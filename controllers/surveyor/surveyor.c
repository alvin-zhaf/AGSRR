#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/compass.h>
#include <webots/distance_sensor.h>
#include <webots/keyboard.h>
#include <webots/camera.h>

// Movement constants 
#define MAX_SPEED 0.25
#define TOLERANCE 0.05
#define SLOW_DOWN_DISTANCE 1.0
#define OBSTACLE_THRESHOLD 1.0
#define OBSTACLE_SUSTAINED_FRAMES 5
#define TERRAIN_MAX_CHANGE 70.0
#define PAUSE_STEPS 50

// SLAM / Mapping constants
#define MAP_SIZE 20
#define WORLD_MIN_X  0.1
#define WORLD_MAX_X  4.0
#define WORLD_MIN_Y  0.0
#define WORLD_MAX_Y  3.9

// Localization constants
#define GRID_SIZE 10

// Victim detection constants
#define MAX_VICTIMS 20
#define VICTIM_STOP_DURATION 50
#define GREEN_THRESHOLD_MIN 80
#define GREEN_DOMINANCE 15
#define GREEN_RATIO_THRESHOLD 0.10

// Predefined waypoints 
#define NUM_WAYPOINTS 9
static const double WAYPOINTS[NUM_WAYPOINTS][2] = {
  {3.36, 2.86},
  {3.32, 2.08},
  {0.32, 2.08},
  {0.33, 1.39},
  {3.32, 1.39},
  {3.32, 0.80},
  {0.33, 0.80},
  {0.32, 0.11},
  {3.32, 0.11}
};

// SLAM Data
static double belief[GRID_SIZE][GRID_SIZE];
static double map_grid[MAP_SIZE][MAP_SIZE];

// Victim Data
typedef struct {
  double x, y;
  int grid_x, grid_y;
} Victim;

static Victim victims[MAX_VICTIMS];
static int victim_count = 0;

// Types
typedef enum {
  MODE_KEYBOARD,
  MODE_AUTONOMOUS
} RobotMode;

typedef enum {
  STATE_INIT,
  STATE_NAVIGATE,
  STATE_OBSTACLE_CHECK,
  STATE_AVOID_OBSTACLE,
  STATE_SLAM_DETOUR,
  STATE_PAUSE,
  STATE_VICTIM_DETECTED,
  STATE_IDLE,
  STATE_COMPLETE
} RobotState;

typedef struct {
  double x, y, heading;
} Pose;

typedef struct {
  WbDeviceTag left, right;
} Motors;

typedef struct {
  RobotState state;
  RobotState prev_state;  // State to return to after victim detection
  double target_x, target_y;
  double prev_front_distance;
  int obstacle_frames;
  double detour_x, detour_y;
  int pause_counter;
  int current_waypoint;
  bool waypoint_reached;
  int victim_stop_counter;
  double victim_x, victim_y;
  // SLAM detour fields
  double slam_detour_x, slam_detour_y;
  double original_target_x, original_target_y;
  int path_check_counter;
} RobotContext;

// Helper functions
static double normalize_angle(double angle) {
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

static Pose get_robot_pose(WbDeviceTag gps, WbDeviceTag compass) {
  Pose pose = {0};
  const double *gps_values = wb_gps_get_values(gps);
  const double *compass_values = wb_compass_get_values(compass);
  if (gps_values) {
    pose.x = gps_values[0];
    pose.y = gps_values[1];
  }
  if (compass_values) {
    pose.heading = atan2(compass_values[0], compass_values[1]);
  }
  return pose;
}

static void set_motor_speeds(Motors motors, double left, double right) {
  wb_motor_set_velocity(motors.left, left);
  wb_motor_set_velocity(motors.right, right);
}

static double distance_to_point(Pose pose, double x, double y) {
  double dx = x - pose.x, dy = y - pose.y;
  return sqrt(dx * dx + dy * dy);
}

// drive_to_target function
static void drive_to_target(Motors motors, Pose pose, double target_x, double target_y) {
  double dx = target_x - pose.x, dy = target_y - pose.y;
  double distance = sqrt(dx * dx + dy * dy);
  double desired_heading = atan2(dy, dx);
  double heading_error = normalize_angle(desired_heading - pose.heading);

  double speed = MAX_SPEED;
  if (distance < SLOW_DOWN_DISTANCE)
    speed *= (0.3 + 0.7 * distance / SLOW_DOWN_DISTANCE);

  if (fabs(heading_error) > TOLERANCE * 3.0) {
    double turn_speed = speed * 0.5;
    set_motor_speeds(motors, heading_error > 0 ? -turn_speed : turn_speed, heading_error > 0 ? turn_speed : -turn_speed);
  }
  else {
    double correction = heading_error * 0.4;
    set_motor_speeds(motors, speed * (1.0 - correction * 0.3), speed * (1.0 + correction * 0.3));
  }
}

// set_next_waypoint 
static bool set_next_waypoint(RobotContext *ctx) {
  if (ctx->current_waypoint < NUM_WAYPOINTS) {
    ctx->target_x = WAYPOINTS[ctx->current_waypoint][0];
    ctx->target_y = WAYPOINTS[ctx->current_waypoint][1];
    printf("Navigating to waypoint %d: (%.2f, %.2f)\n", 
           ctx->current_waypoint + 1, ctx->target_x, ctx->target_y);
    return true;
  }
  return false;
}

// SLAM: Markov localization 
static void markov_init(void) {
  double uniform = 1.0 / (GRID_SIZE * GRID_SIZE);
  for (int i = 0; i < GRID_SIZE; i++)
    for (int j = 0; j < GRID_SIZE; j++)
      belief[i][j] = uniform;
}

static void normalize_belief(void) {
  double total = 0.0;
  for (int i = 0; i < GRID_SIZE; i++)
    for (int j = 0; j < GRID_SIZE; j++)
      total += belief[i][j];

  if (total <= 0.0) {
    markov_init();
    return;
  }

  for (int i = 0; i < GRID_SIZE; i++)
    for (int j = 0; j < GRID_SIZE; j++)
      belief[i][j] /= total;
}

static int world_to_grid(double val, double min_v, double max_v) {
  double r = (val - min_v) / (max_v - min_v);
  int idx = (int)(r * GRID_SIZE);
  if (idx < 0) idx = 0;
  if (idx >= GRID_SIZE) idx = GRID_SIZE - 1;
  return idx;
}

static void markov_step(Pose pose) {
  static double temp[GRID_SIZE][GRID_SIZE];
  memset(temp, 0, sizeof(temp));

  int dx = (int)round(cos(pose.heading));
  int dy = (int)round(sin(pose.heading));

  for (int i = 0; i < GRID_SIZE; i++) {
    for (int j = 0; j < GRID_SIZE; j++) {
      int nx = i + dx, ny = j + dy;
      if (nx >= 0 && nx < GRID_SIZE && ny >= 0 && ny < GRID_SIZE)
        temp[nx][ny] += belief[i][j];
      else
        temp[i][j] += belief[i][j];
    }
  }

  memcpy(belief, temp, sizeof(belief));

  int gx = world_to_grid(pose.x, WORLD_MIN_X, WORLD_MAX_X);
  int gy = world_to_grid(pose.y, WORLD_MIN_Y, WORLD_MAX_Y);

  for (int i = 0; i < GRID_SIZE; i++) {
    for (int j = 0; j < GRID_SIZE; j++) {
      double dist = sqrt((gx - i) * (gx - i) + (gy - j) * (gy - j));
      belief[i][j] *= exp(-dist * 1.5);
    }
  }
  normalize_belief();
}

// SLAM: Occupancy grid mapping
static void map_init(void) {
  for (int i = 0; i < MAP_SIZE; i++)
    for (int j = 0; j < MAP_SIZE; j++)
      map_grid[i][j] = -1.0;  
}

static int world_to_map_x(double x) {
  double r = (x - WORLD_MIN_X) / (WORLD_MAX_X - WORLD_MIN_X);
  int idx = (int)(r * MAP_SIZE);
  if (idx < 0) idx = 0;
  if (idx >= MAP_SIZE) idx = MAP_SIZE - 1;
  return idx;
}

static int world_to_map_y(double y) {
  double r = (y - WORLD_MIN_Y) / (WORLD_MAX_Y - WORLD_MIN_Y);
  int idx = (int)(r * MAP_SIZE);
  if (idx < 0) idx = 0;
  if (idx >= MAP_SIZE) idx = MAP_SIZE - 1;
  return idx;
}

static double ir_raw_to_dist(double raw) {
  double d = 0.4 * (1.0 - raw / 154.0);
  if (d < 0.0) d = 0.0;
  if (d > 0.4) d = 0.4;
  return d;
}

static void update_map_ray(double rx, double ry, double angle, double dist) {
  const double step = 0.05;
  const double max_range = 0.4;

  // Mark free space along the way
  for (double d = 0.0; d < dist && d < max_range; d += step) {
    double wx = rx + d * cos(angle);
    double wy = ry + d * sin(angle);
    int mx = world_to_map_x(wx);
    int my = world_to_map_y(wy);

    double val = map_grid[mx][my];
    if (val < 0.0) val = 0.5;
    val -= 0.05;
    if (val < 0.0) val = 0.0;
    map_grid[mx][my] = val;
  }

  // Mark obstacle at endpoint
  if (dist < max_range) {
    double wx = rx + dist * cos(angle);
    double wy = ry + dist * sin(angle);
    int mx = world_to_map_x(wx);
    int my = world_to_map_y(wy);
    map_grid[mx][my] = 1.0;
  }
}

// function for updating map coordinates
static void update_map_from_sensors(Pose pose, WbDeviceTag ds0, WbDeviceTag ds1, 
                                     WbDeviceTag ds2, WbDeviceTag ds3) {
  if (!ds0 || !ds1 || !ds2 || !ds3) return;

  double raw0 = wb_distance_sensor_get_value(ds0);
  double raw1 = wb_distance_sensor_get_value(ds1);
  double raw2 = wb_distance_sensor_get_value(ds2);
  double raw3 = wb_distance_sensor_get_value(ds3);

  double d0 = ir_raw_to_dist(raw0);
  double d1 = ir_raw_to_dist(raw1);
  double d2 = ir_raw_to_dist(raw2);
  double d3 = ir_raw_to_dist(raw3);

  update_map_ray(pose.x, pose.y, pose.heading, d0);
  update_map_ray(pose.x, pose.y, pose.heading + M_PI_2, d1);
  update_map_ray(pose.x, pose.y, pose.heading + M_PI, d2);
  update_map_ray(pose.x, pose.y, pose.heading - M_PI_2, d3);
}

// function for printing map in a grid like format
static void print_map(void) {
  printf("\n--- MAP GRID ---\n");
  for (int j = MAP_SIZE - 1; j >= 0; j--) {
    for (int i = 0; i < MAP_SIZE; i++) {
      double v = map_grid[i][j];
      char c = '?';
      if (v < 0.0) c = '?';
      else if (v < 0.3) c = '.';
      else if (v > 0.7) c = '#';
      else c = '~';
      printf("%c", c);
    }
    printf("\n");
  }
  printf("--------------------\n");
}

// Victim Detection and Storage
static bool detect_green_victim(WbDeviceTag camera, int *debug_r, int *debug_g, int *debug_b, int *debug_green_pixels, int *debug_total) {
  if (!camera) return false;
  
  const unsigned char *image = wb_camera_get_image(camera);
  if (!image) return false;
  
  int w = wb_camera_get_width(camera);
  int h = wb_camera_get_height(camera);
  int cx = w / 2;
  int cy = h / 2;
  
  // Sample center region of camera
  int green_pixels = 0;
  int total_pixels = 0;
  int avg_r = 0, avg_g = 0, avg_b = 0;
  
  int sample_radius = 15;  // Larger sample area
  for (int dy = -sample_radius; dy <= sample_radius; dy++) {
    for (int dx = -sample_radius; dx <= sample_radius; dx++) {
      int x = cx + dx;
      int y = cy + dy;
      if (x < 0 || x >= w || y < 0 || y >= h) continue;
      
      int r = wb_camera_image_get_red(image, w, x, y);
      int g = wb_camera_image_get_green(image, w, x, y);
      int b = wb_camera_image_get_blue(image, w, x, y);
      
      avg_r += r;
      avg_g += g;
      avg_b += b;
      total_pixels++;
      
      // check pixel colors if green increase the sensitivity for the detection
      if (g > GREEN_THRESHOLD_MIN && 
          g > r + GREEN_DOMINANCE && 
          g > b + GREEN_DOMINANCE) {
        green_pixels++;
      }
    }
  }
  
  if (total_pixels > 0) {
    *debug_r = avg_r / total_pixels;
    *debug_g = avg_g / total_pixels;
    *debug_b = avg_b / total_pixels;
  }
  *debug_green_pixels = green_pixels;
  *debug_total = total_pixels;
  
  // Return true if significant portion of center is green
  double green_ratio = (double)green_pixels / total_pixels;
  return (green_ratio > GREEN_RATIO_THRESHOLD);
}

static bool is_victim_already_found(double x, double y) {
  for (int i = 0; i < victim_count; i++) {
    double dx = victims[i].x - x;
    double dy = victims[i].y - y;
    double dist = sqrt(dx * dx + dy * dy);
    if (dist < 0.3) {  // Within 30cm of known victim
      return true;
    }
  }
  return false;
}

static void store_victim(double x, double y) {
  if (victim_count >= MAX_VICTIMS) {
    printf("[VICTIM] Warning: Maximum victims reached, cannot store more\n");
    return;
  }
  
  if (is_victim_already_found(x, y)) {
    printf("[VICTIM] Victim at (%.2f, %.2f) already recorded\n", x, y);
    return;
  }
  
  // Store victim world coordinates
  victims[victim_count].x = x;
  victims[victim_count].y = y;
  
  // Calculate and store grid position
  int gx = world_to_grid(x, WORLD_MIN_X, WORLD_MAX_X);
  int gy = world_to_grid(y, WORLD_MIN_Y, WORLD_MAX_Y);
  victims[victim_count].grid_x = gx;
  victims[victim_count].grid_y = gy;
  
  // Mark victim location in belief grid with high value
  belief[gx][gy] = 2.0;  // Special marker for victim
  
  victim_count++;
  printf("[VICTIM] *** NEW VICTIM STORED ***\n");
  printf("[VICTIM] Victim #%d at world (%.2f, %.2f), grid (%d, %d)\n", 
         victim_count, x, y, gx, gy);
}

static void print_victims(void) {
  printf("\n--- VICTIMS FOUND: %d ---\n", victim_count);
  for (int i = 0; i < victim_count; i++) {
    printf("  Victim #%d: world (%.2f, %.2f), grid (%d, %d)\n",
           i + 1, victims[i].x, victims[i].y, victims[i].grid_x, victims[i].grid_y);
  }
  printf("-------------------------\n");
}

// SLAM-Based Path Planning
#define PATH_CHECK_DISTANCE 0.5
#define PATH_CHECK_STEP 0.05
#define OCCUPIED_THRESHOLD 0.6
#define DETOUR_OFFSET 0.3

// Check if a specific map cell is occupied
static bool is_cell_occupied(int mx, int my) {
  if (mx < 0 || mx >= MAP_SIZE || my < 0 || my >= MAP_SIZE) {
    return true;  // Out of bounds treated as occupied
  }
  return map_grid[mx][my] > OCCUPIED_THRESHOLD;
}

// Check path from current position toward target, up to PATH_CHECK_DISTANCE
// Returns true if path is blocked, and sets obstacle_x, obstacle_y to obstacle location
static bool check_path_blocked(Pose pose, double target_x, double target_y, 
                                double *obstacle_x, double *obstacle_y) {
  double dx = target_x - pose.x;
  double dy = target_y - pose.y;
  double dist = sqrt(dx * dx + dy * dy);
  
  if (dist < 0.01) return false;  // Already at target
  
  // Normalize direction
  double dir_x = dx / dist;
  double dir_y = dy / dist;
  
  // Check up to PATH_CHECK_DISTANCE or until target
  double check_dist = (dist < PATH_CHECK_DISTANCE) ? dist : PATH_CHECK_DISTANCE;
  
  for (double d = 0.1; d <= check_dist; d += PATH_CHECK_STEP) {
    double wx = pose.x + d * dir_x;
    double wy = pose.y + d * dir_y;
    
    int mx = world_to_map_x(wx);
    int my = world_to_map_y(wy);
    
    if (is_cell_occupied(mx, my)) {
      *obstacle_x = wx;
      *obstacle_y = wy;
      printf("[SLAM] Path blocked at (%.2f, %.2f) - map cell (%d, %d) = %.2f\n",
             wx, wy, mx, my, map_grid[mx][my]);
      return true;
    }
  }
  
  return false;
}

// Check if a detour point is clear (not occupied)
static bool is_detour_clear(double x, double y) {
  int mx = world_to_map_x(x);
  int my = world_to_map_y(y);
  
  // Check the cell and immediate neighbors
  for (int di = -1; di <= 1; di++) {
    for (int dj = -1; dj <= 1; dj++) {
      if (is_cell_occupied(mx + di, my + dj)) {
        return false;
      }
    }
  }
  return true;
}

// Calculate detour waypoint to avoid obstacle
// Returns true if detour found, false if no clear path
static bool calculate_detour(Pose pose, double target_x, double target_y,
                             double obstacle_x, double obstacle_y,
                             double *detour_x, double *detour_y) {
  // Direction to target
  double dx = target_x - pose.x;
  double dy = target_y - pose.y;
  double dist = sqrt(dx * dx + dy * dy);
  
  if (dist < 0.01) return false;
  
  // Perpendicular directions (left and right)
  double perp_left_x = -dy / dist;
  double perp_left_y = dx / dist;
  double perp_right_x = dy / dist;
  double perp_right_y = -dx / dist;
  
  // Calculate potential detour points
  double left_x = obstacle_x + DETOUR_OFFSET * perp_left_x;
  double left_y = obstacle_y + DETOUR_OFFSET * perp_left_y;
  double right_x = obstacle_x + DETOUR_OFFSET * perp_right_x;
  double right_y = obstacle_y + DETOUR_OFFSET * perp_right_y;
  
  bool left_clear = is_detour_clear(left_x, left_y);
  bool right_clear = is_detour_clear(right_x, right_y);
  
  printf("[SLAM] Detour options - Left (%.2f, %.2f): %s, Right (%.2f, %.2f): %s\n",
         left_x, left_y, left_clear ? "CLEAR" : "blocked",
         right_x, right_y, right_clear ? "CLEAR" : "blocked");
  
  if (left_clear && right_clear) {
    // Both clear - 50/50 random choice
    if (rand() % 2 == 0) {
      *detour_x = left_x;
      *detour_y = left_y;
      printf("[SLAM] Randomly chose LEFT detour\n");
    } else {
      *detour_x = right_x;
      *detour_y = right_y;
      printf("[SLAM] Randomly chose RIGHT detour\n");
    }
    return true;
  } else if (left_clear) {
    *detour_x = left_x;
    *detour_y = left_y;
    printf("[SLAM] Chose LEFT detour (right blocked)\n");
    return true;
  } else if (right_clear) {
    *detour_x = right_x;
    *detour_y = right_y;
    printf("[SLAM] Chose RIGHT detour (left blocked)\n");
    return true;
  }
  
  printf("[SLAM] WARNING: No clear detour found!\n");
  return false;
}

// MAIN function
int main(int argc, char **argv) {
  wb_robot_init();
  const int timeStep = wb_robot_get_basic_time_step();

  wb_keyboard_enable(timeStep);

  printf("Select mode (click 3D window first):\n");
  printf("  1 = Keyboard control with Markov localization\n");
  printf("  2 = Autonomous waypoint navigation\n");

  RobotMode mode = MODE_AUTONOMOUS;
  bool mode_selected = false;
  // List of available mode options
  while (wb_robot_step(timeStep) != -1 && !mode_selected) {
    int key = wb_keyboard_get_key();
    if (key == '1') {
      mode = MODE_KEYBOARD;
      mode_selected = true;
      printf("Mode: Keyboard control\n");
    }
    else if (key == '2') {
      mode = MODE_AUTONOMOUS;
      mode_selected = true;
      printf("Mode: Autonomous waypoint navigation\n");
    }
  }

  Motors motors = {wb_robot_get_device("left motor"), wb_robot_get_device("right motor")};
  WbDeviceTag gps = wb_robot_get_device("gps");
  WbDeviceTag compass = wb_robot_get_device("compass");

  wb_gps_enable(gps, timeStep);
  wb_compass_enable(compass, timeStep);
  wb_motor_set_position(motors.left, INFINITY);
  wb_motor_set_position(motors.right, INFINITY);
  set_motor_speeds(motors, 0, 0);

  WbDeviceTag front_ds = 0, camera = 0;
  WbDeviceTag ds0 = 0, ds1 = 0, ds2 = 0, ds3 = 0;

  // Initialize SLAM
  markov_init();
  map_init();

  // Enable camera for both modes
  // Try front-facing camera first, fall back to ground_cam if fail
  camera = wb_robot_get_device("camera");
  if (!camera) {
    camera = wb_robot_get_device("front_camera");
  }
  if (!camera) {
    camera = wb_robot_get_device("cam");
  }
  if (!camera) {
    camera = wb_robot_get_device("ground_cam");
  }
  
  if (camera) {
    wb_camera_enable(camera, timeStep);
    printf("Camera enabled for victim detection\n");
  } else {
    printf("WARNING: No camera found for victim detection!\n");
  }

  if (mode == MODE_KEYBOARD) {
    printf("\nKeyboard mode: W=forward, S=backward, A=left, D=right, M=print map, V=print victims\n");
  }
  else {
    front_ds = wb_robot_get_device("ds0");
    if (front_ds)
      wb_distance_sensor_enable(front_ds, timeStep);
    
    printf("\nAutonomous mode: Traversing %d waypoints...\n", NUM_WAYPOINTS);
    printf("Waypoint list:\n");
    for (int i = 0; i < NUM_WAYPOINTS; i++) {
      printf("  %d: (%.2f, %.2f)\n", i + 1, WAYPOINTS[i][0], WAYPOINTS[i][1]);
    }
    printf("\nGreen victim detection ENABLED\n");
    printf("\n");
  }

  // Enable all distance sensors for SLAM mapping
  ds0 = wb_robot_get_device("ds0");
  ds1 = wb_robot_get_device("ds1");
  ds2 = wb_robot_get_device("ds2");
  ds3 = wb_robot_get_device("ds3");
  if (ds0) wb_distance_sensor_enable(ds0, timeStep);
  if (ds1) wb_distance_sensor_enable(ds1, timeStep);
  if (ds2) wb_distance_sensor_enable(ds2, timeStep);
  if (ds3) wb_distance_sensor_enable(ds3, timeStep);

  RobotContext ctx = {
    .state = STATE_INIT,
    .prev_state = STATE_INIT,
    .current_waypoint = 0,
    .obstacle_frames = 0,
    .pause_counter = 0,
    .waypoint_reached = false,
    .victim_stop_counter = 0,
    .victim_x = 0,
    .victim_y = 0,
    .slam_detour_x = 0,
    .slam_detour_y = 0,
    .original_target_x = 0,
    .original_target_y = 0,
    .path_check_counter = 0
  };
  int init_counter = 0;
  int map_print_counter = 0;
  int victim_debug_counter = 0;
  
  // Seed random for 50/50 detour choice
  srand(42);

  while (wb_robot_step(timeStep) != -1) {
    Pose pose = get_robot_pose(gps, compass);

    // SLAM updates - run every step
    markov_step(pose);
    update_map_from_sensors(pose, ds0, ds1, ds2, ds3);

    // Check for map print request
    int key = wb_keyboard_get_key();
    if (key == 'M' || key == 'm') {
      print_map();
    }
    if (key == 'V' || key == 'v') {
      print_victims();
    }

    if (mode == MODE_KEYBOARD) {
      double left_speed = 0, right_speed = 0;
      if (key == 'W' || key == 'w') {
        left_speed = 0.3;
        right_speed = 0.3;
      }
      else if (key == 'S' || key == 's') {
        left_speed = -0.3;
        right_speed = -0.3;
      }
      else if (key == 'A' || key == 'a') {
        left_speed = -0.2;
        right_speed = 0.2;
      }
      else if (key == 'D' || key == 'd') {
        left_speed = 0.2;
        right_speed = -0.2;
      }
      set_motor_speeds(motors, left_speed, right_speed);

      if (camera) {
        const unsigned char *image = wb_camera_get_image(camera);
        if (image) {
          int w = wb_camera_get_width(camera), h = wb_camera_get_height(camera);
          printf("Pos:(%.2f,%.2f) Cam RGB:(%d,%d,%d)\n", pose.x, pose.y,
                 wb_camera_image_get_red(image, w / 2, h / 2, w),
                 wb_camera_image_get_green(image, w / 2, h / 2, w),
                 wb_camera_image_get_blue(image, w / 2, h / 2, w));
        }
      }
    }
    // Autonomous waypoint navigation
    else 
    {
      double front_distance = front_ds ? wb_distance_sensor_get_value(front_ds) : 0;
      
      // Victim detection
      int debug_r = 0, debug_g = 0, debug_b = 0, debug_green_px = 0, debug_total_px = 0;
      bool green_detected = detect_green_victim(camera, &debug_r, &debug_g, &debug_b, &debug_green_px, &debug_total_px);
      
      if (++victim_debug_counter >= 16) {
        printf("[DEBUG] Cam RGB avg: R=%d G=%d B=%d | Green pixels: %d/%d | Detected: %s\n", 
               debug_r, debug_g, debug_b, debug_green_px, debug_total_px,
               green_detected ? "YES" : "no");
        victim_debug_counter = 0;
      }
      
      // Check for new victim 
      if (green_detected && 
          ctx.state != STATE_VICTIM_DETECTED && 
          ctx.state != STATE_INIT &&
          ctx.state != STATE_COMPLETE &&
          ctx.state != STATE_IDLE &&
          ctx.state != STATE_PAUSE) {
        
        // Calculate victim position 
        double victim_dist = 0.15;  // Approximate distance to victim
        double vx = pose.x + victim_dist * cos(pose.heading);
        double vy = pose.y + victim_dist * sin(pose.heading);
        
        if (!is_victim_already_found(vx, vy)) {
          printf("\n[VICTIM] !!! GREEN VICTIM DETECTED !!!\n");
          printf("[VICTIM] Robot at (%.2f, %.2f), heading %.2f rad\n", 
                 pose.x, pose.y, pose.heading);
          printf("[VICTIM] Estimated victim position: (%.2f, %.2f)\n", vx, vy);
          
          ctx.prev_state = ctx.state;
          ctx.state = STATE_VICTIM_DETECTED;
          ctx.victim_stop_counter = 0;
          ctx.victim_x = vx;
          ctx.victim_y = vy;
          set_motor_speeds(motors, 0, 0);
        }
      }

      switch (ctx.state) {
      case STATE_INIT:
        if (++init_counter >= 10) {
          ctx.prev_front_distance = front_distance;
          if (set_next_waypoint(&ctx)) {
            ctx.state = STATE_NAVIGATE;
          } else {
            ctx.state = STATE_COMPLETE;
          }
        }
        break;
      
      case STATE_IDLE:
        set_motor_speeds(motors, 0, 0);
        break;
      
      case STATE_VICTIM_DETECTED:
        set_motor_speeds(motors, 0, 0);  // Stay stopped
        ctx.victim_stop_counter++;
        
        if (ctx.victim_stop_counter == 1) {
          printf("[VICTIM] Robot stopped, looking at victim...\n");
        }
        
        if (ctx.victim_stop_counter >= VICTIM_STOP_DURATION) {
          // Store the victim
          store_victim(ctx.victim_x, ctx.victim_y);
          print_victims();
          
          printf("[VICTIM] Resuming navigation...\n\n");
          ctx.state = ctx.prev_state;
          ctx.victim_stop_counter = 0;
        }
        break;
      
      case STATE_COMPLETE:
        set_motor_speeds(motors, 0, 0);
        printf("All waypoints completed! Robot stopped.\n");
        print_map();
        print_victims();
        ctx.state = STATE_IDLE; 
        break;
        
      case STATE_NAVIGATE:
      {
        double change = fabs(front_distance - ctx.prev_front_distance);
        ctx.prev_front_distance = front_distance;
        
        if (distance_to_point(pose, ctx.target_x, ctx.target_y) < TOLERANCE)
        {
          printf("Reached waypoint %d: (%.2f, %.2f)\n", 
                 ctx.current_waypoint + 1, ctx.target_x, ctx.target_y);
          ctx.current_waypoint++;
          ctx.pause_counter = 0;
          ctx.waypoint_reached = true;  
          set_motor_speeds(motors, 0, 0);
          ctx.state = STATE_PAUSE;
        }
        // path checking using SLAM, set to once every 10 steps preventing overload/spamming
        else if (++ctx.path_check_counter >= 10) {
          ctx.path_check_counter = 0;
          
          double obstacle_x, obstacle_y;
          if (check_path_blocked(pose, ctx.target_x, ctx.target_y, &obstacle_x, &obstacle_y)) {
            double detour_x, detour_y;
            if (calculate_detour(pose, ctx.target_x, ctx.target_y, 
                                 obstacle_x, obstacle_y, &detour_x, &detour_y)) {
              printf("[SLAM] Inserting detour waypoint (%.2f, %.2f) before target (%.2f, %.2f)\n",
                     detour_x, detour_y, ctx.target_x, ctx.target_y);
              
              // Save original target
              ctx.original_target_x = ctx.target_x;
              ctx.original_target_y = ctx.target_y;
              
              // Set detour as immediate target
              ctx.slam_detour_x = detour_x;
              ctx.slam_detour_y = detour_y;
              
              ctx.state = STATE_SLAM_DETOUR;
            } else {
              // No clear detour, fall back to reactive obstacle avoidance
              drive_to_target(motors, pose, ctx.target_x, ctx.target_y);
            }
          } else {
            drive_to_target(motors, pose, ctx.target_x, ctx.target_y);
          }
        }
        else if (change <= TERRAIN_MAX_CHANGE && front_distance > OBSTACLE_THRESHOLD)
        {
          ctx.obstacle_frames = 1;
          ctx.state = STATE_OBSTACLE_CHECK;
        }
        else
        {
          ctx.obstacle_frames = 0;
          drive_to_target(motors, pose, ctx.target_x, ctx.target_y);
        }
        break;
      }
      
      case STATE_SLAM_DETOUR:
      {
        // Navigate to SLAM-calculated detour point
        double dist_to_detour = distance_to_point(pose, ctx.slam_detour_x, ctx.slam_detour_y);
        
        if (dist_to_detour < TOLERANCE) {
          printf("[SLAM] Detour point reached, resuming to original target (%.2f, %.2f)\n",
                 ctx.original_target_x, ctx.original_target_y);
          
          // Restore original target
          ctx.target_x = ctx.original_target_x;
          ctx.target_y = ctx.original_target_y;
          ctx.path_check_counter = 0;
          ctx.state = STATE_NAVIGATE;
        } else {
          drive_to_target(motors, pose, ctx.slam_detour_x, ctx.slam_detour_y);
        }
        break;
      }
      
      case STATE_OBSTACLE_CHECK:
      {
        double change = fabs(front_distance - ctx.prev_front_distance);
        ctx.prev_front_distance = front_distance;

        if (change > TERRAIN_MAX_CHANGE)
        {
          ctx.obstacle_frames = 0;
          ctx.state = STATE_NAVIGATE;
        }
        else if (front_distance > OBSTACLE_THRESHOLD)
        {
          if (++ctx.obstacle_frames >= OBSTACLE_SUSTAINED_FRAMES)
          {
            printf("Obstacle detected! Initiating detour...\n");
            ctx.detour_x = pose.x + 0.3 * sin(pose.heading);
            ctx.detour_y = pose.y - 0.3 * cos(pose.heading);
            ctx.obstacle_frames = 0;
            ctx.state = STATE_AVOID_OBSTACLE;
          }
          else
          {
            drive_to_target(motors, pose, ctx.target_x, ctx.target_y);
          }
        }
        else
        {
          ctx.obstacle_frames = 0;
          ctx.state = STATE_NAVIGATE;
        }
        break;
      }
      
      case STATE_AVOID_OBSTACLE:
        if (distance_to_point(pose, ctx.detour_x, ctx.detour_y) < TOLERANCE) {
          printf("Detour complete, resuming navigation to waypoint %d...\n", ctx.current_waypoint + 1);
          ctx.pause_counter = 0;
          ctx.waypoint_reached = false;
          set_motor_speeds(motors, 0, 0);
          ctx.state = STATE_PAUSE;
        }
        else {
          drive_to_target(motors, pose, ctx.detour_x, ctx.detour_y);
        }
        break;
      
      case STATE_PAUSE:
        if (++ctx.pause_counter >= PAUSE_STEPS) {
          if (ctx.waypoint_reached) {
            if (ctx.current_waypoint < NUM_WAYPOINTS) {
              if (set_next_waypoint(&ctx)) {
                ctx.state = STATE_NAVIGATE;
              } else {
                ctx.state = STATE_COMPLETE;
              }
            } else {
              ctx.state = STATE_COMPLETE;
            }
          }
          else {
            printf("Resuming navigation to waypoint %d: (%.2f, %.2f)\n",
                   ctx.current_waypoint + 1, ctx.target_x, ctx.target_y);
            ctx.state = STATE_NAVIGATE;
          }
        }
        break;
      }

      // Print map periodically during autonomous mode
      if (++map_print_counter >= 500) {
        print_map();
        map_print_counter = 0;
      }
    }
  }

  wb_robot_cleanup();
  return 0;
}