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
#include <webots/receiver.h>

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

// Predefined waypoints for SLAM mode
#define NUM_WAYPOINTS 9
static const double WAYPOINTS[NUM_WAYPOINTS][2] = {
  {3.36, 2.86},
  {3.32, 2.08},
  {0.32, 2.08},
  {0.33, 1.39},
  {3.32, 1.39},
  {3.32, 0.80},
  {0.33, 0.80},
  {0.32, 0.40},
  {3.32, 0.40}
};

// SLAM Data
static double map_grid[MAP_SIZE][MAP_SIZE];

// Victim Data
typedef struct {
  double x, y;
  double detection_angle;
  int grid_x, grid_y;
} Victim;

static Victim victims[MAX_VICTIMS];
static int victim_count = 0;

// Main Operation Modes
typedef enum {
  MODE_AUTONOMOUS,
  MODE_AUTONOMOUS_SLAM
} RobotMode;

// FSM States
typedef enum {
  STATE_INIT,
  STATE_NAVIGATE,
  STATE_OBSTACLE_CHECK,
  STATE_AVOID_OBSTACLE,
  STATE_SLAM_DETOUR,
  STATE_PAUSE,
  STATE_VICTIM_DETECTED,
  STATE_IDLE,
  STATE_COMPLETE,
  STATE_REVISIT_VICTIMS,
  STATE_ALIGN_TO_VICTIM,
  STATE_APPROACH_VICTIM
} RobotState;

typedef enum {
  PHASE_INITIAL_SCAN,
  PHASE_REVISIT_VICTIMS
} SlamPhase;

typedef struct {
  double x, y, heading;
} Pose;

typedef struct {
  WbDeviceTag left, right;
} Motors;

// Robot Context
typedef struct {
  RobotState state;
  RobotState prev_state;
  double target_x, target_y;
  double prev_front_distance;
  int obstacle_frames;
  double detour_x, detour_y;
  int pause_counter;
  bool has_target;
  // SLAM mode specific fields
  int current_waypoint;
  bool waypoint_reached;
  int victim_stop_counter;
  double victim_x, victim_y;
  double slam_detour_x, slam_detour_y;
  double original_target_x, original_target_y;
  int path_check_counter;
  // Revisit phase fields
  SlamPhase slam_phase;
  int current_revisit_index;
  // Approach phase fields
  double approach_x, approach_y;
  double final_victim_x, final_victim_y;
  double target_angle;
  bool green_was_seen;
} RobotContext;

// SLAM Path Planning constants
#define PATH_CHECK_DISTANCE 0.5
#define PATH_CHECK_STEP 0.05
#define OCCUPIED_THRESHOLD 0.6
#define DETOUR_OFFSET 0.3

// Victim approach constants
#define VICTIM_APPROACH_STANDOFF 0.3
#define VICTIM_APPROACH_SPEED 0.1
#define VICTIM_ALIGN_TOLERANCE 0.05
#define VICTIM_ALIGN_SPEED 0.08

// Helper functions

// Normalizes angle to be between -PI and +PI (Standard Radians)
static double normalize_angle(double angle) {
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

// Reads GPS and Compass to create a Pose struct
static Pose get_robot_pose(WbDeviceTag gps, WbDeviceTag compass) {
  Pose pose = {0};
  const double *gps_values = wb_gps_get_values(gps);
  const double *compass_values = wb_compass_get_values(compass);
  if (gps_values) {
    pose.x = gps_values[0];
    pose.y = gps_values[1];
  }
  // Convert compass vector (North) to Heading Angle (Radians)
  if (compass_values) {
    pose.heading = atan2(compass_values[0], compass_values[1]);
  }
  return pose;
}

// Set motor speed
static void set_motor_speeds(Motors motors, double left, double right) {
  wb_motor_set_velocity(motors.left, left);
  wb_motor_set_velocity(motors.right, right);
}

// Euclidean distance between robot and point
static double distance_to_point(Pose pose, double x, double y) {
  double dx = x - pose.x, dy = y - pose.y;
  return sqrt(dx * dx + dy * dy);
}

static void drive_to_target(Motors motors, Pose pose, double target_x, double target_y) {
  double dx = target_x - pose.x, dy = target_y - pose.y;
  double distance = sqrt(dx * dx + dy * dy);
  double desired_heading = atan2(dy, dx);
  
  // Calculate how much we need to turn
  double heading_error = normalize_angle(desired_heading - pose.heading);

  double speed = MAX_SPEED;
  if (distance < SLOW_DOWN_DISTANCE) // Decelerate smoothly
    speed *= (0.3 + 0.7 * distance / SLOW_DOWN_DISTANCE);

  if (fabs(heading_error) > TOLERANCE * 3.0) {
    // If error is large, turn in place (Pivot)
    double turn_speed = speed * 0.5;
    set_motor_speeds(motors, heading_error > 0 ? -turn_speed : turn_speed, heading_error > 0 ? turn_speed : -turn_speed);
  }
  else {
    // If error is small, drive forward with slight steering correction
    double correction = heading_error * 0.4;
    set_motor_speeds(motors, speed * (1.0 - correction * 0.3), speed * (1.0 + correction * 0.3));
  }
}

// set_next_waypoint for SLAM mode
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

// Set next victim to revisit
static bool set_next_victim_target(RobotContext *ctx) {
  if (ctx->current_revisit_index < victim_count) {
    double vx = victims[ctx->current_revisit_index].x;
    double vy = victims[ctx->current_revisit_index].y;
    double angle = victims[ctx->current_revisit_index].detection_angle;
    
    double standoff_x = vx - VICTIM_APPROACH_STANDOFF * cos(angle);
    double standoff_y = vy - VICTIM_APPROACH_STANDOFF * sin(angle);
    
    ctx->target_x = standoff_x;
    ctx->target_y = standoff_y;
    ctx->approach_x = standoff_x;
    ctx->approach_y = standoff_y;
    ctx->final_victim_x = vx;
    ctx->final_victim_y = vy;
    ctx->target_angle = angle;
    ctx->green_was_seen = false;
    
    printf("[REVISIT] Navigating to victim #%d approach point (%.2f, %.2f)\n", 
           ctx->current_revisit_index + 1, standoff_x, standoff_y);
    printf("[REVISIT] Victim at (%.2f, %.2f), will align to angle: %.2f rad (%.1f deg)\n",
           vx, vy, angle, angle * 180.0 / M_PI);
    return true;
  }
  return false;
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

static int world_to_grid(double val, double min_v, double max_v) {
  double r = (val - min_v) / (max_v - min_v);
  int idx = (int)(r * GRID_SIZE);
  if (idx < 0) idx = 0;
  if (idx >= GRID_SIZE) idx = GRID_SIZE - 1;
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

  if (dist < max_range) {
    double wx = rx + dist * cos(angle);
    double wy = ry + dist * sin(angle);
    int mx = world_to_map_x(wx);
    int my = world_to_map_y(wy);
    map_grid[mx][my] = 1.0;
  }
}

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
  
  int green_pixels = 0;
  int total_pixels = 0;
  int avg_r = 0, avg_g = 0, avg_b = 0;
  
  int sample_radius = 15;
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
  
  double green_ratio = (double)green_pixels / total_pixels;
  return (green_ratio > GREEN_RATIO_THRESHOLD);
}

static bool is_victim_already_found(double x, double y) {
  for (int i = 0; i < victim_count; i++) {
    double dx = victims[i].x - x;
    double dy = victims[i].y - y;
    double dist = sqrt(dx * dx + dy * dy);
    if (dist < 0.3) {
      return true;
    }
  }
  return false;
}

static void store_victim(double x, double y, double angle) {
  if (victim_count >= MAX_VICTIMS) {
    printf("[VICTIM] Warning: Maximum victims reached, cannot store more\n");
    return;
  }
  
  if (is_victim_already_found(x, y)) {
    printf("[VICTIM] Victim at (%.2f, %.2f) already recorded\n", x, y);
    return;
  }
  
  victims[victim_count].x = x;
  victims[victim_count].y = y;
  victims[victim_count].detection_angle = angle;
  
  int gx = world_to_grid(x, WORLD_MIN_X, WORLD_MAX_X);
  int gy = world_to_grid(y, WORLD_MIN_Y, WORLD_MAX_Y);
  victims[victim_count].grid_x = gx;
  victims[victim_count].grid_y = gy;
  
  victim_count++;
  printf("[VICTIM] *** NEW VICTIM STORED ***\n");
  printf("[VICTIM] Victim #%d at world (%.2f, %.2f), grid (%d, %d), angle %.2f rad (%.1f deg)\n", 
         victim_count, x, y, gx, gy, angle, angle * 180.0 / M_PI);
}

static void print_victims(void) {
  printf("\n--- VICTIMS FOUND: %d ---\n", victim_count);
  for (int i = 0; i < victim_count; i++) {
    printf("  Victim #%d: world (%.2f, %.2f), grid (%d, %d), angle %.1f deg\n",
           i + 1, victims[i].x, victims[i].y, victims[i].grid_x, victims[i].grid_y,
           victims[i].detection_angle * 180.0 / M_PI);
  }
  printf("-------------------------\n");
}

// SLAM-Based Path Planning
static bool is_cell_occupied(int mx, int my) {
  if (mx < 0 || mx >= MAP_SIZE || my < 0 || my >= MAP_SIZE) {
    return true;
  }
  return map_grid[mx][my] > OCCUPIED_THRESHOLD;
}

static bool check_path_blocked(Pose pose, double target_x, double target_y, 
                                double *obstacle_x, double *obstacle_y) {
  double dx = target_x - pose.x;
  double dy = target_y - pose.y;
  double dist = sqrt(dx * dx + dy * dy);
  
  if (dist < 0.01) return false;
  
  double dir_x = dx / dist;
  double dir_y = dy / dist;
  
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

static bool is_detour_clear(double x, double y) {
  int mx = world_to_map_x(x);
  int my = world_to_map_y(y);
  
  for (int di = -1; di <= 1; di++) {
    for (int dj = -1; dj <= 1; dj++) {
      if (is_cell_occupied(mx + di, my + dj)) {
        return false;
      }
    }
  }
  return true;
}

static bool calculate_detour(Pose pose, double target_x, double target_y,
                             double obstacle_x, double obstacle_y,
                             double *detour_x, double *detour_y) {
  double dx = target_x - pose.x;
  double dy = target_y - pose.y;
  double dist = sqrt(dx * dx + dy * dy);
  
  if (dist < 0.01) return false;
  
  double perp_left_x = -dy / dist;
  double perp_left_y = dx / dist;
  double perp_right_x = dy / dist;
  double perp_right_y = -dx / dist;
  
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

  // Mode Selection
  printf("Select mode (click 3D window first):\n");
  printf("  1 = Autonomous navigation (drone waypoints)\n");
  printf("  2 = Autonomous navigation with SLAM\n");

  RobotMode mode = MODE_AUTONOMOUS;
  bool mode_selected = false;
  while (wb_robot_step(timeStep) != -1 && !mode_selected) {
    int key = wb_keyboard_get_key();
    if (key == '1') {
      mode = MODE_AUTONOMOUS;
      mode_selected = true;
      printf("Mode: Autonomous navigation (drone waypoints)\n");
    }
    else if (key == '2') {
      mode = MODE_AUTONOMOUS_SLAM;
      mode_selected = true;
      printf("Mode: Autonomous navigation with SLAM\n");
    }
  }
  
  // Hardware Initialisation
  Motors motors = {wb_robot_get_device("left motor"), wb_robot_get_device("right motor")};
  WbDeviceTag gps = wb_robot_get_device("gps");
  WbDeviceTag compass = wb_robot_get_device("compass");

  wb_gps_enable(gps, timeStep);
  wb_compass_enable(compass, timeStep);
  wb_motor_set_position(motors.left, INFINITY);
  wb_motor_set_position(motors.right, INFINITY);
  set_motor_speeds(motors, 0, 0);

  WbDeviceTag front_ds = 0, camera = 0, receiver = 0;
  WbDeviceTag ds0 = 0, ds1 = 0, ds2 = 0, ds3 = 0;

  // Initialize SLAM map
  map_init();

  if (mode == MODE_AUTONOMOUS) {
    front_ds = wb_robot_get_device("ds0");
    if (front_ds)
      wb_distance_sensor_enable(front_ds, timeStep);
    receiver = wb_robot_get_device("receiver");
    if (receiver)
      wb_receiver_enable(receiver, timeStep);
    printf("\nAutonomous mode: Waiting for drone coordinates...\n");
  }
  else if (mode == MODE_AUTONOMOUS_SLAM) {
    // Enable camera for victim detection
    camera = wb_robot_get_device("camera");
    if (!camera) camera = wb_robot_get_device("front_camera");
    if (!camera) camera = wb_robot_get_device("cam");
    if (!camera) camera = wb_robot_get_device("ground_cam");
    
    if (camera) {
      wb_camera_enable(camera, timeStep);
      printf("Camera enabled for victim detection\n");
    } else {
      printf("WARNING: No camera found for victim detection!\n");
    }

    front_ds = wb_robot_get_device("ds0");
    if (front_ds)
      wb_distance_sensor_enable(front_ds, timeStep);
    
    // Enable all distance sensors for SLAM mapping
    ds0 = wb_robot_get_device("ds0");
    ds1 = wb_robot_get_device("ds1");
    ds2 = wb_robot_get_device("ds2");
    ds3 = wb_robot_get_device("ds3");
    if (ds0) wb_distance_sensor_enable(ds0, timeStep);
    if (ds1) wb_distance_sensor_enable(ds1, timeStep);
    if (ds2) wb_distance_sensor_enable(ds2, timeStep);
    if (ds3) wb_distance_sensor_enable(ds3, timeStep);

    printf("\nAutonomous SLAM mode: Traversing %d waypoints...\n", NUM_WAYPOINTS);
    printf("Waypoint list:\n");
    for (int i = 0; i < NUM_WAYPOINTS; i++) {
      printf("  %d: (%.2f, %.2f)\n", i + 1, WAYPOINTS[i][0], WAYPOINTS[i][1]);
    }
    printf("\nGreen victim detection ENABLED\n");
    printf("Phase 1: Complete lawnmower traversal of all waypoints\n");
    printf("Phase 2: Revisit all detected victim locations\n");
    printf("Press M to print map, V to print victims\n\n");
  }

  RobotContext ctx = {
    .state = STATE_INIT,
    .prev_state = STATE_INIT,
    .has_target = false,
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
    .path_check_counter = 0,
    .slam_phase = PHASE_INITIAL_SCAN,
    .current_revisit_index = 0,
    .approach_x = 0,
    .approach_y = 0,
    .final_victim_x = 0,
    .final_victim_y = 0,
    .target_angle = 0,
    .green_was_seen = false
  };
  
  int init_counter = 0;
  int map_print_counter = 0;
  int victim_debug_counter = 0;
  
  srand(42);
  
  // Main Control Loop
  while (wb_robot_step(timeStep) != -1) {
    Pose pose = get_robot_pose(gps, compass);

    // SLAM updates for SLAM mode
    if (mode == MODE_AUTONOMOUS_SLAM) {
      update_map_from_sensors(pose, ds0, ds1, ds2, ds3);
    }

    // Check for keyboard commands
    int key = wb_keyboard_get_key();
    if (key == 'M' || key == 'm') {
      print_map();
    }
    if (key == 'V' || key == 'v') {
      print_victims();
    }
    
    // Autonomous mode with drone waypoints
    if (mode == MODE_AUTONOMOUS) {
      // Check for incoming radio message
      if (receiver && wb_receiver_get_queue_length(receiver) > 0 && !ctx.has_target) {
        const char *msg = (const char *)wb_receiver_get_data(receiver);
        sscanf(msg, "%lf,%lf", &ctx.target_x, &ctx.target_y);
        printf("New target from drone: (%.2f, %.2f)\n", ctx.target_x, ctx.target_y);
        ctx.has_target = true;
        ctx.state = STATE_NAVIGATE;
        wb_receiver_next_packet(receiver);
      }

      double front_distance = front_ds ? wb_distance_sensor_get_value(front_ds) : 0;

      // Mode 1 FSM
      switch (ctx.state) {
      case STATE_INIT:
        if (++init_counter >= 10) {
          ctx.prev_front_distance = front_distance;
          // If we already have a drone target, go to navigate! Otherwise, idle.
          ctx.state = ctx.has_target ? STATE_NAVIGATE : STATE_IDLE;
        }
        break;
        
      // Wait to get coordinates from receiver
      case STATE_IDLE:
        set_motor_speeds(motors, 0, 0);
        break;

      case STATE_NAVIGATE:
      {
        double distance_to_target = distance_to_point(pose, ctx.target_x, ctx.target_y);
        
        // Calculate how much the sensor reading changed since last frame (Noise Filter)
        double change = fabs(front_distance - ctx.prev_front_distance);
        ctx.prev_front_distance = front_distance;
        
        // 1. CHECK ARRIVAL
        if (distance_to_target < TOLERANCE) {
          printf("Reached target (%.2f, %.2f)\n", ctx.target_x, ctx.target_y);
          ctx.has_target = false;      // Mark job as done
          ctx.pause_counter = 0;       // Reset timer
          set_motor_speeds(motors, 0, 0);
          ctx.state = STATE_PAUSE;     // Stop briefly before next action
        }
        // 2. CHECK FOR POTENTIAL OBSTACLES
        // Only react if:
        //  a) We are still far from target (> 0.5m) - prevents stopping right at the goal.
        //  b) Sensor signal is stable (change <= 70.0) - ignores sudden spikes from terrain.
        //  c) Object is close (value > Threshold).
        else if (distance_to_target > 0.5 && change <= TERRAIN_MAX_CHANGE && front_distance > OBSTACLE_THRESHOLD) {
          ctx.obstacle_frames = 1;      // Start counting how long we see it
          ctx.state = STATE_OBSTACLE_CHECK; // Switch to verification state
        }
        // 3. DRIVE NORMALLY
        else {
          ctx.obstacle_frames = 0;      // Reset counter if path is clear
          drive_to_target(motors, pose, ctx.target_x, ctx.target_y);
        }
        break;
      }

      case STATE_OBSTACLE_CHECK:
      {
        double change = fabs(front_distance - ctx.prev_front_distance);
        ctx.prev_front_distance = front_distance;

        // Condition A: It was just noise (sensor spiked heavily)
        if (change > TERRAIN_MAX_CHANGE) {
          ctx.obstacle_frames = 0;
          ctx.state = STATE_NAVIGATE; // False alarm, keep driving
        }
        // Condition B: Object is still there (consistent reading)
        else if (front_distance > OBSTACLE_THRESHOLD) {
          // Increment "Confidence Counter"
          if (++ctx.obstacle_frames >= OBSTACLE_SUSTAINED_FRAMES) {
            // CONFIRMED: It's a real obstacle.
            // Calculate a "Blind Detour": Point 30cm to the right relative to heading.
            ctx.detour_x = pose.x + 0.3 * sin(pose.heading);
            ctx.detour_y = pose.y - 0.3 * cos(pose.heading);
            
            ctx.obstacle_frames = 0;
            ctx.state = STATE_AVOID_OBSTACLE; // execute evasion
          }
          else {
            // Keep driving while we verify (don't stop immediately)
            drive_to_target(motors, pose, ctx.target_x, ctx.target_y);
          }
        }
        // Condition C: Object disappeared (it was momentary)
        else {
          ctx.obstacle_frames = 0;
          ctx.state = STATE_NAVIGATE;
        }
        break;
      }

      case STATE_AVOID_OBSTACLE:
        if (distance_to_point(pose, ctx.detour_x, ctx.detour_y) < TOLERANCE) {
          // Detour point reached. Stop and re-assess.
          ctx.pause_counter = 0;
          set_motor_speeds(motors, 0, 0);
          ctx.state = STATE_PAUSE;
        }
        else {
          drive_to_target(motors, pose, ctx.detour_x, ctx.detour_y);
        }
        break;

      case STATE_PAUSE:
        if (++ctx.pause_counter >= PAUSE_STEPS)
          // If we still have a target (e.g. after a detour), resume Navigation.
          // If we reached the target, go to Idle.
          ctx.state = ctx.has_target ? STATE_NAVIGATE : STATE_IDLE;
        break;

      default:
        break;
      }
    }
    else if (mode == MODE_AUTONOMOUS_SLAM) {
      // SLAM-based autonomous navigation
      double front_distance = front_ds ? wb_distance_sensor_get_value(front_ds) : 0;
      
      // Victim detection (only during initial scan phase)
      int debug_r = 0, debug_g = 0, debug_b = 0, debug_green_px = 0, debug_total_px = 0;
      bool green_detected = detect_green_victim(camera, &debug_r, &debug_g, &debug_b, &debug_green_px, &debug_total_px);
      
      if (++victim_debug_counter >= 16) {
        printf("[DEBUG] Cam RGB avg: R=%d G=%d B=%d | Green pixels: %d/%d | Detected: %s\n", 
               debug_r, debug_g, debug_b, debug_green_px, debug_total_px,
               green_detected ? "YES" : "no");
        victim_debug_counter = 0;
      }
      
      // Check for new victim (only during initial scan phase)
      if (green_detected && 
          ctx.slam_phase == PHASE_INITIAL_SCAN &&
          ctx.state != STATE_VICTIM_DETECTED && 
          ctx.state != STATE_INIT &&
          ctx.state != STATE_COMPLETE &&
          ctx.state != STATE_IDLE &&
          ctx.state != STATE_PAUSE) {
        
        double victim_dist = 0.15;
        double vx = pose.x + victim_dist * cos(pose.heading);
        double vy = pose.y + victim_dist * sin(pose.heading);
        
        if (!is_victim_already_found(vx, vy)) {
          printf("\n[VICTIM] !!! GREEN VICTIM DETECTED !!!\n");
          printf("[VICTIM] Robot at (%.2f, %.2f), heading %.2f rad (%.1f deg)\n", 
                 pose.x, pose.y, pose.heading, pose.heading * 180.0 / M_PI);
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
        set_motor_speeds(motors, 0, 0);
        ctx.victim_stop_counter++;
        
        if (ctx.victim_stop_counter == 1) {
          printf("[VICTIM] Robot stopped, looking at victim...\n");
        }
        
        if (ctx.victim_stop_counter >= VICTIM_STOP_DURATION) {
          store_victim(ctx.victim_x, ctx.victim_y, pose.heading);
          print_victims();
          
          printf("[VICTIM] Resuming navigation... (%d victims found so far)\n\n", victim_count);
          ctx.state = ctx.prev_state;
          ctx.victim_stop_counter = 0;
        }
        break;
      
      case STATE_REVISIT_VICTIMS:
      {
        double dist_to_standoff = distance_to_point(pose, ctx.target_x, ctx.target_y);
        
        if (dist_to_standoff < TOLERANCE) {
          printf("[REVISIT] Reached standoff point for victim #%d\n", 
                 ctx.current_revisit_index + 1);
          printf("[ALIGN] Rotating to face detection angle: %.1f deg\n",
                 ctx.target_angle * 180.0 / M_PI);
          ctx.state = STATE_ALIGN_TO_VICTIM;
        }
        else {
          double change = fabs(front_distance - ctx.prev_front_distance);
          ctx.prev_front_distance = front_distance;
          
          if (++ctx.path_check_counter >= 10) {
            ctx.path_check_counter = 0;
            
            double obstacle_x, obstacle_y;
            if (check_path_blocked(pose, ctx.target_x, ctx.target_y, &obstacle_x, &obstacle_y)) {
              double detour_x, detour_y;
              if (calculate_detour(pose, ctx.target_x, ctx.target_y, 
                                   obstacle_x, obstacle_y, &detour_x, &detour_y)) {
                printf("[SLAM] Inserting detour waypoint (%.2f, %.2f) before victim location\n",
                       detour_x, detour_y);
                
                ctx.original_target_x = ctx.target_x;
                ctx.original_target_y = ctx.target_y;
                ctx.slam_detour_x = detour_x;
                ctx.slam_detour_y = detour_y;
                
                ctx.state = STATE_SLAM_DETOUR;
              } else {
                drive_to_target(motors, pose, ctx.target_x, ctx.target_y);
              }
            } else {
              drive_to_target(motors, pose, ctx.target_x, ctx.target_y);
            }
          }
          else if (change <= TERRAIN_MAX_CHANGE && front_distance > OBSTACLE_THRESHOLD) {
            ctx.obstacle_frames = 1;
            ctx.prev_state = STATE_REVISIT_VICTIMS;
            ctx.state = STATE_OBSTACLE_CHECK;
          }
          else {
            ctx.obstacle_frames = 0;
            drive_to_target(motors, pose, ctx.target_x, ctx.target_y);
          }
        }
        break;
      }
      
      case STATE_ALIGN_TO_VICTIM:
      {
        double angle_error = normalize_angle(ctx.target_angle - pose.heading);
        
        if (fabs(angle_error) < VICTIM_ALIGN_TOLERANCE) {
          printf("[ALIGN] Aligned! Current heading: %.1f deg, Target: %.1f deg\n",
                 pose.heading * 180.0 / M_PI, ctx.target_angle * 180.0 / M_PI);
          printf("[APPROACH] Driving forward until green disappears...\n");
          ctx.green_was_seen = false;
          ctx.state = STATE_APPROACH_VICTIM;
        }
        else {
          double turn_speed = VICTIM_ALIGN_SPEED;
          if (angle_error > 0) {
            set_motor_speeds(motors, -turn_speed, turn_speed);
          } else {
            set_motor_speeds(motors, turn_speed, -turn_speed);
          }
          
          static int align_debug = 0;
          if (++align_debug >= 16) {
            printf("[ALIGN] Angle error: %.1f deg\n", angle_error * 180.0 / M_PI);
            align_debug = 0;
          }
        }
        break;
      }
      
      case STATE_APPROACH_VICTIM:
      {
        int dummy_r, dummy_g, dummy_b, dummy_green, dummy_total;
        bool green_visible = detect_green_victim(camera, &dummy_r, &dummy_g, &dummy_b, &dummy_green, &dummy_total);
        
        if (green_visible) {
          ctx.green_was_seen = true;
        }
        
        if (ctx.green_was_seen && !green_visible) {
          printf("[APPROACH] *** GREEN DISAPPEARED - PASSED VICTIM #%d ***\n", ctx.current_revisit_index + 1);
          printf("[APPROACH] Robot position: (%.2f, %.2f)\n", pose.x, pose.y);
          set_motor_speeds(motors, 0, 0);
          
          ctx.current_revisit_index++;
          ctx.pause_counter = 0;
          ctx.state = STATE_PAUSE;
        }
        else {
          set_motor_speeds(motors, VICTIM_APPROACH_SPEED, VICTIM_APPROACH_SPEED);
          
          static int approach_debug = 0;
          if (++approach_debug >= 16) {
            printf("[APPROACH] Green: %s | Was seen: %s | Pos: (%.2f, %.2f)\n", 
                   green_visible ? "YES" : "no",
                   ctx.green_was_seen ? "YES" : "no",
                   pose.x, pose.y);
            approach_debug = 0;
          }
        }
        break;
      }
      
      case STATE_COMPLETE:
        set_motor_speeds(motors, 0, 0);
        
        if (ctx.slam_phase == PHASE_INITIAL_SCAN) {
          printf("\n========================================\n");
          printf("[PHASE 1 COMPLETE] All waypoints scanned!\n");
          printf("Victims found during scan: %d\n", victim_count);
          print_victims();
          print_map();
          
          if (victim_count > 0) {
            printf("\n========================================\n");
            printf("[PHASE 2] Starting victim revisit phase...\n");
            printf("Will revisit %d victim location(s)\n", victim_count);
            printf("========================================\n\n");
            
            ctx.slam_phase = PHASE_REVISIT_VICTIMS;
            ctx.current_revisit_index = 0;
            
            if (set_next_victim_target(&ctx)) {
              ctx.state = STATE_REVISIT_VICTIMS;
            } else {
              printf("[COMPLETE] No victims to revisit.\n");
              ctx.state = STATE_IDLE;
            }
          } else {
            printf("\n[COMPLETE] No victims found during scan.\n");
            printf("========================================\n");
            ctx.state = STATE_IDLE;
          }
        }
        else if (ctx.slam_phase == PHASE_REVISIT_VICTIMS) {
          printf("\n========================================\n");
          printf("[ALL PHASES COMPLETE]\n");
          printf("Total victims found and revisited: %d\n", victim_count);
          print_victims();
          print_map();
          printf("========================================\n");
          printf("Robot stopped. Run complete.\n");
          ctx.state = STATE_IDLE;
        }
        break;
        
      case STATE_NAVIGATE:
      {
        double change = fabs(front_distance - ctx.prev_front_distance);
        ctx.prev_front_distance = front_distance;
        
        if (distance_to_point(pose, ctx.target_x, ctx.target_y) < TOLERANCE) {
          printf("Reached waypoint %d: (%.2f, %.2f)\n", 
                 ctx.current_waypoint + 1, ctx.target_x, ctx.target_y);
          ctx.current_waypoint++;
          ctx.pause_counter = 0;
          ctx.waypoint_reached = true;
          set_motor_speeds(motors, 0, 0);
          ctx.state = STATE_PAUSE;
        }
        else if (++ctx.path_check_counter >= 10) {
          ctx.path_check_counter = 0;
          
          double obstacle_x, obstacle_y;
          if (check_path_blocked(pose, ctx.target_x, ctx.target_y, &obstacle_x, &obstacle_y)) {
            double detour_x, detour_y;
            if (calculate_detour(pose, ctx.target_x, ctx.target_y, 
                                 obstacle_x, obstacle_y, &detour_x, &detour_y)) {
              printf("[SLAM] Inserting detour waypoint (%.2f, %.2f) before target (%.2f, %.2f)\n",
                     detour_x, detour_y, ctx.target_x, ctx.target_y);
              
              ctx.original_target_x = ctx.target_x;
              ctx.original_target_y = ctx.target_y;
              ctx.slam_detour_x = detour_x;
              ctx.slam_detour_y = detour_y;
              
              ctx.state = STATE_SLAM_DETOUR;
            } else {
              drive_to_target(motors, pose, ctx.target_x, ctx.target_y);
            }
          } else {
            drive_to_target(motors, pose, ctx.target_x, ctx.target_y);
          }
        }
        else if (change <= TERRAIN_MAX_CHANGE && front_distance > OBSTACLE_THRESHOLD) {
          ctx.obstacle_frames = 1;
          ctx.state = STATE_OBSTACLE_CHECK;
        }
        else {
          ctx.obstacle_frames = 0;
          drive_to_target(motors, pose, ctx.target_x, ctx.target_y);
        }
        break;
      }
      
      case STATE_SLAM_DETOUR:
      {
        double dist_to_detour = distance_to_point(pose, ctx.slam_detour_x, ctx.slam_detour_y);
        
        if (dist_to_detour < TOLERANCE) {
          printf("[SLAM] Detour point reached, resuming to original target (%.2f, %.2f)\n",
                 ctx.original_target_x, ctx.original_target_y);
          
          ctx.target_x = ctx.original_target_x;
          ctx.target_y = ctx.original_target_y;
          ctx.path_check_counter = 0;
          
          if (ctx.slam_phase == PHASE_REVISIT_VICTIMS) {
            ctx.state = STATE_REVISIT_VICTIMS;
          } else {
            ctx.state = STATE_NAVIGATE;
          }
        } else {
          drive_to_target(motors, pose, ctx.slam_detour_x, ctx.slam_detour_y);
        }
        break;
      }
      
      case STATE_OBSTACLE_CHECK:
      {
        double change = fabs(front_distance - ctx.prev_front_distance);
        ctx.prev_front_distance = front_distance;

        if (change > TERRAIN_MAX_CHANGE) {
          ctx.obstacle_frames = 0;
          if (ctx.slam_phase == PHASE_REVISIT_VICTIMS) {
            ctx.state = STATE_REVISIT_VICTIMS;
          } else {
            ctx.state = STATE_NAVIGATE;
          }
        }
        else if (front_distance > OBSTACLE_THRESHOLD) {
          if (++ctx.obstacle_frames >= OBSTACLE_SUSTAINED_FRAMES) {
            printf("Obstacle detected! Initiating detour...\n");
            ctx.detour_x = pose.x + 0.3 * sin(pose.heading);
            ctx.detour_y = pose.y - 0.3 * cos(pose.heading);
            ctx.obstacle_frames = 0;
            ctx.state = STATE_AVOID_OBSTACLE;
          }
          else {
            drive_to_target(motors, pose, ctx.target_x, ctx.target_y);
          }
        }
        else {
          ctx.obstacle_frames = 0;
          if (ctx.slam_phase == PHASE_REVISIT_VICTIMS) {
            ctx.state = STATE_REVISIT_VICTIMS;
          } else {
            ctx.state = STATE_NAVIGATE;
          }
        }
        break;
      }
      
      case STATE_AVOID_OBSTACLE:
        if (distance_to_point(pose, ctx.detour_x, ctx.detour_y) < TOLERANCE) {
          if (ctx.slam_phase == PHASE_REVISIT_VICTIMS) {
            printf("Detour complete, resuming navigation to victim #%d...\n", ctx.current_revisit_index + 1);
          } else {
            printf("Detour complete, resuming navigation to waypoint %d...\n", ctx.current_waypoint + 1);
          }
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
          if (ctx.slam_phase == PHASE_REVISIT_VICTIMS) {
            if (ctx.current_revisit_index < victim_count) {
              if (set_next_victim_target(&ctx)) {
                ctx.state = STATE_REVISIT_VICTIMS;
              } else {
                ctx.state = STATE_COMPLETE;
              }
            } else {
              ctx.state = STATE_COMPLETE;
            }
          }
          else {
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
        }
        break;
      }

      // Print map periodically during SLAM mode
      if (++map_print_counter >= 500) {
        print_map();
        map_print_counter = 0;
      }
    }
  }

  wb_robot_cleanup();
  return 0;
}