#include <stdio.h>
#include <math.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/gps.h>
#include <webots/compass.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>

#define GRID_SIZE 10
#define MAP_SIZE 20
#define WORLD_MIN_X -2.0
#define WORLD_MAX_X  2.0
#define WORLD_MIN_Y -2.0
#define WORLD_MAX_Y  2.0

#define MAX_VICTIMS 20

static double belief[GRID_SIZE][GRID_SIZE];
static double map_grid[MAP_SIZE][MAP_SIZE];

typedef struct {
  double x;
  double y;
} Victim;

static Victim victims[MAX_VICTIMS];
static int victim_count = 0;
static int victim_found_flag = 0;

int world_to_grid_x(double x){
  double r = (x - WORLD_MIN_X) / (WORLD_MAX_X - WORLD_MIN_X);
  int idx = (int)(r * GRID_SIZE);
  if (idx < 0) idx = 0;
  if (idx >= GRID_SIZE) idx = GRID_SIZE - 1;
  return idx;
}

int world_to_grid_y(double y){
  double r = (y - WORLD_MIN_Y) / (WORLD_MAX_Y - WORLD_MIN_Y);
  int idx = (int)(r * GRID_SIZE);
  if (idx < 0) idx = 0;
  if (idx >= GRID_SIZE) idx = GRID_SIZE - 1;
  return idx;
}

int is_free(double v) {
  return (v >= 0.0 && v < 0.3);
}

int is_unknown(double v) {
  return (v < 0.0);
}

int is_frontier(int x, int y) {
  if (!is_free(map_grid[x][y]))
    return 0;

  for (int dx = -1; dx <= 1; dx++) {
    for (int dy = -1; dy <= 1; dy++) {
      if (dx == 0 && dy == 0) continue;
      
      int nx = x + dx;
      int ny = y + dy;
      
      if (nx >= 0 && nx < MAP_SIZE && ny >= 0 && ny < MAP_SIZE) {
        if (is_unknown(map_grid[nx][ny])) return 1;
      }
    }
  }
  return 0;
}


int find_nearest_frontier(double rx, double ry, int *fx, int *fy) {
  double best_d = 1e9;
  int found = 0;

  for (int i = 0; i < MAP_SIZE; i++) {
    for (int j = 0; j < MAP_SIZE; j++) {

      if (!is_frontier(i, j))
        continue;

      double wx = WORLD_MIN_X + (i + 0.5) * (WORLD_MAX_X - WORLD_MIN_X) / MAP_SIZE;
      double wy = WORLD_MIN_Y + (j + 0.5) * (WORLD_MAX_Y - WORLD_MIN_Y) / MAP_SIZE;

      double dx = wx - rx;
      double dy = wy - ry;
      double dist = sqrt(dx*dx + dy*dy);

      if (dist < best_d) {
        best_d = dist;
        *fx = i;
        *fy = j;
        found = 1;
      }
    }
  }

  return found;
}

double grid_to_world_x(int gx){
  return WORLD_MIN_X + (gx + 0.5) * ((WORLD_MAX_X - WORLD_MIN_X) / GRID_SIZE);
}

double grid_to_world_y(int gy){
  return WORLD_MIN_Y + (gy + 0.5) * ((WORLD_MAX_Y - WORLD_MIN_Y) / GRID_SIZE);
}

int world_to_map_x(double x){
  double r = (x - WORLD_MIN_X) / (WORLD_MAX_X - WORLD_MIN_X);
  int idx = (int)(r * MAP_SIZE);
  if (idx < 0) idx = 0;
  if (idx >= MAP_SIZE) idx = MAP_SIZE - 1;
  return idx;
}

int world_to_map_y(double y){
  double r = (y - WORLD_MIN_Y) / (WORLD_MAX_Y - WORLD_MIN_Y);
  int idx = (int)(r * MAP_SIZE);
  if (idx < 0) idx = 0;
  if (idx >= MAP_SIZE) idx = MAP_SIZE - 1;
  return idx;
}

int cell_is_free(int gx, int gy) {
  if (gx < 0 || gy < 0 || gx >= MAP_SIZE || gy >= MAP_SIZE)
    return 0; 
  double v = map_grid[gx][gy];

  if (v < 0.0)
    return 1;        

  return (v < 0.7);  
}


double expected_distance(double x, double y, double angle){
  double max_range = 0.4;  
  double step = 0.01;

  for (double d = 0; d < max_range; d += step) {
    double rx = x + d * cos(angle);
    double ry = y + d * sin(angle);

    if (rx < WORLD_MIN_X || rx > WORLD_MAX_X ||
        ry < WORLD_MIN_Y || ry > WORLD_MAX_Y)
      return d;  
  }
  return max_range;
}

double angle_to_target(double rx, double ry, double heading,
                       int fx, int fy) {

  double tx = WORLD_MIN_X + (fx + 0.5) * (WORLD_MAX_X - WORLD_MIN_X) / MAP_SIZE;
  double ty = WORLD_MIN_Y + (fy + 0.5) * (WORLD_MAX_Y - WORLD_MIN_Y) / MAP_SIZE;

  double desired = atan2(ty - ry, tx - rx);
  double diff = desired - heading;

  while (diff > M_PI) diff -= 2*M_PI;
  while (diff < -M_PI) diff += 2*M_PI;

  return diff;
}

void markov_init() {
  double uniform = 1.0 / (GRID_SIZE * GRID_SIZE);
  for (int i = 0; i < GRID_SIZE; i++)
    for (int j = 0; j < GRID_SIZE; j++)
      belief[i][j] = uniform;
}

void normalize_belief() {
  double total = 0.0;
  for (int i = 0; i < GRID_SIZE; i++)
    for (int j = 0; j < GRID_SIZE; j++)
      total += belief[i][j];

  if (total == 0.0) {
    markov_init();
    return;
  }

  for (int i = 0; i < GRID_SIZE; i++)
    for (int j = 0; j < GRID_SIZE; j++)
      belief[i][j] /= total;
}

void mark_victim(double x, double y){
 int mx = world_to_map_x(x);
 int my = world_to_map_y(y);
 
 map_grid[mx][my] = 2.0;
}

void debug_camera_block(const unsigned char *image, int w, int h) {
  int cx = w / 2;
  int cy = h / 2;

  printf("\n--- CAMERA DEBUG BLOCK (center 10x10) ---\n");

  int minR = 255, minG = 255, minB = 255;
  int maxR = 0,   maxG = 0,   maxB = 0;

  for (int dy = -5; dy <= 5; dy++) {
    for (int dx = -5; dx <= 5; dx++) {
      int x = cx + dx;
      int y = cy + dy;

      int r = wb_camera_image_get_red(image, x, y, w);
      int g = wb_camera_image_get_green(image, x, y, w);
      int b = wb_camera_image_get_blue(image, x, y, w);

      if (r < minR) minR = r;
      if (g < minG) minG = g;
      if (b < minB) minB = b;

      if (r > maxR) maxR = r;
      if (g > maxG) maxG = g;
      if (b > maxB) maxB = b;

      if (r > 180 && g < 80 && b < 80)
        printf(" R ");  
      else if (r > 120 && g < 120)
        printf(" r ");  
      else
        printf(" . ");  
    }
    printf("\n");
  }

  printf("Min RGB in block: (%d, %d, %d)\n", minR, minG, minB);
  printf("Max RGB in block: (%d, %d, %d)\n\n", maxR, maxG, maxB);
}

void markov_sensor_update(double heading,
                          WbDeviceTag ds0, WbDeviceTag ds1,
                          WbDeviceTag ds2, WbDeviceTag ds3)
{
  WbDeviceTag sensors[4] = {ds0, ds1, ds2, ds3};

  double base_dir[4] = {
    heading,           
    heading + M_PI_2, 
    heading + M_PI,    
    heading - M_PI_2   
  };

  for (int gx = 0; gx < GRID_SIZE; gx++) {
    for (int gy = 0; gy < GRID_SIZE; gy++) {

      double wx = grid_to_world_x(gx);
      double wy = grid_to_world_y(gy);

      double total_likelihood = 1.0;

      for (int s = 0; s < 4; s++) {
        double dir = base_dir[s];
        double actual_raw = wb_distance_sensor_get_value(sensors[s]);

        double expected_m = expected_distance(wx, wy, dir);
        
        double expected_raw = 154.0 * (1.0 - expected_m / 0.4);
        if (expected_raw < 0) expected_raw = 0;

        double error = fabs(actual_raw - expected_raw);

        double likelihood = exp(-error * 0.02);

        total_likelihood *= likelihood;
      }

      belief[gx][gy] *= total_likelihood;
    }
  }

  normalize_belief();
}

void mark_victim_on_map(double x, double y) {
  int mx = world_to_map_x(x);
  int my = world_to_map_y(y);

  printf("VICTIM MAPPED TO GRID CELL: (%d, %d)\n", mx, my);

  if (mx >= 0 && mx < MAP_SIZE && my >= 0 && my < MAP_SIZE) {
    map_grid[mx][my] = 2.0;
  }
}


void add_victim_if_new(double x, double y) {
  for (int i = 0; i < victim_count; i++) {
    double dx = x - victims[i].x;
    double dy = y - victims[i].y;
    double dist2 = dx * dx + dy * dy;
    if (dist2 < 0.3 * 0.3) {
      return;
    }
  }

  if (victim_count >= MAX_VICTIMS) {
    printf("Victim list full, cannot store more.\n");
    return;
  }

  victims[victim_count].x = x;
  victims[victim_count].y = y;
  victim_count++;

  mark_victim_on_map(x, y);

  printf("Stored victim #%d at (%.2f, %.2f)\n", victim_count, x, y);
  victim_found_flag = 1;
}

void update_map_from_sensor(double x, double y,
                            double angle, double measured_dist)
{
  double step = 0.05;
  double max_range = 0.4;

  for (double d = 0; d < measured_dist && d < max_range; d += step) {
    double wx = x + d * cos(angle);
    double wy = y + d * sin(angle);
    int mx = world_to_map_x(wx);
    int my = world_to_map_y(wy);

    if (map_grid[mx][my] == 2.0)
      continue;

    double val = map_grid[mx][my];
    if (val < 0.0) val = 0.5;
    val -= 0.05;
    if (val < 0.0) val = 0.0;
    map_grid[mx][my] = val;
  }
  
  if (measured_dist < max_range) {
    double wx = x + measured_dist * cos(angle);
    double wy = y + measured_dist * sin(angle);
    int mx = world_to_map_x(wx);
    int my = world_to_map_y(wy);

    if (map_grid[mx][my] != 2.0) {
      map_grid[mx][my] = 1.0;
    }
  }
}

void get_best_pose(double *x, double *y) {
  int best_i = 0;
  int best_j = 0;
  double best_p = 0.0;

  for (int i = 0; i < GRID_SIZE; i++) {
    for (int j = 0; j < GRID_SIZE; j++) {
      if (belief[i][j] > best_p) {
        best_p = belief[i][j];
        best_i = i;
        best_j = j;
      }
    }
  }

  *x = grid_to_world_x(best_i);
  *y = grid_to_world_y(best_j);
}

void markov_mode_step(WbDeviceTag gps, WbDeviceTag compass,
                      WbDeviceTag ds0, WbDeviceTag ds1,
                      WbDeviceTag ds2, WbDeviceTag ds3)
{
  const double *gps_values = wb_gps_get_values(gps);
  const double *compass_values = wb_compass_get_values(compass);
  if (!gps_values || !compass_values)
    return;

  double heading = atan2(compass_values[0], compass_values[1]);

  static double temp[GRID_SIZE][GRID_SIZE];
  for (int i = 0; i < GRID_SIZE; i++)
    for (int j = 0; j < GRID_SIZE; j++)
      temp[i][j] = 0.0;

  int dx = (int)round(cos(heading));
  int dy = (int)round(sin(heading));

  for (int i = 0; i < GRID_SIZE; i++) {
    for (int j = 0; j < GRID_SIZE; j++) {
      int nx = i + dx;
      int ny = j + dy;

      if (nx >= 0 && nx < GRID_SIZE && ny >= 0 && ny < GRID_SIZE)
        temp[nx][ny] += belief[i][j];
      else
        temp[i][j] += belief[i][j];
    }
  }

  for (int i = 0; i < GRID_SIZE; i++)
    for (int j = 0; j < GRID_SIZE; j++)
      belief[i][j] = temp[i][j];
      
  markov_sensor_update(heading, ds0, ds1, ds2, ds3);
  
  double best_x, best_y;
  get_best_pose(&best_x, &best_y);
  
  double sensor_dirs[4] = {
    heading,           
    heading + M_PI_2,  
    heading + M_PI,    
    heading - M_PI_2   
  };
  
  double ds_vals[4] = {
    wb_distance_sensor_get_value(ds0),
    wb_distance_sensor_get_value(ds1),
    wb_distance_sensor_get_value(ds2),
    wb_distance_sensor_get_value(ds3)
  };
  
  for (int i = 0; i < 4; i++) {
    double raw = ds_vals[i];
    double dist = 0.4 * (1.0 - raw / 154.0);
    if (dist < 0) dist = 0;
    ds_vals[i] = dist;
  }
  
  for (int i = 0; i < 4; i++) {
    update_map_from_sensor(best_x, best_y,
                           sensor_dirs[i], ds_vals[i]);
  }

  double x = gps_values[0];
  double y = gps_values[1];
  double front_raw = wb_distance_sensor_get_value(ds0);
  printf("MARKOV: pos(%.2f, %.2f), heading=%.2f, ds0=%.1f\n",
         x, y, heading, front_raw);
}

void print_map() {
  printf("\n--- MAP GRID ---\n");
  for (int i = 0; i < MAP_SIZE; i++) {
    for (int j = 0; j < MAP_SIZE; j++) {
      double v = map_grid[i][j];
      char c;

      if (v == 2.0)              
        c = 'V';
      else if (v < 0.0)
        c = '?';               
      else if (v < 0.3)
        c = '.';               
      else if (v > 0.7)
        c = '#';               
      else
        c = '~';               
      printf(" %c ", c);
    }
    printf("\n");
  }

  printf("Victims stored: %d\n", victim_count);
}



int main(int argc, char **argv) {
  wb_robot_init();
  int timeStep = wb_robot_get_basic_time_step();

  WbDeviceTag leftMotor = wb_robot_get_device("left motor");
  WbDeviceTag rightMotor = wb_robot_get_device("right motor");
  WbDeviceTag gps = wb_robot_get_device("gps");
  WbDeviceTag compass = wb_robot_get_device("compass");
  WbDeviceTag camera = wb_robot_get_device("ground_cam");

  WbDeviceTag ds0 = wb_robot_get_device("ds0");
  WbDeviceTag ds1 = wb_robot_get_device("ds1");
  WbDeviceTag ds2 = wb_robot_get_device("ds2");
  WbDeviceTag ds3 = wb_robot_get_device("ds3");

  wb_distance_sensor_enable(ds0, timeStep);
  wb_distance_sensor_enable(ds1, timeStep);
  wb_distance_sensor_enable(ds2, timeStep);
  wb_distance_sensor_enable(ds3, timeStep);

  wb_gps_enable(gps, timeStep);
  wb_compass_enable(compass, timeStep);
  wb_camera_enable(camera, timeStep);

  wb_motor_set_position(leftMotor, INFINITY);
  wb_motor_set_position(rightMotor, INFINITY);

  markov_init();
  
  for (int i = 0; i < MAP_SIZE; i++)
    for (int j = 0; j < MAP_SIZE; j++)
      map_grid[i][j] = -1.0;
    
  int counter = 0;

  while (wb_robot_step(timeStep) != -1) {

    const double *compass_values = wb_compass_get_values(compass);
    double heading = atan2(compass_values[0], compass_values[1]);
    
    markov_mode_step(gps, compass, ds0, ds1, ds2, ds3);

    double rx, ry;
    get_best_pose(&rx, &ry);
    
    // predict the next grid cell the robot will enter
    // int next_x = world_to_map_x(rx + 0.1 * cos(heading));
    // int next_y = world_to_map_y(ry + 0.1 * sin(heading));
    
    // int front_blocked = !cell_is_free(next_x, next_y);
    // int ir_hits_wall = wb_distance_sensor_get_value(ds0) > 120; // front IR

    if (victim_found_flag) {
        wb_motor_set_velocity(leftMotor, 0.0);
        wb_motor_set_velocity(rightMotor, 0.0);
    }
    else {
        int fx, fy;
        int has_frontier = find_nearest_frontier(rx, ry, &fx, &fy);
        double diff = 0.0;
        if (has_frontier)
            diff = angle_to_target(rx, ry, heading, fx, fy);
            
        double front_ir = wb_distance_sensor_get_value(ds0);
        double left_ir = wb_distance_sensor_get_value(ds1);
        double right_ir = wb_distance_sensor_get_value(ds3);
        int immediate_obstacle = (front_ir > 100);  
        
        if (immediate_obstacle) {
            printf("Obstacle ahead → choosing best turn direction\n");
            
            if (left_ir < right_ir) {
                wb_motor_set_velocity(leftMotor, -0.3);
                wb_motor_set_velocity(rightMotor, 0.3);
            } else {
                wb_motor_set_velocity(leftMotor, 0.3);
                wb_motor_set_velocity(rightMotor, -0.3);
            }
        }
        else if (has_frontier) {
            printf("Frontier at (%d, %d) | turn diff = %.2f\n", fx, fy, diff);
            
            if (fabs(diff) > 0.5) {  
                double turn_speed = 0.3;
                if (diff > 0) {
                    wb_motor_set_velocity(leftMotor, -turn_speed);
                    wb_motor_set_velocity(rightMotor, turn_speed);
                } else {
                    wb_motor_set_velocity(leftMotor, turn_speed);
                    wb_motor_set_velocity(rightMotor, -turn_speed);
                }
            } else {
                // Move forward towards frontier
                wb_motor_set_velocity(leftMotor, 0.4);
                wb_motor_set_velocity(rightMotor, 0.4);
            }
        }
        else {
            printf("NO FRONTIERS FOUND — MAP COMPLETE!\n");
            wb_motor_set_velocity(leftMotor, 0.0);
            wb_motor_set_velocity(rightMotor, 0.0);
        }
    }

    // === CAMERA PROCESSING ===
    const unsigned char *image = wb_camera_get_image(camera);
    if (image != NULL) {
        int w = wb_camera_get_width(camera);
        int h = wb_camera_get_height(camera);

        int brightCount = 0;
        int total = 0;

        int cx = w / 2;
        int cy = h / 2;

        for (int dy = -10; dy <= 10; dy++) {
            for (int dx = -10; dx <= 10; dx++) {

                int x = cx + dx;
                int y = cy + dy;

                int r = wb_camera_image_get_red(image, x, y, w);
                int g = wb_camera_image_get_green(image, x, y, w);
                int b = wb_camera_image_get_blue(image, x, y, w);

                int brightness = (r + g + b) / 3;

                if (brightness > 120)
                    brightCount++;

                total++;
            }
        }

        if (brightCount > total * 0.20) {
            double vx, vy;
            get_best_pose(&vx, &vy);

            printf(">>> SHEEP DETECTED near (%.2f, %.2f) <<<\n", vx, vy);

            add_victim_if_new(vx, vy);
        }
    }

    counter++;
    if (counter % 20 == 0)
        print_map();
}


  wb_robot_cleanup();
  return 0;
}
