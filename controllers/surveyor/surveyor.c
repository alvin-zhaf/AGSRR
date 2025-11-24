/*
 * Copyright 1996-2024 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:   An example of a controller using the Track node to move
 *                tracked robot.
 */

#include <stdio.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/gps.h>
#include <webots/compass.h>
#include <webots/camera.h>

#define GRID_SIZE 10
#define WORLD_MIN_X -2.0
#define WORLD_MAX_X 2.0
#define WORLD_MIN_Y -2.0
#define WORLD_MAX_Y 2.0

static double belief[GRID_SIZE][GRID_SIZE];

static void print_keyboard_help() {
  printf("Select the 3D window and use the keyboard:\n");
  printf("\n");
  printf(" W: forward\n");
  printf(" A: turn left\n");
  printf(" S: backward\n");
  printf(" D: turn right\n");
  printf("\n");
  printf("-------------------------------------------------\n");
}

// typedef enum {
  // MODE_GOTO,
  // MODE_MARKOV
// } RobotMode;

// static RobotMode curr = MODE_MARKOV;

int world_to_grid_x(double x){
 double r = (x - WORLD_MIN_X) / (WORLD_MAX_X - WORLD_MIN_X);
 int idx = (int)(r * GRID_SIZE);
 if (idx < 0) idx = 0;
 if (idx >= GRID_SIZE) idx = GRID_SIZE - 1;
 return idx;
}

int world_to_grid_y(double y){
  double r = (y -WORLD_MIN_Y) / (WORLD_MAX_Y - WORLD_MIN_Y);
  int idx = (int)(r * GRID_SIZE);
  if (idx < 0) idx = 0;
  if (idx >= GRID_SIZE) idx = GRID_SIZE - 1;
  return idx;
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
  
  for (int i = 0; i < GRID_SIZE; i++)
    for (int j = 0; j < GRID_SIZE; j++)
      belief[i][j] /= total;
}

void markov_mode_step(WbDeviceTag gps, WbDeviceTag compass){
  const double *gps_values = wb_gps_get_values(gps);
  const double *compass_values = wb_compass_get_values(compass);
  if (!gps_values || !compass_values) return;
  
  double x = gps_values[0];
  double y = gps_values[1];
  double heading = atan2(compass_values[0], compass_values[1]);
  
  static double temp[GRID_SIZE][GRID_SIZE];
  for (int i = 0; i < GRID_SIZE; i++)
    for (int j = 0; j < GRID_SIZE; j++)
      temp[i][j] = 0.0;
  
  int dx = (int)round(cos(heading));
  int dy = (int)round(sin(heading));
  
  for (int i = 0; i < GRID_SIZE; i++){
    for (int j = 0; j < GRID_SIZE; j++){
      int nx = i + dx;
      int ny = j + dy;
      
      if (nx >= 0 && nx < GRID_SIZE && ny >= 0 && ny < GRID_SIZE)
        temp[nx][ny] += belief [i][j];
      else
        temp[i][j] += belief[i][j];
    }
  }
  
  for (int i = 0; i < GRID_SIZE; i++)
    for (int j = 0; j < GRID_SIZE; j++)
      belief[i][j] = temp[i][j];
      
  int gx = world_to_grid_x(x);
  int gy = world_to_grid_y(y);
  
  for (int i = 0; i < GRID_SIZE; i++){
    for (int j = 0; j < GRID_SIZE; j++){
      double dist = sqrt((gx-i)*(gx-i) + (gy-j)*(gy-j));
      double likelihood = exp(-dist * 1.5);
      belief[i][j] *= likelihood;
    }
  }
  normalize_belief();
  printf("MARKOV MODE: Robot at (%.2f, %.2f)\n", x, y);  
}
      
  
  
int main(int argc, char **argv) {
  wb_robot_init();
  const int timeStep = wb_robot_get_basic_time_step();
  wb_keyboard_enable(timeStep);

  WbDeviceTag leftMotor = wb_robot_get_device("left motor");
  WbDeviceTag rightMotor = wb_robot_get_device("right motor");
  WbDeviceTag gps = wb_robot_get_device("gps");
  WbDeviceTag compass = wb_robot_get_device("compass");
  WbDeviceTag camera = wb_robot_get_device("ground_cam");
  
  wb_gps_enable(gps, timeStep);
  wb_compass_enable(compass, timeStep);
  wb_camera_enable(camera, timeStep);
  
  wb_motor_set_position(leftMotor, INFINITY);
  wb_motor_set_position(rightMotor, INFINITY);
  
  markov_init();
  
  
  print_keyboard_help();

  while (wb_robot_step(timeStep) != -1) {
    double test_speed_l = 0.3;
    double test_speed_r = 0.28;
    wb_motor_set_velocity(leftMotor, test_speed_l);
    wb_motor_set_velocity(rightMotor,test_speed_r);
    markov_mode_step(gps, compass);
    const unsigned char *image = wb_camera_get_image(camera);
    if (image != NULL) {
      int width = wb_camera_get_width(camera);
      int height = wb_camera_get_height(camera);
      
      int r = wb_camera_image_get_red(image, width/2, height/2, width);
      int g = wb_camera_image_get_green(image, width/2, height/2, width);
      int b = wb_camera_image_get_blue(image, width/2, height/2, width);
      printf("Camera pixel = R:%d G:%d B:%d\n", r, g, b);
    }
  }

  wb_robot_cleanup();

  return 0;
}
