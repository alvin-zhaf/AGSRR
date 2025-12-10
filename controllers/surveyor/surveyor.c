#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/compass.h>
#include <webots/distance_sensor.h>
#include <webots/keyboard.h>
#include <webots/camera.h>
#include <webots/receiver.h>

#define MAX_SPEED 0.25
#define TOLERANCE 0.05
#define SLOW_DOWN_DISTANCE 1.0
#define OBSTACLE_THRESHOLD 1.0
#define OBSTACLE_SUSTAINED_FRAMES 5
#define TERRAIN_MAX_CHANGE 70.0
#define PAUSE_STEPS 50

#define GRID_SIZE 10
#define WORLD_MIN_X -2.0
#define WORLD_MAX_X 2.0
#define WORLD_MIN_Y -2.0
#define WORLD_MAX_Y 2.0

typedef enum {
  MODE_KEYBOARD,
  MODE_AUTONOMOUS
} RobotMode;

typedef enum {
  STATE_INIT,
  STATE_NAVIGATE,
  STATE_OBSTACLE_CHECK,
  STATE_AVOID_OBSTACLE,
  STATE_PAUSE,
  STATE_IDLE
} RobotState;

typedef struct {
  double x, y, heading;
} Pose;

typedef struct {
  WbDeviceTag left, right;
} Motors;

typedef struct {
  RobotState state;
  double target_x, target_y;
  double prev_front_distance;
  int obstacle_frames;
  double detour_x, detour_y;
  int pause_counter;
  bool has_target;
} RobotContext;

static double belief[GRID_SIZE][GRID_SIZE];

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

/* Markov localization functions */
static void markov_init() {
  double uniform = 1.0 / (GRID_SIZE * GRID_SIZE);
  for (int i = 0; i < GRID_SIZE; i++)
    for (int j = 0; j < GRID_SIZE; j++)
      belief[i][j] = uniform;
}

static void normalize_belief() {
  double total = 0.0;
  for (int i = 0; i < GRID_SIZE; i++)
    for (int j = 0; j < GRID_SIZE; j++)
      total += belief[i][j];
  for (int i = 0; i < GRID_SIZE; i++)
    for (int j = 0; j < GRID_SIZE; j++)
      belief[i][j] /= total;
}

static int world_to_grid(double val, double min_v, double max_v) {
  double r = (val - min_v) / (max_v - min_v);
  int idx = (int)(r * GRID_SIZE);
  return idx < 0 ? 0 : (idx >= GRID_SIZE ? GRID_SIZE - 1 : idx);
}

static void markov_step(Pose pose) {
  static double temp[GRID_SIZE][GRID_SIZE];
  for (int i = 0; i < GRID_SIZE; i++)
    for (int j = 0; j < GRID_SIZE; j++)
      temp[i][j] = 0.0;

  int dx = (int)round(cos(pose.heading)), dy = (int)round(sin(pose.heading));
  for (int i = 0; i < GRID_SIZE; i++) {
    for (int j = 0; j < GRID_SIZE; j++) {
      int nx = i + dx, ny = j + dy;
      if (nx >= 0 && nx < GRID_SIZE && ny >= 0 && ny < GRID_SIZE)
        temp[nx][ny] += belief[i][j];
      else
        temp[i][j] += belief[i][j];
    }
  }
  for (int i = 0; i < GRID_SIZE; i++)
    for (int j = 0; j < GRID_SIZE; j++)
      belief[i][j] = temp[i][j];

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

int main(int argc, char **argv) {
  wb_robot_init();
  const int timeStep = wb_robot_get_basic_time_step();

  wb_keyboard_enable(timeStep);

  printf("Select mode (click 3D window first):\n");
  printf("  1 = Keyboard control with Markov localization\n");
  printf("  2 = Autonomous navigation (drone waypoints)\n");

  RobotMode mode = MODE_AUTONOMOUS;
  bool mode_selected = false;
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
      printf("Mode: Autonomous navigation\n");
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

  WbDeviceTag front_ds = 0, camera = 0, receiver = 0;

  if (mode == MODE_KEYBOARD) {
    camera = wb_robot_get_device("ground_cam");
    if (camera)
      wb_camera_enable(camera, timeStep);
    markov_init();
    printf("\nKeyboard mode: W=forward, S=backward, A=left, D=right\n");
  }
  else {
    front_ds = wb_robot_get_device("ds0");
    if (front_ds)
      wb_distance_sensor_enable(front_ds, timeStep);
    receiver = wb_robot_get_device("receiver");
    if (receiver)
      wb_receiver_enable(receiver, timeStep);
    printf("\nAutonomous mode: Waiting for drone coordinates...\n");
  }

  RobotContext ctx = {.state = STATE_INIT, .has_target = false};
  int init_counter = 0;

  while (wb_robot_step(timeStep) != -1) {
    Pose pose = get_robot_pose(gps, compass);

    if (mode == MODE_KEYBOARD) {
      int key = wb_keyboard_get_key();
      double left_speed = 0, right_speed = 0;
      if (key == 'W') {
        left_speed = 0.3;
        right_speed = 0.3;
      }
      else if (key == 'S') {
        left_speed = -0.3;
        right_speed = -0.3;
      }
      else if (key == 'A') {
        left_speed = -0.2;
        right_speed = 0.2;
      }
      else if (key == 'D') {
        left_speed = 0.2;
        right_speed = -0.2;
      }
      set_motor_speeds(motors, left_speed, right_speed);
      markov_step(pose);

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
    else
    {
      /* Check for new waypoint from drone */
      if (receiver && wb_receiver_get_queue_length(receiver) > 0 && !ctx.has_target) {
        const char *msg = (const char *)wb_receiver_get_data(receiver);
        sscanf(msg, "%lf,%lf", &ctx.target_x, &ctx.target_y);
        printf("New target from drone: (%.2f, %.2f)\n", ctx.target_x, ctx.target_y);
        ctx.has_target = true;
        ctx.state = STATE_NAVIGATE;
        wb_receiver_next_packet(receiver);
      }

      double front_distance = front_ds ? wb_distance_sensor_get_value(front_ds) : 0;

      switch (ctx.state) {
      case STATE_INIT:
        if (++init_counter >= 10) {
          ctx.prev_front_distance = front_distance;
          ctx.state = ctx.has_target ? STATE_NAVIGATE : STATE_IDLE;
        }
        break;

      case STATE_IDLE:
        set_motor_speeds(motors, 0, 0);
        break;

      case STATE_NAVIGATE:
      {
        double change = fabs(front_distance - ctx.prev_front_distance);
        ctx.prev_front_distance = front_distance;

        if (distance_to_point(pose, ctx.target_x, ctx.target_y) < TOLERANCE)
        {
          printf("Reached target (%.2f, %.2f)\n", ctx.target_x, ctx.target_y);
          ctx.has_target = false;
          ctx.pause_counter = 0;
          set_motor_speeds(motors, 0, 0);
          ctx.state = STATE_PAUSE;
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
          ctx.state = ctx.has_target ? STATE_NAVIGATE : STATE_IDLE;
        break;
      }
    }
  }

  wb_robot_cleanup();
  return 0;
}