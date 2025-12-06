#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/compass.h>
#include <webots/distance_sensor.h>

#define MAX_SPEED 0.25
#define TOLERANCE 0.05
#define SLOW_DOWN_DISTANCE 1.0
#define OBSTACLE_THRESHOLD 1.0
#define OBSTACLE_SUSTAINED_FRAMES 5
#define TERRAIN_MAX_CHANGE 70.0
#define PAUSE_STEPS 50

static double waypoints[][2] = {
  {1.0, 1.0},
  {1.0, -1.0},
  {-1.0, -1.0},
  {-1.0, 1.0}
};
static int num_waypoints = 4;

typedef enum {
  STATE_INIT,
  STATE_NAVIGATE,
  STATE_OBSTACLE_CHECK,
  STATE_AVOID_OBSTACLE,
  STATE_PAUSE
} RobotState;

typedef struct {
  double x;
  double y;
  double heading;
} Pose;

typedef struct {
  WbDeviceTag left;
  WbDeviceTag right;
} Motors;

typedef struct {
  RobotState state;
  int current_waypoint;
  double prev_front_distance;
  int obstacle_frames;
  double detour_x, detour_y;
  int pause_counter;
  bool advance_after_pause;
} RobotContext;

static double normalize_angle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
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
  double dx = x - pose.x;
  double dy = y - pose.y;
  return sqrt(dx * dx + dy * dy);
}

static void drive_to_target(Motors motors, Pose pose, double target_x, double target_y) {
  double dx = target_x - pose.x;
  double dy = target_y - pose.y;
  double distance = sqrt(dx * dx + dy * dy);
  
  double desired_heading = atan2(dy, dx);
  double heading_error = normalize_angle(desired_heading - pose.heading);
  
  double speed = MAX_SPEED;
  if (distance < SLOW_DOWN_DISTANCE) {
    speed *= (0.3 + 0.7 * distance / SLOW_DOWN_DISTANCE);
  }
  
  if (fabs(heading_error) > TOLERANCE * 3.0) {
    double turn_speed = speed * 0.5;
    set_motor_speeds(motors, 
                    heading_error > 0 ? -turn_speed : turn_speed,
                    heading_error > 0 ? turn_speed : -turn_speed);
  } else {
    double correction = heading_error * 0.4;
    set_motor_speeds(motors, 
                    speed * (1.0 - correction * 0.3),
                    speed * (1.0 + correction * 0.3));
  }
}

int main(int argc, char **argv) {
  wb_robot_init();
  
  const int timeStep = wb_robot_get_basic_time_step();
  
  Motors motors;
  motors.left = wb_robot_get_device("left motor");
  motors.right = wb_robot_get_device("right motor");
  WbDeviceTag gps = wb_robot_get_device("gps");
  WbDeviceTag compass = wb_robot_get_device("compass");
  WbDeviceTag front_ds = wb_robot_get_device("ds0");
  
  if (motors.left == 0 || motors.right == 0 || gps == 0 || compass == 0 || front_ds == 0) {
    printf("Error: Required devices not found!\n");
    wb_robot_cleanup();
    return 1;
  }
  
  wb_gps_enable(gps, timeStep);
  wb_compass_enable(compass, timeStep);
  wb_distance_sensor_enable(front_ds, timeStep);
  
  wb_motor_set_position(motors.left, INFINITY);
  wb_motor_set_position(motors.right, INFINITY);
  set_motor_speeds(motors, 0, 0);
  
  RobotContext ctx = {
    .state = STATE_INIT,
    .current_waypoint = 0,
    .prev_front_distance = 0,
    .obstacle_frames = 0,
    .detour_x = 0,
    .detour_y = 0,
    .pause_counter = 0,
    .advance_after_pause = false
  };
  
  int init_counter = 0;
  
  while (wb_robot_step(timeStep) != -1) {
    Pose pose = get_robot_pose(gps, compass);
    double front_distance = wb_distance_sensor_get_value(front_ds);
    
    switch (ctx.state) {
      
      case STATE_INIT:
        init_counter++;
        if (init_counter >= 10) {
          ctx.prev_front_distance = front_distance;
          ctx.state = STATE_NAVIGATE;
        }
        break;
      
      case STATE_NAVIGATE: {
        double change = fabs(front_distance - ctx.prev_front_distance);
        ctx.prev_front_distance = front_distance;
        
        if (distance_to_point(pose, waypoints[ctx.current_waypoint][0], 
                              waypoints[ctx.current_waypoint][1]) < TOLERANCE) {
          ctx.advance_after_pause = true;
          ctx.pause_counter = 0;
          set_motor_speeds(motors, 0, 0);
          ctx.state = STATE_PAUSE;
        } else if (change <= TERRAIN_MAX_CHANGE && front_distance > OBSTACLE_THRESHOLD) {
          ctx.obstacle_frames = 1;
          ctx.state = STATE_OBSTACLE_CHECK;
        } else {
          ctx.obstacle_frames = 0;
          drive_to_target(motors, pose, waypoints[ctx.current_waypoint][0],
                          waypoints[ctx.current_waypoint][1]);
        }
        break;
      }
      
      case STATE_OBSTACLE_CHECK: {
        double change = fabs(front_distance - ctx.prev_front_distance);
        ctx.prev_front_distance = front_distance;
        
        if (change > TERRAIN_MAX_CHANGE) {
          ctx.obstacle_frames = 0;
          ctx.state = STATE_NAVIGATE;
        } else if (front_distance > OBSTACLE_THRESHOLD) {
          ctx.obstacle_frames++;
          if (ctx.obstacle_frames >= OBSTACLE_SUSTAINED_FRAMES) {
            ctx.detour_x = pose.x + 0.3 * sin(pose.heading);
            ctx.detour_y = pose.y - 0.3 * cos(pose.heading);
            ctx.obstacle_frames = 0;
            ctx.state = STATE_AVOID_OBSTACLE;
          } else {
            drive_to_target(motors, pose, waypoints[ctx.current_waypoint][0],
                            waypoints[ctx.current_waypoint][1]);
          }
        } else {
          ctx.obstacle_frames = 0;
          ctx.state = STATE_NAVIGATE;
        }
        break;
      }
      
      case STATE_AVOID_OBSTACLE:
        if (distance_to_point(pose, ctx.detour_x, ctx.detour_y) < TOLERANCE) {
          ctx.advance_after_pause = false;
          ctx.pause_counter = 0;
          set_motor_speeds(motors, 0, 0);
          ctx.state = STATE_PAUSE;
        } else {
          drive_to_target(motors, pose, ctx.detour_x, ctx.detour_y);
        }
        break;
      
      case STATE_PAUSE:
        ctx.pause_counter++;
        if (ctx.pause_counter >= PAUSE_STEPS) {
          if (ctx.advance_after_pause) {
            ctx.current_waypoint++;
            if (ctx.current_waypoint >= num_waypoints) {
              ctx.current_waypoint = 0;
            }
          }
          ctx.state = STATE_NAVIGATE;
        }
        break;
    }
  }
  
  wb_robot_cleanup();
  return 0;
}