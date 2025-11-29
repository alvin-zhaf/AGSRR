#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/compass.h>

#define MAX_SPEED 1.0
#define POSITION_TOLERANCE 0.05
#define ANGLE_TOLERANCE 0.05
#define MIN_DISTANCE_FOR_FULL_SPEED 1.0
#define STUCK_THRESHOLD 0.005
#define STUCK_TIME_LIMIT 2.0
#define NEAR_TARGET_DISTANCE 0.15
#define VERY_CLOSE_DISTANCE 0.1

static double target_x = 1.0;
static double target_y = 0.5;

static double normalize_angle(double angle) {
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

int main(int argc, char **argv) {
  wb_robot_init();
  
  const int timeStep = wb_robot_get_basic_time_step();
  
  WbDeviceTag leftMotor = wb_robot_get_device("left motor");
  WbDeviceTag rightMotor = wb_robot_get_device("right motor");
  WbDeviceTag gps = wb_robot_get_device("gps");
  WbDeviceTag compass = wb_robot_get_device("compass");
  
  if (leftMotor == 0 || rightMotor == 0 || gps == 0 || compass == 0) {
    printf("Error: Required devices not found!\n");
    wb_robot_cleanup();
    return 1;
  }
  
  wb_gps_enable(gps, timeStep);
  wb_compass_enable(compass, timeStep);
  
  wb_motor_set_position(leftMotor, INFINITY);
  wb_motor_set_position(rightMotor, INFINITY);
  wb_motor_set_velocity(leftMotor, 0);
  wb_motor_set_velocity(rightMotor, 0);

  printf("Target: (%.2f, %.2f)\n", target_x, target_y);
  
  for (int i = 0; i < 10; i++) {
    wb_robot_step(timeStep);
  }
  
  double prev_x = 0, prev_y = 0;
  double stuck_timer = 0;
  int recovery_phase = 0;
  int recovery_counter = 0;
  double last_distance = 999.0;
  int oscillation_counter = 0;
  
  while (wb_robot_step(timeStep) != -1) {
    const double *gps_values = wb_gps_get_values(gps);
    if (gps_values == NULL)
      continue;
    
    double current_x = gps_values[0];
    double current_y = gps_values[1];
    
    const double *compass_values = wb_compass_get_values(compass);
    if (compass_values == NULL)
      continue;
    
    double current_heading = atan2(compass_values[0], compass_values[1]);
    
    double dx = target_x - current_x;
    double dy = target_y - current_y;
    double distance = sqrt(dx * dx + dy * dy);
    
    if (distance < POSITION_TOLERANCE) {
      printf("Final Position: (%.3f, %.3f)\n", current_x, current_y);
      printf("Error: %.3f meters (%.1f cm)\n", distance, distance * 100);
      wb_motor_set_velocity(leftMotor, 0);
      wb_motor_set_velocity(rightMotor, 0);
      break;
    }
    
    double movement = sqrt(pow(current_x - prev_x, 2) + pow(current_y - prev_y, 2));
    bool is_approaching = (distance < last_distance - 0.001);
    bool near_target = (distance < NEAR_TARGET_DISTANCE);
    bool very_close = (distance < VERY_CLOSE_DISTANCE);
    
    if (very_close && fabs(distance - last_distance) < 0.002) {
      oscillation_counter++;
      if (oscillation_counter > 100) {
        printf("\nClose enough! Stopping.\n");
        wb_motor_set_velocity(leftMotor, 0);
        wb_motor_set_velocity(rightMotor, 0);
        break;
      }
    } else {
      oscillation_counter = 0;
    }
    
    if (recovery_phase == 0 && !very_close) {
      if (!near_target && !is_approaching && movement < STUCK_THRESHOLD) {
        stuck_timer += timeStep / 1000.0;
        if (stuck_timer > STUCK_TIME_LIMIT) {
          printf("STUCK! Starting recovery...\n");
          recovery_phase = 1;
          recovery_counter = 0;
          stuck_timer = 0;
        }
      } else {
        stuck_timer = 0;
      }
    }
    
    prev_x = current_x;
    prev_y = current_y;
    last_distance = distance;
    
    if (recovery_phase > 0) {
      recovery_counter++;
      
      if (recovery_phase == 1) {
        wb_motor_set_velocity(leftMotor, -MAX_SPEED * 0.5);
        wb_motor_set_velocity(rightMotor, -MAX_SPEED * 0.5);
        
        if (recovery_counter > (1500 / timeStep)) {
          recovery_phase = 2;
          recovery_counter = 0;
        }
      } else if (recovery_phase == 2) {
        static int turn_dir = 0;
        if (turn_dir == 0) {
          turn_dir = (rand() % 2) * 2 - 1;
        }
        
        wb_motor_set_velocity(leftMotor, turn_dir * MAX_SPEED * 0.6);
        wb_motor_set_velocity(rightMotor, -turn_dir * MAX_SPEED * 0.6);
        
        if (recovery_counter > (1000 / timeStep)) {
          recovery_phase = 0;
          turn_dir = 0;
          printf("Recovery complete.\n");
        }
      }
      continue;
    }
    
    double desired_heading = atan2(dy, dx);
    double heading_error = normalize_angle(desired_heading - current_heading);
    
    double speed_factor = 1.0;
    if (distance < MIN_DISTANCE_FOR_FULL_SPEED) {
      speed_factor = 0.3 + 0.7 * (distance / MIN_DISTANCE_FOR_FULL_SPEED);
      if (speed_factor < 0.2) speed_factor = 0.2;
    }
    
    double speed = MAX_SPEED * speed_factor;
    
    if (very_close) {
      double direct_speed = MAX_SPEED * 0.3;
      wb_motor_set_velocity(leftMotor, direct_speed);
      wb_motor_set_velocity(rightMotor, direct_speed);
    } else if (fabs(heading_error) > ANGLE_TOLERANCE * 3.0) {
      double turn_speed = speed * 0.5;
      wb_motor_set_velocity(leftMotor, heading_error > 0 ? -turn_speed : turn_speed);
      wb_motor_set_velocity(rightMotor, heading_error > 0 ? turn_speed : -turn_speed);
    } else {
      double angular_correction = heading_error * 0.4;
      double left_speed = speed * (1.0 - angular_correction * 0.3);
      double right_speed = speed * (1.0 + angular_correction * 0.3);
      wb_motor_set_velocity(leftMotor, left_speed);
      wb_motor_set_velocity(rightMotor, right_speed);
    }
  }
  
  wb_robot_cleanup();
  return 0;
}