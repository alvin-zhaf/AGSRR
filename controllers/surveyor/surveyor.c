#include <stdio.h>
#include <math.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/compass.h>

#define MAX_SPEED 0.04
#define POSITION_TOLERANCE 0.05
#define ANGLE_TOLERANCE 0.05

static double target_x = 1.0;
static double target_y = 0.5;

static double normalize_angle(double angle)
{
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

int main(int argc, char **argv)
{
  wb_robot_init();

  const int timeStep = wb_robot_get_basic_time_step();

  WbDeviceTag leftMotor = wb_robot_get_device("left motor");
  WbDeviceTag rightMotor = wb_robot_get_device("right motor");
  WbDeviceTag gps = wb_robot_get_device("gps");
  WbDeviceTag compass = wb_robot_get_device("compass");

  if (leftMotor == 0 || rightMotor == 0 || gps == 0 || compass == 0)
  {
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

  for (int i = 0; i < 10; i++)
  {
    wb_robot_step(timeStep);
  }

  while (wb_robot_step(timeStep) != -1)
  {
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

    if (distance < POSITION_TOLERANCE)
    {
      printf("Final Position: (%.3f, %.3f)\n", current_x, current_y);
      printf("Error: %.3f meters\n", distance);
      wb_motor_set_velocity(leftMotor, 0);
      wb_motor_set_velocity(rightMotor, 0);
      break;
    }

    double desired_heading = atan2(dy, dx);
    double heading_error = normalize_angle(desired_heading - current_heading);

    double angular_correction = heading_error * 0.5;

    if (fabs(heading_error) > ANGLE_TOLERANCE)
    {
      wb_motor_set_velocity(leftMotor, -angular_correction * MAX_SPEED);
      wb_motor_set_velocity(rightMotor, angular_correction * MAX_SPEED);
    }
    else
    {
      wb_motor_set_velocity(leftMotor, MAX_SPEED);
      wb_motor_set_velocity(rightMotor, MAX_SPEED);
    }
  }

  wb_robot_cleanup();
  return 0;
}