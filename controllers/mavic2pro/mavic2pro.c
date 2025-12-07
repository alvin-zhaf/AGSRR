#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <webots/robot.h>

#include <webots/camera.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/supervisor.h>
#include <webots/emitter.h>


#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

#define NUM_VICTIMS 3
const char *victim_defs[NUM_VICTIMS] = {"victim1", "victim2", "victim3"};

int main(int argc, char **argv) {
  wb_robot_init();
  int timestep = (int)wb_robot_get_basic_time_step();

  // Get and enable devices.
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, timestep);
  WbDeviceTag front_left_led = wb_robot_get_device("front left led");
  WbDeviceTag front_right_led = wb_robot_get_device("front right led");
  WbDeviceTag imu = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(imu, timestep);
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, timestep);
  WbDeviceTag compass = wb_robot_get_device("compass");
  wb_compass_enable(compass, timestep);
  WbDeviceTag gyro = wb_robot_get_device("gyro");
  wb_gyro_enable(gyro, timestep);
  wb_keyboard_enable(timestep);
  WbDeviceTag camera_roll_motor = wb_robot_get_device("camera roll");
  WbDeviceTag camera_pitch_motor = wb_robot_get_device("camera pitch");
  // WbDeviceTag camera_yaw_motor = wb_robot_get_device("camera yaw");  // Not used in this example.

  // Get propeller motors and set them to velocity mode.
  WbDeviceTag front_left_motor = wb_robot_get_device("front left propeller");
  WbDeviceTag front_right_motor = wb_robot_get_device("front right propeller");
  WbDeviceTag rear_left_motor = wb_robot_get_device("rear left propeller");
  WbDeviceTag rear_right_motor = wb_robot_get_device("rear right propeller");
  WbDeviceTag motors[4] = {front_left_motor, front_right_motor, rear_left_motor, rear_right_motor};
  int m;
  for (m = 0; m < 4; ++m) {
    wb_motor_set_position(motors[m], INFINITY);
    wb_motor_set_velocity(motors[m], 1.0);
  }
  
  WbDeviceTag emitter = wb_robot_get_device("emitter");
  wb_emitter_set_channel(emitter, 1);

 // Constants, empirically found.
  const double k_vertical_thrust = 68.5;  // with this thrust, the drone lifts.
  const double k_vertical_offset = 0.6;   // Vertical offset where the robot actually targets to stabilize itself.
  const double k_vertical_p = 3.0;        // P constant of the vertical PID.
  const double k_roll_p = 50.0;           // P constant of the roll PID.
  const double k_pitch_p = 30.0;          // P constant of the pitch PID.

  // Target positions
  double waypoints[][3] = {
  {0.5, 3.5, 1.0},
  {0.5, 0.5, 1.0},
  {1.0, 0.5, 1.0},
  {1.0, 3.5, 1.0},
  {1.5, 3.5, 1.0},
  {1.5, 0.5, 1.0},
  {2.0, 0.5, 1.0},
  {2.0, 3.5, 1.0},
  {2.5, 3.5, 1.0},
  {2.5, 0.5, 1.0},
  {3.0, 0.5, 1.0},
  {3.0, 3.5, 1.0},
  {3.5, 3.5, 1.0},
  {3.5, 0.5, 1.0},
  {0.5, 3.5, 1.0}
  };
  int num_waypoints = sizeof(waypoints) / sizeof(waypoints[0]);
  int current_target = 0;
  printf("Starting waypoint navigation. Total points: %d\n", num_waypoints);
  

  // Main loop
  while (wb_robot_step(timestep) != -1) {
    const double time = wb_robot_get_time();  // in seconds.

    // Retrieve robot position using the sensors.
    const double roll = wb_inertial_unit_get_roll_pitch_yaw(imu)[0];
    const double pitch = wb_inertial_unit_get_roll_pitch_yaw(imu)[1];
    const double x = wb_gps_get_values(gps)[0];
    const double y = wb_gps_get_values(gps)[1];
    const double altitude = wb_gps_get_values(gps)[2];
    const double roll_velocity = wb_gyro_get_values(gyro)[0];
    const double pitch_velocity = wb_gyro_get_values(gyro)[1];
    
    // Blink the front LEDs alternatively with a 1 second rate.
    const bool led_state = ((int)time) % 2;
    wb_led_set(front_left_led, led_state);
    wb_led_set(front_right_led, !led_state);

    // Stabilize the Camera by actuating the camera motors according to the gyro feedback.
    wb_motor_set_position(camera_roll_motor, -0.115 * roll_velocity);
    wb_motor_set_position(camera_pitch_motor, -0.1 * pitch_velocity);

    // Update target
    double target_x = waypoints[current_target][0];
    double target_y = waypoints[current_target][1];
    double target_altitude = waypoints[current_target][2];
    
    // Calculate xy position error
    double error_x = target_x - x;
    double error_y = target_y - y;
    double error_altitude = target_altitude - altitude;
    double error_xy = sqrt(error_x*error_x + error_y*error_y);

    // Compute the roll, pitch, yaw and disturbances.
    double pitch_disturbance = CLAMP(-error_x * 2.0, -2.0, 2.0); // Along sagittal plane
    double roll_disturbance  = CLAMP(error_y * 2.0, -2.0, 2.0);  // Along frontal plane
    double yaw_disturbance = 0.0;  // Rotation
    const double clamped_difference_altitude = CLAMP(target_altitude - altitude + k_vertical_offset, -1.0, 1.0);
    const double vertical_input = k_vertical_p * pow(clamped_difference_altitude, 3.0);

    // Compute the roll, pitch, yaw and vertical inputs.
    const double roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_velocity + roll_disturbance;
    const double pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) + pitch_velocity + pitch_disturbance;
    const double yaw_input = yaw_disturbance;

    /// Actuate the motors taking into consideration all the computed inputs.
    const double front_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input;
    const double front_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input;
    const double rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input;
    const double rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input;
    wb_motor_set_velocity(front_left_motor, front_left_motor_input);
    wb_motor_set_velocity(front_right_motor, -front_right_motor_input);
    wb_motor_set_velocity(rear_left_motor, -rear_left_motor_input);
    wb_motor_set_velocity(rear_right_motor, rear_right_motor_input);
    
    // Check if waypoint reached (within 20 cm horizontally, 10 cm vertically)
    if (error_xy < 0.20 && fabs(error_altitude) < 0.10) {
      printf("Reached waypoint %d: (%f, %f, %f)\n", current_target, target_x, target_y, target_altitude);
      if (current_target < num_waypoints - 1) {
        current_target++;
        printf("Moving to next waypoint %d...\n", current_target);
      } else {
        printf("Final waypoint reached. Holding position.\n");
      }
      
    // Victim Detection and Tagging
    const unsigned char *image = wb_camera_get_image(camera);
    if (image != NULL) {
      int width = wb_camera_get_width(camera);
      int height = wb_camera_get_height(camera);

      for (int v = 0; v < NUM_VICTIMS; v++) {
        WbNodeRef victim = wb_supervisor_node_get_from_def(victim_defs[v]);
        if (!victim) continue;

        const double *pos = wb_supervisor_node_get_position(victim);

        // Detecting green
        int r = wb_camera_image_get_red(image, width/2, height/2, width);
        int g = wb_camera_image_get_green(image, width/2, height/2, width);
        int b = wb_camera_image_get_blue(image, width/2, height/2, width);

        if (g > 200 && r < 50 && b < 50) {  // green detected
          // Tag victim by changing shirt to red
          WbFieldRef appearance = wb_supervisor_node_get_field(victim, "appearance");
          WbNodeRef app_node = wb_supervisor_field_get_sf_node(appearance);
          WbFieldRef color_field = wb_supervisor_node_get_field(app_node, "baseColor");
          double red[3] = {1, 0, 0};
          wb_supervisor_field_set_sf_color(color_field, red);

          // Sending coordinates to ground robot
          char message[128];
          sprintf(message, "Victim tagged at: %f %f %f", pos[0], pos[1], pos[2]);
          wb_emitter_send(emitter, message, strlen(message)+1);
          printf("Tagged and sent: %s\n", message);
        }
      }
    }
  }
  };
  
  wb_robot_cleanup();
  return EXIT_SUCCESS;
}
