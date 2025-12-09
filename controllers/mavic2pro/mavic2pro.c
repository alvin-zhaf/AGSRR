#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h> 

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

// --- GLOBAL VICTIM DATA STORAGE ---
typedef struct {
    char name[16];
    double x, y;
    bool detected; 
} VictimReport;

static VictimReport g_victim_reports[NUM_VICTIMS];
static bool g_all_waypoints_reached = false;
static bool g_summary_printed = false;

// Target positions
double waypoints[][3] = {
  {0.50, 3.50, 1.00},
  {0.50, 0.50, 2.00},
  {1.00, 0.50, 2.00},
  {1.00, 3.50, 2.00},
  {1.50, 3.50, 2.00},
  {1.50, 0.50, 2.00},
  {2.00, 0.50, 2.00},
  {2.00, 3.50, 2.00},
  {2.50, 3.50, 2.00},
  {2.50, 0.50, 2.00},
  {3.00, 0.50, 2.00},
  {3.00, 3.50, 2.00},
  {3.50, 3.50, 2.00},
  {3.50, 0.50, 2.00},
  {0.50, 3.50, 2.00},
};
int num_waypoints = sizeof(waypoints) / sizeof(waypoints[0]);
int current_target = 0;

// --- SUMMARY AND FINAL ACTIONS FUNCTION ---
void print_summary_and_finalize() {
    if (g_summary_printed) return;

    printf("\n==========================================\n");
    printf(" SAR MISSION SUMMARY\n");
    printf("==========================================\n");

    int tagged_count = 0;
    double red_final[3] = {1, 0, 0}; // The final color: RED
    
    for (int i = 0; i < NUM_VICTIMS; i++) {
        if (g_victim_reports[i].detected) {
            tagged_count++;
        }
    }
    
    printf("Area Scanned: 100%% (Completed all %d waypoints)\n", num_waypoints);
    printf("Total Victims Found and Tagged: %d", tagged_count);
    printf("\n------------------------------------------\n");
    printf("List of Victims:\n");

    for (int i = 0; i < NUM_VICTIMS; i++) {
        WbNodeRef victim = wb_supervisor_node_get_from_def(victim_defs[i]);
        
        if (g_victim_reports[i].detected) {
            printf(" - %s: found at X=%.2f, Y=%.2f (Tagged RED)\n", 
                   g_victim_reports[i].name, 
                   g_victim_reports[i].x, 
                   g_victim_reports[i].y);
            
            // Detected victims are tagged to red
            if (victim) {
                WbFieldRef color_field = wb_supervisor_node_get_field(victim, "shirtColor");
                if (color_field) {
                    wb_supervisor_field_set_sf_color(color_field, red_final);
                }
            }
            
        } else {
             printf(" - %s: NOT FOUND\n", g_victim_reports[i].name);
        }
    }
    printf("==========================================\n");
    
    g_summary_printed = true;
}


int main(int argc, char **argv) {
  wb_robot_init();
  int timestep = (int)wb_robot_get_basic_time_step();

  // Initialize victim reports
  for (int i = 0; i < NUM_VICTIMS; i++) {
      strcpy(g_victim_reports[i].name, victim_defs[i]);
      g_victim_reports[i].detected = false; 
  }
  
  // ----- DEVICE SETUP AND ENABLE -----
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
  
  const double k_vertical_thrust = 68.5; 
  const double k_vertical_offset = 0.6;
  const double k_vertical_p = 3.0;
  const double k_roll_p = 50.0;
  const double k_pitch_p = 30.0;

  printf("Starting waypoint navigation. Total points: %d\n", num_waypoints);
  
  // ----- MAIN SIMULATION LOOP -----
  while (wb_robot_step(timestep) != -1) {
    const double time = wb_robot_get_time();

    const double roll = wb_inertial_unit_get_roll_pitch_yaw(imu)[0];
    const double pitch = wb_inertial_unit_get_roll_pitch_yaw(imu)[1];
    const double x = wb_gps_get_values(gps)[0];
    const double y = wb_gps_get_values(gps)[1];
    const double altitude = wb_gps_get_values(gps)[2];
    const double roll_velocity = wb_gyro_get_values(gyro)[0];
    const double pitch_velocity = wb_gyro_get_values(gyro)[1];
    
    // Blink the front LEDs.
    const bool led_state = ((int)time) % 2;
    wb_led_set(front_left_led, led_state);
    wb_led_set(front_right_led, !led_state);

    // Stabilize the Camera.
    wb_motor_set_position(camera_roll_motor, -0.115 * roll_velocity);
    wb_motor_set_position(camera_pitch_motor, 1.57);

    // Update target
    double target_x = waypoints[current_target][0];
    double target_y = waypoints[current_target][1];
    double target_altitude = waypoints[current_target][2];
    
    double error_x = target_x - x;
    double error_y = target_y - y;
    double error_altitude = target_altitude - altitude;
    double error_xy = sqrt(error_x*error_x + error_y*error_y);

    double pitch_disturbance = CLAMP(-error_x * 2, -2.0, 2.0); 
    double roll_disturbance  = CLAMP(error_y * 2, -2.0, 2.0);  
    double yaw_disturbance = 0.0;
    const double clamped_difference_altitude = CLAMP(target_altitude - altitude + k_vertical_offset, -1.0, 1.0);
    const double vertical_input = k_vertical_p * pow(clamped_difference_altitude, 3.0);

    const double roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_velocity + roll_disturbance;
    const double pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) + pitch_velocity + pitch_disturbance;
    const double yaw_input = yaw_disturbance;

    const double front_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input;
    const double front_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input;
    const double rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input;
    const double rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input;
    wb_motor_set_velocity(front_left_motor, front_left_motor_input);
    wb_motor_set_velocity(front_right_motor, -front_right_motor_input);
    wb_motor_set_velocity(rear_left_motor, -rear_left_motor_input);
    wb_motor_set_velocity(rear_right_motor, rear_right_motor_input);
    
    // VICTIM DETECTION
    const unsigned char *image = wb_camera_get_image(camera);

    if (image != NULL) {
      int width = wb_camera_get_width(camera);
      int height = wb_camera_get_height(camera);

      int r = wb_camera_image_get_red(image, width/2, height/2, width);
      int g = wb_camera_image_get_green(image, width/2, height/2, width);
      int b = wb_camera_image_get_blue(image, width/2, height/2, width);

      // Detecting green colour (G dominant, G > 100)
      if (g > r + 30 && g > b + 30 && g > 100) { 
        
        // Loop through all victims and process untagged ones
        for (int v = 0; v < NUM_VICTIMS; v++) {
          // Skip if already detected
          if (g_victim_reports[v].detected) continue;

          WbNodeRef victim = wb_supervisor_node_get_from_def(victim_defs[v]);
          
          if (!victim) continue; 
          
          WbFieldRef color_field = wb_supervisor_node_get_field(victim, "shirtColor");
          
          if (color_field) {
             const double *current_color = wb_supervisor_field_get_sf_color(color_field);
             
             // Check if the victim is currently green (R is close to 0)
             if (current_color[0] < 0.1) { 
                 
                 const double *pos = wb_supervisor_node_get_position(victim);
                 
                 // Coordinates
                 g_victim_reports[v].x = pos[0];
                 g_victim_reports[v].y = pos[1];
                 g_victim_reports[v].detected = true;
                 
                 // Send Coordinates to Ground Robot (Surveyor)
                 char message[128];
                 sprintf(message, "%.2f,%.2f", pos[0], pos[1]); 
                 wb_emitter_send(emitter, message, strlen(message) + 1);
                 
              
             }
          }
        }
      }
    }

    // ----- WAYPOINT CHECK -----
    if (error_xy < 0.20 && fabs(error_altitude) < 0.10) {
      printf("Reached waypoint %d: (%.2f, %.2f, %.2f)\n", current_target, target_x, target_y, target_altitude);
      if (current_target < num_waypoints - 1) {
        current_target++;
        printf("Moving to next waypoint %d...\n", current_target);
      } else {
        // All waypoints reached! Prepare for summary and cleanup.
        g_all_waypoints_reached = true; 
        printf("Final waypoint reached. Mission complete.\n");
        break; // Exit the main loop to proceed to summary
      }
    } 
  }

  // --- MISSION END & SUMMARY ---
  if (g_all_waypoints_reached) {
      print_summary_and_finalize();
  }

  wb_robot_cleanup();
  return EXIT_SUCCESS;
}