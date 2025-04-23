#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/keyboard.h>
#include <webots/lidar.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/camera.h>

#define TIME_STEP 16
#define TARGET_POINTS_SIZE 2
#define DISTANCE_TOLERANCE 0.2
#define MAX_SPEED 5
#define TURN_COEFFICIENT 4.0
#define OBSTACLE_THRESHOLD 0.8

enum XYZAComponents { X = 0, Y, Z, ALPHA };
enum Sides { LEFT, RIGHT };

typedef struct _Vector {
  double u;
  double v;
} Vector;

static WbDeviceTag motors[2];
static WbDeviceTag gps;
static WbDeviceTag compass;
static WbDeviceTag lidar;
static WbDeviceTag camera;
static WbDeviceTag camera_roll_motor;
static WbDeviceTag camera_pitch_motor;

static Vector targets[TARGET_POINTS_SIZE] = {
  {1.5,1.5},
  {-1.5, -1.5},
};

static int current_target_index = 0;
static bool autopilot = true;
static bool old_autopilot = true;
static int old_key = -1;

static double modulus_double(double a, double m) {
  const int div = (int)(a / m);
  double r = a - div * m;
  if (r < 0.0)
    r += m;
  return r;
}

static void robot_set_speed(double left, double right) {
  wb_motor_set_velocity(motors[0], left);
  wb_motor_set_velocity(motors[1], right);
}

static void check_keyboard() {
  double speeds[2] = {0.0, 0.0};
  int key = wb_keyboard_get_key();

  if (key >= 0) {
    switch (key) {
      case WB_KEYBOARD_UP:
        speeds[LEFT] = MAX_SPEED;
        speeds[RIGHT] = MAX_SPEED;
        autopilot = false;
        break;
      case WB_KEYBOARD_DOWN:
        speeds[LEFT] = -MAX_SPEED;
        speeds[RIGHT] = -MAX_SPEED;
        autopilot = false;
        break;
      case WB_KEYBOARD_RIGHT:
        speeds[LEFT] = MAX_SPEED;
        speeds[RIGHT] = -MAX_SPEED;
        autopilot = false;
        break;
      case WB_KEYBOARD_LEFT:
        speeds[LEFT] = -MAX_SPEED;
        speeds[RIGHT] = MAX_SPEED;
        autopilot = false;
        break;
      case 'P':
        if (key != old_key) {
          const double *position_3d = wb_gps_get_values(gps);
          printf("position: {%f, %f, %f}\n", position_3d[X], position_3d[Y], position_3d[Z]);
        }
        break;
      case 'A':
        if (key != old_key)
          autopilot = !autopilot;
        break;
      case 'C':
        if (key != old_key) {
          printf("Saving camera image\n");
          wb_camera_save_image(camera, "turtlebot_camera_snapshot.png", 100);
        }
        break;
    }
  }

  if (autopilot != old_autopilot) {
    old_autopilot = autopilot;
    printf("%s control\n", autopilot ? "auto" : "manual");
  }

  robot_set_speed(speeds[LEFT], speeds[RIGHT]);
  old_key = key;
}

static double norm(const Vector *v) {
  return sqrt(v->u * v->u + v->v * v->v);
}

static void normalize(Vector *v) {
  double n = norm(v);
  if (n > 0.0) {
    v->u /= n;
    v->v /= n;
  }
}

static void minus(Vector *v, const Vector *const v1, const Vector *const v2) {
  v->u = v1->u - v2->u;
  v->v = v1->v - v2->v;
}

double gaussian(double x, double mu, double sigma) {
  return (1.0 / (sigma * sqrt(2.0 * M_PI))) * exp(-((x - mu) * (x - mu)) / (2 * sigma * sigma));
}

static void stabilize_camera(double roll_velocity, double pitch_velocity) {
  if (camera_roll_motor != 0 && camera_pitch_motor != 0) {
    wb_motor_set_position(camera_roll_motor, -0.115 * roll_velocity);
    wb_motor_set_position(camera_pitch_motor, -0.1 * pitch_velocity);
  }
}

static void log_point_cloud_and_path() {
  const WbLidarPoint *point_cloud = wb_lidar_get_point_cloud(lidar);
  int point_count = wb_lidar_get_number_of_points(lidar);
  const double *robot_pos = wb_gps_get_values(gps);

  FILE *pc_file = fopen("point_cloud.csv", "a");
  if (pc_file) {
    for (int i = 0; i < point_count; i++) {
      double global_x = robot_pos[0] + point_cloud[i].x;
      double global_y = robot_pos[1] + point_cloud[i].y;
      double global_z = robot_pos[2] + point_cloud[i].z;
      fprintf(pc_file, "%f,%f,%f\n", global_x, global_y, global_z);
    }
    fclose(pc_file);
  }

  FILE *path_file = fopen("robot_path.csv", "a");
  if (path_file) {
    fprintf(path_file, "%f,%f,%f\n", robot_pos[0], robot_pos[1], robot_pos[2]);
    fclose(path_file);
  }
}

static void run_autopilot() {
  double speeds[2] = {0.0, 0.0};
  const double *position_3d = wb_gps_get_values(gps);
  const double *north_3d = wb_compass_get_values(compass);
  const Vector position = {position_3d[X], position_3d[Y]};

  Vector direction;
  minus(&direction, &(targets[current_target_index]), &position);
  const double distance = norm(&direction);
  normalize(&direction);

  const double robot_angle = atan2(north_3d[0], north_3d[1]);
  const double target_angle = atan2(direction.v, direction.u);
  double beta = modulus_double(target_angle - robot_angle + M_PI, 2 * M_PI) - M_PI;

  if (distance < DISTANCE_TOLERANCE) {
    printf("Reached target %d\n", current_target_index + 1);
    current_target_index = (current_target_index + 1) % TARGET_POINTS_SIZE;
  } else {
    const float *lidar_values = wb_lidar_get_range_image(lidar);
    const int lidar_width = wb_lidar_get_horizontal_resolution(lidar);
    const double lidar_max_range = wb_lidar_get_max_range(lidar);

    double *braitenberg_coefficients = malloc(sizeof(double) * lidar_width);
    for (int i = 0; i < lidar_width; i++)
      braitenberg_coefficients[i] = 25 * gaussian(i, lidar_width / 2.0, lidar_width / 8.0);

    double left_speed = MAX_SPEED;
    double right_speed = MAX_SPEED;

    for (int i = 0; i < lidar_width; i++) {
      if (lidar_values[i] < OBSTACLE_THRESHOLD && !isnan(lidar_values[i])) {
        double influence = 2.0 * (1.0 - lidar_values[i] / lidar_max_range);
        if (i < lidar_width / 2) {
          left_speed += braitenberg_coefficients[i] * influence;
          right_speed -= braitenberg_coefficients[i] * influence;
        } else {
          left_speed -= braitenberg_coefficients[i] * influence;
          right_speed += braitenberg_coefficients[i] * influence;
        }
      }
    }

    speeds[LEFT] = left_speed + TURN_COEFFICIENT * beta;
    speeds[RIGHT] = right_speed - TURN_COEFFICIENT * beta;

    double norm_factor = fmax(fabs(speeds[LEFT]), fabs(speeds[RIGHT]));
    if (norm_factor > MAX_SPEED) {
      speeds[LEFT] *= MAX_SPEED / norm_factor;
      speeds[RIGHT] *= MAX_SPEED / norm_factor;
    }

    free(braitenberg_coefficients);
  }

  double roll_velocity = (speeds[RIGHT] - speeds[LEFT]) * 0.2;
  double pitch_velocity = (speeds[LEFT] + speeds[RIGHT]) * 0.1;
  stabilize_camera(roll_velocity, pitch_velocity);
  robot_set_speed(speeds[LEFT], speeds[RIGHT]);

  // Log 3D map and path
  log_point_cloud_and_path();
}

static void process_camera() {
  if (camera == 0)
    return;

  int width = wb_camera_get_width(camera);
  int height = wb_camera_get_height(camera);
  const unsigned char *image = wb_camera_get_image(camera);

  if (image != NULL) {
    long red_sum = 0, green_sum = 0, blue_sum = 0;
    int center_x = width / 2, center_y = height / 2;
    int sample_size = 10;

    for (int y = center_y - sample_size/2; y < center_y + sample_size/2; y++) {
      for (int x = center_x - sample_size/2; x < center_x + sample_size/2; x++) {
        int r = wb_camera_image_get_red(image, width, x, y);
        int g = wb_camera_image_get_green(image, width, x, y);
        int b = wb_camera_image_get_blue(image, width, x, y);
        red_sum += r; green_sum += g; blue_sum += b;
      }
    }

    static int step_counter = 0;
    if (step_counter++ % 100 == 0) {
      double total = sample_size * sample_size;
      printf("Camera center RGB: (%.1f, %.1f, %.1f)\n", red_sum/total, green_sum/total, blue_sum/total);
    }
  }
}

int main() {
  wb_robot_init();
  printf("Robot initialized.\n");

  const char *names[2] = {"left wheel motor", "right wheel motor"};
  for (int i = 0; i < 2; i++) {
    motors[i] = wb_robot_get_device(names[i]);
    wb_motor_set_position(motors[i], INFINITY);
  }

  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);
  compass = wb_robot_get_device("compass");
  wb_compass_enable(compass, TIME_STEP);
  lidar = wb_robot_get_device("LDS-01");
  wb_lidar_enable(lidar, TIME_STEP);
  wb_lidar_enable_point_cloud(lidar);
  camera = wb_robot_get_device("Camera");
  if (camera) wb_camera_enable(camera, TIME_STEP);

  camera_roll_motor = wb_robot_get_device("camera roll");
  camera_pitch_motor = wb_robot_get_device("camera pitch");

  wb_keyboard_enable(TIME_STEP);

  while (wb_robot_step(TIME_STEP) != -1) {
    process_camera();
    check_keyboard();
    if (autopilot)
      run_autopilot();
  }

  wb_robot_cleanup();
  return 0;
}
