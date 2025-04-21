#include <Pixy2.h>
#include <DualMAX14870MotorShield.h>
#include <Servo.h>
#include <Adafruit_BNO055.h>
#include <math.h>

Pixy2 pixy;
DualMAX14870MotorShield motors;
Servo shooter;
Adafruit_BNO055 imu_board = Adafruit_BNO055(55); //IMU setup

struct Coordinate {
  int x; // cm
  int y; // cm
};

const struct Coordinate GOAL_CENTER_COORD = {150, 500}; // TODO: I'm pretty sure the zigbee returns x-y coordinates in cm. Fill these in appropriately
const int GOAL_WIDTH = 100; // in cm - TODO: Change this accordingly


void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  pixy.init();
  motors.enableDrivers();
  // motors.flipM2(true);
  shooter.attach(9); // Attach servo to pin 9
  //for IMU
  Serial.println("Setup begin");
  if (!imu_board.begin()) {
    Serial.print("oops");
    while(1);
  }
  delay(500);
  imu_board.setExtCrystalUse(true);
  Serial.println("Setup done");
}

void loop() {
  drive_straight();
  Serial.println("---");
  double yaw = readYaw();
  Serial.println(yaw);

  double shoot_angle = calcShootAngleToGoal({0, 0}, {10, 10}, 2, 0);
  Serial.println(shoot_angle);


}

void drive_straight() {
  motors.setM1Speed(125);
  motors.setM2Speed(125);
}

// Converts radians to degrees
double radToDeg(double rad) {
  return rad * 180.0 / PI;
}

// current - the current x,y coordinate of the robot
// goal - this will be GOAL_CENTER_COORD
// goal_width - this will be GOAL_WIDTH (the center x,y coord for the goal)
// offset_pct - 0 if aiming to center of goal, +1 if aiming towards right post, -1 if aiming towards left post, any value in between for more granular control
double calcShootAngleToGoal(Coordinate current, Coordinate goal, int goal_width, double offset_pct) {
  // Clamp offset to range [-1, 1] just in case
  if (offset_pct < -1.0) offset_pct = -1.0;
  if (offset_pct > 1.0) offset_pct = 1.0;

  // Compute x-offset within the goal
  double xOffset = (goal_width / 2.0) * offset_pct;

  // Compute direction vector to the desired target point on the goal
  double dx = (goal.x + xOffset) - current.x;
  double dy = goal.y - current.y;

  // Angle off y-axis (in radians), then converted to degrees
  double angleRad = atan2(dx, dy);
  return radToDeg(angleRad);
}

void aim(double shoot_angle) {
  // pivot cw or ccw until readYaw() is shoot_angle
  // this should be easy, just send commands to both motors to rotate
  // something like this...
  // while (readYaw() != shoot_angle) {
  //    // rotate cw/ccw
  // }
}

// reads yaw
double readYaw() {
  /* Get a new sensor event */ 
  sensors_event_t event; 
  imu_board.getEvent(&event);
  double yaw = event.orientation.x; // consider subtracting setpoint
  return yaw;
}
