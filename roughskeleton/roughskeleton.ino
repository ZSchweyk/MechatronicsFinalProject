#include <Pixy2.h>
#include <DualMAX14870MotorShield.h>
#include <Servo.h>
#include <Adafruit_BNO055.h>
#include <math.h>

Pixy2 pixy;
Servo wind_servo;
Servo release_servo;
DualMAX14870MotorShield motors;
//Adafruit_BNO055 imu_board = Adafruit_BNO055(55); //IMU setup

enum State {
  SPIN_TO_FIND_PUCK,
  AIM_PUCK,
  APPROACH_PUCK,
  FIND_GOAL,
  SHOOT
};

struct Coordinate {
  int x; // cm
  int y; // cm
};

const Coordinate GOAL_CENTER_COORD = {150, 500}; // TODO: I'm pretty sure the zigbee returns x-y coordinates in cm. Fill these in appropriately
const int GOAL_WIDTH = 100; // in cm - TODO: Change this accordingly

State currentState = SPIN_TO_FIND_PUCK;
unsigned long lastSeenTime = 0;
const unsigned long puckLostTimeout = 2000;

const int PUCK_SIGNATURE = 1;   // Orange
const int GOAL_SIGNATURE = 2;   // Green

const int minspeed = 100;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  pixy.init();
  motors.flipM2(true);
  // wind_servo.attach(9);
  // release_servo.attach(10);
  // //for IMU
  // if (!imu.begin()) {
  //   Serial.print("oops");
  //   while(1);
  // }
  // delay(500);
  // imu.setExtCrystalUse(true);
}

void loop() {
  motors.enableDrivers();
  //pixy.ccc.getBlocks();

  switch (currentState) {
    case SPIN_TO_FIND_PUCK:
      motors.setM1Speed(minspeed);
      motors.setM2Speed(-minspeed);
      pixy.ccc.getBlocks();
      if (index(PUCK_SIGNATURE) >= 0) {
        stop();
        currentState = APPROACH_PUCK;
      }
      break;

    case AIM_PUCK:
      // if (index(&pixy, PUCK_SIGNATURE) >= 0) {
      //   lastSeenTime = millis();
      //   trackPuck(pixy.ccc.blocks[index(&pixy, PUCK_SIGNATURE)].m_x);
      // } else if (millis() - lastSeenTime > puckLostTimeout) {
      //   currentState = SPIN_TO_FIND_PUCK;
      // }
      spinTowardsPuck();
      pixy.ccc.getBlocks();
      if (index(PUCK_SIGNATURE) >= 0) {
        currentState = FIND_GOAL;
      } else {
        currentState = SPIN_TO_FIND_PUCK;
      }

      break;
    case APPROACH_PUCK:
      
    case FIND_GOAL:

      // spin();
      // if (pixy.ccc.numBlocks > 0 && pixy.ccc.blocks[0].m_signature == GOAL_SIGNATURE) {
      //   moveForwardBriefly();
      //   // You can later compare pixy.ccc.blocks[0].m_x with goal center
      //   currentState = SHOOT;
      // }
      break;

    case SHOOT:
      shootPuck();
      currentState = SPIN_TO_FIND_PUCK;  // Reset
      break;
  }
  motors.disableDrivers();
}

int index(int colorindex) {
  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    if (pixy.ccc.blocks[i].m_signature == colorindex) {
      return i;
    }
  }
  return -1;
}

void spin() {
  motors.enableDrivers();
  motors.setM1Speed(minspeed);
  motors.setM2Speed(-minspeed);
}

void stop() {
  motors.setM1Speed(0);
  motors.setM2Speed(0);
}



void moveForwardBriefly() {
  motors.setM1Speed(-100);
  motors.setM2Speed(-100);
  delay(500);
  stop();
}

void spinTowardsPuck() {

  pixy.ccc.getBlocks();

  int target = 145;

  double thresh = 20;

  double kp = 1.0;
  double ki = 0;
  double kd = 0;

  double integral = 0;

  double deriv = 0;

  if (index(PUCK_SIGNATURE) < 0) {
    stop();
    return;
  }
  double error = pixy.ccc.blocks[index(PUCK_SIGNATURE)].m_x - target;

  long dt = 15; //ms

  while (index(PUCK_SIGNATURE) >= 0 && abs(error) > thresh) {
    Serial.println(error);
    double newerror = pixy.ccc.blocks[index(PUCK_SIGNATURE)].m_x - target;
    integral += dt * newerror;
    deriv = (newerror - error) / dt;
    double ctrlsig = kp * newerror + ki * integral - kd * deriv;
    motors.setM1Speed(ctrlsig);
    motors.setM2Speed(-1 * ctrlsig);
    error = newerror;
    delay(dt);
    pixy.ccc.getBlocks();
  }
  stop();
}

void trackPuck(int x) {
  int center = 158;
  int tolerance = 20;

  if (x < center - tolerance) {
    motors.setM1Speed(-minspeed);
    motors.setM2Speed(minspeed);  // Turn left
  } else if (x > center + tolerance) {
    motors.setM1Speed(minspeed);
    motors.setM2Speed(-minspeed);   // Turn right
  } else {
    stop();
    // motors.setM1Speed(minspeed);
    // motors.setM2Speed(minspeed); // Move forward
  }

}

void shootPuck() {
  Serial.println("Shooting!");
  // shooter.write(0);   // Adjust angle for shooting
  delay(500);
  // shooter.write(90);  // Reset
  delay(500);
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

// void aim(double shoot_angle) {

//   int speed = 100;

//   int sign = (readYaw() - shoot_angle > 0) ? 1 : -1;
//   motors.setM1Speed(-1 * sign * speed);
//   motors.setM2Speed(sign * speed);
//   while (abs(readYaw() - shoot_angle) > 5) {}
//   motors.setM1Speed(0);
//   motors.setM2Speed(0);

// }

// void aimpid(double shoot_angle) {
//   // pivot cw or ccw until readYaw() is shoot_angle
//   // this should be easy, just send commands to both motors to rotate
//   // something like this...
//   // while (readYaw() != shoot_angle) {
//   //    // rotate cw/ccw
//   // }

//   double thresh = 10;
//   double kp = 1.0;
//   double ki = .5;
//   double kd = .25;

//   double integral = 0;

//   double deriv = 0;

//   double error = readYaw() - shoot_angle;

//   long dt = 15; //ms

//   while (abs(error) > thresh) {
//     double newerror = readYaw() - shoot_angle;
//     integral += dt * newerror;
//     deriv = (newerror - error) / dt;
//     double ctrlsig = kp * newerror + ki * integral - kd * deriv;
//     motors.setM1Speed(-1 * ctrlsig);
//     motors.setM2Speed(ctrlsig);
//     error = newerror;
//     delay(dt);
//   }

// }

// reads yaw
// double readYaw() {
//   /* Get a new sensor event */ 
//   sensors_event_t event; 
//   imu_board.getEvent(&event);
//   double yaw = event.orientation.x; // consider subtracting setpoint
//   return yaw <= 179 ? yaw : yaw - 360;
// }

//ZigBee check coordinates 
Coordinate getZigBeeCoords(int* match_byte_int){
  // Send data from the serial monitor to Xbee module
  Serial1.print('?');
  String data = "";
  int count = 0;
  if(!Serial1.available()){
    Serial.println("error ZigBee");
  }
  // Receive data from the Xbee module one char at a time and print to serial monitor
  while (Serial1.available() && count < 14)
  {
    char incoming = Serial1.read();
    data += incoming;
    count++;
    //Serial.print(incoming);
    delay(20);
  }
  if(count == 14) { // this is a dumb redundancy check
    String x = data.substring(7,10);
    // Serial.println("X: " + x);
    String y = data.substring(11);
    // Serial.println("Y: " + y);
    String match_byte = data.substring(0, 1);
    // Serial.println("Match Byte: " + match_byte);

    *match_byte_int = match_byte.toInt();
    Coordinate coord = {x.toInt(), y.toInt()};
  }
}