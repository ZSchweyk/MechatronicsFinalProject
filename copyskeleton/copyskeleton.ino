#include <Pixy2.h>
#include <DualMAX14870MotorShield.h>
#include <Servo.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <math.h>

Pixy2 pixy;
Servo wind_servo;
Servo release_servo;
DualMAX14870MotorShield motors;
Adafruit_BNO055 imu_board = Adafruit_BNO055(55);

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

const Coordinate GOAL_CENTER_COORD = {150, 500};
const int GOAL_WIDTH = 100; // cm

State currentState = SPIN_TO_FIND_PUCK;
unsigned long lastSeenTime = 0;
const unsigned long puckLostTimeout = 2000;

const int PUCK_SIGNATURE = 1;   // Orange
const int GOAL_SIGNATURE = 2;   // Green

const int minspeed = 130;

bool winded = false;

int lastPuckX = 90;  // assume center (Pixy2 image width ~ 180)
bool puckSeen = false;
const int closeEnoughY = 20;  // Adjust based on your camera setup

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  pixy.init();
  motors.flipM2(true);
  wind_servo.attach(9);
  wind_servo.write(0);
  release_servo.attach(10);
  release_servo.write(0);

  if (!imu_board.begin()) {
    Serial.println("Failed to initialize BNO055 IMU!");
    while (1);
  }
  delay(500);
  imu_board.setExtCrystalUse(true);
}

void loop() {
  motors.enableDrivers();
  pixy.ccc.getBlocks();  // Always get the latest blocks

  switch (currentState) {
    case SPIN_TO_FIND_PUCK:
      motors.setM1Speed(minspeed);
      motors.setM2Speed(-minspeed);

      if (index(PUCK_SIGNATURE) >= 0) {
        lastPuckX = pixy.ccc.blocks[index(PUCK_SIGNATURE)].m_x;
        puckSeen = true;
        stop();
        currentState = APPROACH_PUCK;
      }
      break;

    case APPROACH_PUCK:
      if (index(PUCK_SIGNATURE) >= 0) {
        lastPuckX = pixy.ccc.blocks[index(PUCK_SIGNATURE)].m_x;
        puckSeen = true;

        int puckX = pixy.ccc.blocks[index(PUCK_SIGNATURE)].m_x;
        int puckY = pixy.ccc.blocks[index(PUCK_SIGNATURE)].m_y;

        if (puckY <= closeEnoughY) {
          stop();
          currentState = FIND_GOAL;
          break;
        }

        int error = puckX - 90;  // center is ~90
        int turnSpeed = constrain(error * 0.5, -100, 100);

        motors.setM1Speed(-minspeed + turnSpeed);
        motors.setM2Speed(-minspeed - turnSpeed);

      } else if (puckSeen) {
        // Lost puck — turn back toward last seen x
        int error = lastPuckX - 90;
        int turnSpeed = constrain(error * 0.5, -100, 100);

        motors.setM1Speed(turnSpeed);
        motors.setM2Speed(-turnSpeed);
      } else {
        // No memory, spin to search again
        currentState = SPIN_TO_FIND_PUCK;
      }
      break;

    case FIND_GOAL:
      // Logic for goal-seeking (e.g., align and shoot)
      break;

    case SHOOT: {
      Coordinate current = getZigBeeCoords();
      double shoot_angle = calcShootAngleToGoal(current, GOAL_CENTER_COORD, GOAL_WIDTH, 0);
      aimToAngle(shoot_angle);
      shootPuck();
      currentState = SPIN_TO_FIND_PUCK;
      break;
    }
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

void wind() {
  release_servo.write(90);
  delay(500);
  wind_servo.write(180);
  delay(500);
  release_servo.write(180);
  delay(500);
  wind_servo.write(0);
  winded = true;
}

void shootPuck() {
  Serial.println("Shooting!");
  if (winded) {
    release_servo.write(90);
    winded = false;
  }
}

double radToDeg(double rad) {
  return rad * 180.0 / PI;
}

double readYaw() {
  imu::Vector<3> euler = imu_board.getVector(Adafruit_BNO055::VECTOR_EULER);
  double yaw = euler.x();
  return yaw;
}

void aimToAngle(double target_angle) {
  double kp = 1.2;
  double ki = 0.0;
  double kd = 0.3;
  double threshold = 3.0;
  double prev_error = 0;
  double integral = 0;
  long dt_ms = 20;
  double dt_s = dt_ms / 1000.0;

  while (true) {
    double current_yaw = readYaw();
    double error = target_angle - current_yaw;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    if (abs(error) < threshold) break;

    integral += error * dt_s;
    double derivative = (error - prev_error) / dt_s;
    double ctrl = kp * error + ki * integral + kd * derivative;
    int max_speed = 150;
    ctrl = constrain(ctrl, -max_speed, max_speed);

    motors.setM1Speed(-ctrl);
    motors.setM2Speed(ctrl);
    prev_error = error;
    delay(dt_ms);
  }
  stop();
}

double calcShootAngleToGoal(Coordinate current, Coordinate goal, int goal_width, double offset_pct) {
  if (offset_pct < -1.0) offset_pct = -1.0;
  if (offset_pct > 1.0) offset_pct = 1.0;

  double xOffset = (goal_width / 2.0) * offset_pct;
  double dx = (goal.x + xOffset) - current.x;
  double dy = goal.y - current.y;

  double angleRad = atan2(dx, dy);
  double angleDeg = radToDeg(angleRad);
  if (angleDeg < 0) angleDeg += 360;
  return angleDeg;
}

Coordinate getZigBeeCoords() {
  Serial1.print('?');
  String data = "";
  int count = 0;
  Coordinate coord = {0, 0};

  while (Serial1.available() && count < 14) {
    char incoming = Serial1.read();
    data += incoming;
    count++;
    delay(20);
  }

  if (count == 14) {
    String x = data.substring(7,10);
    String y = data.substring(11);
    coord.x = x.toInt();
    coord.y = y.toInt();
  }
  return coord;
}
