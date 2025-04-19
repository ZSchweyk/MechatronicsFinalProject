#include <Pixy2.h>
#include <DualMAX14870MotorShield.h>
#include <Servo.h>
#include <Adafruit_BNO055.h>

Pixy2 pixy;
DualMAX14870MotorShield motors;
Servo shooter;
Adafruit_BNO055 imu = Adafruit_BNO055(55); //IMU setup

enum State {
  SPIN_TO_FIND_PUCK,
  APPROACH_PUCK,
  FIND_GOAL,
  SHOOT
};

State currentState = SPIN_TO_FIND_PUCK;
unsigned long lastSeenTime = 0;
const unsigned long puckLostTimeout = 2000;

const int PUCK_SIGNATURE = 1;   // Orange
const int GOAL_SIGNATURE = 2;   // Green

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  pixy.init();
  motors.flipM2(true);
  shooter.attach(9); // Attach servo to pin 9
  //for IMU
  if (!imu.begin()) {
    Serial.print("oops");
    while(1);
  }
  delay(500);
  imu.setExtCrystalUse(true);
}

void loop() {
  pixy.ccc.getBlocks();

  switch (currentState) {
    case SPIN_TO_FIND_PUCK:
      spin();
      if (pixy.ccc.numBlocks > 0 && pixy.ccc.blocks[0].m_signature == PUCK_SIGNATURE) {
        lastSeenTime = millis();
        currentState = APPROACH_PUCK;
      }
      break;

    case APPROACH_PUCK:
      if (pixy.ccc.numBlocks > 0 && pixy.ccc.blocks[0].m_signature == PUCK_SIGNATURE) {
        lastSeenTime = millis();
        trackPuck(pixy.ccc.blocks[0].m_x);
      } else if (millis() - lastSeenTime > puckLostTimeout) {
        currentState = SPIN_TO_FIND_PUCK;
      }
      break;

    case FIND_GOAL:
      spin();
      if (pixy.ccc.numBlocks > 0 && pixy.ccc.blocks[0].m_signature == GOAL_SIGNATURE) {
        moveForwardBriefly();
        // You can later compare pixy.ccc.blocks[0].m_x with goal center
        currentState = SHOOT;
      }
      break;

    case SHOOT:
      shootPuck();
      currentState = SPIN_TO_FIND_PUCK;  // Reset
      break;
  }
}

void spin() {
  motors.enableDrivers();
  motors.setM1Speed(100);
  motors.setM2Speed(-100);
}

void moveForwardBriefly() {
  motors.setM1Speed(-100);
  motors.setM2Speed(-100);
  delay(500);
  motors.setM1Speed(0);
  motors.setM2Speed(0);
}

void trackPuck(int x) {
  int center = 158;
  int tolerance = 20;

  if (x < center - tolerance) {
    motors.setM1Speed(70);
    motors.setM2Speed(-70);  // Turn left
  } else if (x > center + tolerance) {
    motors.setM1Speed(-70);
    motors.setM2Speed(70);   // Turn right
  } else {
    motors.setM1Speed(-100);
    motors.setM2Speed(-100); // Move forward
  }
}

void shootPuck() {
  Serial.println("Shooting!");
  shooter.write(0);   // Adjust angle for shooting
  delay(500);
  shooter.write(90);  // Reset
  delay(500);
}

// reads yaw
double readYaw() {
  /* Get a new sensor event */ 
  sensors_event_t event; 
  imu.getEvent(&event);
  double yaw = event.orientation.x; // consider subtracting setpoint
  return yaw;
}

//ZigBee check coordinates 
void getZigBeeCoords(int* x_int, int* y_int, int* match_byte_int){
  // Send data from the serial monitor to Xbee module
  Serial1.print('?');
  String data = "";
  int count = 0;
  if(!Serial1.available()){
    Serial.println("error ZigBee");
  }
  // Receive data from the Xbee module and print to serial monitor
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
    *x_int = x.toInt();
    //Serial.println("X: " + x);
    String y = data.substring(11);
    *y_int = y.toInt();
    //Serial.println("Y: " + y);
    String match_byte = data.substring(0, 1);
    *match_byte_int = match_byte.toInt();
  }
}