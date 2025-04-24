#include <Pixy2.h>
#include <DualMAX14870MotorShield.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define LED_PIN 13

DualMAX14870MotorShield motors;
Pixy2 pixy;
String color;

enum State { MOVE_FORWARD,
             READ_COLOR,
            // TURNING,
};

State currentState = MOVE_FORWARD;

int signature;
int dist_threshold = 10;
i6nt x_cord;

int signal = 13;
float distance;
float dist;
unsigned long pulseduration = 0;

// PID constants
double Kp = .5;  // Proportional gain
double Kd = 0.1;  // Derivative gain
double Ki = 0.1;  //Integral gain

// Target position
double setpoint;

// Variables for PID control
double input, output;
double integral = 0;
double derivative;
double previous_error = 0;
double ang;

// Refresh rate
unsigned long refreshRate = 100;  // Refresh rate in milliseconds
unsigned long lastTime = 0;

Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup() {
  Serial.begin(115200);
  Serial.print("Starting...\n");
  motors.flipM2(true);
  motors.flipM1(true);
  pixy.init();

  //initilaize LED for debugg
  pinMode(A8, OUTPUT);
  pinMode(A9, OUTPUT);
  pinMode(A10, OUTPUT);
  pinMode(A11, OUTPUT);

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
   // while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);
}

void loop() {
  switch (currentState) {
    
    case READ_COLOR:
      analogWrite(A9, 255);
      Serial.println("scanning");
      getColorFromSensor();
      if (color == "") {
        Serial.println("no color detected");
       
        motors.setM1Speed(100);
        motors.setM2Speed(-100);
        delay(200);
        //currentState = MOVE_FORWARD;
        break;
      } else {
        Serial.print("got color: ");
        Serial.println(color);
        currentState = MOVE_FORWARD;
        analogWrite(A9, 0);
        break;
      }
      break;
    case MOVE_FORWARD:
      analogWrite(A8, 255);
      dist = measureDistance();
      Serial.println(dist);
      delay(500);
      //while (dist > dist_threshold) {
        while(1){
          Serial.print("moving - ");
          Serial.println(dist);
          moveForward();
          delay(500);

          pixy.ccc.getBlocks();
          x_cord = pixy.ccc.blocks[0].m_x;

          if (x_cord > 158 + 30) {
            turnRight();
            delay(100);
          } else if (x_cord < 158 - 30) {
            turnLeft();
            delay(100);
          }
       // dist = measureDistance();
      }
      stopMotors();
      Serial.println("stopped");
      currentState = MOVE_FORWARD;
      analogWrite(A8, 0);
      break;

    // case TURNING:
    //   analogWrite(A10, 255);
    //   if (color == "GREEN") {
    //     left90();
    //     Serial.println("done turning");
    //     currentState = MOVE_FORWARD;
    //     motors.disableDrivers();
    //     analogWrite(A10, 0);
    //     break;
    //   }
    //   if (color == "RED") {
    //     right90();
    //     Serial.println("done turning");
    //     currentState = MOVE_FORWARD;
    //     analogWrite(A10, 0);
    //     motors.disableDrivers();
    //     break;
    //   }
    //   if (color == "BLUE") {
    //     uturn();
    //     currentState = MOVE_FORWARD;
    //     analogWrite(A10, 0);
    //     motors.disableDrivers();
    //     break;
    //   }
    //   currentState = MOVE_FORWARD;
    //   delay(1000);
    //   analogWrite(A11, 0);
    //   break;
  }
}

void getColorFromSensor() {
  color = "";
  int i = 0;
  // grab blocks!
  pixy.ccc.getBlocks();

  signature = pixy.ccc.blocks[i].m_signature;

  //Serial.println(signature);
  if (signature == 1) {
    Serial.println("set to ORANGE");
    color = "ORANGE";
  }
  // if (signature == 3) {
  //   Serial.println("set to BLUE");
  //   color = "BLUE";
  // }
  // if (signature == 2) {
  //   Serial.println("set to RED");
  //   color = "RED";
  // }
}

void moveForward() {
  motors.enableDrivers();
  motors.setM1Speed(-100);
  motors.setM2Speed(-100);
}

void stopMotors() {
  motors.setM1Speed(0);
  motors.setM2Speed(0);
}

float measureDistance() {
  // set pin as output so we can send a pulse
  pinMode(signal, OUTPUT);
  // set output to LOW
  digitalWrite(signal, LOW);
  delayMicroseconds(5);

  // now send the 5uS pulse out to activate Ping)))
  digitalWrite(signal, HIGH);
  delayMicroseconds(5);
  digitalWrite(signal, LOW);

  // now we need to change the digital pin
  // to input to read the incoming pulse
  pinMode(signal, INPUT);

  // finally, measure the length of the incoming pulse
  pulseduration = pulseIn(signal, HIGH);
  distance = (343.0 * pulseduration) / 20000;
  return distance;
}

void turnLeft() {
  motors.enableDrivers();
  motors.setM1Speed(150);
  motors.setM2Speed(-150);
}

void turnRight() {
  motors.enableDrivers();
  motors.setM1Speed(-150);
  motors.setM2Speed(150);
}
double readPosition() {
  sensors_event_t event;
  bno.getEvent(&event);
  ang = event.orientation.x;
  if (color != 'BLUE') {
    if (ang > 180) {
      ang -= 360;
    }
  }
  Serial.print("ang: ");
  Serial.println(ang);
  return ang;  // Return the actual sensor value
}

void applyOutput(double output) {
  //Serial.print("output: ");
  //Serial.println(output);
  if (output < 0) {
    turnLeft();
    delay(1 + (-1 * output));
    stopMotors();
    delay(250);
  }
  if (output > 0) {
    turnRight();
    delay(1 + output);
    stopMotors();
    delay(250);
  }
}

void right90() {
  delay(1000);
  bno.begin();
  delay(500);
  setpoint = 90;
/*
  ang = readPosition();
  while (ang <= 90) {
    turnRight();
    delay(250);
    ang = readPosition();
  }
*/
  output = 51;
  while (abs(output) > 30) {
    unsigned long now = millis();
    if (now - lastTime >= refreshRate) {
      lastTime = now;

      // Read current position
      input = readPosition();  // Placeholder for position reading function

      // Calculate PID
      double error = setpoint - input;
      integral += error * (refreshRate / 1000.0);
      derivative = (error - previous_error) / (refreshRate / 1000.0);
      output = Kp * error + Ki * integral + Kd * derivative;
      previous_error = error;
      Serial.println(output);

      // Apply output to your system
      applyOutput(output);  // Placeholder for output application function
    }
  }

  motors.setM1Speed(0);
  motors.setM2Speed(0);
  motors.disableDrivers();
  delay(1000);
}

void left90() {
  delay(2000);
  bno.begin();
  setpoint = -90;
/*
  ang = readPosition();
  while (ang >= -90) {
    turnLeft();
    delay(250);
    ang = readPosition();
  }
  */
  output = 51;
  while (abs(output) > 50) {
    unsigned long now = millis();
    if (now - lastTime >= refreshRate) {
      lastTime = now;

      // Read current position
      input = readPosition();  // Placeholder for position reading function

      // Calculate PID
      double error = setpoint - input;
      integral += error * (refreshRate / 1000.0);
      derivative = (error - previous_error) / (refreshRate / 1000.0);
      output = Kp * error + Ki * integral + Kd * derivative;
      previous_error = error;
      Serial.print("output: ");
      Serial.println(output);

      // Apply output to your system
      applyOutput(output);  // Placeholder for output application function
    }
  }
  motors.setM1Speed(0);
  motors.setM2Speed(0);
  motors.disableDrivers();
  delay(1000);
}

void uturn() {
  delay(1000);
  bno.begin();
  delay(500);
  setpoint = 180;
/*
  ang = readPosition();
  while (ang <= 180) {
    turnRight();
    delay(250);
    ang = readPosition();
  }

*/
  output = 51;
  while (abs(output) > 50) {
    unsigned long now = millis();
    if (now - lastTime >= refreshRate) {
      lastTime = now;

      // Read current position
      input = readPosition();  // Placeholder for position reading function

      // Calculate PID
      double error = setpoint - input;
      integral += error * (refreshRate / 1000.0);
      derivative = (error - previous_error) / (refreshRate / 1000.0);
      output = Kp * error + Ki * integral + Kd * derivative;
      previous_error = error;
      Serial.println(output);

      // Apply output to your system
      applyOutput(output);  // Placeholder for output application function
    }
    motors.setM1Speed(0);
    motors.setM2Speed(0);
    motors.disableDrivers();
  }
}