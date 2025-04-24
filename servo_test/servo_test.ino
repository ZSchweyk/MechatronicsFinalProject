/* #include <Servo.h>

Servo servo;
const int angleIncrement = 10;  // Degrees to move per space press
int currentAngle = 0;

void setup() {
  // Serial.begin(9600);
  servo.attach(9);  // attaches the servo on pin 9 to the servo object
  servo.write(currentAngle);  // Set initial position
}

// void loop() {
//   if (Serial.available() > 0) {
//     char incomingChar = Serial.read();
    
//     if (incomingChar == ' ') {  // Check if space bar was pressed
//       currentAngle += angleIncrement;  // Increase angle by 10 degrees
      
//       // Keep angle within valid range (0-180)
//       if (currentAngle > 180) {
//         currentAngle = 180;
//         Serial.println("Maximum angle reached (180°)");
//       }
      
//       servo.write(currentAngle);  // Move servo to new position
      
//       Serial.print("Servo moved to: ");
//       Serial.print(currentAngle);
//       Serial.println("°");
//     }
//   }
// }

void loop() {
  servo.write(0);
  delay(2000);
  servo.write(180);
  delay(2000);
  servo.detach();  // This releases the servo motor
  while (1) {}

  // servo.write(0);
  // for (int angle = 10; angle <= 180; angle += 10) {
  //   servo.write(angle);
  //   delay(250);
  // }
  
  
} */

















#include <Servo.h>

Servo myServo;
int currentAngle = 90;
const int servoPin = 9;

void setup() {
  Serial.begin(9600);
  myServo.attach(servoPin);
  myServo.write(currentAngle);
  delay(2000);
  Serial.println("Space: +10° | r: release torque | a: reattach");
  myServo.write(90);
  delay(2000);
  myServo.write(0);
  delay(2000);
  myServo.write(currentAngle);
  pinMode(13, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    char incomingChar = Serial.read();
    Serial.print("Character is "); Serial.println(incomingChar);
    
    if (incomingChar == ' ') {
      currentAngle += 10;
      if (currentAngle > 180) currentAngle = 180;
      myServo.write(currentAngle);
      delay(2000);
      Serial.print("Position: ");
      Serial.print(currentAngle);
      Serial.println("°");
    }
    else if (incomingChar == 'r') {
      myServo.detach();
      delay(1000); // Allow time for the servo to free-wheel
      digitalWrite(servoPin, LOW);  // After detaching
      Serial.println("Torque released - servo moves freely");
    }
    else if (incomingChar == 'a') {
      myServo.attach(servoPin);
      Serial.println("Servo reattached");
    } else if (incomingChar == 'h') {
      digitalWrite(13, HIGH);
      Serial.println("Gate activated");
    } else if (incomingChar == 'l') {
      digitalWrite(13, LOW);
      Serial.println("Gate deactivated");
    }
  }
}









