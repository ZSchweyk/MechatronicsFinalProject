#include <Servo.h>

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

  // servo.write(0);
  // for (int angle = 10; angle <= 180; angle += 10) {
  //   servo.write(angle);
  //   delay(250);
  // }
  
  
}