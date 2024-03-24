#define m1 4  //Right Motor MA1
#define m2 5  //Right Motor MA2
#define m3 2  //Left Motor MB1
#define m4 3  //Left Motor MB2
#define e1 9  //Right Motor Enable Pin EA
#define e2 10 //Left Motor Enable Pin EB

//**********5 Channel IR Sensor Connection**********//
#define ir1 A0  // Left Most Sensor
#define ir2 A1
#define ir3 A2  // Middle
#define ir4 A3
#define ir5 A4  // Right Most Sensor
//*************************************************//
int junctionDetected = 0;
unsigned long junctionDetectedTime = 0;
unsigned long junctionTimeInterval = 200; // Adjust as needed

int motorSpeed1 = 255; // Default motor speed
int motorSpeed2 = 255; // Default motor speed

void setup() {
  pinMode(m1, OUTPUT);
  pinMode(m2, OUTPUT);
  pinMode(m3, OUTPUT);
  pinMode(m4, OUTPUT);
  pinMode(e1, OUTPUT);
  pinMode(e2, OUTPUT);
  pinMode(ir1, INPUT);
  pinMode(ir2, INPUT);
  pinMode(ir3, INPUT);
  pinMode(ir4, INPUT);
  pinMode(ir5, INPUT);
}

void moveForward() {
  analogWrite(e1, motorSpeed1); //you can adjust the speed of the motors from 0-255
  analogWrite(e2, motorSpeed2); //you can adjust the speed of the motors from 0-255
  digitalWrite(m1, HIGH);
  digitalWrite(m2, LOW);
  digitalWrite(m3, HIGH);
  digitalWrite(m4, LOW);
}

void moveRight() {
  analogWrite(e1, motorSpeed1); //you can adjust the speed of the motors from 0-255
  analogWrite(e2, motorSpeed2); //you can adjust the speed of the motors from 0-255
  digitalWrite(m1, HIGH);
  digitalWrite(m2, LOW);
  digitalWrite(m3, LOW);
  digitalWrite(m4, HIGH);
}

void moveLeft() {
  analogWrite(e1, motorSpeed1); //you can adjust the speed of the motors from 0-255
  analogWrite(e2, motorSpeed2); //you can adjust the speed of the motors from 0-255
  digitalWrite(m1, LOW);
  digitalWrite(m2, HIGH);
  digitalWrite(m3, HIGH);
  digitalWrite(m4, LOW);
}

void stopMotors() {
  digitalWrite(m1, LOW);
  digitalWrite(m2, LOW);
  digitalWrite(m3, LOW);
  digitalWrite(m4, LOW);
}

void followLine() {
  // Reading Sensor Values
  int s1 = digitalRead(ir1);  // Left Most Sensor
  int s2 = digitalRead(ir2);  // Left Sensor
  int s3 = digitalRead(ir3);  // Middle Sensor
  int s4 = digitalRead(ir4);  // Right Sensor
  int s5 = digitalRead(ir5);  // Right Most Sensor

  // Determining movement based on sensor readings
  if ((s1 == 1) && (s2 == 1) && (s3 == 0) && (s4 == 1) && (s5 == 1)) {
    // Forward
    moveForward();
  } else if ((s1 == 1) && (s2 == 0) && (s3 == 1) && (s4 == 1) && (s5 == 1)) {
    // Right
    moveRight();
  } else if ((s1 == 0) && (s2 == 1) && (s3 == 1) && (s4 == 1) && (s5 == 1)) {
    // Right
    moveRight();
  } else if ((s1 == 1) && (s2 == 1) && (s3 == 1) && (s4 == 0) && (s5 == 1)) {
    // Left
    moveLeft();
  } else if ((s1 == 1) && (s2 == 1) && (s3 == 1) && (s4 == 1) && (s5 == 0)) {
    // Left
    moveLeft();
  } else if ((s1 == 1) && (s2 == 1) && (s3 == 0) && (s4 == 0) && (s5 == 1)) {
    // Left
    moveLeft();
  } else if ((s1 == 1) && (s2 == 0) && (s3 == 0) && (s4 == 1) && (s5 == 1)) {
    // Right
    moveRight();
  } else if ((s1 == 0) && (s2 == 0) && (s3 == 0) && (s4 == 1) && (s5 == 1)) {
    // Right
    moveRight();
  } else if ((s1 == 1) && (s2 == 1) && (s3 == 0) && (s4 == 0) && (s5 == 0)) {
    // Left
    moveLeft();
  } else if ((s1 == 0) && (s2 == 0) && (s3 == 0) && (s4 == 0) && (s5 == 0)) {
    // Forward
    moveForward();
  } else {
    // Stop
    moveForward();
  }

// Checking if all sensors detect black and the previous state was not all black
  //if ((s1 == 0) && (s2 == 0) && (s3 == 0) && (s4 == 0) && (s5 == 0) && (previousState != 0)) {
    //junctionDetected++; // Increment junction count
  //}
//  previousState = (s1 == 0 && s2 == 0 && s3 == 0 && s4 == 0 && s5 == 0); // Update previous state
  
    // Check for junction detection based on time interval
  if ((millis() - junctionDetectedTime) > junctionTimeInterval) {
    junctionDetected++; // Increment junction count
    junctionDetectedTime = millis(); // Reset junction detection timer
  }
}

void loop() {
  if (junctionDetected < 3) {
    followLine();
  } else {
    stopMotors();
    // Perform additional movements after detecting three junctions
    for (int i = 0; i < 10; i++) {
      moveForward();
    }
    stopMotors();
    for (int i = 0; i < 10; i++) {
      moveRight();
    }
    stopMotors();
    for (int i = 0; i < 50; i++) {
      moveForward();
    }
  }
}
