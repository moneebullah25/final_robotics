int sMostRight = 12;

// irSensorPins[0] - Left
// irSensorPins[1] - Middle
// irSensorPins[2] - Right
int irSensorPins[3] = {
    26,
    27,
    14
};

int sMostLeft = 25;

int motorR_EN = 4;
int motorL_EN = 19;
int motorRA = 16;
int motorRB = 17;
int motorLA = 5;
int motorLB = 18;

int value1 = 100;
int value2 = 100;

int junctionDetected = 0;
unsigned long junctionDetectedTime = 0;
unsigned long junctionTimeInterval = 500;

void setup() {
    pinMode(sMostRight, INPUT);
    pinMode(sMostLeft, INPUT);
    for (int i = 0; i < 3; i++) {
        pinMode(irSensorPins[i], INPUT);
    }

    pinMode(motorR_EN, OUTPUT);
    pinMode(motorL_EN, OUTPUT);
    pinMode(motorRA, OUTPUT);
    pinMode(motorRB, OUTPUT);
    pinMode(motorLA, OUTPUT);
    pinMode(motorLB, OUTPUT);
    Serial.begin(9600);
}

void loop() {
    if (junctionDetected >= 3) {
        Stop();
        delay(3000);
        Forward();
        delay(2000);
        Stop();
        delay(1000);
        Right();
        delay(2000);
        Stop();
        delay(1000);
        Forward();
        delay(5000);
    } else {
        FollowLine();
    }

}

void Forward() {
    // Serial.println("F");
    analogWrite(motorR_EN, value1); // Enable pin for motorR to control speed
    digitalWrite(motorRA, HIGH);
    digitalWrite(motorRB, LOW);
    analogWrite(motorL_EN, value2 / 1.5); // Enable pin for motorL to control speed
    digitalWrite(motorLA, HIGH);
    digitalWrite(motorLB, LOW);
}
void Left() {
    Serial.println("L");
    analogWrite(motorR_EN, value1); // Enable pin for motorR to control speed
    digitalWrite(motorRA, HIGH);
    digitalWrite(motorRB, LOW);
    analogWrite(motorL_EN, value2 / 1.5); // Enable pin for motorL to control speed
    digitalWrite(motorLA, LOW);
    digitalWrite(motorLB, LOW);
}
void Right() {
    Serial.println("R");
    analogWrite(motorR_EN, value1); // Enable pin for motorR to control speed
    digitalWrite(motorRA, LOW);
    digitalWrite(motorRB, LOW);
    analogWrite(motorL_EN, value2 / 1.5); // Enable pin for motorL to control speed
    digitalWrite(motorLA, HIGH);
    digitalWrite(motorLB, LOW);
}
void Stop() {
    Serial.println("S");
    analogWrite(motorR_EN, value1); // Enable pin for motorR to control speed
    digitalWrite(motorRA, LOW);
    digitalWrite(motorRB, LOW);
    analogWrite(motorL_EN, value2 / 1.5); // Enable pin for motorL to control speed
    digitalWrite(motorLA, LOW);
    digitalWrite(motorLB, LOW);
}

void FollowLine() {
    int irSensorValues[3];

    for (int i = 0; i < 3; i++) {
        irSensorValues[i] = digitalRead(irSensorPins[i]);
        if (irSensorValues[i] == 0) {
            irSensorValues[i] = 1;
        } else {
            irSensorValues[i] = 0;
        }
    }

    int ValSensor1 = digitalRead(sMostRight);
    int ValSensor5 = digitalRead(sMostLeft);

    if (ValSensor1 == 1) {
        Right();
    } else if (ValSensor5 == 1) {
        Left();
    }

    // HIGH means detected black line
    // LOW means doesn't detected black line
    if (irSensorValues[0] == HIGH && irSensorValues[1] == LOW && irSensorValues[2] == LOW) {
        Stop();
        Left();
    } //100 
    else if (irSensorValues[0] == HIGH && irSensorValues[1] == HIGH && irSensorValues[2] == LOW) {
        Left();
    } //110
    else if (irSensorValues[0] == LOW && irSensorValues[1] == LOW && irSensorValues[2] == HIGH) {
        Stop();
        Right();
    } //001
    else if (irSensorValues[0] == LOW && irSensorValues[1] == HIGH && irSensorValues[2] == HIGH) {
        Right();
    } //011
    else {
        Forward();
    } //111

    if (irSensorValues[0] == HIGH && irSensorValues[1] == HIGH && irSensorValues[2] == HIGH && (millis() - junctionDetectedTime) > junctionTimeInterval) {
        junctionDetected++;
        junctionDetectedTime = millis();
    }
    Serial.println(junctionDetected);

}