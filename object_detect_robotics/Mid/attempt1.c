/*------ Arduino Line Follower Code----- */
/*-------definning Inputs------*/
#define LS 2 // left sensor
#define RS 3 // right sensor

/*-------definning Outputs------*/
#define LM1 4 // left motor
#define LM2 5 // left motor
#define RM1 6 // right motor
#define RM2 7 // right motor

void setup()
{
	pinMode(LS, INPUT);
	pinMode(RS, INPUT);
	pinMode(LM1, OUTPUT);
	pinMode(LM2, OUTPUT);
	pinMode(RM1, OUTPUT);
	pinMode(RM2, OUTPUT);
}

void move_forward() {
	digitalWrite(LM1, HIGH);
	digitalWrite(LM2, LOW);
	digitalWrite(RM1, HIGH);
	digitalWrite(RM2, LOW);
}

void move_right() {
	digitalWrite(LM1, LOW);
	digitalWrite(LM2, LOW);
	digitalWrite(RM1, HIGH);
	digitalWrite(RM2, LOW);
}

void move_left() {
	digitalWrite(LM1, HIGH);
	digitalWrite(LM2, LOW);
	digitalWrite(RM1, LOW);
	digitalWrite(RM2, LOW);
}

void stop_motor() {
	digitalWrite(LM1, LOW);
	digitalWrite(LM2, LOW);
	digitalWrite(RM1, LOW);
	digitalWrite(RM2, LOW);
}

void loop()
{
	if(!(digitalRead(LS)) && !(digitalRead(RS))) // Move Forward (digitalRead(LS) && digitalRead(RS))
	{
		move_forward();	
	}

	if(!(digitalRead(LS)) && digitalRead(RS)) // Turn right
	{
		move_right();
	}

	if(digitalRead(LS) && !(digitalRead(RS))) // turn left
	{
		move_right();
	}

	if(digitalRead(LS) && digitalRead(RS)) // stop !(digitalRead(LS)) && !(digitalRead(RS))
	{
		stop_motor();
	}
}