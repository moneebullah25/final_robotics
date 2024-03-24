const int tp = 9;
const int ep = 10;

float dtime, dist;

void setup() {
  pinMode(tp, OUTPUT);
  pinMode(ep, INPUT);
  Serial.begin(9600);
}

void loop() {
  digitalWrite(tp, LOW);
  delayMicroseconds(2);
  digitalWrite(tp, HIGH);
  delayMicroseconds(10);
  digitalWrite(tp, LOW);

  dtime = pulseIn(ep, HIGH);
  dist = (dtime*.0343)/2;
  Serial.print("Distance: ");
  Serial.println(dist);
  delay(50);
}
