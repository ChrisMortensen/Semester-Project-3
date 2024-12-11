const int trigPin1 = 23;
const int echoPin1 = 22;
const int powerPin1 = 7;
//const int trigPin2 = 11;
//const int echoPin2 = 12;

float duration, distance, duration2, distance2;

void setup() {
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(powerPin1, OUTPUT);
  digitalWrite(powerPin1, HIGH);
  //pinMode(trigPin2, OUTPUT);
  //pinMode(echoPin2, INPUT);
  Serial.begin(9600);
}

void calcDistance(int trigPin, int echoPin, char name[]) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);

  distance = (duration*.0343)/2;
  Serial.print(name);
  Serial.println(distance);
}

void loop(){
  calcDistance(trigPin1, echoPin1, "Sensor 1: ");
  delay(500);
  //calcDistance(trigPin2, echoPin2, "Sensor 2: ");
  //delay(500);
}
