const int lightSensorPin = A0;    // Define the analog pin for the light sensor
const int numReadings = 10;      // Number of readings to take for averaging

int readings[numReadings];
int index = 0;
int total = 0;

void setup() {
  pinMode(lightSensorPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  int sensorValue = analogRead(lightSensorPin);
  total = total - readings[index];
  readings[index] = sensorValue;
  total = total + sensorValue;
  index = (index + 1) % numReadings;
  int average = total / numReadings;
  Serial.println("Average Light Sensor Value: " + String(average));
  delay(1000);
}
