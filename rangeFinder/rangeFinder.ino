/* Arduino example code for MaxBotix MB1240 XL-MaxSonar-EZ4 ultrasonic distance sensor: analog voltage output. More info: www.makerguides.com */
 
#define sensorPin A0
 
int distance = 0;
 
void setup() {
 Serial.begin(9600);
}
 
void read_sensor() {
 distance = analogRead(sensorPin) * 1;
}
 
void print_data() {
 Serial.print("distance = ");
 Serial.print(distance);
 Serial.println(" cm");
}
 
void loop() {
 read_sensor();
 print_data();
 delay(1000);
}
