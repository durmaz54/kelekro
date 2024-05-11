#include <Arduino.h>
#include <QTRSensors.h>
#include <PID_v1.h>

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

double Setpoint = 2500, Input, Output;
double Kp=0.06, Ki=0, Kd=0.7;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


void setup() {
    Serial.begin(9600);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7, A8}, SensorCount);
  
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
  myPID.SetMode(AUTOMATIC);

}

void loop() {
    uint16_t position = qtr.readLineBlack(sensorValues);
    Serial.println(position);
    delay(100);
}

