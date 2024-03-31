#include <SimpleKalmanFilter.h>
#include <PID_v1_bc.h>

const int motorPin = 5;       // PWM pin for controlling the motor
const int encoderPin = 2;     // Digital pin connected to encoder's signal
volatile unsigned long count; // Variable to store encoder pulse count
unsigned long previousMillis;  // Previous time for calculating RPM
unsigned long rpm;             // Variable to store RPM value

double Setpoint, Input, Output;
double Kp=2.0, Ki=0.1, Kd=0.1; // PID coefficients, adjust these values based on your application

SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.01);
const long SERIAL_REFRESH_TIME = 50;
long refresh_time;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  pinMode(motorPin, OUTPUT);
  pinMode(encoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPin), countPulses, RISING);
  Serial.begin(115200);
  count = 0;
  previousMillis = 0;
  rpm = 0;

  // Initialize the PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(100); // Adjust PID calculation interval
  myPID.SetOutputLimits(0, 255); // Limits for PWM

  Setpoint = 255; // Set your desired RPM here
}

void loop() {
  // Set duty cycle to control motor speed
  //analogWrite(motorPin, 255); // Adjust duty cycle as needed

  // Calculate RPM
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 1000) {
    rpm = (count * 60000UL) / (currentMillis - previousMillis) / 20; // Assuming 20 pulses per revolution
    float estimated_value = simpleKalmanFilter.updateEstimate(rpm);

    Input = estimated_value;
    myPID.Compute();

    Serial.println(rpm);
    Serial.print(",");
    Serial.print(estimated_value,4);
    Serial.print(",");
    Serial.print(Output);
    Serial.println();
    analogWrite(motorPin, Output);
    count = 0;
    previousMillis = currentMillis;
  }

  // Add other code if needed
  
}

void countPulses() {
  count++;
}