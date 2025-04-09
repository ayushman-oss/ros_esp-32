#include <Arduino.h>

// Motor control pins
const int leftP = 25;
const int leftN = 26;
const int rightP = 27;
const int rightN = 14;

// Encoder pins
const int encoder1 = 34;
const int encoder2 = 35;

// Encoder tick counters
volatile int encoder1Ticks = 0;
volatile int encoder2Ticks = 0;

// RPM calculation
unsigned long lastRPMUpdate = 0;
float rpmMotor1 = 0.0;
float rpmMotor2 = 0.0;
const int ticksPerRevolution = 20;

// PWM settings
const int pwmFreq = 1000;
const int pwmResolution = 8; // 8-bit resolution: 0-255
const int pwmMax = 255;
const int pwmStep = 50; // Step size
const int pwmHoldTime = 5000; // 5 seconds per level

// PWM channels
const int leftChannel = 0;
const int rightChannel = 1;

void IRAM_ATTR onEncoder1Tick() {
  encoder1Ticks++;
}

void IRAM_ATTR onEncoder2Tick() {
  encoder2Ticks++;
}

void setup() {
  Serial.begin(115200);

  pinMode(encoder1, INPUT_PULLUP);
  pinMode(encoder2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder1), onEncoder1Tick, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder2), onEncoder2Tick, RISING);

  pinMode(leftN, OUTPUT);
  pinMode(rightN, OUTPUT);
  digitalWrite(leftN, LOW);
  digitalWrite(rightN, LOW);

  // PWM setup
  ledcSetup(leftChannel, pwmFreq, pwmResolution);
  ledcAttachPin(leftP, leftChannel);
  ledcSetup(rightChannel, pwmFreq, pwmResolution);
  ledcAttachPin(rightP, rightChannel);

  Serial.println("Starting PWM-RPM test...");
}

void loop() {
  for (int duty = 50; duty <= pwmMax; duty += pwmStep) {
    Serial.println("===================================");
    Serial.printf("Testing PWM Duty: %d\n", duty);

    ledcWrite(leftChannel, duty);
    ledcWrite(rightChannel, duty);

    unsigned long startTime = millis();
    while (millis() - startTime < pwmHoldTime) {
      unsigned long currentTime = millis();
      if (currentTime - lastRPMUpdate >= 1000) {
        noInterrupts();
        rpmMotor1 = (encoder1Ticks / (float)ticksPerRevolution) * 60.0;
        rpmMotor2 = (encoder2Ticks / (float)ticksPerRevolution) * 60.0;
        encoder1Ticks = 0;
        encoder2Ticks = 0;
        interrupts();

        lastRPMUpdate = currentTime;
        Serial.printf("PWM: %d => RPM1: %.2f, RPM2: %.2f\n", duty, rpmMotor1, rpmMotor2);
      }
    }
  }

  // Stop motors after test
  ledcWrite(leftChannel, 0);
  ledcWrite(rightChannel, 0);
  Serial.println("Test complete.");
  while (1); // Stop loop after one test round
}
