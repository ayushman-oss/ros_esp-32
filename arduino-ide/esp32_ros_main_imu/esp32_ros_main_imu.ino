#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <MPU9250_asukiaaa.h>
#include <MadgwickAHRS.h>


const char* ssid = "Ayken";
const char* password = "Jake1044";

//Motor Pins
const int leftP = 25;
const int leftN = 26;
const int rightP = 27;
const int rightN = 14;

//PWM
const int pwmfreq = 1000;
const int pwmResolution = 8; 
const int pwmMax = 255;

//Speed Encoder 
const int encoder1 = 34;
const int encoder2 = 35;
volatile int encoder1Ticks = 0;
volatile int encoder2Ticks = 0;

unsigned long lastRPMUpdate = 0;
float rpmMotor1 = 0.0;
float rpmMotor2 = 0.0;
const int ticksPerRevolution = 20;


// WebSocket
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
AsyncWebSocket ws_rpm("/ws_rpm"); 
AsyncWebSocket ws_imu("/ws_imu");
AsyncWebSocket ws_constants("/ws_constants");

//isr
void IRAM_ATTR onEncoder1Tick() {
  encoder1Ticks++;
}
void IRAM_ATTR onEncoder2Tick() {
  encoder2Ticks++;
}

//imu
#ifdef ESP32_HAL_I2C_H
#define SDA_PIN 16
#define SCL_PIN 4
#endif
#define DEG_TO_RAD 0.01745329251

MPU9250_asukiaaa imu;
Madgwick filter;

float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
float accelBiasX = 0, accelBiasY = 0, accelBiasZ = 0;

float roll, pitch, yaw;
float initialYaw = 0;
unsigned long lastUpdateMicros = 0;


//kalman
float est_rpm1 = 0.0;
float err_est1 = 1.0;
float q1 = 0.5;  
float r1 = 5.0;  
float est_rpm2 = 0.0;
float err_est2 = 1.0;
float q2 = 0.5;
float r2 = 5.0;

float kalmanUpdate(float meas, float &est, float &err_est, float q, float r) {
  err_est += q;
  float k = err_est / (err_est + r);
  est = est + k * (meas - est);
  err_est = (1 - k) * err_est;

  return est;
}

//pid
float targetRPM = 2000.0; 
float kp = 2.5, ki = 0.05, kd = 0.5; 
float error1 = 0.0, error2 = 0.0;
float prevError1 = 0.0, prevError2 = 0.0;
float integral1 = 0.0, integral2 = 0.0;
int pwm1;
int pwm2;


//Heading Control (Yaw PID) 
float currentYaw = 0.0;
float targetYaw = 0.0;
bool maintainHeading = false;
float headingKp = 2.0, headingKi = 0.0, headingKd = 0.5;
float headingError = 0.0, prevHeadingError = 0.0, headingIntegral = 0.0;


void calibrateSensor(int samples = 500) {
  Serial.println("Calibrating... Please keep the sensor still.");

  float sumAx = 0, sumAy = 0, sumAz = 0;
  float sumGx = 0, sumGy = 0, sumGz = 0;

  for (int i = 0; i < samples; i++) {
    imu.accelUpdate();
    imu.gyroUpdate();

    sumAx += imu.accelX();
    sumAy += imu.accelY();
    sumAz += imu.accelZ() - 1.0; 
    sumGx += imu.gyroX();
    sumGy += imu.gyroY();
    sumGz += imu.gyroZ();

    delay(5); // Small delay for stability
      }
    accelBiasX = sumAx / samples;
    accelBiasY = sumAy / samples;
    accelBiasZ = sumAz / samples;
  
    gyroBiasX = sumGx / samples;
    gyroBiasY = sumGy / samples;
    gyroBiasZ = sumGz / samples;
  
    Serial.println("Calibration done.");
  }

void setup() {
  Serial.begin(115200);

  WiFi.softAP(ssid, password);

  
  IPAddress IP = WiFi.softAPIP();
  Serial.print("Soft AP IP Address: ");
  Serial.println(IP);



  
  ws.onEvent(onWsEvent);
  ws_constants.onEvent(onConstantsEvent);
  server.addHandler(&ws);
  server.addHandler(&ws_constants);
  server.addHandler(&ws_rpm);
  server.addHandler(&ws_imu);
  
  server.begin();

  pinMode(encoder1, INPUT_PULLUP);
  pinMode(encoder2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder1), onEncoder1Tick, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder2), onEncoder2Tick, RISING);
 
  
  pinMode(leftP, OUTPUT);
  pinMode(leftN, OUTPUT);
  pinMode(rightP, OUTPUT);
  pinMode(rightN, OUTPUT);
  ledcAttach(rightP,pwmfreq,pwmResolution);
  ledcAttach(leftP,pwmfreq,pwmResolution);
  ledcAttach(rightN,pwmfreq,pwmResolution);
  ledcAttach(leftN,pwmfreq,pwmResolution);
  
  stopbot();

  #ifdef ESP32_HAL_I2C_H
  Wire.begin(SDA_PIN, SCL_PIN);
  imu.setWire(&Wire);
  #endif
   imu.beginAccel();
   imu.beginGyro();
   imu.beginMag();
   calibrateSensor();
   filter.begin(50);
   for (int i = 0; i < 100; i++) {
    imu.accelUpdate();
    imu.gyroUpdate();

    float ax = imu.accelX() - accelBiasX;
    float ay = imu.accelY() - accelBiasY;
    float az = imu.accelZ() - accelBiasZ;

    float gx = (imu.gyroX() - gyroBiasX) * DEG_TO_RAD;
    float gy = (imu.gyroY() - gyroBiasY) * DEG_TO_RAD;
    float gz = (imu.gyroZ() - gyroBiasZ) * DEG_TO_RAD;

    filter.updateIMU(gx, gy, gz, ax, ay, az);
    delay(10);
  }
  initialYaw = filter.getYaw();
  lastUpdateMicros = micros();
  
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastRPMUpdate >= 1000) {
        noInterrupts();

        float raw_rpm1 = (encoder1Ticks / (float)ticksPerRevolution) * 60.0;
        float raw_rpm2 = (encoder2Ticks / (float)ticksPerRevolution) * 60.0;

        rpmMotor1 = kalmanUpdate(raw_rpm1, est_rpm1, err_est1, q1, r1);
        rpmMotor2 = kalmanUpdate(raw_rpm2, est_rpm2, err_est2, q2, r2);

        encoder1Ticks = 0;
        encoder2Ticks = 0;
        interrupts();

        lastRPMUpdate = currentTime;
        String rpmMessage = "RPM1: " + String(rpmMotor1, 2) + ", RPM2: " + String(rpmMotor2, 2);
        //Serial.println("[Broadcast] " + rpmMessage);
        ws_rpm.textAll(rpmMessage);
       
    }
    
        error1 = targetRPM - rpmMotor1;
        error2 = targetRPM - rpmMotor2;
    
        integral1 += error1;
        integral2 += error2;
    
        float derivative1 = error1 - prevError1;
        float derivative2 = error2 - prevError2;
    
        pwm1 = constrain(kp * error1 + ki * integral1 + kd * derivative1, 150, 255);
        pwm2 = constrain(kp * error2 + ki * integral2 + kd * derivative2, 150, 255);
    
        prevError1 = error1;
        prevError2 = error2;

          if (maintainHeading) {
              float error = targetYaw - currentYaw;
              if (error > 180) error -= 360;
              if (error < -180) error += 360;
          
              headingError = error;
              if (abs(headingError) > 2.0) {
                headingIntegral += headingError;
                float derivative = headingError - prevHeadingError;
                float correction = headingKp * headingError + headingKi * headingIntegral + headingKd * derivative;
                pwm1 = constrain(pwm1 - correction, 150, 255);
                pwm2 = constrain(pwm2 + correction, 150, 255);
                prevHeadingError = headingError;
              }
            }

        
        unsigned long now = micros();
        float deltaT = (now - lastUpdateMicros) / 1e6;  // seconds
        if (deltaT < 0.005) return; // ~200Hz max
        lastUpdateMicros = now;
      
        float freq = 1.0 / deltaT;
        filter.begin(freq); // update filter with new sampling rate
      
        imu.accelUpdate();
        imu.gyroUpdate();
      
        float ax = imu.accelX() - accelBiasX;
        float ay = imu.accelY() - accelBiasY;
        float az = imu.accelZ() - accelBiasZ;
      
        float gx = (imu.gyroX() - gyroBiasX) * DEG_TO_RAD;
        float gy = (imu.gyroY() - gyroBiasY) * DEG_TO_RAD;
        float gz = (imu.gyroZ() - gyroBiasZ) * DEG_TO_RAD;
      
        filter.updateIMU(gx, gy, gz, ax, ay, az);
      
        roll = filter.getRoll() * RAD_TO_DEG;
        pitch = filter.getPitch() * RAD_TO_DEG;
        yaw = (filter.getYaw() - initialYaw) * RAD_TO_DEG;
      
        if (yaw > 180) yaw -= 360;
        if (yaw < -180) yaw += 360;
      
        Serial.print(" | Yaw (rel): "); Serial.println(yaw, 2);
        String imuOrientationString = "Roll: " + String(roll, 2) + 
                              " | Pitch: " + String(pitch, 2) + 
                              " | Yaw: " + String(yaw, 2);
                              
        ws_imu.textAll(imuOrientationString);

}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.println("Client connected");
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.println("Client disconnected");
  } else if (type == WS_EVT_DATA) {
    int message = atoi((char*)data);
    Serial.print("Received: ");
    Serial.println(message);
    movement(message);
  }
}

void onConstantsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                      AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.println("Constants client connected");
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.println("Constants client disconnected");
  } else if (type == WS_EVT_DATA) {
    String payload = String((char*)data).substring(0, len);
    Serial.print("Received constants: ");
    Serial.println(payload);

    float values[6];
    int idx = 0;

    for (int i = 0; i < 6; i++) {
      int commaIndex = payload.indexOf(',', idx);
      if (commaIndex == -1 && i < 5) {
        Serial.println("Error parsing constants");
        return;
      }

      String part = (i < 5) ? payload.substring(idx, commaIndex) : payload.substring(idx);
      values[i] = part.toFloat();
      idx = commaIndex + 1;
    }

    kp = values[0];
    ki = values[1];
    kd = values[2];
    q1 = q2 = values[3];
    r1 = r2 = values[4];
    targetRPM = values[5];

    Serial.println("Constants updated:");
    Serial.println("Kp: " + String(kp));
    Serial.println("Ki: " + String(ki));
    Serial.println("Kd: " + String(kd));
    Serial.println("Q: " + String(q1));
    Serial.println("R: " + String(r1));
    Serial.println("targetRPM: " + String(targetRPM));
  }
}


void movement(int cmdValue) {

  switch (cmdValue) {
    case 1:
      Serial.println("Executing Forward");
      Forward();
      break;
    case 3:
      Serial.println("Executing Backward");
      Backward();
      break;
    case 2:
      Serial.println("Executing Left Turn");
      Left();
      break;
    case 4:
      Serial.println("Executing Right Turn");
      Right();
      break;
    case 5:
      Serial.println("Stopping");
      stopbot();
      break;
    default:
      Serial.println("Unknown Command ++ stopping");
      stopbot();
      break;
  }
}

void Forward() {
  targetYaw = currentYaw; maintainHeading = true;
  ledcWrite(leftP, pwm1);
  ledcWrite(leftN, 0);
  ledcWrite(rightP, pwm2);
  ledcWrite(rightN, 0);
}

void Backward() {
  targetYaw = currentYaw; maintainHeading = true;
  ledcWrite(leftN, pwm1);
  ledcWrite(leftP, 0);
  ledcWrite(rightN, pwm2);
  ledcWrite(rightP, 0);
}

void Left() {
  maintainHeading = false;
  ledcWrite(leftP, 0);
  ledcWrite(leftN, pwm1);
  ledcWrite(rightP, pwm2);
  ledcWrite(rightN, 0);
}

void Right() {
  maintainHeading = false;
  ledcWrite(leftP, 0);
  ledcWrite(leftN, pwm1);
  ledcWrite(rightP, 0);
  ledcWrite(rightN, pwm2);
}

void stopbot() {
  maintainHeading = false;
  ledcWrite(leftP, 0);
  ledcWrite(leftN, 0);
  ledcWrite(rightP, 0);
  ledcWrite(rightN, 0);

}
