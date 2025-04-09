#include <Wire.h>
#include <MPU6050.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Ticker.h>  

const char* ssid = "Ayken"; 
const char* password = ""; 

const int leftP = 25;
const int leftN = 26;
const int rightP = 27;
const int rightN = 14;

// Speed encoder pins
const int encoder1 = 34;
const int encoder2 = 35;

// Encoder tick counters
volatile unsigned long encoder1Ticks = 0;
volatile unsigned long encoder2Ticks = 0;

// RPM calculations
float rpmMotor1 = 0.0;
float rpmMotor2 = 0.0;

// Constants
const int ticksPerRevolution = 20;
const int debounceDelay = 50;  // Debounce delay in milliseconds

// Debouncing state
unsigned long lastDebounceTime1 = 0;
unsigned long lastDebounceTime2 = 0;

// IMU sensor instance
MPU6050 mpu;

// Calibration values
float accelXOffset = 0;
float accelYOffset = 0;
float accelZOffset = 0;
float gyroXOffset = 0;
float gyroYOffset = 0;
float gyroZOffset = 0;

// AsyncWebServer and WebSockets
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
AsyncWebSocket ws_rpm("/ws_rpm");
AsyncWebSocket ws_imu("/ws_imu");

// Timer for RPM calculation (interrupt every 1000 ms)
Ticker rpmTimer;
Ticker imuTimer;

// Debounced encoder tick functions
void IRAM_ATTR onEncoder1Tick() {
  unsigned long currentTime = millis();
  if (currentTime - lastDebounceTime1 > debounceDelay) {
    encoder1Ticks++;
    lastDebounceTime1 = currentTime;
  }
}

void IRAM_ATTR onEncoder2Tick() {
  unsigned long currentTime = millis();
  if (currentTime - lastDebounceTime2 > debounceDelay) {
    encoder2Ticks++;
    lastDebounceTime2 = currentTime;
  }
}

// Timer interrupt function to update RPM every second
void updateRPM() {
  noInterrupts();  // Disable interrupts while we process the data
  rpmMotor1 = (encoder1Ticks / ticksPerRevolution) * 60.0;
  rpmMotor2 = (encoder2Ticks / ticksPerRevolution) * 60.0;

  encoder1Ticks = 0;
  encoder2Ticks = 0;
  interrupts();  // Re-enable interrupts

  String rpmMessage = "RPM1: " + String(rpmMotor1, 2) + ", RPM2: " + String(rpmMotor2, 2);
  ws_rpm.textAll(rpmMessage);
}

// IMU data reading function
void updateIMUData() {
  int16_t ax_raw, ay_raw, az_raw;
  int16_t gx_raw, gy_raw, gz_raw;

  mpu.getAcceleration(&ax_raw, &ay_raw, &az_raw);
  mpu.getRotation(&gx_raw, &gy_raw, &gz_raw);

  // Convert to float and apply calibration
  float ax = ax_raw - accelXOffset;
  float ay = ay_raw - accelYOffset;
  float az = az_raw - accelZOffset;
  float gx = gx_raw - gyroXOffset;
  float gy = gy_raw - gyroYOffset;
  float gz = gz_raw - gyroZOffset;

  String imuData = "AX: " + String(ax, 2) + ", AY: " + String(ay, 2) + ", AZ: " + String(az, 2) +
                   ", GX: " + String(gx, 2) + ", GY: " + String(gy, 2) + ", GZ: " + String(gz, 2);
  ws_imu.textAll(imuData);
}


// Calibration function for MPU6050
void calibrateIMU() {
  long ax = 0, ay = 0, az = 0;
  long gx = 0, gy = 0, gz = 0;

  for (int i = 0; i < 1000; i++) {
    int16_t tempAx, tempAy, tempAz;
    int16_t tempGx, tempGy, tempGz;

    mpu.getAcceleration(&tempAx, &tempAy, &tempAz);
    mpu.getRotation(&tempGx, &tempGy, &tempGz);

    ax += tempAx;
    ay += tempAy;
    az += tempAz;
    gx += tempGx;
    gy += tempGy;
    gz += tempGz;

    delay(1);
  }

  accelXOffset = ax / 1000.0;
  accelYOffset = ay / 1000.0;
  accelZOffset = az / 1000.0;
  gyroXOffset = gx / 1000.0;
  gyroYOffset = gy / 1000.0;
  gyroZOffset = gz / 1000.0;

  Serial.println("IMU Calibration completed.");
}

void Forward() {
  digitalWrite(leftP, HIGH);
  digitalWrite(leftN, LOW);
  digitalWrite(rightP, HIGH);
  digitalWrite(rightN, LOW);
}

void Backward() {
  digitalWrite(leftP, LOW);
  digitalWrite(leftN, HIGH);
  digitalWrite(rightP, LOW);
  digitalWrite(rightN, HIGH);
}

void Left() {
  digitalWrite(leftP, LOW);
  digitalWrite(leftN, HIGH);
  digitalWrite(rightP, HIGH);
  digitalWrite(rightN, LOW);
}

void Right() {
  digitalWrite(leftP, HIGH);
  digitalWrite(leftN, LOW);
  digitalWrite(rightP, LOW);
  digitalWrite(rightN, HIGH);
}

void stopbot() {
  digitalWrite(leftP, LOW);
  digitalWrite(leftN, LOW);
  digitalWrite(rightP, LOW);
  digitalWrite(rightN, LOW);
}

void setup() {
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 20) {
    delay(500);
    Serial.print(".");
    retries++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("Connected to WiFi network with IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Failed to connect to Wi-Fi");
  }

  // Motor pins setup
  pinMode(leftP, OUTPUT);
  pinMode(leftN, OUTPUT);
  pinMode(rightP, OUTPUT);
  pinMode(rightN, OUTPUT);
  
  stopbot();

  // Encoder pins setup
  pinMode(encoder1, INPUT_PULLUP);
  pinMode(encoder2, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(encoder1), onEncoder1Tick, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder2), onEncoder2Tick, RISING);
  // Setup WebSocket
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.addHandler(&ws_rpm);
  server.addHandler(&ws_imu);
  server.begin();

  // Initialize MPU6050
  Wire.begin();
  mpu.initialize();

  // Call the calibration function
  calibrateIMU();

  // Set up a timer to call updateRPM() every 1 second (1000 ms)
  rpmTimer.attach(1.0, updateRPM);  // 1 second interval

  // Set up a timer to call updateIMUData() every 1 second (1000 ms)
  imuTimer.attach(1.0, updateIMUData);  // 1 second interval
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

void loop() {
  // Main loop remains empty, as the timer interrupt will handle RPM and IMU data
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

    // Send acknowledgment to WebSocket client
    client->text("Message received: " + String(message));

    // Handle motor movement command
    movement(message);
}}
