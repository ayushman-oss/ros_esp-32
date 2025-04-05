#include <WiFi.h>
#include <ESPAsyncWebServer.h>


const char* ssid = "Ayken";
const char* password = "Jake1044";

const int leftP = 25;
const int leftN = 26;
const int rightP = 27;
const int rightN = 14;

// Speed encoder pins
const int encoder1 = 34;
const int encoder2 = 35;

// Encoder tick counters
volatile int encoder1Ticks = 0;
volatile int encoder2Ticks = 0;

// RPM calculations
unsigned long lastRPMUpdate = 0;
float rpmMotor1 = 0.0;
float rpmMotor2 = 0.0;

// Constants
const int ticksPerRevolution = 20;


AsyncWebServer server(80);

// WebSocket on "/ws" endpoint
AsyncWebSocket ws("/ws");

void IRAM_ATTR onEncoder1Tick() {
    encoder1Ticks++;
}
void IRAM_ATTR onEncoder2Tick() {
    encoder2Ticks++;
}
void setup() {
  Serial.begin(115200);

  WiFi.softAP(ssid, password);
   

  IPAddress IP = WiFi.softAPIP();
  Serial.print("Soft AP IP Address: ");
  Serial.println(IP);

    
  pinMode(leftP, OUTPUT);
  pinMode(leftN, OUTPUT);
  pinMode(rightP, OUTPUT);
  pinMode(rightN, OUTPUT);
  
  stopbot();

  pinMode(encoder1, INPUT_PULLUP);
  pinMode(encoder2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder1), onEncoder1Tick, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder2), onEncoder2Tick, RISING);

  
  
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();

  
}

void loop() {
   unsigned long currentTime = millis();
  if (currentTime - lastRPMUpdate >= 1000) {
    noInterrupts();
    rpmMotor1 = (encoder1Ticks / (float)ticksPerRevolution) * 60.0;
    rpmMotor2 = (encoder2Ticks / (float)ticksPerRevolution) * 60.0;
    encoder1Ticks = 0;
    encoder2Ticks = 0;
    interrupts();

    lastRPMUpdate = currentTime;
    //Serial.printf("Motor 1 RPM: %.2f, Motor 2 RPM: %.2f\n", rpmMotor1, rpmMotor2);
    String rpmMessage = "RPM1: " + String(rpmMotor1, 2) + ", RPM2: " + String(rpmMotor2, 2);
    ws.textAll(rpmMessage);
  }

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

    // Send acknowledgment to ROS
    client->text("Message received: " + message);


    movement(message);
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
