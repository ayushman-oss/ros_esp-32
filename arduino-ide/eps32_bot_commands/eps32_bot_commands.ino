#include <WiFi.h>
#include <ESPAsyncWebServer.h>


const char* ssid = "Ayken";
const char* password = "Jake1044";

const int leftP = 25;
const int leftN = 26;
const int rightP = 27;
const int rightN = 14;

const int pwmfreq = 1000;
const int pwmResolution = 8; // 8-bit resolution: 0-255
const int pwmMax = 255;



AsyncWebServer server(80);

// WebSocket on "/ws" endpoint
AsyncWebSocket ws("/ws");

// Function prototypes
void handleCommand(String command);

void setup() {
  Serial.begin(115200);

  WiFi.softAP(ssid, password);
  

  IPAddress IP = WiFi.softAPIP();
  Serial.print("Soft AP IP Address: ");
  Serial.println(IP);
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();
  
  pinMode(leftP, OUTPUT);
  pinMode(leftN, OUTPUT);
  pinMode(rightP, OUTPUT);
  pinMode(rightN, OUTPUT);
  ledcAttach(rightP,pwmfreq,pwmResolution);
  ledcAttach(leftP,pwmfreq,pwmResolution);
  ledcAttach(rightN,pwmfreq,pwmResolution);
  ledcAttach(leftN,pwmfreq,pwmResolution);
  
  
  
  stopbot();
  
}

void loop() {

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
  ledcWrite(leftP, 150);
  ledcWrite(leftN, 0);
  ledcWrite(rightP, 150);
  ledcWrite(rightN, 0);
}

void Backward() {
  ledcWrite(leftN, 150);
  ledcWrite(leftP, 0);
  ledcWrite(rightN, 150);
  ledcWrite(rightP, 0);
}

void Left() {
  ledcWrite(leftP, 0);
  ledcWrite(leftN, 150);
  ledcWrite(rightP, 150);
  ledcWrite(rightN, 0);
}

void Right() {
  ledcWrite(leftP, 0);
  ledcWrite(leftN, 150);
  ledcWrite(rightP, 0);
  ledcWrite(rightN, 150);
}

void stopbot() {
  ledcWrite(leftP, 0);
  ledcWrite(leftN, 0);
  ledcWrite(rightP, 0);
  ledcWrite(rightN, 0);

}
