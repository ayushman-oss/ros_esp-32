#include <WiFi.h>
#include <ESPAsyncWebServer.h>

// Define the SSID and Password for the ESP32 Soft AP
const char* ssid = "Ayken";
const char* password = "Jake1044";

// Create an AsyncWebServer object on port 80
AsyncWebServer server(80);

// WebSocket on "/ws" endpoint
AsyncWebSocket ws("/ws");

void setup() {
  Serial.begin(115200);

  // Start the Soft AP with SSID and Password
  WiFi.softAP(ssid, password);
  
  // Get the IP address of the ESP32 in Soft AP mode
  IPAddress IP = WiFi.softAPIP();
  Serial.print("Soft AP IP Address: ");
  Serial.println(IP);

  // WebSocket event handler
  ws.onEvent(onWsEvent);

  // Add WebSocket handler to the server
  server.addHandler(&ws);

  // Start the server
  server.begin();
}

void loop() {
  // Nothing needed here, WebSocket events are handled asynchronously
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.println("Client connected");
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.println("Client disconnected");
  } else if (type == WS_EVT_DATA) {
    String message = String((char*)data);
    Serial.print("Received: ");
    Serial.println(message);
    
    // Respond with the same message (echo)
    client->text("Echo: " + message);
  }
}
