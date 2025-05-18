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

// Encoder pins
const int encoder1 = 34;
const int encoder2 = 35;

// Encoder tick counters
volatile int encoder1Ticks = 0;
volatile int encoder2Ticks = 0;

unsigned long lastRPMUpdate = 0;
float rpmMotor1 = 0.0;
float rpmMotor2 = 0.0;
const int ticksPerRevolution = 20;


AsyncWebServer server(80);

// WebSocket on "/ws" endpoint
AsyncWebSocket ws("/ws");
AsyncWebSocket ws_rpm("/ws_rpm"); 
AsyncWebSocket ws_constants("/ws_constants");




void IRAM_ATTR onEncoder1Tick() {
  encoder1Ticks++;
}
void IRAM_ATTR onEncoder2Tick() {
  encoder2Ticks++;
}

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


void setup() {
  Serial.begin(115200);

  //WiFi.begin(ssid, password);
  WiFi.softAP(ssid, password);
  
//  while (WiFi.status() != WL_CONNECTED) {
//    delay(500);
//    Serial.print(".");
//  }
  


  IPAddress IP = WiFi.softAPIP();
  Serial.print("Soft AP IP Address: ");
  Serial.println(IP);


  
  ws.onEvent(onWsEvent);
  ws_constants.onEvent(onConstantsEvent);
  server.addHandler(&ws);
  server.addHandler(&ws_constants);
  server.addHandler(&ws_rpm);

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
  
}

void loop() {



  unsigned long currentTime = millis();
  if (currentTime - lastRPMUpdate >= 1000) {
        noInterrupts();
        //rpmMotor1 = (encoder1Ticks / (float)ticksPerRevolution) * 60.0;
        //rpmMotor2 = (encoder2Ticks / (float)ticksPerRevolution) * 60.0;

        float raw_rpm1 = (encoder1Ticks / (float)ticksPerRevolution) * 60.0;
        float raw_rpm2 = (encoder2Ticks / (float)ticksPerRevolution) * 60.0;

        rpmMotor1 = kalmanUpdate(raw_rpm1, est_rpm1, err_est1, q1, r1);
        rpmMotor2 = kalmanUpdate(raw_rpm2, est_rpm2, err_est2, q2, r2);

        encoder1Ticks = 0;
        encoder2Ticks = 0;
        interrupts();

        lastRPMUpdate = currentTime;
        String rpmMessage = "RPM1: " + String(rpmMotor1, 2) + ", RPM2: " + String(rpmMotor2, 2);
        
        ws_rpm.textAll(rpmMessage);
        
       //Serial.println("[Broadcast] " + rpmMessage);
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
  

}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.println("Client connected");
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.println("Client disconnected");
  } else if (type == WS_EVT_DATA) {
    char messageStr[16];
    memcpy(messageStr, data, len);
    messageStr[len] = '\0';
    
    int message = atoi(messageStr);
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
      if (commaIndex == -1 && i < 4) {
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
  ledcWrite(leftP, pwm1);
  ledcWrite(leftN, 0);
  ledcWrite(rightP, pwm2);
  ledcWrite(rightN, 0);
}

void Backward() {
  ledcWrite(leftN, pwm1);
  ledcWrite(leftP, 0);
  ledcWrite(rightN, pwm2);
  ledcWrite(rightP, 0);
}

void Left() {
  ledcWrite(leftP, 0);
  ledcWrite(leftN, pwm1);
  ledcWrite(rightP, pwm2);
  ledcWrite(rightN, 0);
}

void Right() {
  ledcWrite(leftP, 0);
  ledcWrite(leftN, pwm1);
  ledcWrite(rightP, 0);
  ledcWrite(rightN, pwm2);
}

void stopbot() {
  ledcWrite(leftP, 0);
  ledcWrite(leftN, 0);
  ledcWrite(rightP, 0);
  ledcWrite(rightN, 0);

}
