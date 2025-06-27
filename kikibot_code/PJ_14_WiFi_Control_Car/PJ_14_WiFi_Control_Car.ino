// ESP32 2WD Car WiFi Control - Improved Version
// Hardware: ESP32 WROOM-32E + DRV8837 Motor Driver
#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiClient.h>

//#define DEBUG 1

// WiFi Configuration
const char* WIFI_SSID = "copaland";
const char* WIFI_PASSWORD = "24868080";
const int SERVER_PORT = 80;

// Motor Pin Definitions
const int MOTOR1_PWM_PIN = 25;        // Motor 1 PWM
const int MOTOR1_DIR_PIN = 33;        // Motor 1 Direction
const int MOTOR1_EN_PIN = 32;         // Motor 1 Enable

const int MOTOR2_PWM_PIN = 14;        // Motor 2 PWM  
const int MOTOR2_DIR_PIN = 27;        // Motor 2 Direction
const int MOTOR2_EN_PIN = 26;         // Motor 2 Enable

// LED Pin Definitions
const int LED_LEFT_PIN = 4;           // Left LED
const int LED_RIGHT_PIN = 13;         // Right LED

// PWM Configuration
const int PWM_CHANNEL_1 = 1;          // Motor 1 PWM Channel
const int PWM_CHANNEL_2 = 2;          // Motor 2 PWM Channel
const int PWM_FREQUENCY = 1200;       // PWM Frequency (Hz)
const int PWM_RESOLUTION = 8;         // PWM Resolution (8-bit = 0-255)

// Speed Configuration
const int SPEED_FORWARD = 200;        // Forward speed
const int SPEED_BACKWARD = 55;        // Backward speed  
const int SPEED_TURN_FAST = 200;      // Fast wheel speed for turning
const int SPEED_TURN_SLOW = 160;      // Slow wheel speed for turning
const int SPEED_STOP = 0;             // Stop speed

// LED Blink Configuration
const unsigned long LED_BLINK_INTERVAL = 200;  // 200ms blink interval

// Global Variables
String receivedCommand = "0";
unsigned long previousLedTime = 0;
bool ledState = false;
WiFiServer server(SERVER_PORT);

// Function Prototypes
void setupWiFi();
void setupMotors();
void setupLEDs();
void handleClient();
void executeMotorCommand(String command);
void setMotorSpeed(int motor1Speed, int motor2Speed, bool motor1Dir, bool motor2Dir);
void stopMotors();
void updateLEDs();

void setup() {
#ifdef DEBUG  
  Serial.begin(115200);
  Serial.println("ESP32 2WD Car Starting...");
#endif
  
  setupWiFi();
  setupMotors();
  setupLEDs();
  
  server.begin();
  MDNS.addService("http", "tcp", SERVER_PORT);
  
#ifdef DEBUG    
  Serial.println("Setup completed. Car ready for control!");
#endif
  
  previousLedTime = millis();
}

void loop() {
  handleClient();
  updateLEDs();
}

void setupWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
#ifdef DEBUG       
    Serial.print(".");
#endif      
  }
  
#ifdef DEBUG    
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(WIFI_SSID);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("TCP server started");
#endif
}

void setupMotors() {
  // Setup PWM channels
  ledcSetup(PWM_CHANNEL_1, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(MOTOR1_PWM_PIN, PWM_CHANNEL_1);
  
  ledcSetup(PWM_CHANNEL_2, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(MOTOR2_PWM_PIN, PWM_CHANNEL_2);
  
  // Setup direction pins
  pinMode(MOTOR1_DIR_PIN, OUTPUT);
  pinMode(MOTOR2_DIR_PIN, OUTPUT);
  
  // Setup enable pins (active HIGH)
  pinMode(MOTOR1_EN_PIN, OUTPUT);
  pinMode(MOTOR2_EN_PIN, OUTPUT);
  digitalWrite(MOTOR1_EN_PIN, HIGH);  // Enable Motor 1
  digitalWrite(MOTOR2_EN_PIN, HIGH);  // Enable Motor 2
  
  // Initialize motors to stop
  stopMotors();
}

void setupLEDs() {
  pinMode(LED_LEFT_PIN, OUTPUT);
  pinMode(LED_RIGHT_PIN, OUTPUT);
  digitalWrite(LED_LEFT_PIN, LOW);
  digitalWrite(LED_RIGHT_PIN, HIGH);
}

void handleClient() {
  WiFiClient client = server.available();
  if (!client) {
    return;
  }
  
  // Wait for client data
  while(client.connected() && !client.available()) {
    delay(1);
  }
  
  // Read HTTP request
  String request = client.readStringUntil('\r');
  int addr_start = request.indexOf(' ');
  int addr_end = request.indexOf(' ', addr_start + 1);
  
  if (addr_start == -1 || addr_end == -1) {
#ifdef DEBUG     
    Serial.print("Invalid request: ");
    Serial.println(request);
#endif
    client.stop();
    return;
  }
  
  // Extract command from URL
  receivedCommand = request.substring(addr_start + 1, addr_end);
  
#ifdef DEBUG   
  Serial.println("Command: " + receivedCommand);
#endif

  // Execute motor command
  executeMotorCommand(receivedCommand);
  
  // Send HTTP response
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/plain");
  client.println("Connection: close");
  client.println();
  client.println("OK");
  client.stop();
}

void executeMotorCommand(String command) {
  if (command == "/btn/S") {           // Stop
    stopMotors();
  }
  else if (command == "/btn/F") {      // Forward
    setMotorSpeed(SPEED_FORWARD, SPEED_FORWARD, false, false);
  }
  else if (command == "/btn/B") {      // Backward
    setMotorSpeed(SPEED_BACKWARD, SPEED_BACKWARD, true, true);
  }
  else if (command == "/btn/L") {      // Left turn
    setMotorSpeed(SPEED_TURN_SLOW, SPEED_TURN_FAST, false, false);
  }
  else if (command == "/btn/R") {      // Right turn
    setMotorSpeed(SPEED_TURN_FAST, SPEED_TURN_SLOW, false, false);
  }
  else if (command == "/btn/q") {      // Backward Left
    setMotorSpeed(SPEED_BACKWARD, SPEED_FORWARD, true, false);
  }
  else if (command == "/btn/p") {      // Backward Right
    setMotorSpeed(SPEED_FORWARD, SPEED_BACKWARD, false, true);
  }
  else if (command == "/btn/x") {      // Emergency Stop
    stopMotors();
  }
#ifdef DEBUG
  else {
    Serial.println("Unknown command: " + command);
  }
#endif
}

void setMotorSpeed(int motor1Speed, int motor2Speed, bool motor1Dir, bool motor2Dir) {
  // Set motor directions
  digitalWrite(MOTOR1_DIR_PIN, motor1Dir);
  digitalWrite(MOTOR2_DIR_PIN, motor2Dir);
  
  // Set motor speeds
  ledcWrite(PWM_CHANNEL_1, motor1Speed);
  ledcWrite(PWM_CHANNEL_2, motor2Speed);
}

void stopMotors() {
  digitalWrite(MOTOR1_DIR_PIN, LOW);
  digitalWrite(MOTOR2_DIR_PIN, LOW);
  ledcWrite(PWM_CHANNEL_1, SPEED_STOP);
  ledcWrite(PWM_CHANNEL_2, SPEED_STOP);
}

void updateLEDs() {
  unsigned long currentTime = millis();
  
  if (currentTime - previousLedTime >= LED_BLINK_INTERVAL) {
    ledState = !ledState;
    
    // Alternate LEDs (when left is ON, right is OFF and vice versa)
    digitalWrite(LED_LEFT_PIN, ledState);
    digitalWrite(LED_RIGHT_PIN, !ledState);
    
    previousLedTime = currentTime;
  }
}
