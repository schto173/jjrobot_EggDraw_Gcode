/*
 * DrawBot Controller v1.1
 * Refactored version with improved movement handling and state management
 */

#include <EEPROM.h>
#include <Servo.h>

// Version
#define VERSION "DrawBot 1.1"

// Hardware Configuration
#define ENABLE_MOTORS 4
#define Y_STEP_PIN 12
#define Y_DIR_PIN 5
#define X_STEP_PIN 7
#define X_DIR_PIN 8
#define SERVO1_PIN 10
#define LED_PIN 13

// Servo Configuration
#define SERVO_UP 10
#define SERVO_DOWN 40

// Motor Parameters
#define Y_AXIS_STEPS_PER_UNIT 8.88889  // Steps per deg, 16x microstepping
#define X_AXIS_STEPS_PER_UNIT 8.88889
#define Y_MIN 0
#define Y_MAX 90

// Movement State Management
struct MovementState {
  volatile long position_y;
  volatile long position_x;
  long target_position_y;
  long target_position_x;
  uint8_t dir_y;
  uint8_t dir_x;
  bool isMoving;
  bool movementComplete;
  bool commandReceived;
};

// Communication State Management
struct CommState {
  String inputBuffer;
  int currentConnId;
  bool servo_enabled;
};

// Global State Objects
MovementState moveState = { 0, 0, 0, 0, 0, 0, false, true, false };
CommState commState = { "", -1, false };
Servo penServo;

// Function Declarations
void initializeHardware();
void initializeWiFi();
void processIncomingData();
void processGCode(int connId, String gcode);
void updateMotorPosition();
void handleMovement(float x, float y, float z, int connId);
void penUp();
void penDown();
float parseValue(String gcode, char param);
void sendResponse(int connId, String response);
void ESPflush();
bool ESPwaitFor(const char* response, int timeout);

void setup() {
  initializeHardware();
  initializeWiFi();
}

void loop() {
  processIncomingData();
  updateMotorPosition();
}

void initializeHardware() {
  // Configure pins
  pinMode(ENABLE_MOTORS, OUTPUT);
  pinMode(Y_STEP_PIN, OUTPUT);
  pinMode(Y_DIR_PIN, OUTPUT);
  pinMode(X_STEP_PIN, OUTPUT);
  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  // Initial pin states
  digitalWrite(ENABLE_MOTORS, HIGH);  // Motors disabled
  digitalWrite(LED_PIN, LOW);

  // Serial initialization
  Serial.begin(115200);   // Debug serial
  Serial1.begin(115200);  // ESP8266 serial

  // Initialize servo
  penServo.attach(SERVO1_PIN);
  penServo.write(SERVO_UP);
  delay(1000);
  penServo.detach();

  // Timer3 setup for stepper control
  TCCR3A = 0;
  TCCR3B = (1 << WGM32) | (1 << CS31);  // CTC mode, prescaler=8
  OCR3A = 1000;                         // 0.065ms interval
  TCNT3 = 0;
  TIMSK3 |= (1 << OCIE3A);
}

void initializeWiFi() {
  Serial.println(F("Initializing WiFi..."));

  ESPflush();
  Serial1.println(F("AT+RST"));
  ESPwaitFor("ready", 10);
  delay(1000);

  Serial1.println(F("AT+CWMODE=2"));
  ESPwaitFor("OK", 3);

  Serial1.println(F("AT+CWSAP=\"DrawBot\",\"\",5,0"));
  ESPwaitFor("OK", 5);

  Serial1.println(F("AT+CIPMUX=1"));
  ESPwaitFor("OK", 3);

  Serial1.println(F("AT+CIPSERVER=1,8080"));
  ESPwaitFor("OK", 3);

  Serial.println(F("WiFi initialized!"));
  Serial.println(F("SSID: DrawBot"));
  Serial.println(F("Port: 8080"));

  digitalWrite(LED_PIN, HIGH);
}
void processIncomingData() {
  while (Serial1.available()) {
    char c = Serial1.read();

    if (c != '\n') {
      commState.inputBuffer += c;
    } else {
      commState.inputBuffer.trim();

      if (commState.inputBuffer.startsWith("+IPD")) {
        int firstComma = commState.inputBuffer.indexOf(',');
        int secondComma = commState.inputBuffer.indexOf(',', firstComma + 1);
        int colon = commState.inputBuffer.indexOf(':', secondComma);

        if (firstComma != -1 && colon != -1) {
          commState.currentConnId = commState.inputBuffer.substring(firstComma + 1, secondComma).toInt();
          String data = commState.inputBuffer.substring(colon + 1);
          processGCode(commState.currentConnId, data);
        }
      }

      commState.inputBuffer = "";
    }
  }
}

void processGCode(int connId, String gcode) {
  gcode.trim();
  gcode.toUpperCase();

  Serial.print(F("Processing G-code: "));
  Serial.println(gcode);
  Serial.print(F("Current Position - X: "));
  Serial.print(moveState.position_x / X_AXIS_STEPS_PER_UNIT);
  Serial.print(F(" Y: "));
  Serial.println(moveState.position_y / Y_AXIS_STEPS_PER_UNIT);

  if (gcode.startsWith("G0 ") || gcode.startsWith("G1 ")) {
    float x = parseValue(gcode, 'X');
    float y = constrain(parseValue(gcode, 'Y'), Y_MIN, Y_MAX);
    float z = parseValue(gcode, 'Z');
    handleMovement(x, y, z, connId);
  } else if (gcode.startsWith("M280")) {
    int angle = parseValue(gcode, 'S');
    if (angle >= 0 && angle <= 180) {
      penServo.attach(SERVO1_PIN);
      penServo.write(angle);
      delay(200);
      penServo.detach();
      sendResponse(connId, "OK\r\n");
    }
  }
}

void handleMovement(float x, float y, float z, int connId) {
  bool positionChanged = false;

  // Handle Z movement first (pen up/down)
    if (z > 0) {
      penUp();
    } else {
      penDown();
    }


  // Calculate new positions
  long new_target_y = y * Y_AXIS_STEPS_PER_UNIT;

  // Normalize X to 0-360 range and calculate steps
  while (x < 0) x += 360;
  x = fmod(x, 360);
  long new_target_x = x * X_AXIS_STEPS_PER_UNIT;

  // Check if position actually changes
  if (new_target_x != moveState.target_position_x || new_target_y != moveState.target_position_y) {
    moveState.target_position_x = new_target_x;
    moveState.target_position_y = new_target_y;
    positionChanged = true;
  }

  // Determine if movement is needed
  bool needsToMove = (moveState.position_x != moveState.target_position_x) || (moveState.position_y != moveState.target_position_y);

  if (positionChanged && needsToMove) {
    digitalWrite(ENABLE_MOTORS, LOW);
    moveState.isMoving = true;
    moveState.commandReceived = true;
    moveState.movementComplete = false;
    Serial.println(F("Movement started"));
  } else {
    Serial.println(F("No movement needed"));
    sendResponse(connId, "OK\r\n");
  }
}

void updateMotorPosition() {
  static bool wasMoving = false;
  bool currentlyMoving = false;

  // Update Y axis state
  if (moveState.position_y != moveState.target_position_y) {
    if (moveState.position_y < moveState.target_position_y) {
      digitalWrite(Y_DIR_PIN, HIGH);
      moveState.dir_y = 1;
    } else {
      digitalWrite(Y_DIR_PIN, LOW);
      moveState.dir_y = 2;
    }
    currentlyMoving = true;
  } else {
    moveState.dir_y = 0;
  }

  // Update X axis state
  if (moveState.position_x != moveState.target_position_x) {
    if (moveState.position_x < moveState.target_position_x) {
      digitalWrite(X_DIR_PIN, HIGH);
      moveState.dir_x = 1;
    } else {
      digitalWrite(X_DIR_PIN, LOW);
      moveState.dir_x = 2;
    }
    currentlyMoving = true;
  } else {
    moveState.dir_x = 0;
  }

  // Handle movement state changes
  if (currentlyMoving && !wasMoving) {
    digitalWrite(ENABLE_MOTORS, LOW);
  } else if (!currentlyMoving && wasMoving) {
    delay(50);
    digitalWrite(ENABLE_MOTORS, HIGH);

    if (moveState.commandReceived) {
      sendResponse(commState.currentConnId, "OK\r\n");
      moveState.commandReceived = false;
    }
    moveState.isMoving = false;
    moveState.movementComplete = true;
  }

  wasMoving = currentlyMoving;
}
void penUp() {
  penServo.attach(SERVO1_PIN);
  penServo.write(SERVO_UP);
  delay(300);
  penServo.detach();
}

void penDown() {
  penServo.attach(SERVO1_PIN);
  penServo.write(SERVO_DOWN);
  delay(300);
  penServo.detach();
}

float parseValue(String gcode, char param) {
  int index = gcode.indexOf(param);
  if (index != -1) {
    int nextSpace = gcode.indexOf(' ', index);
    if (nextSpace == -1) nextSpace = gcode.length();
    return gcode.substring(index + 1, nextSpace).toFloat();
  }
  return 0;
}

void sendResponse(int connId, String response) {
  String cmd = "AT+CIPSEND=";
  cmd += connId;
  cmd += ",";
  cmd += response.length();
  Serial1.println(cmd);
  ESPwaitFor(">", 1);
  Serial1.print(response);
}

void ESPflush() {
  while (Serial1.available()) {
    Serial1.read();
  }
}

bool ESPwaitFor(const char* response, int timeout) {
  long start = millis();
  String buffer;

  while ((millis() - start) < (timeout * 1000)) {
    if (Serial1.available()) {
      char c = Serial1.read();
      buffer += c;
      if (buffer.indexOf(response) != -1) {
        return true;
      }
    }
  }
  return false;
}

// Timer Interrupt Service Routine
ISR(TIMER3_COMPA_vect) {
  static uint8_t motor_select = 0;

  if (motor_select == 0) {
    // Y motor
    if (moveState.dir_y == 1 && moveState.position_y < moveState.target_position_y) {
      digitalWrite(Y_STEP_PIN, HIGH);
      delayMicroseconds(1);
      digitalWrite(Y_STEP_PIN, LOW);
      moveState.position_y++;
    } else if (moveState.dir_y == 2 && moveState.position_y > moveState.target_position_y) {
      digitalWrite(Y_STEP_PIN, HIGH);
      delayMicroseconds(1);
      digitalWrite(Y_STEP_PIN, LOW);
      moveState.position_y--;
    }
  } else {
    // X motor
    if (moveState.dir_x == 1 && moveState.position_x < moveState.target_position_x) {
      digitalWrite(X_STEP_PIN, HIGH);
      delayMicroseconds(1);
      digitalWrite(X_STEP_PIN, LOW);
      moveState.position_x++;
    } else if (moveState.dir_x == 2 && moveState.position_x > moveState.target_position_x) {
      digitalWrite(X_STEP_PIN, HIGH);
      delayMicroseconds(1);
      digitalWrite(X_STEP_PIN, LOW);
      moveState.position_x--;
    }
  }

  motor_select = !motor_select;
}
