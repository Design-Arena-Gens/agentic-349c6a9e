#include <SoftwareSerial.h>

namespace {
constexpr uint8_t PIN_BT_RX = 10;  // HC-05 TX -> Arduino RX
constexpr uint8_t PIN_BT_TX = 11;  // HC-05 RX <- Arduino TX

constexpr uint8_t PIN_M1_IN1 = 2;   // Steering
constexpr uint8_t PIN_M1_IN2 = 3;
constexpr uint8_t PIN_M1_EN = 5;

constexpr uint8_t PIN_M3_IN3 = 4;   // Propulsion
constexpr uint8_t PIN_M3_IN4 = 6;
constexpr uint8_t PIN_M3_EN = 7;

constexpr uint8_t PWM_MAX = 255;
constexpr unsigned long ACK_PULSE_MS = 50UL;
constexpr unsigned long SELF_TEST_ON_MS = 300UL;
constexpr unsigned long SELF_TEST_OFF_MS = 220UL;

SoftwareSerial btSerial(PIN_BT_RX, PIN_BT_TX);
bool ledState = false;
}

void pulseSelfTest();
void resetMotors();
void runCommand(char command);
char normalizeCommand(char command);
void setPropulsionForward();
void setPropulsionBackward();
void setSteeringLeft();
void setSteeringRight();
void setAllStop();
void acknowledgeCommand();

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(PIN_M1_IN1, OUTPUT);
  pinMode(PIN_M1_IN2, OUTPUT);
  pinMode(PIN_M1_EN, OUTPUT);

  pinMode(PIN_M3_IN3, OUTPUT);
  pinMode(PIN_M3_IN4, OUTPUT);
  pinMode(PIN_M3_EN, OUTPUT);

  resetMotors();
  pulseSelfTest();

  btSerial.begin(9600);
}

void loop() {
  if (btSerial.available() > 0) {
    const char incoming = static_cast<char>(btSerial.read());
    const char command = normalizeCommand(incoming);
    runCommand(command);
  }
}

void pulseSelfTest() {
  for (uint8_t i = 0; i < 3; ++i) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(SELF_TEST_ON_MS);
    digitalWrite(LED_BUILTIN, LOW);
    delay(SELF_TEST_OFF_MS);
  }
  ledState = false;
}

void resetMotors() {
  digitalWrite(PIN_M1_IN1, LOW);
  digitalWrite(PIN_M1_IN2, LOW);
  analogWrite(PIN_M1_EN, 0);

  digitalWrite(PIN_M3_IN3, LOW);
  digitalWrite(PIN_M3_IN4, LOW);
  analogWrite(PIN_M3_EN, 0);
}

void runCommand(const char command) {
  switch (command) {
    case 'F':
      setPropulsionForward();
      break;
    case 'B':
      setPropulsionBackward();
      break;
    case 'L':
      setSteeringLeft();
      break;
    case 'R':
      setSteeringRight();
      break;
    case 'S':
    default:
      setAllStop();
      break;
  }
  acknowledgeCommand();
}

char normalizeCommand(const char command) {
  switch (command) {
    case 'F':
    case 'B':
    case 'L':
    case 'R':
    case 'S':
      return command;
    default:
      return 'S';
  }
}

void setPropulsionForward() {
  digitalWrite(PIN_M3_IN3, HIGH);
  digitalWrite(PIN_M3_IN4, LOW);
  analogWrite(PIN_M3_EN, PWM_MAX);
}

void setPropulsionBackward() {
  digitalWrite(PIN_M3_IN3, LOW);
  digitalWrite(PIN_M3_IN4, HIGH);
  analogWrite(PIN_M3_EN, PWM_MAX);
}

void setSteeringLeft() {
  digitalWrite(PIN_M1_IN1, HIGH);
  digitalWrite(PIN_M1_IN2, LOW);
  analogWrite(PIN_M1_EN, PWM_MAX);
}

void setSteeringRight() {
  digitalWrite(PIN_M1_IN1, LOW);
  digitalWrite(PIN_M1_IN2, HIGH);
  analogWrite(PIN_M1_EN, PWM_MAX);
}

void setAllStop() {
  resetMotors();
}

void acknowledgeCommand() {
  ledState = !ledState;
  digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
  delay(ACK_PULSE_MS);
  ledState = !ledState;
  digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
}
