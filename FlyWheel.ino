#include <Servo.h>

/*---------------- PIN ASSIGNMENT ----------------*/
#define START_LED 15
#define START_BTN 16
#define GEN_LED 17        
#define GEN_BTN 18
#define CLUTCH 41
#define LAMP 1            
#define HALL_PIN 30
#define METER_SERVO_PIN 22
#define GEN_SERVO_PIN 21
#define DEBUG_LED LED_BUILTIN

/*---------------- CONSTANTS ----------------*/
#define MAX_RPM 120
#define READY_RPM 100       
#define MIN_GEN_RPM 40      
#define GEN_SERVO_ENGAGED 125 
#define GEN_SERVO_HOME 90    

#define SERVO_STEP_MS 25     
#define GEN_BUTTON_LOCKOUT 500 
#define SHUTDOWN_DELAY_MS 1500 
#define STATE_TRANSITION_LOCKOUT 250 

#define INACTIVITY_TIMEOUT_MS 10000 

#define LED_ON LOW        
#define LED_OFF HIGH
#define RPM_TIMEOUT_US 2500000 
#define MIN_PULSE_INTERVAL 2000
#define PULSES_PER_REV 6

/*---------------- STATE MACHINE ----------------*/
enum State { STANDBY, CRANKING, READY_TO_GEN, GENERATING };
State currentState = STANDBY;
unsigned long stateStartTime = 0; 
unsigned long lastDebugPrint = 0; 
unsigned long lowSpeedTimer = 0; 
unsigned long lastServoStep = 0; 
unsigned long inactivityTimer = 0; 

/*---------------- RPM GLOBALS ----------------*/
volatile unsigned long firstPulseTime = 0, lastPulseTime = 0, lastValidPulseTime = 0;
volatile int pulseCount = 0;
volatile unsigned long revTime = 0;
volatile bool newRev = false, debugState = false;
float rpm = 0, rpmSmoothed = 0;

Servo meterServo, genServo;
int genServoCurrentPos = GEN_SERVO_HOME; 

class Button {
  int pin; bool last;
public:
  Button(int p) : pin(p) { pinMode(pin, INPUT_PULLUP); last = digitalRead(pin); }
  bool pressed() { bool now = digitalRead(pin); bool res = (last == HIGH && now == LOW); last = now; return res; }
  void reset() { last = digitalRead(pin); } 
};
Button startBtn(START_BTN), genBtn(GEN_BTN);

void runBlinker(int pin, int interval) {
  static unsigned long lastBlink = 0; static bool state = false;
  if (millis() - lastBlink > interval) { lastBlink = millis(); state = !state; digitalWrite(pin, state ? LED_ON : LED_OFF); }
}

void hallISR() {
  unsigned long now = micros();
  if (now - lastPulseTime < MIN_PULSE_INTERVAL) return;
  lastPulseTime = now; lastValidPulseTime = now;
  debugState = !debugState; digitalWrite(DEBUG_LED, debugState);
  if (pulseCount == 0) firstPulseTime = now;
  pulseCount++;
  if (pulseCount >= PULSES_PER_REV) { revTime = now - firstPulseTime; pulseCount = 0; newRev = true; }
}

void setup() {
  Serial.begin(115200);
  pinMode(START_LED, OUTPUT); pinMode(GEN_LED, OUTPUT); pinMode(CLUTCH, OUTPUT);
  pinMode(DEBUG_LED, OUTPUT); pinMode(LAMP, OUTPUT);
  analogWriteFrequency(1, 441); analogWriteFrequency(22, 441);
  analogWrite(LAMP, 0); 
  digitalWrite(START_LED, LED_OFF); digitalWrite(GEN_LED, LED_OFF); digitalWrite(CLUTCH, LOW);
  pinMode(HALL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallISR, FALLING);
  meterServo.attach(METER_SERVO_PIN); genServo.attach(GEN_SERVO_PIN); 
  genServo.write(GEN_SERVO_HOME);
}

void loop() {
  updateRPM();

  // ⚡ INACTIVITY MONITOR
  if (rpmSmoothed > 0.5) {
    inactivityTimer = millis(); 
  }

  // ⚡ IMPROVED TIMEOUT WITH HARD EXIT
  if ((currentState == CRANKING || currentState == READY_TO_GEN) && (millis() - inactivityTimer > INACTIVITY_TIMEOUT_MS)) {
    Serial.println("TIMEOUT: Resetting.");
    
    // Hardware Reset
    digitalWrite(CLUTCH, LOW); 
    analogWrite(LAMP, 0);
    digitalWrite(START_LED, LED_OFF);
    digitalWrite(GEN_LED, LED_OFF);
    
    // Software Reset
    startBtn.reset();          
    genBtn.reset();
    currentState = STANDBY;
    
    return; // ⚡ CRITICAL: Skip the rest of the loop so STANDBY button isn't checked yet
  }

  if (millis() - lastDebugPrint > 200) {
    lastDebugPrint = millis();
    Serial.print("RPM: "); Serial.print(rpmSmoothed);
    Serial.print(" | State: ");
    switch(currentState) {
      case STANDBY: Serial.println("STANDBY"); break;
      case CRANKING: Serial.println("CRANKING"); break;
      case READY_TO_GEN: Serial.println("READY_TO_GEN"); break;
      case GENERATING: Serial.println("GENERATING"); break;
    }
  }

  switch (currentState) {
    case STANDBY:
      runBlinker(START_LED, 500); digitalWrite(GEN_LED, LED_OFF);
      digitalWrite(CLUTCH, LOW); analogWrite(LAMP, 0);
      updateServo(GEN_SERVO_HOME); 
      if (startBtn.pressed()) { 
        digitalWrite(START_LED, LED_OFF); digitalWrite(CLUTCH, HIGH); 
        inactivityTimer = millis(); 
        currentState = CRANKING; 
      }
      break;

    case CRANKING:
      if (rpmSmoothed >= READY_RPM) { digitalWrite(CLUTCH, LOW); stateStartTime = millis(); currentState = READY_TO_GEN; }
      break;

    case READY_TO_GEN:
      runBlinker(GEN_LED, 200);
      if (millis() - stateStartTime > GEN_BUTTON_LOCKOUT) {
        if (genBtn.pressed()) { digitalWrite(GEN_LED, LED_OFF); currentState = GENERATING; }
      }
      if (rpmSmoothed < 10) currentState = STANDBY;
      break;

    case GENERATING:
      updateServo(GEN_SERVO_ENGAGED); 
      if (rpmSmoothed < MIN_GEN_RPM) {
        if (lowSpeedTimer == 0) lowSpeedTimer = millis(); 
        if (millis() - lowSpeedTimer > SHUTDOWN_DELAY_MS) {
          analogWrite(LAMP, 0); currentState = STANDBY;
        }
      } else { lowSpeedTimer = 0; }

      if (genServoCurrentPos >= GEN_SERVO_ENGAGED) {
        int brightness = map((int)rpmSmoothed, MIN_GEN_RPM, MAX_RPM, 30, 255);
        brightness = constrain(brightness, 0, 255);
        analogWrite(LAMP, brightness);
      } else { analogWrite(LAMP, 0); }
      break;
  }
  meterServo.write(map((int)rpmSmoothed, 0, MAX_RPM, 115, 0));
}

void updateServo(int target) {
  if (millis() - lastServoStep > SERVO_STEP_MS) {
    lastServoStep = millis();
    if (genServoCurrentPos < target) genServoCurrentPos++;
    else if (genServoCurrentPos > target) genServoCurrentPos--;
    genServo.write(genServoCurrentPos);
  }
}

void updateRPM() {
  bool isClutchNoiseWindow = (currentState == READY_TO_GEN && (millis() - stateStartTime < STATE_TRANSITION_LOCKOUT));
  if (newRev) {
    noInterrupts(); unsigned long rTime = revTime; newRev = false; interrupts();
    if (rTime > 0 && !isClutchNoiseWindow) {
      float nextRPM = 60000000.0 / rTime;
      if (nextRPM > 0.5) {
        rpm = nextRPM; 
        if (rpmSmoothed < 1.0) rpmSmoothed = rpm * 0.5;
      }
    }
  }
  unsigned long timeSinceLastPulse = micros() - lastValidPulseTime;
  float targetRPM = rpm;
  if (timeSinceLastPulse > RPM_TIMEOUT_US) { targetRPM = 0; rpm = 0; } 
  else if (rpmSmoothed > 10.0) { targetRPM = (rpm > 10.0) ? rpm : rpmSmoothed; } 
  else { targetRPM = rpm; }
  if (!isClutchNoiseWindow) { rpmSmoothed = (rpmSmoothed * 0.94) + (targetRPM * 0.06); }
  if (rpmSmoothed < 0.5) rpmSmoothed = 0;
}
