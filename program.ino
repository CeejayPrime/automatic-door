/*
  About Project:

  This project is a model of an automatic door that opens and closes when a person passes by. First, a motion sensor detects the presence of a passerby, then sends signal
  for the door to open. The door closes when a set timer has elapsed, then the door closes.

  Materials used for the project:

  Nema 17 stepper motor
  Aluminum coupling
  TB6600 stepper driver
  On/Off Switch
  Potentiometer
  Arduino Uno
  Threaded rod: Pitch = 1.25mm. Full revolution will move the glass slide 1.25mm

*/

#define motionSeen 8
#define openEndLimit 9
#define closeEndLimit 7
#define setDoorOpenPosition A0
#define setDoorClosePosition A1
#define dirPin 5
#define stepPin 4
#define stopMotor A2

int openingTravelDistance = 0;
int closingTravelDistance = 0;

bool isMovementOn = false;

int openEndState;
int lastOpenEndState = HIGH;

byte openEndIsPressed;

unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

unsigned long closeTime = 0;

enum {
  doorIsClosed,
  doorIsOpen,
  doorOpening,
  doorClosing
};

unsigned char doorState = doorIsClosed;

void openDoor() {
  Serial.println("Door open");

  digitalWrite(dirPin, HIGH);
  for (int i = 0; i < openingTravelDistance; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(600);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(600);
  }
}

void closeDoor() {
  Serial.println("Door close");

  digitalWrite(dirPin, LOW);
  for (int i = 0; i < closingTravelDistance; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(600);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(600);
  }
}

void motorStop() {
  digitalWrite(stopMotor, LOW);
  Serial.println("Motor Has Stopped");
}

void setup() {
  Serial.begin(115200);

  pinMode(motionSeen, INPUT);
  pinMode(openEndLimit, INPUT_PULLUP);
  pinMode(closeEndLimit, INPUT_PULLUP);

  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(stopMotor, OUTPUT);

}

void loop() {

  int doorOpenPosition = analogRead(setDoorOpenPosition);
  int doorClosePosition = analogRead(setDoorClosePosition);

  openingTravelDistance = map(doorOpenPosition, 0, 1023, 1, 10000);
  closingTravelDistance = map(doorClosePosition, 0, 1023, 1, 10000);

  int openEndVal = digitalRead(openEndLimit);
  int closeEndVal = digitalRead(closeEndLimit);

  byte doorOpenComplete;

  bool startOpen = false;
  bool startClose = false;

  int sensorActive = digitalRead(motionSeen);
  int pirState = LOW;

//  if (openEndVal != lastOpenEndState) {
//    lastDebounceTime = millis();
//  }
//
//  if ((millis() - lastDebounceTime) > debounceDelay) {
//    if (openEndVal != openEndState) {
//      openEndState = openEndVal;
//
//      if (openEndState == LOW) {
//        openEndIsPressed = true;
//      }
//    }
//  }

  if (sensorActive == 1) {
    pirState = 0;
    isMovementOn = true;

  } else {
    if (pirState == 0) {
      isMovementOn = false;
    }
  }

  switch (doorState) {
    case doorIsClosed:
      if (isMovementOn) {
        doorState = doorOpening;
        break;
      }
      else {
        break;
      }

    case doorOpening:
      openDoor();
      doorState = doorIsOpen;
      break;

    case doorIsOpen:
      if (millis() - closeTime > 20000) {
        doorState = doorClosing;
        break;
      }

    case doorClosing:
      if (isMovementOn) {
        motorStop();
        doorState = doorOpening;
        break;
      }
      closeDoor();
      doorState = doorIsClosed;
      break;

    default:
      doorState = doorIsClosed;
      break;
  }
}
