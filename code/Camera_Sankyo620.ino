/*
Project "Mod-8mm"
Code to run a modified Super-8mm camera in "Mod-8" mode
with sprocketless film advance by stepper motor with pinch roller.
Copyright (C) 2026 John Calder some rights reserved
This Mod-8mm code applies the Open Source "MIT License".
https://choosealicense.com/licenses/mit/
Mod-8mm general (other than code) documents apply Creative Commons CC4.0-BY licensing.
Details in those documents.
Sankyo 620 Camera special mod is the use of a mini stepper motor for aperture control.
Started 2026-04-07 JPC John Calder
2026-04-19 test with drive motor - add diagnostics
2026-04-25 add aperture control
2026-05-08 JPC aperture control is available at all times including RUN
2026-05-09 JPC trials of power supplies, batteries versus bench supply - acting the same
2026-05-13 JPC problem with "Nucleo" pin 9 not responding to digitalWrite code
 solution - discover this STM32 Nucleo is not liking 5V inputs into A1, A2, A3
 but it is going OK after moving these inputs to D11, D12 and D3
*/
//*******************************************************
//Configuration Section START - alter values here for individual camera setup
//all pulse widths are the time on HIGH
//the pulse step cycle is 2 x these values
const unsigned long APERTURE_PULSEWIDTH = 40000;
const unsigned long SHUTTER_PULSEWIDTH = 1389;  //18fps (1/18) * (1/20) * (1/2)
//const unsigned long SHUTTER_PULSEWIDTH = 1389 * 18 / 2; //2fps testing
const unsigned long DRIVE_PULSEWIDTH = 300;  //nominally 300, increase for testing
//const unsigned long DRIVE_PULSEWIDTH = 3000; // advance in about 1/6 sec
//adjust time for optimal mark exposure adjusting according to film results
const unsigned long LASER_ZAP_TIME = 20000;  //start with 1/50 second
//
//On building the camera it is a challenge to know which direction
//drives will run from stepper motors.
//Therefore on first testing, observe the direction
//each mechanical sub-system is turning,
//and if going in reverse change its LOW to HIGH
const byte DIR_SHUTTER_DEFAULT = HIGH;
const byte DIR_DRIVE_DEFAULT = LOW;
//Configuration Section END
//*******************************************************
// Outport ports for motor control
const byte ADVANCE_SIGNAL = 2;
const byte STEP_APERTURE = 4;
const byte DIR_APERTURE = 5;
const byte STEP_DRIVE = 6;
const byte DIR_DRIVE = 7;
const byte STEP_SHUTTER = 8;  //2026-05-07 testing was 11
const byte DIR_SHUTTER = 9;   //2026-05-07 testing was 12
const byte REGISTRATION_MARK = 10;
// Input ports for control switches
const byte RUN_SWITCH = A0;
//2026-05-13 JPC discover this STM32 Nucleo is not liking 5V inputs into A1, A2, A3
// but it is going OK with moving these inputs to D11, D12 and D3
const byte APERTURE_PLUS = 11;
const byte APERTURE_MINUS = 12;
const byte SINGLE_FRAME = 3;
//Shutter is a repeating run while the run switch is ON,
//toggling the output every SHUTTER_PULSEWIDTH microseconds
unsigned long shutterPulseStartTime;
//Main drive is more complex because of tracking 27 steps of varying pulse width
byte driveState = 0;                  // 0,1,2 ... 26, 27
unsigned long drivePulseStartTime;    //timing of each of the 27 steps
unsigned long driveAdvanceStartTime;  //timing of the frame advance
unsigned long aperturePulseStartTime;
unsigned long prevTime;
unsigned long thisTime;
unsigned long cycleHalf;
unsigned long cycleTime;
unsigned long startTime;
void setup() {
  // pin defaults, Serial monitoring setup, initialise polling check values
  pinMode(ADVANCE_SIGNAL, INPUT);
  pinMode(STEP_APERTURE, OUTPUT);  //Aperture special for Sankyo 620
  pinMode(DIR_APERTURE, OUTPUT);   //Aperture special for Sankyo 620
  pinMode(STEP_DRIVE, OUTPUT);     //Drive Motor Step control
  pinMode(DIR_DRIVE, OUTPUT);      //Drive Motor Configure Direction
  pinMode(STEP_SHUTTER, OUTPUT);   //Shutter Motor Step control
  pinMode(DIR_SHUTTER, OUTPUT);    //Shutter Motor Configure Direction
  pinMode(REGISTRATION_MARK, OUTPUT);
  pinMode(RUN_SWITCH, INPUT);  // RUN!
  pinMode(SINGLE_FRAME, INPUT);
  pinMode(APERTURE_PLUS, INPUT);   //Aperture Step control special for Sankyo 620
  pinMode(APERTURE_MINUS, INPUT);  //Aperture Step control special for Sankyo 620
  //for indication that this code is running
  pinMode(LED_BUILTIN, OUTPUT);
  //2026-05-13 set all digitalWrite pins to gnd
  for (int i = 4; i <= 10; i++) {
    digitalWrite(i, LOW);
  }
  //then apply exceptions from config section above
  //2026-05-07 set above
  digitalWrite(DIR_SHUTTER, DIR_SHUTTER_DEFAULT);
  digitalWrite(DIR_DRIVE, DIR_DRIVE_DEFAULT);
  // aperture applied below in void loop()
  Serial.begin(115200);
  digitalWrite(LED_BUILTIN, HIGH);
  //give the human time to do startup things like clear the serial monitor
  delay(5000);
  Serial.println("PROGRAM START. A2 = " + (String)A2);
  thisTime = micros();
  prevTime = thisTime;
  startTime = thisTime;
  cycleHalf = thisTime;
  cycleTime = thisTime;
}
void loop() {
  //timing
  prevTime = thisTime;
  thisTime = micros();
  //manage case of running more than 70 minutes causing micros() to reset
  if (thisTime < prevTime) {
    Serial.println("micros() reset at " + (String)(prevTime / 60000000) + " min.");
    prevTime = thisTime;
    startTime = thisTime;
    cycleHalf = thisTime;
    cycleTime = thisTime;
    delayMicroseconds(10);
    return;
  }
  if (digitalRead(RUN_SWITCH) == HIGH) {
    delayMicroseconds(10);
    shutter();
    drive();
  } else {
    //reset motors
    if (driveState > 0) {
      digitalWrite(STEP_SHUTTER, LOW);
      digitalWrite(STEP_DRIVE, LOW);
      driveState = 0;
    }
    delayMicroseconds(100);
    shutterPulseStartTime = thisTime;
  }
  if (digitalRead(APERTURE_MINUS) == HIGH) {
    digitalWrite(DIR_APERTURE, LOW);
    if (SHUTTER_PULSEWIDTH >= 10000) Serial.print("AL, ");
    aperture();
  } else if (digitalRead(APERTURE_PLUS) == HIGH) {
    digitalWrite(DIR_APERTURE, HIGH);
    if (SHUTTER_PULSEWIDTH >= 10000) Serial.print("AH, ");
    aperture();
  } else {
    digitalWrite(STEP_APERTURE, LOW);
    aperturePulseStartTime = thisTime;
  }
  //flash LED on an 8 second cycle, include state reporting
  if (thisTime - cycleHalf >= 4000000) {  //half cycle for each of led-on and led-off
    byte ledStatus = digitalRead(LED_BUILTIN);
    bool isMonitor = (thisTime / 1000000 <= 54);
    //Limit serial monitoring to checking near the beginning of the run
    if (isMonitor) {
      Serial.println("LED=" + (String)ledStatus + "; RUN=" + (String)digitalRead(A0));
    }
    if (ledStatus == HIGH) {
      digitalWrite(LED_BUILTIN, LOW);
    } else {
      digitalWrite(LED_BUILTIN, HIGH);
      if (isMonitor) {  //stop serial monitoring but allow led to keep flashing
        Serial.println("Run time = " + (String)((micros() - startTime) / 1000000) + " sec; Cycle time = " + (String)(thisTime - cycleTime) + " microsec.");
      }
      cycleTime = thisTime;
    }
    cycleHalf = thisTime;
  }
}
void drive() {
  if (driveState == 0) {
    if (digitalRead(ADVANCE_SIGNAL) == LOW) {
      return;
    } else {
      driveState = 1;
      drivePulseStartTime = thisTime;
      driveAdvanceStartTime = thisTime;
      //TESTING 2026-04-19
      if (DRIVE_PULSEWIDTH >= 100000) {
        Serial.println("Advance. driveState =");
        Serial.print((String)driveState);
      }
      //delay(1000);
    }
  }
  // registration mark laser zap device
  if (thisTime - driveAdvanceStartTime <= LASER_ZAP_TIME) {
    digitalWrite(REGISTRATION_MARK, HIGH);
  } else {
    digitalWrite(REGISTRATION_MARK, LOW);
  }
  long unsigned pulseTime = thisTime - drivePulseStartTime;
  switch (driveState) {
    case 1:
    case 27:
      // check for step ending
      if (pulseTime >= DRIVE_PULSEWIDTH * 2 * 2) {
        if (driveState == 1) {
          driveState = 2;
        } else {
          driveState = 0;
        }
        drivePulseStartTime = thisTime;  //needed for 2
        if (DRIVE_PULSEWIDTH >= 100000) {
          Serial.print(", " + (String)driveState);  //TESTING
          if (driveState == 0) Serial.println(", DONE.");
        }
        return;
      } else if (pulseTime >= DRIVE_PULSEWIDTH * 2) {
        digitalWrite(STEP_DRIVE, LOW);
      } else {
        digitalWrite(STEP_DRIVE, HIGH);
      }
      break;
    case 2:
    case 26:
      if (pulseTime >= DRIVE_PULSEWIDTH * 3) {
        driveState++;
        //TESTING – do more monitoring when configured to run slowly
        if (DRIVE_PULSEWIDTH >= 100000) Serial.print(", " + (String)driveState);
        drivePulseStartTime = thisTime;
        return;
      } else if (pulseTime >= DRIVE_PULSEWIDTH * 3 / 2) {
        digitalWrite(STEP_DRIVE, LOW);
      } else {
        digitalWrite(STEP_DRIVE, HIGH);
      }
      break;
    default:  //cases 3 to 25
      if (pulseTime >= DRIVE_PULSEWIDTH * 2) {
        driveState++;
        //TESTING – do more monitoring when configured to run slowly
        if (DRIVE_PULSEWIDTH >= 100000) Serial.print(", " + (String)driveState);
        drivePulseStartTime = thisTime;
        return;
      } else if (pulseTime >= DRIVE_PULSEWIDTH) {
        digitalWrite(STEP_DRIVE, LOW);
      } else {
        digitalWrite(STEP_DRIVE, HIGH);
      }
      break;
  }
}
void shutter() {
  long unsigned pulseTime = thisTime - shutterPulseStartTime;
  if (pulseTime >= SHUTTER_PULSEWIDTH * 2) {
    shutterPulseStartTime = thisTime;
    return;
  }
  if (pulseTime >= SHUTTER_PULSEWIDTH) {
    digitalWrite(STEP_SHUTTER, LOW);
  } else {
    digitalWrite(STEP_SHUTTER, HIGH);
  }
}
void aperture() {
  long unsigned pulseTime = thisTime - aperturePulseStartTime;
  if (pulseTime >= APERTURE_PULSEWIDTH * 2) {
    //2026-04-26 JPC aperture into state of waiting for a switch on
    aperturePulseStartTime = thisTime;
    digitalWrite(STEP_APERTURE, LOW);
    return;
  }
  if (pulseTime >= APERTURE_PULSEWIDTH) {
    digitalWrite(STEP_APERTURE, LOW);
  } else {
    digitalWrite(STEP_APERTURE, HIGH);
  }
}