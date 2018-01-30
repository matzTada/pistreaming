#include <SoftwareSerial.h>

#define MOT_1_H 4
#define MOT_1_L 5
#define MOT_1_PWM 6
#define MOT_2_H 7
#define MOT_2_L 8
#define MOT_2_PWM 9

SoftwareSerial mySerial(2, 3);

void motorControl(int number, int H, int L, int pwm) {
  if (number == 1) {
    digitalWrite(MOT_1_H, H);
    digitalWrite(MOT_1_L, L);
    analogWrite(MOT_1_PWM, pwm);
  }
  else if (number == 2) {
    digitalWrite(MOT_2_H, H);
    digitalWrite(MOT_2_L, L);
    analogWrite(MOT_2_PWM, pwm);
  }
}

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
  Serial.println(F("Simple Motor Control"));
  mySerial.println(F("Simple Motor Control"));

  //motor setup
  pinMode(MOT_1_H, OUTPUT);
  pinMode(MOT_1_L, OUTPUT);
  pinMode(MOT_1_PWM, OUTPUT);
  pinMode(MOT_2_H, OUTPUT);
  pinMode(MOT_2_L, OUTPUT);
  pinMode(MOT_2_PWM, OUTPUT);
}

// continuously reads packets, looking for ZB Receive or Modem Status
void loop() {
  if (mySerial.available() > 0) {
    String payload = mySerial.readStringUntil("\n");
    Serial.print(payload);
    char payloadType = payload.charAt(0);
    if (payloadType == 't') { //stop
      mySerial.print(F("===STOP==="));
      Serial.print(F("===STOP==="));
      motorControl(1, LOW, LOW, 0);
      motorControl(2, LOW, LOW, 0);
    }
    else if (payloadType == 's') {
      mySerial.print(F("===STRAIGHT==="));
      Serial.print(F("===STRAIGHT==="));
      motorControl(1, LOW, HIGH, 127);
      motorControl(2, LOW, HIGH, 127);
    }
    else if (payloadType == 'r') {
      mySerial.print(F("===RIGHT ROTATE==="));
      Serial.print(F("===RIGHT ROTATE==="));
      motorControl(1, HIGH, LOW, 200);
      motorControl(2, LOW, HIGH, 200);
    }
    else if (payloadType == 'l') {
      mySerial.print(F("===LEFT ROTATE==="));
      Serial.print(F("===LEFT ROTATE==="));
      motorControl(1, LOW, HIGH, 200);
      motorControl(2, HIGH, LOW, 200);
    }
    else if (payloadType == 'b') {
      mySerial.print(F("===BACK==="));
      Serial.print(F("===BACK==="));
      motorControl(1, HIGH, LOW, 127);
      motorControl(2, HIGH, LOW, 127);
    }
    else if (payloadType == 'R') {
      mySerial.print(F("===RIGHT ROTATE==="));
      Serial.print(F("===RIGHT ROTATE==="));
      motorControl(1, LOW, HIGH, 127);
      motorControl(2, LOW, HIGH, 200);
    }
    else if (payloadType == 'L') {
      mySerial.print(F("===LEFT ROTATE==="));
      Serial.print(F("===LEFT ROTATE==="));
      motorControl(1, LOW, HIGH, 200);
      motorControl(2, LOW , HIGH, 127);
    }
  }
}


