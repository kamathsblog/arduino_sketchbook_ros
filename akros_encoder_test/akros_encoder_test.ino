/* Encoder Library - TwoKnobs Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

#include <Encoder.h>
#define ENC_CPR 300

// Change these pin numbers to the pins connected to your encoder.
//   Best Pe4ormance: both pins have interrupt capability
//   Good Pe4ormance: only the first pin has interrupt capability
//   Low Pe4ormance:  neither pin has interrupt capability
//motors 1:lf, 2:lb, 3:rb, 4:rf
Encoder enc1(19, 17);
Encoder enc2(18, 16);
Encoder enc3(20, 24);
Encoder enc4(21, 22);
//   avoid using pins with LEDs attached

void setup() {
  
  Serial.begin(9600);
  Serial.println("TwoKnobs Encoder Test:");
}

long position1 = -999;
long position2 = -999;
long position3 = -999;
long position4 = -999;

void loop() {
  long new1, new2, new3, new4;
  new1 = enc1.read();
  new2 = enc2.read();
  new3 = enc3.read();
  new4 = enc4.read();

  int vel1, vel2, vel3, vel4;
  vel1 = enc1.readRPM(ENC_CPR);
  vel2 = enc2.readRPM(ENC_CPR);
  vel3 = -1 * enc3.readRPM(ENC_CPR);
  vel4 = -1 * enc4.readRPM(ENC_CPR);

  if (new1 != position1 || new2 != position2 || new3 != position3 || new4 != position4 ) {
    Serial.print("Position");
    Serial.print(" | Left Front = ");
    Serial.print(new1);
    Serial.print(" | Left Back = ");
    Serial.print(new2);
    Serial.print(" | Right Front = ");
    Serial.print(new4);
    Serial.print(" | Right Back = ");
    Serial.print(new3);
    Serial.println();
    /*
    Serial.print("Velocity");
    Serial.print(" | Left Front = ");
    Serial.print(vel1);
    Serial.print(" | Left Back = ");
    Serial.print(vel2);
    Serial.print(" | Right Front = ");
    Serial.print(vel4);
    Serial.print(" | Right Back = ");
    Serial.print(vel3);
    Serial.println();
 */
    position1 = new1;
    position2 = new2;
    position3 = new3;
    position4 = new4;
  }
  // if a character is sent from the serial monitor,
  // reset both back to zero.
  /*
  if (Serial.available()) {
    Serial.read();
    Serial.println("Reset both knobs to zero");
    enc1.write(0);
    enc2.write(0);
    enc3.write(0);
    enc4.write(0);
  }
  */

  delay(10);
}
