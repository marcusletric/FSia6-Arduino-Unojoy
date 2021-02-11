/** iBus2USB v1.2.1 ****************************************************************************

  By: Patrick Kerr  
  Target: Arduino Leonardo Pro Micro
  Prerequisite: Have the Joystick library included in your "Arduino/libraries" folder
  Wiring receiver (~5V):
     # + on RAW pin (near USB port)
     # - on GND pin
     # Signal on Rx1 pin 
          
  This Sketch takes FlySky i-Bus Serial data from the receiver 
  and turns it into an USB Joystick to use with various drone simulators 
  (Tested in LiftOff, VelociDrone and DRL Simulator)

  Using the ArduinoJoystickLibrary from MHeironimus on GitHub : https://git.io/Jvb7j
  
  Also inspired by iBus2PPM from povlhp on GitHub for the iBus data reading loop 
  iBus2PPM : https://git.io/Jvb5e adapted for Leonardo board.

  I am using a FlySky FS-i6 transmitter with custom firmware from benb0jangles on GitHub,
  FlySky-i6-Mod- : https://git.io/Jvb54 which unlocks all 10 channels when using iBus and 8 in PPM,
  but who wants to use PPM when almost every receiver can do iBus for the same price.
  
        Copyright (c) 2017, Patrick Kerr
        
        This source code is free software; you can redistribute it and/or
        modify it under the terms of the GNU Lesser General Public
        License as published by the Free Software Foundation; either
        version 3 of the License, or (at your option) any later version.
        This source code is distributed in the hope that it will be useful,
        but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
        Lesser General Public License for more details.
        You should have received a copy of the GNU Lesser General Public
        License along with this source code; if not, write to the Free Software
        Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

**********************************************************************************************/

#include "UnoJoy.h"
#include <SoftwareSerial.h>
//#define LED 4 // Error LED pin
//#define RC_CHAN 11  // Maximum 14 due to buffer size limitations. // using 11 here due to Joystick analog axies limitation
#define RC_CHAN 14  // All 14 needed for FSiA6 checksum
#define MIN_COMMAND 1000 // Minimum value that the RC can send (Typically 1000us)
#define MAX_COMMAND 2000 // Maximum value that the RC can send (Typically 2000us)
#define STICK_CENTER (MIN_COMMAND+((MAX_COMMAND-MIN_COMMAND)/2))
//#define IBUS_BUFFSIZE 32 // iBus packet size (2 byte header, space for 14 channels x 2 bytes, 2 byte checksum)
#define IBUS_BUFFSIZE 31 // hacked FSia6 RX packet size (1 byte header, space for 14 channels x 2 bytes, 2 byte checksum)

SoftwareSerial mySerial(2, 3);

enum {  // enum defines the order of channels
  ROLL, // Channel 1 , index 0
  PITCH,
  THROTTLE,
  YAW,
  AUX1, // First Auxilary, Channel 5, index 4...
  AUX2,
  AUX3,
  AUX4,
  AUX5,
  AUX6,
  AUX7,
  AUX8,
  AUX9,
  AUX10,
};

// static Joystick_ Joystick(0x420, JOYSTICK_TYPE_JOYSTICK, 0, 0, true, true, true, true, true, true, true, true, true, true, true); // Maxed out 11 Analog Aux inputs
//     Joystick_ Name( Joystick ID, Joystick Type, Btn,Hat, X,    Y,    Z,   rX,   rY,   rZ, Rud., Thr., Acc., Brk., Str.

static uint8_t ibusIndex = 0; // Index counter, obviously...
static uint8_t ibus[IBUS_BUFFSIZE] = {0}; // iBus Buffer array
static uint16_t rcValue[RC_CHAN] = {0}; // RC/Joystick Values array

void setup() {
  setupUnoJoy();
  mySerial.begin(115200);
  #ifdef LED
    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH); // LED ON Until Proper iBUS RX signal is detected
  #endif
}

void loop() {
  if (mySerial.available()) {
    uint8_t val = mySerial.read();
    
    // Hacked FSiA6 RX protocol
    if (ibusIndex == 0 && val != 0x55) {  // Not the expected 0x55 at index 0, 
      ibusIndex = 0;                      // so we have to reset the index.
      return; // Skip all and wait next loop for another byte.
    }
    if (ibusIndex < IBUS_BUFFSIZE) ibus[ibusIndex] = val; // populate ibus array with current byte
    ibusIndex++; 
    if (ibusIndex == IBUS_BUFFSIZE) { // End of packet, Verify integrity
      ibusIndex = 0;
      
        rcValue[ 0] = (ibus[ 2] << 8) + ibus[ 1];
        rcValue[ 1] = (ibus[ 4] << 8) + ibus[ 3];
        rcValue[ 2] = (ibus[ 6] << 8) + ibus[ 5];
        rcValue[ 3] = (ibus[ 8] << 8) + ibus[ 7];
        rcValue[ 4] = (ibus[10] << 8) + ibus[ 9];
        rcValue[ 5] = (ibus[12] << 8) + ibus[11];
        rcValue[ 6] = (ibus[14] << 8) + ibus[13];
        rcValue[ 7] = (ibus[16] << 8) + ibus[15];
        rcValue[ 8] = (ibus[18] << 8) + ibus[17];
        rcValue[ 9] = (ibus[20] << 8) + ibus[19];
        rcValue[10] = (ibus[22] << 8) + ibus[21];
        rcValue[11] = (ibus[24] << 8) + ibus[23];
        rcValue[12] = (ibus[26] << 8) + ibus[25];
        rcValue[13] = (ibus[28] << 8) + ibus[27];

        int checksum = (ibus[30] << 8) + ibus[29];
        int myChecksum = 0;
        for( int c=0; c<14; c++){
          myChecksum += rcValue[c];
        }
        
        if( myChecksum == checksum ){
          dataForController_t controllerData = getBlankDataForController();
          controllerData.leftStickX = (rcValue[YAW] - 1000) >> 2;
          controllerData.leftStickY = (rcValue[THROTTLE] -1000) >> 2;
          controllerData.rightStickX = (rcValue[ROLL] - 1000) >> 2;
          controllerData.rightStickY = (rcValue[PITCH] - 1000) >> 2;
          controllerData.circleOn = rcValue[AUX1] > 1100;
          controllerData.squareOn = rcValue[AUX2] > 1300 && rcValue[AUX2] < 1800;
          controllerData.crossOn = rcValue[AUX2] > 1800;
          setControllerData(controllerData);
        } else {
          ibusIndex = 0;
        }
    }
  }
}


