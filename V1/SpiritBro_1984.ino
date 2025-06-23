
/**
 *  A2ThreeD SpiritBro Adapter - Determines the state of a Hasbro 1984/Spengler wand and uses the information to turn on/off a Spirit Life-Size pack electronics
 *   
 *   This code uses code and work derived from the GPStar proton pack source code, re-used according to the GPL license.
 *   Specifically, it uses modified sections of the PowerMeter.h code and some from the ProtonPack.ino sections.
 *   See https://github.com/gpstar81/GPStar-proton-pack/blob/v6.0/develop/source/ProtonPack/PowerMeter.h for the original code.
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, see <https://www.gnu.org/licenses/>.
 *   
 *   Version: 1.0
 *   Date: 4/27/2025
 *   Notes: Initial version for sending out beta boards.

*/

// Pin assignments
#define PACK_LED_PIN 8
#define POWERCELL_LED_PIN 9
#define SPIRIT_POWER_OUT 6
#define SPIRIT_FIRE_OUT 7
#define AUDIO_OUT 26                // GPIO for PWM Audio Output
#define BUILTIN_LED 25

//LED Size definitions
#define POWERCELL_LED_COUNT 15          //Number of LEDs in powercell
#define POWERCELL_80PCT_LED_COUNT 12    //Number of LEDs in powercell for 80% scale pack 
#define CYCLOTRON_LED_COUNT 40          //Number of LEDs in ring for AL/FE modes
#define CYCLOTRON_80PCT_LED_COUNT 32    //Number of LEDs in ring for 80% scale pack 
#define CYCLOTRON84_LED_COUNT 4         //Defaults to 4 cyclotron LEDs for 84/89 modes

#define CYC_START_SPEED 255 //Speed for cyclotron during boot (max 255 is slowest)
#define CYC_IDLE_SPEED 10  //Speed for cyclotron during idle, lower number is faster (default 100) 
#define CYC_84_SPEED 400
#define CYC_FIRE_SPEED 5   //Speed for cyclotron during firing, lower number is faster (default 25) 
#define CYC_BOOT_LEN 9000  //Number of milliseconds to fade in cyclotron ring before boot (2.9sec for Hasbro 1984 wand)
#define BRIGHTNESS 255    // Maximum brightness

//Mode Settings
#define SPIRIT_FIRE false //Set to true if you want the spirt pack to play it's firing audio when the wand is firing.

// Status variables
bool b_afterlifemode = true;   //Determines which version of the cyclotron animation is running. If afterlifemode is true then it assumes a ring LED, otherwise uses 4 LEDs.
bool b_wand_firing = false;
bool b_firing_intensify = false;
bool b_wand_connected = false;
bool b_wand_syncing = false;
bool b_wand_on = false;
bool b_use_power_meter = true;
bool b_show_power_data = false;
bool b_powerbooted = false;
bool b_cyclotronbooted = false;
bool b_picoled = false;

const uint8_t i_wand_power_level_max = 5; // Max power level of the wand.
uint8_t i_wand_power_level = 1; // Power level of the wand.

//Import needed libraries
#include <millisDelay.h>
#include <FastLED.h>

// Boot-up LED animation speeds
uint32_t intBootSpeed = CYC_START_SPEED;
uint32_t intSpeed = CYC_START_SPEED;
uint32_t cyc_boot_interval = CYC_BOOT_LEN / 255; //calculate the delay between ramp up steps (255 steps)

// LED animation settings
const uint32_t pwr_interval = 55;      // Interval between powercell climbing steps
const uint32_t updateInterval = CYC_84_SPEED;   // Interval in milliseconds for LED updates
uint8_t currentLED = 0;     // LED position tracker
uint8_t currentPCellLED = 0;
uint8_t fadeInterval = 30;  // Interval between fade steps in 84 mode
uint8_t fadeAmount = 30;    // Amount to fade each time in 84 mode
uint8_t ringPosition = 0;                            //Position of the cyclotron LED ring animation
uint8_t cyclo_led_count = CYCLOTRON_LED_COUNT;

//Status LED blink settings
uint32_t previousBlinkMillis = 0; // Store last time the LED was updated
uint32_t blinkinterval = 10000;      // 10 seconds interval
uint32_t onDuration = 250;      // 500ms ON duration
uint32_t previousPicoLEDMillis = 0;
uint32_t intPicoLEDSpeed = 1000;

// LED Objects
CRGB pack_leds[CYCLOTRON_LED_COUNT];
CRGB powercell_leds[POWERCELL_LED_COUNT];

CLEDController &cyclotronController = FastLED.addLeds<NEOPIXEL, PACK_LED_PIN>(pack_leds, CYCLOTRON_LED_COUNT);

//Timers
uint32_t currentMillis = 0;
uint32_t previousMillis = 0;  
uint32_t currentPCellMillis = 0;
uint32_t previousPCellMillis = 0;  
uint32_t startMillis = 0;
uint32_t bootMillis = 0;
uint32_t fadePreviousMillis = 0;

// Define states for the different operational modes of the pack
enum PackState { ACTION_IDLE, ACTION_BOOTING, ACTION_POWERDOWN, ACTION_BOOTED, MODE_ON, MODE_OFF, WAND_FIRING, WAND_STOPPED_FIRING };
enum PackState PACK_STATUS;

//Import the PowerMeter code that handles reading the INA219 current sensor and Hasbro wand states
#include "PowerMeter.h"

//Initial setup of hardware
void setup() {

  pinMode(SPIRIT_POWER_OUT, OUTPUT);
  digitalWrite(SPIRIT_POWER_OUT, HIGH);

  pinMode(SPIRIT_FIRE_OUT, OUTPUT);
  digitalWrite(SPIRIT_FIRE_OUT, HIGH);

  pinMode(BUILTIN_LED, OUTPUT);

  Serial.begin(115200);
  Serial.println("SpiritBro V1.0 1984 Full Code - May 1, 2025 (c) A2ThreeD");

  // Power Cell, Cyclotron Lid, and N-Filter.
  FastLED.addLeds<NEOPIXEL, PACK_LED_PIN>(pack_leds, CYCLOTRON_LED_COUNT);
  FastLED.addLeds<NEOPIXEL, POWERCELL_LED_PIN>(powercell_leds, POWERCELL_LED_COUNT);
  FastLED.setDither(0); // Disables the "temporal dithering" feature as this software will set brightness on a per-pixel level by device.
  FastLED.show();

  // Initialize the power metering on the i2c bus
  powerMeterInit();

  //Set the intial pack state
  PACK_STATUS = MODE_OFF;
}

//The main loop of the code for checking wand state and performing actions based on the state
void loop() {

    // get the current time
  currentMillis = millis();

  // Check current voltage/amperage draw of the wand and perform actions based on the values
  checkPowerMeter();

  //Blink the Pico builtin LED
  blinkPicoLED(intPicoLEDSpeed);

  //Perform operations based on the state of the system
  switch(PACK_STATUS) {
  
  case ACTION_IDLE:
    // Do nothing.
  break;

  case ACTION_POWERDOWN:
    Serial.println("ACTION_POWERDOWN Processing");
    packShutdown();
  break;

  case ACTION_BOOTING:
    //Start the LEDs for the powercell/cyclotron in bootup mode
    Serial.println("ACTION_BOOTING Processing");

    //cyclotronBootUp();
      if (b_afterlifemode == true){
      if(currentMillis - bootMillis >= cyc_boot_interval){
        intBootSpeed = intBootSpeed - 1;
        bootMillis = currentMillis;

        if (intBootSpeed <= CYC_IDLE_SPEED){
        b_cyclotronbooted = true;
        }else{
          cyclotronUpdateRing(intBootSpeed);
          powerCellUpdate();

        }
      }
    } else {
      cyclotronUpdate();
      powerCellUpdate();
    }

    if (b_cyclotronbooted == true){
        PACK_STATUS = ACTION_BOOTED;
    }

  break;

  case ACTION_BOOTED:
    Serial.println("ACTION_BOOTED Processing");
    blinkPicoLED(200);

    if (b_afterlifemode == true){
      cyclotronRingIdle();
    } else {
      cyclotronUpdate();
    }
    powerCellUpdate();
  break;

  case WAND_FIRING:
    Serial.println("WAND_FIRING Processing");

    if (b_afterlifemode == true){
      cyclotronRingFiring();
    } else {
      cyclotronUpdate();
    }
    powerCellUpdate();
  break;

  case WAND_STOPPED_FIRING:
    Serial.println("WAND_STOPPED_FIRING Processing");

    if (b_afterlifemode == true){
      cyclotronRingIdle();
    } else {
      cyclotronUpdate();
    }
    
    PACK_STATUS = ACTION_BOOTED;
  break;

  case MODE_OFF:
    blinkStatusLED();
  break;
  }

}

void packStartup() {
  PACK_STATUS = ACTION_BOOTING;
  Serial.println("Powering up pack");
  
  //Send power on pulse to Spirit Life-Size Proton Pack electronics
  sendPowerPulse();

  //Blink Pico LED fast to show wand active
  intPicoLEDSpeed = 200;
}

void packShutdown() {
  // Turn the pack off.
  if(PACK_STATUS != MODE_OFF) {
    Serial.println("Shutting Down pack");

    //If the wand is firing, stop the mode.
    //Send power off pulse to Spirit Life-Size Proton Pack electronics
    //Clear all the LEDs and reset the boot variables.

    wandStoppedFiring();
    sendPowerPulse();
    FastLED.clear();
    FastLED.show();
    b_powerbooted = false;
    b_cyclotronbooted = false;
  }

  Serial.println("Pack is off");
  intPicoLEDSpeed = 1000;
  PACK_STATUS = MODE_OFF;
}

void sendPowerPulse(){
  digitalWrite(SPIRIT_POWER_OUT, LOW);
  delay(200);
  digitalWrite(SPIRIT_POWER_OUT, HIGH);  
}

void wandFiring(){
  if(SPIRIT_FIRE == true){
    digitalWrite(SPIRIT_FIRE_OUT, LOW);
  }
  
  PACK_STATUS = WAND_FIRING;
}

void wandStoppedFiring(){
    if(SPIRIT_FIRE == true){
      digitalWrite(SPIRIT_FIRE_OUT, HIGH);
    }
}

void cyclotronBootUp(){
  cyclotronUpdateRing(CYC_START_SPEED); //Every 100 milliseconds, advance the ring led (slow)
}

void cyclotronRingIdle(){
  cyclotronUpdateRing(CYC_IDLE_SPEED); //Every 100 milliseconds, advance the ring led (slow)
}

void cyclotronRingFiring(){
  cyclotronUpdateRing(CYC_FIRE_SPEED); //Every 25 milliseconds, advance the ring led (fast)
}

//AL and FE style ring cyclotron animation and fading
void cyclotronUpdateRing(uint8_t intSpeed){

  if(currentMillis - startMillis >= intSpeed){
      startMillis = currentMillis;
      fadeToBlackBy(pack_leds, CYCLOTRON_LED_COUNT, 170);    //Dims the LEDs by 64/256 (1/4) and thus sets the trail's length.
      pack_leds[ringPosition] = CRGB::Red; 
      ringPosition++;    //Shifts all LEDs one step in the currently active direction    
      if (ringPosition == CYCLOTRON_LED_COUNT) ringPosition = 0;    //If one end is reached, reset the position to loop around
      FastLED.show();   
    }
}

//84 style cyclotron animation and fading (default)
void cyclotronUpdate() {
  // Get the current time
  uint32_t currentMillis = millis();

  // Check if it's time to update the LEDs
  if (currentMillis - previousMillis >= updateInterval) {
    // Save the last update time
    previousMillis = currentMillis;
    // Light up the current LED
    pack_leds[currentLED] = CRGB::Red;  // Change color to Red

      // Move to the next LED
    currentLED++;
    if (currentLED >= CYCLOTRON84_LED_COUNT) {
      currentLED = 0;  // Loop back to the first LED
    }

    // Show the updated LEDs
    FastLED.show();
  }

  // Handle fading the LEDs out in steps every fadeInterval
  if (currentMillis - fadePreviousMillis >= fadeInterval) {
    fadePreviousMillis = currentMillis;

    // Gradually fade out all LEDs except the current one
    for (int i = 0; i < CYCLOTRON84_LED_COUNT; i++) {
      if (i != currentLED) {
        pack_leds[i].fadeToBlackBy(fadeAmount);
      }
    }

    FastLED.show();
  }

}

//Powercell Animation
void powerCellUpdate() {
  uint32_t currentPCellMillis = millis();

  // Check if it's time to light the next LED
  if (currentPCellMillis - previousPCellMillis >= pwr_interval) {
    previousPCellMillis = currentPCellMillis;

    // Check if we have reached the end of the strip
    if (currentPCellLED >= POWERCELL_LED_COUNT) {
      // Turn off all LEDs
      fill_solid(powercell_leds, POWERCELL_LED_COUNT, CRGB::Black);
      FastLED.show();
      
      // Reset currentLED to start over
      currentPCellLED = 0;
    } else {
      // Light up the current LED (keep previous ones lit)
      powercell_leds[currentPCellLED] = CRGB::Blue;

      // Show the updated LEDs
      FastLED.show();

      // Move to the next LED in the sequence
      currentPCellLED++;
    }
  }
}

void blinkPicoLED(uint32_t intPicoLEDSpeed){
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousPicoLEDMillis >= intPicoLEDSpeed) {
    previousPicoLEDMillis = currentMillis;
    
    // Toggle the LED state
    b_picoled = !b_picoled;
    digitalWrite(BUILTIN_LED, b_picoled ? HIGH : LOW);
  }
}

void blinkStatusLED(){

  unsigned long currentMillis = millis();

  // Check if it's time to turn on the LED (every 10 seconds)
  if (currentMillis - previousBlinkMillis >= blinkinterval) {
    previousBlinkMillis = currentMillis; // Update time
    powercell_leds[0] = CRGB::Blue;          // Turn the LED on
    Serial.println("Blinking Powercell Status LED");
    FastLED.show();

    // Use another non-blocking delay for 500ms ON time
    while (millis() - currentMillis < onDuration) {
      // Do nothing and wait 500ms
    }

    powercell_leds[0] = CRGB::Black;          // Turn the LED off
    FastLED.show();
  }
}



