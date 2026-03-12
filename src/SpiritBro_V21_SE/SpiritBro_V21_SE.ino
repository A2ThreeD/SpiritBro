/**
 * A2ThreeD SpiritBro Adapter - Determines the state of a Hasbro 1984/Spengler wand and uses the information to turn on/off a Spirit Life-Size pack electronics
 * * This code uses code and work derived from the GPStar proton pack source code, re-used according to the GPL license.
 * Specifically, it uses modified sections of the PowerMeter.h code and some from the ProtonPack.ino sections.
 * See https://github.com/gpstar81/GPStar-proton-pack/blob/v6.0/develop/source/ProtonPack/PowerMeter.h for the original code.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <https://www.gnu.org/licenses/>.
 *
 * Code Name: Spirit Bro - Spirit Universal
 * Description: For SpiritBro boards V1, V2, and V2.2 to use with Spirit stock electronics.
 * Date: 8/17/2025
 * Notes: V1.0 - First version of universal code, and splitting into versions for Spirit and Frankengeek board.
 *        V1.01 - Added code to handle the function button input and create a basic menu system for Spirit packs. Long press disables wand shutdown. Short press shuts down pack or starts it. Double-click turns on/off spirit firing sounds.
 *        V1.02 - 8/23/25 - Added EEPROM saving for the settings so they survive reboots. 
 *        V1.03 - 9/13/25 - Tweaked debug settings so that the board boots up faster
*/

// Set to 1 to enable built-in debug messages
#define DEBUG 0

// Define SpiritBro PCB Revision - Default to SB_V2 to support latest hardware
#define SB_V1 1
#define SB_V2 2
#define SB_V21 3
#define SB_VERSION SB_V21

// Define Output PCB Type (Spirit or FrankenGeek)
// Spirit mode outputs shorter power on/off pulse, and has no output for the overheating pin.
#define OUTPUT_SPIRIT 1
#define OUTPUT_FG 2
#define OUTPUT_MODE OUTPUT_SPIRIT //Options OUTPUT_SPIRIT for Spirit PCB or OUTPUT_FG for FrankenGeek

// Conditionally define pin assignments based on PCB Revision.
// V1 PCB has a full size Pi Pico 2040, and the V2+ are based on the WaveShare RP2040 Zero
#if SB_VERSION == 1
  #define PWR_BTN_PIN 6
  #define FIRE_BTN_PIN 7
  #define PACK_LED_PIN 8
  #define PCELL_LED_PIN 9
  #define FUNC_BTN_PIN 14
  #define PI_LED_PIN 25
#elif SB_VERSION == 2
  #define FIRE_BTN_PIN 6
  #define PWR_BTN_PIN 7
  #define PACK_LED_PIN 8
  #define FUNC_BTN_PIN 10
  #define PI_LED_PIN 16
#elif SB_VERSION == 3
  #define EXTRA_IO_PIN_PIN 3
  #define FIRE_BTN_PIN 6
  #define PWR_BTN_PIN 7
  #define PACK_LED_PIN 8
  #define FUNC_BTN_PIN 9
  #define FG_FIRE_PIN 14
  #define FG_DIO1_PIN 15
  #define PI_LED_PIN 16
  #define FG_DIO2_PIN 26
  #define FG_OH_PIN 27
#else
  #error "Unsupported SB_VERSION"
#endif

// Conditionally define  assignments based output selection
#if OUTPUT_MODE == OUTPUT_SPIRIT
  #define PWRPULSE_LENGTH 200
  #define SPIRIT_FIRE false //Set to true if you want the spirt pack to play it's firing audio when the wand is firing.
#elif OUTPUT_MODE == OUTPUT_FG
  #define PWRPULSE_LENGTH 300
  #define SPIRIT_FIRE true //Set to true if you want the spirt pack to play it's firing audio when the wand is firing.
#else
  #error "Unsupported OUTPUT_MODE"
#endif

// Import 3rd-Party Libraries
#include <millisDelay.h>
#include <FastLED.h>
#include <avdweb_Switch.h>
#include <EEPROM.h>

//EEPROM savings for states
#define EEPROM_SIZE 64   // plenty for just 2 flags
#define ADDR_SHUTDOWN_DISABLED 0
#define ADDR_SPIRIT_FIRE_SOUND 1

// Debug macros (replace #ifdef DEBUG blocks with these)
#ifdef DEBUG
  #define DEBUG_PRINT(x)   Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

// Configuration and status variables
bool ledOn = false;
bool b_wand_connected = false;
bool b_wand_on = false;
bool b_use_power_meter = true;
bool b_show_power_data = false;
bool b_wand_firing = false;
bool b_skip_next_power_on_pulse = false;

bool shutdownDisabled = false;
bool spiritFireSoundEnabled = false;
const unsigned long LONG_PRESS_TIME = 1000; // 1 seconds

//Button controller
Switch functionButton(FUNC_BTN_PIN);

//Pico LED blink settings
uint32_t previousPicoLEDMillis = 0;
uint32_t intPicoLEDSpeed = 1000;

//Timer definitions and intialization
uint32_t currentMillis = 0;
uint32_t previousMillis = 0;  
uint32_t currentPCellMillis = 0;
uint32_t previousPCellMillis = 0;  
uint32_t startMillis = 0;
uint32_t bootMillis = 0;
uint32_t fadePreviousMillis = 0;

millisDelay ms_delay_post_2;
millisDelay ms_delay_post_3;

// Define states for the different operational modes of the pack
enum PackState { MODE_IDLE, MODE_BOOTING, MODE_POWERDOWN, MODE_BOOTED, MODE_ON, MODE_OFF, WAND_FIRING, WAND_STOPPED_FIRING };
enum PackState PACK_STATUS;

// Import Additional Local Code (do this after all the variable definitions so they are available)
#include "PowerMeter.h"

// LED Controllers
CRGB status_led[1]; // For the onboard RP2040 LED
CRGB picoLEDColor = CRGB::Green;
CLEDController* statusController = nullptr;

//Initial setup of hardware
void setup() {

  pinMode(PWR_BTN_PIN, OUTPUT);
  digitalWrite(PWR_BTN_PIN, HIGH);

  pinMode(FIRE_BTN_PIN, OUTPUT);
  digitalWrite(FIRE_BTN_PIN, HIGH);

  //Only SpiritBro V1.0 uses a full Pi Pico with a non-RGB LED
  #if SB_VERSION == SB_V1
    pinMode(PI_LED_PIN, OUTPUT);
  #endif

  EEPROM.begin(EEPROM_SIZE);
  loadSettings();   // restore saved flags

  #if DEBUG == 1
    Serial.begin(115200);
    while (!Serial && millis() < 2000); // Wait for Serial to connect
  #endif

  DEBUG_PRINTLN("SpiritBro V1 Spirit Universal Firmware - V1.03 - A2ThreeD 2025");
  DEBUG_PRINT("PCB Revision: ");
  DEBUG_PRINTLN(SB_VERSION);
  DEBUG_PRINT("Output Mode Selection: ");
  DEBUG_PRINTLN(OUTPUT_MODE);
  DEBUG_PRINT("Enable Firing Output: ");
  DEBUG_PRINTLN(spiritFireSoundEnabled);
  DEBUG_PRINTLN("");

  //Initialize all Frankengeek pins to defaults
  #if SB_VERSION == SB_V21
    pinMode(FG_DIO1_PIN, OUTPUT);
    digitalWrite(FG_DIO1_PIN, LOW);

    pinMode(FG_DIO2_PIN, OUTPUT);
    digitalWrite(FG_DIO2_PIN, LOW);

    pinMode(FG_FIRE_PIN, OUTPUT);
    digitalWrite(FG_FIRE_PIN, HIGH);

    pinMode(FUNC_BTN_PIN, HIGH);
    digitalWrite(FUNC_BTN_PIN, HIGH);
  #endif

  //Intialize function button
  functionButton.longPressPeriod = LONG_PRESS_TIME;

  // Initialize the INA219 current monitor on the i2c bus
  if(b_use_power_meter) {
    powerMeterInit();
  }

  //Set the intial pack state
  PACK_STATUS = MODE_OFF;
  DEBUG_PRINTLN("Startup/Setup Complete - Running main loop");

  // Initialize LEDs
  #if SB_VERSION == SB_V2 || SB_VERSION == SB_V21
  statusController = &FastLED.addLeds<WS2812, PI_LED_PIN, GRB>(status_led, 1);
  #endif
  FastLED.setBrightness(128);
  FastLED.clear();
  FastLED.show();
}

// MAIN CODE LOOP
void loop() {
    currentMillis = millis();

    //Check for function button input, and process any presses
    functionButton.poll();
    buttonPressHandler();

    // Check for wand or switch inputs and update state
    checkPowerMeter();

    // Handle state-specific transitions
    switch(PACK_STATUS) {
        case MODE_IDLE:
            DEBUG_PRINTLN("State: MODE_IDLE");
            break;
        case MODE_POWERDOWN:
            DEBUG_PRINTLN("State: MODE_POWERDOWN - Starting pack shutdown");
            PACK_STATUS = MODE_OFF;    // Transition to the final OFF state
            sendPowerPulse();
            DEBUG_PRINTLN("State: MODE_OFF - Pack is now off");
            break;
        case MODE_BOOTING:
            DEBUG_PRINTLN("State: MODE_BOOTING");
            break;
        case MODE_BOOTED:
            break;
        case WAND_FIRING:
            DEBUG_PRINTLN("State: WAND_FIRING");
            break;
        case WAND_STOPPED_FIRING:
            DEBUG_PRINTLN("State: WAND_STOPPED_FIRING");
            PACK_STATUS = MODE_BOOTED; // Immediately transition back to booted
            break;
        case MODE_OFF:;
            break;
    }

    // Update LED based on current state
    updateStatusLED();
    
    // Update all animations based on the current state
    if (PACK_STATUS != MODE_OFF && PACK_STATUS != MODE_IDLE) {
        //Placeholder
    }
    
    // Render all LED changes at once at the end of the loop
    FastLED.show();
}

void packStartup() {
  PACK_STATUS = MODE_BOOTING;
  DEBUG_PRINTLN("Powering up pack");

  //Send power on pulse to Spirit Life-Size Proton Pack electronics
  sendPowerPulse();
  PACK_STATUS = MODE_BOOTED;
}

void packShutdown() {
  if (PACK_STATUS != MODE_POWERDOWN && PACK_STATUS != MODE_OFF) {
    DEBUG_PRINTLN("Initiating shutdown sequence...");
    PACK_STATUS = MODE_POWERDOWN;
  }
}

void sendPowerPulse(){
  if (shutdownDisabled) {
    DEBUG_PRINT("Shutdown/Startup blocked by function button option, ignoring pack shutdown signal");
  } else {
    digitalWrite(PWR_BTN_PIN, LOW);
    delay(PWRPULSE_LENGTH);              // safe here since it's rare
    digitalWrite(PWR_BTN_PIN, HIGH);
  }
}

void wandFiring(){
  if(spiritFireSoundEnabled == true){
    digitalWrite(FIRE_BTN_PIN, LOW);
  }
  PACK_STATUS = WAND_FIRING;
}

void wandStoppedFiring(){

    if(PACK_STATUS != MODE_POWERDOWN && PACK_STATUS != MODE_OFF) {
      PACK_STATUS = WAND_STOPPED_FIRING;
    }
   
    if(spiritFireSoundEnabled == true){
      //Set the pin high to tell the Spirit pack to stop playing the firing sound
      digitalWrite(FIRE_BTN_PIN, HIGH);
    }
}

void pulseFireButton(unsigned long duration) {
  digitalWrite(FIRE_BTN_PIN, LOW);
  delay(duration);              // safe here since it's rare (long press only)
  digitalWrite(FIRE_BTN_PIN, HIGH);
}

void updateStatusLED() {
  uint32_t speed = 1000;
  CRGB color = CRGB::Black;

  switch (PACK_STATUS) {
    case MODE_IDLE:
      speed = 200;
      color = CRGB::Blue;
      break;
    case MODE_BOOTING:
      speed = 50;
      color = CRGB::Blue;
      break;
    case MODE_BOOTED:
      speed = 100;
      color = CRGB::Blue;
      break;
    case MODE_POWERDOWN:
      // could do a fade or just turn off
      speed = 0;
      color = CRGB::Black;
      break;
    case WAND_FIRING:
      speed = 100;
      color = CRGB::Red;
      break;
    case WAND_STOPPED_FIRING:
      speed = 300;
      color = CRGB::Blue;
      break;
    case MODE_OFF:
      speed = intPicoLEDSpeed; // default 1000ms
      color = CRGB::Green;
      break;
  }

  if (speed > 0) {
    blinkPicoLED(speed, color);
  } else {
    // Force LED off if speed is 0
    ledOn = false;
    if (SB_VERSION == SB_V1) {
      digitalWrite(PI_LED_PIN, LOW);
    } else {
      status_led[0] = CRGB::Black;
    }
  }
}

//Blinking the status LED on the PCB 
void blinkPicoLED(uint32_t intPicoLEDSpeed, CRGB picoLEDColor) { 
  unsigned long currentMillis = millis(); 
  if (currentMillis - previousPicoLEDMillis >= intPicoLEDSpeed) { 
    previousPicoLEDMillis = currentMillis; 
    ledOn = !ledOn; 

  //Blink the correct LED based on PCB version (Pi Pico has a standard LED and Pi Pico Zero has RGB) 
  if (SB_VERSION == SB_V1) { 
    digitalWrite(PI_LED_PIN, ledOn ? HIGH : LOW); 
  } else { 
    status_led[0] = ledOn ? picoLEDColor : CRGB::Black; 
    }
  }
}

void flashStatusLED(CRGB color, int flashes, int duration) {
  for (int i = 0; i < flashes; i++) {
    if (SB_VERSION == SB_V1) {
      digitalWrite(PI_LED_PIN, HIGH);
      delay(duration);
      digitalWrite(PI_LED_PIN, LOW);
    } else {
      status_led[0] = color;
      FastLED.show();
      delay(duration);
      status_led[0] = CRGB::Black;
      FastLED.show();
    }
    delay(duration);
  }
}

void pulsePin(int pin, unsigned long ms) {
  digitalWrite(pin, LOW);
  delay(ms);
  digitalWrite(pin, HIGH);
}

void buttonPressHandler() {
  // Long Press - Disable pack shutdown/startup from wand
  if (functionButton.longPress()) {
    shutdownDisabled = !shutdownDisabled;
    DEBUG_PRINT("Long Press -> Shutdown Disabled Mode: ");
    DEBUG_PRINTLN(shutdownDisabled ? "ON" : "OFF");

    if (shutdownDisabled) {
      flashStatusLED(CRGB::Yellow, 2, 200); // two quick yellow flashes
    } else {
      flashStatusLED(CRGB::Purple, 2, 200); // two quick purple flashes
    }

    //Play confirmation sound
    pulsePin(FIRE_BTN_PIN, 500);
  }

    // Single Click - Power on/off pack
    if (functionButton.singleClick()) {
      DEBUG_PRINTLN("Single-Click -> Pulsing Power Pin");
      pulsePin(PWR_BTN_PIN, 200);
    }  

    // Double-Click - Enabled or Disable pack firing sounds
    if (functionButton.doubleClick()) {
      spiritFireSoundEnabled = !spiritFireSoundEnabled;
      DEBUG_PRINT("Double-Click -> Spirit Pack Firing Sound: ");
      DEBUG_PRINTLN(spiritFireSoundEnabled ? "ON" : "OFF");
      saveSettings();   // save to EEPROM
      pulsePin(FIRE_BTN_PIN, 500);
    }  
}

void saveSettings() {
  EEPROM.write(ADDR_SPIRIT_FIRE_SOUND, spiritFireSoundEnabled);
  EEPROM.commit();  // store to flash
}

void loadSettings() {
  spiritFireSoundEnabled = EEPROM.read(ADDR_SPIRIT_FIRE_SOUND);
}


