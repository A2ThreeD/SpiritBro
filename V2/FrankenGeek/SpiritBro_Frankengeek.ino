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
 * Date: 7/7/2025
 * Notes: V1.8 - Consolidated Cyclotron and Power Cell LEDs into a single strip.
 *        V2.0 - Added conditional logic to account for Spirit or FrankenGeek outputs, as well as support V1 and V2 PCBs both.
 *        V2.1 - 7/7/25 - Cleaned up conditional logic and added overheat pin definition.
 *        V2.2 - 7/24/25 - Added FG overheat pin assignment.
*/

// Set to 1 to enable built-in debug messages
#define DEBUG 1

// Debug macros
#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

//LED Size definitions for Cyclotrons
#define CYCLOTRON_AL_LED_COUNT_FS 40    //Number of LEDs in ring for AL/FE modes
#define CYCLOTRON_AL_LED_COUNT_DLX 30   //Number of LEDs in ring for Deluxe (80%) scale packs
#define CYCLOTRON_84_LED_COUNT 4        //Defaults to 4 cyclotron LEDs for 84/89 modes

// Define which version you're building - 1984 is 4 LED Cyclotron, and 2021 is a ring
#define PACK_YEAR 1984  // Change to 2021 for Afterlife version
#define PACK_SIZE 100   // Change to 80 for 80% scale pack

// Define SpiritBro PCB Revision - Default to SB_V2 to support latest hardware
#define SB_V1 1
#define SB_V2 2
#define SB_VERSION SB_V2

// Define Output PCB Type (Spirit or FrankenGeek)
// Spirit mode outputs shorter power on/off pulse, and has no output for the overheating pin.
#define OUTPUT_SPIRIT 1
#define OUTPUT_FG 2
#define OUTPUT_MODE OUTPUT_FG //Options OUTPUT_SPIRIT for Spirit PCB or OUTPUT_FG for FrankenGeek

// Define Wand Model Options - For future development and tuning
#define WAND_TYPE_1984 1
#define WAND_TYPE_SPENGLER 2
#define WAND_TYPE_ZAPNBLAST 3
#define WAND_MODEL WAND_TYPE_1984 // Options: WAND_TYPE_1984, WAND_TYPE_SPENGLER, WAND_TYPE_ZAPNBLAST

// Conditionally define pin assignments based on PCB Revision.
// V1 PCB has a full size Pi Pico 2040, and the V2+ are based on the WaveShare RP2040 Zero
#if SB_VERSION == 1
  #define PACK_LED_PIN 8
  #define POWERCELL_LED_PIN 9
  #define POWER_BTN_OUT 6
  #define FIRE_BTN_OUT 7
  #define BUILTIN_LED_PIN 25
#elif SB_VERSION == 2
  #define OVERHEAT_PIN 9
  #define PACK_LED_PIN 8
  #define POWER_BTN_OUT 7
  #define FIRE_BTN_OUT 6
  #define BUILTIN_LED_PIN 16
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

// Conditionally define LED counts based on pack scale
#if PACK_SIZE == 100
  #define POWERCELL_LED_COUNT 15
  #define CYCLOTRON_AL_LED_COUNT  CYCLOTRON_AL_LED_COUNT_FS
#elif PACK_SIZE == 80
  #define POWERCELL_LED_COUNT 12
  #define CYCLOTRON_AL_LED_COUNT  CYCLOTRON_AL_LED_COUNT_DLX
#else
  #error "Unsupported PACK_SIZE. Please use 100 or 80."
#endif

// Conditionally define TOTAL_LED_COUNT
#if PACK_YEAR == 1984
  #define TOTAL_LED_COUNT (POWERCELL_LED_COUNT + CYCLOTRON_84_LED_COUNT)
  #define CYCLOTRON_LED_COUNT CYCLOTRON_84_LED_COUNT
#elif PACK_YEAR == 2021
  #define TOTAL_LED_COUNT (POWERCELL_LED_COUNT + CYCLOTRON_AL_LED_COUNT)
  #define CYCLOTRON_LED_COUNT CYCLOTRON_AL_LED_COUNT
#else
  #error "Unsupported PACK_YEAR. Please use 1984 or 2020."
#endif

// I2S Audio Output PINS for DAC / Amps (Future PCB revisions)
#define I2S_BCLK 11
#define I2S_LRCK 12
#define I2S_DOUT 13

// Main Configuration options
#define POWERCELL_START_INDEX CYCLOTRON_LED_COUNT
#define CYCLOTRON_START_INDEX 0
#define CYC_START_SPEED 255 //Speed for cyclotron during boot (max 255 is slowest)
#define CYC_IDLE_SPEED 10   //Speed for cyclotron during idle, lower number is faster (default 100) 
#define CYC_84_SPEED 400    //Speed for the standard 4-LED Cyclotron to rotate at
#define CYC_FIRE_SPEED 5    //Speed for cyclotron during firing, lower number is faster (default 25) 
#define CYC_BOOT_LEN 9000   //Number of milliseconds to fade in cyclotron ring before boot (2.9sec for Hasbro 1984 wand)
#define BRIGHTNESS 255      //Maximum brightness
#define CYCLOTRON_TRAIL_FADE 170

//Conditionally define the wand power-on threshold based on WAND_MODEL
#if WAND_MODEL == WAND_TYPE_1984 || WAND_MODEL == WAND_TYPE_SPENGLER
  //const float f_wand_power_on_threshold = 0.65;
#elif WAND_MODEL == WAND_TYPE_ZAPNBLAST
 //const float f_wand_power_on_threshold = 0.10;
#else
  #error "Unsupported WAND_MODEL. Please use WAND_1984, WAND_SPENGLER, or WAND_ZAPNBLAST."
#endif

// Import 3rd-Party Libraries
#include <Arduino.h>
#include <millisDelay.h>
#include <FastLED.h>
#include <AudioFileSourceID3.h>
#include <AudioGeneratorWAV.h>
#include <AudioOutputI2S.h>
#include <AudioFileSourceLittleFS.h>

// Configuration and status variables
bool b_afterlifemode = false;   //Determines which version of the cyclotron animation is running. If afterlifemode is true then it assumes a ring LED, otherwise uses 4 LEDs.
bool b_cyclotronbooted = false;
bool ledOn = false;
bool b_wand_connected = false;
bool b_wand_syncing = false;
bool b_wand_on = false;
bool b_use_power_meter = true;
bool b_show_power_data = false;
bool b_wand_firing = false;
bool b_firing_intensify = false;
bool b_overheating = false;

uint8_t i_wand_power_level = 1; // Power level of the wand.

// Boot-up LED animation speeds
uint32_t intBootSpeed = CYC_START_SPEED;
uint32_t cyc_boot_interval = CYC_BOOT_LEN / 255; //calculate the delay between ramp up steps (255 steps)

// LED Controllers
CRGB leds[TOTAL_LED_COUNT];
CRGB status_led[1]; // For the onboard RP2040 LED
CRGB picoLEDColor = CRGB::Green;

CLEDController* stripController = nullptr;
CLEDController* statusController = nullptr;

// LED animation settings
const uint32_t pwr_interval = 55;      // Interval between powercell climbing steps
const uint32_t updateInterval = CYC_84_SPEED;   // Interval in milliseconds for LED updates
uint8_t currentCyclotronLED = 0;     // LED position tracker
uint8_t currentPCellLED = 0;
uint8_t fadeInterval = 30;  // Interval between fade steps in 84 mode
uint8_t fadeAmount = 30;    // Amount to fade each time in 84 mode
uint8_t ringPosition = 0;   //Position of the cyclotron LED ring animation

//Status LED blink settings
uint32_t previousBlinkMillis = 0; // Store last time the LED was updated
uint32_t blinkinterval = 10000;      // 10 seconds interval
uint32_t onDuration = 500;      // 500ms ON duration
bool statusLedIsOn = false;
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
bool isPulsing = false;

millisDelay powerPulseTimer;
millisDelay ms_delay_post_2;
millisDelay ms_delay_post_3;

//  Audio Playback Setup 
AudioFileSourceLittleFS *file = nullptr;
AudioGeneratorWAV *wav = nullptr;
AudioOutputI2S *out = nullptr;
AudioFileSource *id3 = nullptr;

// Audio File name and location - Stored in LitteFS partition and uploaded
const char* audioFiles[] = {
    "/bootup.wav",  
    "/idlehum.wav",
    "/shutdown.wav"  
};
const int NUM_AUDIO_FILES = sizeof(audioFiles) / sizeof(audioFiles[0]);
int currentAudioIndex = -1;
bool isLooping = false;

// Define states for the different operational modes of the pack
enum PackState { ACTION_IDLE, ACTION_BOOTING, ACTION_POWERDOWN, ACTION_BOOTED, MODE_ON, MODE_OFF, WAND_FIRING, WAND_STOPPED_FIRING };
enum PackState PACK_STATUS;

// Import Additional Local Code (do this after all the variable definitions so they are available)
#include "PowerMeter.h"

// Audio - Stop all playing audio 
void stopAudio() {
    if (wav && wav->isRunning()) {
        wav->stop();
    }
    // Free memory of the previous sound
    delete wav;  wav = nullptr;
    delete id3;  id3 = nullptr;
    delete file; file = nullptr;
}

// Audio - Play WAV files
void playAudioByIndex(int index, bool loop = false) {
    if (index < 0 || index >= NUM_AUDIO_FILES) {
        
        #ifdef DEBUG
        Serial.println("Invalid audio index!");
        #endif

        return;
    }

    // Stop and clean up any previously playing audio
    stopAudio();

    currentAudioIndex = index;
    isLooping = loop;
    const char* filename = audioFiles[currentAudioIndex];
    
    #ifdef DEBUG
    Serial.printf("Preparing: %s | Loop: %s\n", filename, isLooping ? "YES" : "NO");
    #endif

    file = new AudioFileSourceLittleFS(filename);
    id3 = new AudioFileSourceID3(file);
    wav = new AudioGeneratorWAV();
    wav->begin(id3, out);
    
    #ifdef DEBUG
    Serial.println("Audio ready.");
    #endif
}

//Initial setup of hardware
void setup() {

  pinMode(POWER_BTN_OUT, OUTPUT);
  digitalWrite(POWER_BTN_OUT, HIGH);

  pinMode(FIRE_BTN_OUT, OUTPUT);
  digitalWrite(FIRE_BTN_OUT, HIGH);

  pinMode(OVERHEAT_PIN, OUTPUT);
  digitalWrite(OVERHEAT_PIN, LOW);  

  #if SB_VERSION == SB_V1
    pinMode(BUILTIN_LED_PIN, OUTPUT);
  #endif

  Serial.begin(115200);
  while (!Serial && millis() < 2000); // Wait for Serial to connect
  
  //Set the afterlife mode based on PACK_YEAR  
  #if PACK_YEAR == 2021
    b_afterlifemode = true;
  #else
    b_afterlifemode = false;
  #endif

  #ifdef DEBUG
    Serial.println("SpiritBro - V2.0 - A2ThreeD 2025");
    Serial.print("PCB Revision: ");
    Serial.println(SB_VERSION);
    Serial.print("Output Mode Selection: ");
    Serial.println(OUTPUT_MODE);
    Serial.print("Build Year Selection: ");
    Serial.println(PACK_YEAR);
    Serial.print("Enable Firing Output: ");
    Serial.println(SPIRIT_FIRE);
    Serial.print("Cyclotron LED count: ");
    Serial.println(CYCLOTRON_LED_COUNT);
    Serial.print("Power Cell start index: ");
    Serial.println(POWERCELL_START_INDEX);
    Serial.println("");
  #endif

  // Audio Init
  #ifdef DEBUG
  Serial.println("Configuring Audio DAC");
  #endif

  out = new AudioOutputI2S();
  out->SetPinout(I2S_BCLK, I2S_LRCK, I2S_DOUT);
  out->SetGain(1.0);

  #if SB_VERSION == SB_V2
    statusController = &FastLED.addLeds<WS2812, BUILTIN_LED_PIN, RGB>(status_led, 1);
  #endif

  stripController = &FastLED.addLeds<WS2812, PACK_LED_PIN, GRB>(leds, TOTAL_LED_COUNT);
  FastLED.setBrightness(128);

  // Clear LEDs after self-test
  FastLED.clear();
  FastLED.show();

  // Initialize the INA219 current monitor on the i2c bus
  if(b_use_power_meter) {
    powerMeterInit();
  }

  //Set the intial pack state
  PACK_STATUS = MODE_OFF;
}

// Check to see if power on pulse is being processed
void checkPowerPulse() {
    if (isPulsing && powerPulseTimer.justFinished()) {
        digitalWrite(POWER_BTN_OUT, HIGH); // End the pulse
        isPulsing = false;
    }
}

// MAIN CODE LOOP
void loop() {
    currentMillis = millis();

    // Handle non-blocking power pulse
    checkPowerPulse();
    
    // Check for wand or switch inputs and update state
    checkPowerMeter();

    // Handle state-specific transitions and audio
    switch(PACK_STATUS) {
        case ACTION_IDLE:
            #ifdef DEBUG
            Serial.println("State: ACTION_IDLE");
            #endif
            blinkPicoLED(200, CRGB::Blue);
            break;
        case ACTION_POWERDOWN:
          
            // We are in this state waiting for the shutdown sound to finish.
            if (!wav || !wav->isRunning()) {
                #ifdef DEBUG
                Serial.println("Shutdown sound finished. Pack is now off.");
                #endif

                // Reset boot variables and indexes
                b_cyclotronbooted = false; 
                currentPCellLED = 0;
                currentCyclotronLED = 0;

                PACK_STATUS = MODE_OFF;    // Transition to the final OFF state
                sendPowerPulse();

                // Clear LEDs
                FastLED.clear();
                FastLED.show();
            }
            break;
        case ACTION_BOOTING:
            #ifdef DEBUG
            Serial.println("State: ACTION_BOOTING");
            #endif

            blinkPicoLED(50, CRGB::Blue);
            playAudioByIndex(0, false);

            break;
        case ACTION_BOOTED:

            blinkPicoLED(100, CRGB::Blue);
            if (wav && !wav->isRunning()) {
              playAudioByIndex(1, true); // Play idle hum
            }
            break;
        case WAND_FIRING:
            //#ifdef DEBUG
            //Serial.println("State: WAND_FIRING");
            //#endif

            stopAudio();
            blinkPicoLED(100, CRGB::Red);
            break;
        case WAND_STOPPED_FIRING:
            #ifdef DEBUG
            Serial.println("State: WAND_STOPPED_FIRING");
            #endif

            blinkPicoLED(300, CRGB::Blue);
            playAudioByIndex(1, true); // Play idle hum

            PACK_STATUS = ACTION_BOOTED; // Immediately transition back to booted
            break;
        case MODE_OFF:

            blinkStatusLED();
            blinkPicoLED(intPicoLEDSpeed, CRGB::Green);
            stopAudio();
            break;
    }

    // Update all animations based on the current state
    if (PACK_STATUS != MODE_OFF && PACK_STATUS != ACTION_IDLE) {
        cyclotronStates();
        powerCellUpdate();
    }
    
    // Render all LED changes at once at the end of the loop
    FastLED.show();

    // Handle audio processing
    if (wav && wav->isRunning() && !wav->loop()) {
        //stopAudio(); // Clean up if audio finished and isn’t looping
    }
}

void packStartup() {
  PACK_STATUS = ACTION_BOOTING;

  #ifdef DEBUG
  Serial.println("Powering up pack");
  #endif
  
  //Send power on pulse to Spirit Life-Size Proton Pack electronics
  sendPowerPulse();
}

void packShutdown() {
  // This function is called ONCE to start the shutdown sequence.
  if(PACK_STATUS != ACTION_POWERDOWN && PACK_STATUS != MODE_OFF) {
    #ifdef DEBUG
      Serial.println("Initiating shutdown sequence...");
    #endif

    PACK_STATUS = ACTION_POWERDOWN; // Set state to wait for sound
    
    sendPowerPulse();
    playAudioByIndex(2, false); // Play shutdown sound (index 2)
  }

}

void sendPowerPulse(){
  if (!isPulsing) {
      digitalWrite(POWER_BTN_OUT, LOW);
      powerPulseTimer.start(PWRPULSE_LENGTH);
      isPulsing = true;
  }
}

void wandFiring(){
  if(SPIRIT_FIRE == true){
    digitalWrite(FIRE_BTN_OUT, LOW);
  }
  PACK_STATUS = WAND_FIRING;
}

void wandStoppedFiring(){

    if(PACK_STATUS != ACTION_POWERDOWN && PACK_STATUS != MODE_OFF) {
      PACK_STATUS = WAND_STOPPED_FIRING;
    }
   
    if(SPIRIT_FIRE == true){
      //Set the pin high to tell the Spirit pack to stop playing the firing sound
      digitalWrite(FIRE_BTN_OUT, HIGH);
    }
}

// New function to consolidate cyclotron logic
void cyclotronStates() {
    if (b_afterlifemode) {
        switch (PACK_STATUS) {
            case ACTION_BOOTING:
                if(currentMillis - bootMillis >= cyc_boot_interval){
                  bootMillis = currentMillis;
                  intBootSpeed--;
                  if (intBootSpeed <= CYC_IDLE_SPEED){
                    b_cyclotronbooted = true;
                    PACK_STATUS = ACTION_BOOTED; // Transition state here
                  }
                }
                cyclotronUpdateRing(intBootSpeed);
                break;
            case WAND_FIRING:
                cyclotronUpdateRing(CYC_FIRE_SPEED);
                break;
            case ACTION_BOOTED:
            case WAND_STOPPED_FIRING: // Falls through
            default: // Idle is the default state
              cyclotronUpdateRing(CYC_IDLE_SPEED);
              break;
        }
    } else {
        switch (PACK_STATUS) {
            case ACTION_BOOTING:
              b_cyclotronbooted = true;
              PACK_STATUS = ACTION_BOOTED; // Transition state here
              cyclotron84Update();
              break;
            case WAND_FIRING:
              cyclotron84Update();
              break;
            case ACTION_BOOTED:
              cyclotron84Update();
              break;
            case WAND_STOPPED_FIRING: // Falls through
              cyclotron84Update();
              break;
            default: // Idle is the default state
              cyclotron84Update();
              break;
        }
    }
}

//AL and FE style ring cyclotron animation and fading
void cyclotronUpdateRing(uint8_t intSpeed){
  if(currentMillis - startMillis >= intSpeed){
      startMillis = currentMillis;

      fadeToBlackBy(&leds[CYCLOTRON_START_INDEX], CYCLOTRON_LED_COUNT, CYCLOTRON_TRAIL_FADE);    
      leds[CYCLOTRON_START_INDEX + ringPosition] = CRGB::Red; 
      ringPosition++;      
      if (ringPosition >= CYCLOTRON_LED_COUNT) ringPosition = 0;
    }
}

//84 style cyclotron animation and fading (default)
void cyclotron84Update() {
  if (currentMillis - previousMillis >= updateInterval) {
    previousMillis = currentMillis;

    leds[CYCLOTRON_START_INDEX + currentCyclotronLED] = CRGB::Red;
    currentCyclotronLED++;
    if (currentCyclotronLED >= CYCLOTRON_LED_COUNT) {
      currentCyclotronLED = 0;
    }
  }

  if (currentMillis - fadePreviousMillis >= fadeInterval) {
    fadePreviousMillis = currentMillis;
    for (int i = 0; i < CYCLOTRON_LED_COUNT; i++) {
      if (i != currentCyclotronLED) {
        leds[CYCLOTRON_START_INDEX + i].fadeToBlackBy(fadeAmount);
      }
    }
  }
}

//Powercell Animation
void powerCellUpdate() {
  if (currentMillis - previousPCellMillis >= pwr_interval) {
    previousPCellMillis = currentMillis;

    if (currentPCellLED >= POWERCELL_LED_COUNT) {
      fill_solid(&leds[POWERCELL_START_INDEX], POWERCELL_LED_COUNT, CRGB::Black);
      currentPCellLED = 0;
    } else {
      leds[POWERCELL_START_INDEX + currentPCellLED] = CRGB::Blue;
      currentPCellLED++;
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
      digitalWrite(BUILTIN_LED_PIN, ledOn ? HIGH : LOW);  
    } else {
      status_led[0] = ledOn ? picoLEDColor : CRGB::Black;  
    }
  }
}

//Blinking the status LED on the powercell to indicate power is on to the board but the system isn't running.
void blinkStatusLED() {

  unsigned long currentMillis = millis();
  if (!statusLedIsOn && (currentMillis - previousBlinkMillis >= blinkinterval)) {

    previousBlinkMillis = currentMillis;
    leds[POWERCELL_START_INDEX] = CRGB::Blue;
    statusLedIsOn = true;
  }

  if (statusLedIsOn && (currentMillis - previousBlinkMillis >= onDuration)) {

    leds[POWERCELL_START_INDEX] = CRGB::Black;
    statusLedIsOn = false;
  }
}



