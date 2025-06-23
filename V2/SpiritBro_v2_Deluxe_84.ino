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
 * * Version: 1.8
 * Date: 6/22/2025
 * Notes: V1.8 - Consolidated Cyclotron and Power Cell LEDs into a single strip.
*/

//LED Size definitions
#define CYCLOTRON_AL_LED_COUNT_FS 40       //Number of LEDs in ring for AL/FE modes
#define CYCLOTRON_AL_LED_COUNT_DLX  30  //Number of LEDs in ring for Deluxe (80%) scale packs
#define CYCLOTRON_84_LED_COUNT 4        //Defaults to 4 cyclotron LEDs for 84/89 modes

// Define which version you're building - 1984 is 4 LED Cyclotron, and 2021 is a ring
#define PACK_YEAR 1984  // Change to 2021 for Afterlife version
#define PACK_SIZE 100   // Change to 80 for 80% scale pack
#define WAND_MODEL WAND_1984  // Options: WAND_1984, WAND_SPENGLER, WAND_ZAPNBLAST

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

// Pin assignments
#define PACK_LED_PIN 8
#define RP2040_LED_PIN 16
#define SPIRIT_POWER_OUT 7
#define SPIRIT_FIRE_OUT 6
#define BUILTIN_LED 25

// ======== I2S Audio Pins ========
#define I2S_BCLK 11
#define I2S_LRCK 12
#define I2S_DOUT 13

#define POWERCELL_START_INDEX CYCLOTRON_LED_COUNT
#define CYCLOTRON_START_INDEX 0

#define CYC_START_SPEED 255 //Speed for cyclotron during boot (max 255 is slowest)
#define CYC_IDLE_SPEED 10  //Speed for cyclotron during idle, lower number is faster (default 100) 
#define CYC_84_SPEED 400
#define CYC_FIRE_SPEED 5   //Speed for cyclotron during firing, lower number is faster (default 25) 
#define CYC_BOOT_LEN 9000  //Number of milliseconds to fade in cyclotron ring before boot (2.9sec for Hasbro 1984 wand)
#define BRIGHTNESS 255    // Maximum brightness
#define CYCLOTRON_TRAIL_FADE 170

//Mode Settings
#define SPIRIT_FIRE false //Set to true if you want the spirt pack to play it's firing audio when the wand is firing.

#define DEBUG //Comment this line out to disable debugging via serial

// Status variables
bool b_afterlifemode = false;   //Determines which version of the cyclotron animation is running. If afterlifemode is true then it assumes a ring LED, otherwise uses 4 LEDs.
bool b_wand_firing = false;
bool b_wand_connected = false;
bool b_wand_on = false;
bool b_show_power_data = false;
bool b_cyclotronbooted = false;
bool ledOn = false;

const uint8_t i_wand_power_level_max = 5; // Max power level of the wand.
uint8_t i_wand_power_level = 1; // Power level of the wand.

//Import needed libraries
#include <millisDelay.h>
#include <FastLED.h>
#include <AudioFileSourceID3.h>
#include <AudioGeneratorWAV.h>
#include <AudioOutputI2S.h>
#include <AudioFileSourceLittleFS.h>
#include <INA219.h>

// Boot-up LED animation speeds
uint32_t intBootSpeed = CYC_START_SPEED;
uint32_t cyc_boot_interval = CYC_BOOT_LEN / 255; //calculate the delay between ramp up steps (255 steps)

// LED animation settings
const uint32_t pwr_interval = 55;      // Interval between powercell climbing steps
const uint32_t updateInterval = CYC_84_SPEED;   // Interval in milliseconds for LED updates
uint8_t currentLED = 0;     // LED position tracker
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

// <<< MODIFIED: Single LED array for the entire strip
CRGB leds[TOTAL_LED_COUNT];
CRGB status_led[1]; // For the onboard RP2040 LED
CRGB picoLEDColor = CRGB::Green;

CLEDController* stripController = nullptr;
CLEDController* statusController = nullptr;

// <<< MODIFIED: Obsolete - FastLED.addLeds is now handled directly in setup()
// CLEDController &cyclotronController = FastLED.addLeds<NEOPIXEL, PACK_LED_PIN>(pack_leds, CYCLOTRON_LED_COUNT);

//Timers
uint32_t currentMillis = 0;
uint32_t previousMillis = 0;  
uint32_t currentPCellMillis = 0;
uint32_t previousPCellMillis = 0;  
uint32_t startMillis = 0;
uint32_t bootMillis = 0;
uint32_t fadePreviousMillis = 0;
millisDelay powerPulseTimer;
bool isPulsing = false;


// ======== Audio Setup ========
AudioFileSourceLittleFS *file = nullptr;
AudioGeneratorWAV *wav = nullptr;
AudioOutputI2S *out = nullptr;
AudioFileSource *id3 = nullptr;

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

// Custom values for calibrating the current-sensing device.
#define SHUNT_R     0.1  // Shunt resistor in ohms (default: 0.1ohm)
#define SHUNT_MAX_V 0.2  // Max voltage across shunt (default: 0.2V)
#define BUS_MAX_V   16.0 // Sets max based on expected range (< 16V)
#define MAX_CURRENT 2.0  // Sets the expected max amperage draw (2A)

// General Variables
INA219 monitor; // Power monitor object on i2c bus using the INA219 chip.
bool b_power_meter_available = false; // Whether a power meter device exists on i2c bus, per setup() -> powerMeterInit()
const uint16_t f_wand_power_up_delay = 1000; // How long to wait and ignore any wand firing events after initial power-up (ms).
const float f_ema_alpha = 0.2; // Smoothing factor (<1) for Exponential Moving Average (EMA) [Lower Value = Smoother Averaging].

//Conditionally define the wand power-on threshold based on WAND_MODEL
#if WAND_MODEL == WAND_1984 || WAND_MODEL == WAND_SPENGLER
  const float f_wand_power_on_threshold = 0.65;
#elif WAND_MODEL == WAND_ZAPNBLAST
  const float f_wand_power_on_threshold = 0.20;
#else
  #error "Unsupported WAND_MODEL. Please use WAND_1984, WAND_SPENGLER, or WAND_ZAPNBLAST."
#endif

// Special Timers and Timeouts
millisDelay ms_powerup_debounce; // Timer to lock out firing when the wand powers on.

// Define an object which can store
struct PowerMeter {
  const static uint16_t StateChangeDuration = 80; // Duration (ms) for a current change to persist for action
  const static float StateChangeThreshold; // Minimum change in current (A) to consider as a potential state change
  float ShuntVoltage = 0; // mV - Millivolts read to calculate the amperage draw across the shunt resistor
  float ShuntCurrent = 0; // A - The current (amperage) reading via the shunt resistor
  float BusVoltage = 0;   // mV - Voltage reading from the measured device
  float BattVoltage = 0;  // V - Reference voltage from device power source
  float BusPower = 0;     // W - Calculation of power based on the bus mV*A values
  float AmpHours = 0;     // Ah - An estimation of power consumed over regular intervals
  float RawPower = 0;     // W - Calculation of power based on raw V*A values (non-smoothed)
  float AvgPower = 0;     // A - Running average from the RawPower value (smoothed)
  float LastAverage = 0;  // A - Last average used when determining a state change
  unsigned int PowerReadDelay = (int) (StateChangeDuration / 4); // How often (ms) to read levels for changes
  unsigned long StateChanged = 0; // Time when a potential state change was detected
  unsigned long LastRead = 0;     // Used to calculate Ah consumed since battery power-on
  unsigned long ReadTick = 0;     // Difference of current read time - last read
  millisDelay ReadTimer;          // Timer for reading latest values from power meter
};

// Set the static constant for considering a "change" based on latest current reading average.
// IOW, if we measure a difference of this much since the last average, the user initiated input.
const float PowerMeter::StateChangeThreshold = 0.2;

// Create instances of the PowerMeter object.
PowerMeter wandReading;

// ======== Audio Control ========
void stopAudio() {
    if (wav && wav->isRunning()) {
        wav->stop();
    }
    // Free memory of the previous sound
    delete wav;  wav = nullptr;
    delete id3;  id3 = nullptr;
    delete file; file = nullptr;
}

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

  pinMode(SPIRIT_POWER_OUT, OUTPUT);
  digitalWrite(SPIRIT_POWER_OUT, HIGH);

  pinMode(SPIRIT_FIRE_OUT, OUTPUT);
  digitalWrite(SPIRIT_FIRE_OUT, HIGH);

  pinMode(BUILTIN_LED, OUTPUT);

  Serial.begin(115200);
  while (!Serial && millis() < 2000); // Wait for Serial to connect
  
  //Set the afterlife mode based on PACK_YEAR  
  #if PACK_YEAR == 2021
    b_afterlifemode = true;
  #else
    b_afterlifemode = false;
  #endif

  #ifdef DEBUG
    Serial.println("SpiritBro V1.7 - Combined LED Strip - (c) A2ThreeD");
    Serial.print("Build Year: ");
    Serial.println(PACK_YEAR);
    Serial.print("Cyclotron LED count: ");
    Serial.println(CYCLOTRON_LED_COUNT);
    Serial.print("Power Cell start index: ");
    Serial.println(POWERCELL_START_INDEX);
  #endif

  // Audio Init
  #ifdef DEBUG
  Serial.println("Setting up audio amp...");
  #endif

  out = new AudioOutputI2S();
  out->SetPinout(I2S_BCLK, I2S_LRCK, I2S_DOUT);
  out->SetGain(1.0);

  statusController = &FastLED.addLeds<WS2812, RP2040_LED_PIN, RGB>(status_led, 1);
  stripController = &FastLED.addLeds<WS2812, PACK_LED_PIN, GRB>(leds, TOTAL_LED_COUNT);
  FastLED.setBrightness(128);

  //#ifdef DEBUG
  //Serial.println("Power-On Self-Test: Power Cell LEDs");
  //#endif

  // Light up Power Cell LEDs white for 2 seconds as self-test
  //fill_solid(&leds[POWERCELL_START_INDEX], POWERCELL_LED_COUNT, CRGB::Blue);
  //FastLED.show();
  //delay(2000);

  // Clear LEDs after self-test
  FastLED.clear();
  FastLED.show();

  // Initialize the power metering on the i2c bus
  powerMeterInit();

  //Set the intial pack state
  PACK_STATUS = MODE_OFF;
}

void checkPowerPulse() {
    if (isPulsing && powerPulseTimer.justFinished()) {
        digitalWrite(SPIRIT_POWER_OUT, HIGH); // End the pulse
        isPulsing = false;
    }
}

//The main loop of the code for checking wand state and performing actions based on the state
void loop() {
    currentMillis = millis();

    // Handle non-blocking power pulse
    checkPowerPulse();
    
    // 1. Check for wand or switch inputs and update state
    checkPowerMeter();

    // 2. Handle state-specific transitions and audio
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
                b_cyclotronbooted = false; // Reset boot variable

                // Clear LEDs
                FastLED.clear();
                FastLED.show();

                PACK_STATUS = MODE_OFF;    // Transition to the final OFF state
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
            #ifdef DEBUG
            Serial.println("State: WAND_FIRING");
            #endif

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

    // 3. Update all animations based on the current state
    if (PACK_STATUS != MODE_OFF && PACK_STATUS != ACTION_IDLE) {
        cyclotronStates();
        powerCellUpdate();
    }
    
    // 4. Render all LED changes at once at the end of the loop
    FastLED.show();

    // 5. Handle audio processing
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
    
    playAudioByIndex(2, false); // Play shutdown sound (index 2)
    wandStoppedFiring();
    sendPowerPulse();
    FastLED.clear(); // Clear LEDs immediately
    FastLED.show();  // Push the cleared state to the LEDs
  }
}

void sendPowerPulse(){
  if (!isPulsing) {
      digitalWrite(SPIRIT_POWER_OUT, LOW);
      powerPulseTimer.start(200);
      isPulsing = true;
  }
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

    leds[CYCLOTRON_START_INDEX + currentLED] = CRGB::Red;
    currentLED++;
    if (currentLED >= CYCLOTRON_LED_COUNT) {
      currentLED = 0;
    }
  }

  if (currentMillis - fadePreviousMillis >= fadeInterval) {
    fadePreviousMillis = currentMillis;
    for (int i = 0; i < CYCLOTRON_LED_COUNT; i++) {
      if (i != currentLED) {
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

void blinkPicoLED(uint32_t intPicoLEDSpeed, CRGB picoLEDColor) {
  unsigned long currentMillis = millis();
  if (currentMillis - previousPicoLEDMillis >= intPicoLEDSpeed) {
    previousPicoLEDMillis = currentMillis;
    ledOn = !ledOn;
    status_led[0] = ledOn ? picoLEDColor : CRGB::Black;
  }
}

void blinkStatusLED() {

  unsigned long currentMillis = millis();
  if (!statusLedIsOn && (currentMillis - previousBlinkMillis >= blinkinterval)) {
    #ifdef DEBUG
    Serial.println("Status LED on");
    #endif
    previousBlinkMillis = currentMillis;
    leds[POWERCELL_START_INDEX] = CRGB::Blue;
    statusLedIsOn = true;
  }

  if (statusLedIsOn && (currentMillis - previousBlinkMillis >= onDuration)) {
    // <<< MODIFIED: Turn off the first LED of the power cell segment
   #ifdef DEBUG
    Serial.println("Status LED off");
    #endif

    leds[POWERCELL_START_INDEX] = CRGB::Black;
    statusLedIsOn = false;
  }
}

// Configure and calibrate the power meter device.
void powerMeterConfig() {
  #ifdef DEBUG
    Serial.print(F("Configuring Power Meter... "));
  #endif
  
  monitor.configure(INA219::RANGE_16V, INA219::GAIN_1_40MV, INA219::ADC_64SAMP, INA219::ADC_64SAMP, INA219::CONT_SH_BUS);
  monitor.calibrate(SHUNT_R, SHUNT_MAX_V, BUS_MAX_V, MAX_CURRENT);

}

// Initialize the power meter device on the i2c bus.
void powerMeterInit() {

    uint8_t i_monitor_status = monitor.begin();
    
    #ifdef DEBUG
      Serial.print(F("Power Meter Result: "));
      Serial.print(i_monitor_status);
    #endif

    if (i_monitor_status == 0) {
      #ifdef DEBUG
        Serial.println(F(" (Success)"));
      #endif
      
      b_power_meter_available = true;
      powerMeterConfig();
      wandReading.LastRead = millis();
      wandReading.ReadTimer.start(wandReading.PowerReadDelay);
    }
    else {
      #ifdef DEBUG
        Serial.println(F(" (Failed - Check Wiring)"));
      #endif
    } 
}

// Perform a reading of values from the power meter for the wand.
void doWandPowerReading() {
  if (b_power_meter_available) {
    wandReading.ShuntVoltage = monitor.shuntVoltage();
    wandReading.ShuntCurrent = monitor.shuntCurrent();
    wandReading.BusVoltage = monitor.busVoltage();
    wandReading.BusPower = monitor.busPower();

    wandReading.BattVoltage = wandReading.BusVoltage + wandReading.ShuntVoltage;
    wandReading.RawPower = wandReading.BattVoltage * wandReading.ShuntCurrent;
    wandReading.AvgPower = (f_ema_alpha * wandReading.RawPower) + ((1 - f_ema_alpha) * wandReading.AvgPower);

    unsigned long i_new_time = millis();
    wandReading.ReadTick = i_new_time - wandReading.LastRead;
    wandReading.AmpHours += (wandReading.ShuntCurrent * wandReading.ReadTick) / 3600000.0;
    wandReading.LastRead = i_new_time;

    monitor.recalibrate();
    monitor.reconfig();
  }
}

void updateWandPowerState() {
  const uint8_t POWER_STATE_UPDATE_INTERVAL = 20;
  static uint8_t si_update;
  si_update = (si_update + 1) % POWER_STATE_UPDATE_INTERVAL;

  if (!b_wand_connected) {
    float f_avg_power = wandReading.AvgPower;
    unsigned long current_time = millis();
    unsigned long change_time;
    bool b_state_change_lower = f_avg_power < wandReading.LastAverage - (PowerMeter::StateChangeThreshold * 1.4);
    bool b_state_change_higher = f_avg_power > wandReading.LastAverage + PowerMeter::StateChangeThreshold;

    if(b_state_change_lower || b_state_change_higher) {
      if(wandReading.StateChanged == 0) {
        wandReading.StateChanged = current_time;
      }

      change_time = current_time - wandReading.StateChanged;
      if(change_time >= PowerMeter::StateChangeDuration) {
        wandReading.LastAverage = f_avg_power;

        if(f_avg_power > f_wand_power_on_threshold) {
          b_wand_on = true;
          #ifdef DEBUG
          Serial.println("Wand is powered up");
          #endif
          if(PACK_STATUS == MODE_OFF) {
            packStartup();
            ms_powerup_debounce.start(f_wand_power_up_delay);
          }
        }

        if(b_wand_on && PACK_STATUS != MODE_OFF) {
          if(b_state_change_higher && !b_wand_firing && ms_powerup_debounce.remaining() < 1) {
            i_wand_power_level = 5;
            b_wand_firing = true;
            PACK_STATUS = WAND_FIRING;
            wandFiring();
          }

          if(b_state_change_lower && b_wand_firing) {
            PACK_STATUS = WAND_STOPPED_FIRING;
            b_wand_firing = false;
            wandStoppedFiring();
          }
        }
      }
    }
    else {
      wandReading.StateChanged = 0;
    }
    
    if(f_avg_power <= f_wand_power_on_threshold) {
      b_wand_on = false;

      if(PACK_STATUS != MODE_OFF && PACK_STATUS != ACTION_POWERDOWN) {
        packShutdown();
      }

      wandReading.StateChanged = 0;
      wandReading.LastAverage = f_avg_power;
    }
  }
  else {
    wandReading.StateChanged = 0;
    wandReading.LastAverage = 0;
  }
}


// Displays the latest gathered power meter values (for debugging only!).
void wandPowerDisplay() {
  if(b_power_meter_available && b_show_power_data) {
     Serial.print("W.Shunt(mV):"); Serial.print(wandReading.ShuntVoltage); Serial.print(",");
     Serial.print("W.Shunt(A):"); Serial.print(wandReading.ShuntCurrent); Serial.print(",");
     Serial.print("W.Raw(W):"); Serial.print(wandReading.RawPower); Serial.print(",");
     Serial.print("W.AvgPow(W):"); Serial.print(wandReading.AvgPower); Serial.print(",");
     Serial.print("W.State:"); Serial.println(wandReading.LastAverage);
  }
}

// Check the available timers for reading power meter data.
void checkPowerMeter() {
  if(wandReading.ReadTimer.justFinished()) {
    if(b_power_meter_available) {
      doWandPowerReading();
      wandPowerDisplay();
      updateWandPowerState();
      wandReading.ReadTimer.start(wandReading.PowerReadDelay);
    }
  }
}
