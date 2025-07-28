/**
 * GPStar Proton Pack - Ghostbusters Proton Pack & Neutrona Wand.
 * Copyright (C) 2024-2025 Michael Rajotte <michael.rajotte@gpstartechnologies.com>
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
 * Modified by: Aaron Morris
 * Date: 6/24/24
 * Modification Revision: 1.1 - This version appears to be working properly with the SpiritBro V2 board. 
 *                        Added magic number definitions, refactored the states, and tuned values based on test wand.
 *                        Tested overheating, startup, shutdown, mode changing and firing sequences.
 */

#pragma once

/**
 * Power Meter (using the INA219 chip) - https://github.com/flav1972/ArduinoINA219
 * Provides support for a power-sensing chip which can detect the voltage and current
 * being provided to the Neutrona Wand. Intended for those users who want to utilize
 * a stock wand but still trigger the pack power-on and firing animations/effects.
 */
#include <INA219.h>

// Custom values for calibrating the current-sensing device.
#define SHUNT_R     0.1  // Shunt value in ohms (default: 0.1ohm)
#define SHUNT_MAX_V 0.2  // Max voltage across shunt (default: 0.2V)
#define BUS_MAX_V   16.0 // Sets max based on expected range (< 16V)
#define MAX_CURRENT 2.0  // Sets the expected max amperage draw (2A)
#define DIAGNOSTIC_MODE false // Set to true to find new thresholds, false for normal operation.


// These values define the thresholds for detecting wand state changes based on power draw.
const float POWER_ON_THRESHOLD_FAST         = 0.09;   // A rapid power increase suggesting the wand is turning on.
const float POWER_ON_THRESHOLD_SLOW_AVG     = 0.0025; // A slower, sustained power increase.
const float POWER_ON_MIN_IDLE_POWER         = 0.8;    // Minimum average power to confirm the wand is on (filters out noise).
const float POWER_OFF_THRESHOLD_FAST        = -0.02;  // A rapid power decrease suggesting the wand is turning off.
const float POWER_OFF_MAX_POWER             = 0.55;   // Power level below which a fast drop confirms shutdown.
const float POWER_OFF_ABSOLUTE_MAX          = 0.1;    // Absolute power level below which the wand is considered off.
const float FIRING_START_AVG_RATE_MIN       = 0.0285; // Min average rate of power increase to be considered firing.
const float FIRING_START_AVG_RATE_MAX       = 0.045;  // Max average rate of power increase to be considered firing.
const float FIRING_START_POWER_JUMP         = 0.26;   // A significant jump in power over the detection window, indicating firing start.
const float FIRING_STOP_THRESHOLD_FROM_IDLE = 0.11;   // How close the power level must return to idle to be considered "stopped firing".
const float FIRING_STOP_ABSOLUTE_MAX_POWER  = 0.70;   // A fallback absolute power level to consider firing stopped.
// ----------------------------------------------------


// General Variables
INA219 monitor; // Power monitor object on i2c bus using the INA219 chip.
bool b_power_meter_available = false; // Whether a power meter device exists on i2c bus, per setup() -> powerMeterInit()
bool b_pack_started_by_meter = false; // Whether the pack was started via detection through the power meter.
bool b_wand_just_started = false; // Whether the wand was just started via the power meter, used to debounce the startup process.
bool b_wand_overheated = false; // Whether the wand overheated, as if it did we should ignore power off events.
const uint16_t i_wand_overheat_delay = 14480; // How many milliseconds of continuous firing before we lock into overheating mode.
const uint16_t i_wand_overheat_duration = 2500; // How long to play the alarm for before going into the full overheat sequence on the pack.
const uint16_t i_wand_startup_delay = 2750; // How many milliseconds after wand startup before we allow detecting firing events.
const float f_ema_alpha = 0.2; // Smoothing factor (<1) for Exponential Moving Average (EMA) [Lower Value = Smoother Averaging].
float f_sliding_window[20] = {}; // Sliding window for detecting state changes, initialized to 0.
float f_accumulator = 0.0; // Accumulator used for sliding window averaging operations.
float f_diff_average = 0.0; // Stores the result of the sliding window average operation.
float f_idle_value = 0.0; // Stores the previous idle value to be used for stop firing checks.

// Define an object which can store
struct PowerMeter {
  float ShuntVoltage = 0; // mV - Millivolts read to calculate the amperage draw across the shunt resistor
  float ShuntCurrent = 0; // A - The current (amperage) reading via the shunt resistor
  float BusVoltage = 0;   // mV - Voltage reading from the measured device
  float BattVoltage = 0;  // V - Reference voltage from device power source
  float BusPower = 0;     // W - Calculation of power based on the bus mV*A values
  float AmpHours = 0;     // Ah - An estimation of power consumed over regular intervals
  float RawPower = 0;     // W - Calculation of power based on raw V*A values (non-smoothed)
  float AvgPower = 0;     // A - Running average from the RawPower value (smoothed)
  uint16_t PowerReadDelay = 20; // How often (ms) to read levels for changes
  unsigned long LastRead = 0;     // Used to calculate Ah consumed since battery power-on
  unsigned long ReadTick = 0;     // Difference of current read time - last read
  millisDelay ReadTimer;          // Timer for reading latest values from power meter
};

// Create instances of the PowerMeter object.
PowerMeter wandReading;

// Forward function declarations.
void packStartup();
void packShutdown();
void wandFiring();
void wandStoppedFiring();

// Configure and calibrate the power meter device.
void powerMeterConfig() {
  debugln(F("Configuring Power Meter"));
  monitor.configure(INA219::RANGE_16V, INA219::GAIN_1_40MV, INA219::ADC_64SAMP, INA219::ADC_64SAMP, INA219::CONT_SH_BUS);
  monitor.calibrate(SHUNT_R, SHUNT_MAX_V, BUS_MAX_V, MAX_CURRENT);
}

// Initialize the power meter device on the i2c bus.
void powerMeterInit() {
  uint8_t i_monitor_status = monitor.begin();
  debug(F("Power Meter Initialization Result: "));
  debugln(i_monitor_status);

  if(i_monitor_status == 0) {
    b_power_meter_available = true;
    powerMeterConfig();
    wandReading.LastRead = millis();
    wandReading.ReadTimer.start(wandReading.PowerReadDelay);
  } else {
    debugln(F("Unable to find power monitoring device on i2c."));
  }
}

// Perform a reading of values from the power meter for the wand.
void doWandPowerReading() {
  // Slide the 20-parameter-wide window.
  for(uint8_t i = 0; i < 19; i++) {
    f_sliding_window[i] = f_sliding_window[i+1];
  }

  f_accumulator = 0.0;

  // Reads the latest values from the monitor.
  wandReading.ShuntVoltage = monitor.shuntVoltage();
  wandReading.ShuntCurrent = monitor.shuntCurrent();
  wandReading.BusVoltage = monitor.busVoltage();
  wandReading.BusPower = monitor.busPower();

  // Update the smoothed current (A) values using an exponential moving average.
  wandReading.BattVoltage = wandReading.BusVoltage + wandReading.ShuntVoltage;
  wandReading.RawPower = wandReading.BattVoltage * wandReading.ShuntCurrent;
  wandReading.AvgPower = (f_ema_alpha * wandReading.RawPower) + ((1 - f_ema_alpha) * wandReading.AvgPower);

  // Add the latest EMA'd reading to the end of the window.
  f_sliding_window[19] = wandReading.AvgPower;

  // Use time and current (A) to calculate amp-hours consumed.
  unsigned long i_new_time = millis();
  wandReading.ReadTick = i_new_time - wandReading.LastRead;
  wandReading.AmpHours += (wandReading.ShuntCurrent * wandReading.ReadTick) / 3600000.0;
  wandReading.LastRead = i_new_time;

  // Prepare for next read
  monitor.recalibrate();
  monitor.reconfig();
}

// Update the states of the pack based on the wand power states
void updateWandPowerState() {
  static uint8_t si_update = (si_update + 1) % 20;

  // --- State Transition Check ---
  // First, check if we need to TRANSITION INTO the overheat state.
  // This happens only if we are currently firing and the 14.5s timer finishes.
  if (b_wand_firing && !b_wand_overheated && ms_delay_post_2.justFinished()) {
    Serial.println(">>>> TIMER FINISHED: TRANSITIONING TO OVERHEAT STATE");
    b_wand_overheated = true;
    // Immediately start the 2.5 second alarm timer.
    ms_delay_post_2.start(i_wand_overheat_duration); 
  }

  // --- State Execution ---
  if (b_wand_overheated) {
    // --- STATE: We are OVERHEATED ---
    // The only thing to do in this state is wait for the 2.5s alarm timer to finish.
    
    // Check if the alarm timer is done
    if (ms_delay_post_2.justFinished()) {
      Serial.println("Wand Stopped Firing (Overheat Sequence Finished)");
      wandStoppedFiring();
      b_wand_firing = false; 
      b_wand_overheated = false; // Transition out of the overheat state
    }
    
    // Allow manual power-off to break out of the overheat state
    if (PACK_STATUS == MODE_OFF) {
      b_wand_overheated = false;
      ms_delay_post_2.stop();
    }

  } else {
    // --- STATE: We are NOT OVERHEATED ---
    // All normal logic for power on/off and firing start/stop goes here.
    
    for (uint8_t i = 17; i < 19; i++) {
      f_accumulator += (f_sliding_window[i + 1] - f_sliding_window[i]);
    }
    f_diff_average = f_accumulator / 2.0;
    f_accumulator = 0.0;

    if (!b_wand_on) {
      // POWER ON LOGIC
      for (uint8_t i = 0; i < 20; i++) { f_accumulator += f_sliding_window[i]; }
      float f_on_average = f_accumulator / 20.0;
      f_accumulator = 0.0;
      if (f_diff_average > POWER_ON_THRESHOLD_FAST || (f_diff_average > POWER_ON_THRESHOLD_SLOW_AVG && f_on_average > POWER_ON_MIN_IDLE_POWER)) {
        for (uint8_t i = 0; i < 20; i++) { f_sliding_window[i] = 0.0; }
        b_wand_on = true;
        b_wand_just_started = true;
        ms_delay_post_3.start(i_wand_startup_delay);
        if (PACK_STATUS == MODE_OFF) { packStartup(); b_pack_started_by_meter = true; }
      }
    } else { // b_wand_on is true
      if (ms_delay_post_3.justFinished()) {
        b_wand_just_started = false;
      }

      // POWER OFF LOGIC
      if ((f_diff_average < POWER_OFF_THRESHOLD_FAST && f_sliding_window[19] < POWER_OFF_MAX_POWER) || f_sliding_window[19] < POWER_OFF_ABSOLUTE_MAX) {
        if (b_wand_firing) {
          ms_delay_post_2.stop();
          wandStoppedFiring();
          b_wand_firing = false;
        }
        if (PACK_STATUS != MODE_OFF) { 
          packShutdown(); 
        }
        
        b_wand_on = false;
        b_pack_started_by_meter = false;

      } else if (PACK_STATUS == MODE_OFF) {

        b_pack_started_by_meter = false;

      } else if (!b_wand_just_started) {

        if (!b_wand_firing) {
          // START FIRING LOGIC
          for(uint8_t i = 9; i < 19; i++) { f_accumulator += (f_sliding_window[i + 1] - f_sliding_window[i]); }
          f_diff_average = f_accumulator / 10.0;
          f_accumulator = 0.0;
          float f_range = f_sliding_window[19] - f_sliding_window[9];
          if ((f_diff_average > FIRING_START_AVG_RATE_MIN && f_diff_average < FIRING_START_AVG_RATE_MAX) || (f_range > FIRING_START_POWER_JUMP)) {
            ms_delay_post_2.start(i_wand_overheat_delay); // Start the 14.5s overheat timer
            f_idle_value = f_sliding_window[9];
            wandFiring();
            b_wand_firing = true;
          }
        } else {
          // NORMAL STOP FIRING LOGIC
          float f_recent_average = 0.0;
          for(uint8_t i = 15; i < 20; i++) { f_recent_average += f_sliding_window[i]; }
          f_recent_average /= 5.0;
          if ((f_recent_average - f_idle_value < FIRING_STOP_THRESHOLD_FROM_IDLE) || f_recent_average < FIRING_STOP_ABSOLUTE_MAX_POWER) {
            ms_delay_post_2.stop();
            wandStoppedFiring();
            b_wand_firing = false;
          }
        }
      }
    }
  }
}


// Displays the latest gathered power meter values (for debugging only!).
void wandPowerDisplay() {
  if(b_show_power_data) {
    Serial.print(F("W.Raw(W):"));
    Serial.print(wandReading.RawPower, 4);
    Serial.println(F(","));
    //Serial.print(F("W.AvgPow(W):"));
    //Serial.print(wandReading.AvgPower, 4);
    //Serial.print(F(","));
  }
}

// Check the available timers for reading power meter data.
void checkPowerMeter() {
  if(wandReading.ReadTimer.justFinished()) {
    if(!b_wand_connected && !b_wand_syncing) {
      doWandPowerReading();
      wandPowerDisplay();
      updateWandPowerState();
    }
    wandReading.ReadTimer.repeat();
  }
}