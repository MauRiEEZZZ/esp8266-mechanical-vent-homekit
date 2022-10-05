/*
 * Original Author: Klusjesman
 *
 * Tested with STK500 + ATMega328P
 * GCC-AVR compiler
 * 
 * Modified by supersjimmie:
 * Code and libraries made compatible with Arduino and ESP8266 
 * Tested with Arduino IDE v1.6.5 and 1.6.9
 * For ESP8266 tested with ESP8266 core for Arduino v 2.1.0 and 2.2.0 Stable
 * (See https://github.com/esp8266/Arduino/ )
 */

/*
CC11xx pins    ESP pins Arduino pins  Description
1 - VCC        VCC      VCC           3v3
2 - GND        GND      GND           Ground
3 - MOSI       13=D7    Pin 11        Data input to CC11xx
4 - SCK        14=D5    Pin 13        Clock pin
5 - MISO/GDO1  12=D6    Pin 12        Data output from CC11xx / serial clock from CC11xx
6 - GDO0       ?        Pin 2?        Serial data to CC11xx
7 - GDO2       ?        Pin  ?        output as a symbol of receiving or sending data
8 - CSN        15=D8    Pin 10        Chip select / (SPI_SS)
*/

#include "Arduino.h"
#include <SPI.h>
#include <arduino_homekit_server.h>
#include <arduino-timer.h>
#include "IthoCC1101.h"
#include "IthoPacket.h"
#include "wifi_info.h"

#define LOG_D(fmt, ...)   printf_P(PSTR(fmt "\n") , ##__VA_ARGS__);

typedef enum State 
{
  StateIgnore = 0,
  StateStandby = 1, 
  StateLow = 2, 
  StateMedium = 3,
  StateHigh = 4,
  StateFull = 5,
  StateTimerShort = 6,
  StateTimerLong = 7
};

char *state_to_string[] = {"ignore", "standby", "low", "medium", "high", "full", "short_timer", "long_timer"};

typedef enum Action
{
  ActionDeactivate = 1, 
  ActionActivate = 2,
  ActionSpeedStandby = 3,
  ActionSpeedLow = 4,
  ActionSpeedMedium = 5,
  ActionSpeedHigh = 6,
  ActionSpeedFull = 7,
  ActionTimerShortOn = 8,
  ActionTimerShortOff = 9,
  ActionTimerLongOn = 10,
  ActionTimerLongOff = 11
};

char *action_to_string[] = {"none", "deactivate", "activate", "standby", "low", "medium", "high", "full", "short_timer_on", "short_timer_off", "long_timer_on", "long_timer_off"};

IthoCC1101 rf;
IthoPacket packet;
State current_state = StateLow;
State last_speed = StateLow;
Timer<1, millis> timer_short;
Timer<1, millis> timer_long;
int timer_short_ms = 10 * 60 * 1000; // 10 min
int timer_long_ms = 40 * 60 * 1000; // 40 min

// access the HomeKit characteristics defined in fan_accessory.c
extern "C" homekit_server_config_t config;
extern "C" homekit_characteristic_t ch_fan_active;
extern "C" homekit_characteristic_t ch_fan_rotation_speed;
extern "C" homekit_characteristic_t ch_timer_short_on;
extern "C" homekit_characteristic_t ch_timer_long_on;

static uint32_t next_heap_millis = 0;

void setup(void) {
  Serial.begin(115200);
  Serial.println("setup begin");

  wifi_connect(); 
  delay(500);
  // homekit_storage_reset(); // to remove the previous HomeKit pairing storage when you first run this new HomeKit example
  homekit_setup();
  delay(500);
  rf_setup();
  Serial.println("setup done");
  // sendRegister();
  // Serial.println("join command sent");
}

void loop() {
  homekit_loop();
  timer_short.tick();
  timer_long.tick();
  delay(10);
}

// Called when the fan value is enabled/disabled by iOS Home APP
void ch_fan_active_setter(const homekit_value_t value) {
  LOG_D("Homekit sends active: %d", value.bool_value);
  
  switch (value.bool_value) {
    case true:  transition(ActionActivate);   break;
    case false: transition(ActionDeactivate); break;
  }
}

void ch_timer_short_setter(const homekit_value_t value) {
  LOG_D("Homekit sends short timer: %d", value.bool_value);
  
  switch (value.bool_value) {
    case true: transition(ActionTimerShortOn); break;
    case false: transition(ActionTimerShortOff); break;
  }
}

void ch_timer_long_setter(const homekit_value_t value) {
  LOG_D("Homekit sends long timer: %d", value.bool_value);
  
  switch (value.bool_value) {
    case true: transition(ActionTimerLongOn); break;
    case false: transition(ActionTimerLongOff); break;
  }
}

// Called when the fan rotation speed is changed by iOS Home APP
void ch_fan_rotation_speed_setter(const homekit_value_t value) {
  LOG_D("Homekit sends rotation speed: %.6f", value.float_value); 

  if (compare_float(value.float_value, 0)) {
    transition(ActionSpeedLow); 
  } else if (compare_float(value.float_value, 50)) {
    transition(ActionSpeedMedium);
  } else if (compare_float(value.float_value, 100)) {
    transition(ActionSpeedHigh);
  }
}

bool time_long_expired(void *) {
  LOG_D("Long timer expired"); 
  transition(ActionTimerLongOff);
  
  return false; // to repeat the action - false to stop
}

bool time_short_expired(void *) {
  LOG_D("Short timer expired"); 
  transition(ActionTimerShortOff);
 
  return false; // to repeat the action - false to stop
}

// Determine what speed to set.
// Homekit often sends 'rotation speed' and 'active' at the same time.
// The state machine avoids invalid transitions.
State get_next_state(State current_state, Action action) {
  switch (current_state) {        
      case StateLow:
        switch (action) {
          case ActionActivate: return last_speed != StateLow ? last_speed : StateLow;
          case ActionDeactivate: return StateLow; 
          case ActionSpeedMedium: return StateMedium; 
          case ActionSpeedHigh: return StateHigh; 
          case ActionTimerShortOn: return StateTimerShort;
          case ActionTimerLongOn: return StateTimerLong;
        }
        break;

      case StateMedium:
        switch (action) {
          case ActionDeactivate: return StateLow; 
          case ActionSpeedLow: return StateLow; 
          case ActionSpeedHigh: return StateHigh; 
          case ActionTimerShortOn: return StateTimerShort;
          case ActionTimerLongOn: return StateTimerLong;
        }
        break;
        
      case StateHigh:
        switch (action) {
          case ActionDeactivate: return StateLow; 
          case ActionSpeedLow: return StateLow; 
          case ActionSpeedMedium: return StateMedium; 
          case ActionTimerShortOn: return StateTimerShort;
          case ActionTimerLongOn: return StateTimerLong;
        }
        break;      

      case StateTimerShort:
        switch (action) {
          case ActionTimerShortOn: return StateTimerShort; // Restart the timer if turned on again
          case ActionTimerShortOff: timer_short.cancel(); return StateLow; // When the timer ends or is disabled, go back to the previous speed
          case ActionTimerLongOn: timer_short.cancel(); return StateTimerLong; // A short timer can be overwritten by a long timer
          // No other action can overwrite the timer
        }
        break; 

      case StateTimerLong:
        switch (action) {
          case ActionTimerLongOn: return StateTimerLong; // Restart the timer if turned on again
          case ActionTimerLongOff: timer_long.cancel(); return StateLow; // When the timer ends or is disabled, go back to the previous speed
          // No other action can overwrite the timer
        }
        break; 
    }
    
    return StateIgnore;
}

void ch_fan_rotation_for_state(State state) {
    // Change Homekit rotation speed
    switch (state) {
      case StateLow:
        ch_fan_rotation_speed.value.float_value = 0.0;
        break;
      case StateMedium:
      case StateTimerShort:
        ch_fan_rotation_speed.value.float_value = 50.0;
        break;
      case StateHigh:
      case StateTimerLong:
        ch_fan_rotation_speed.value.float_value = 100.0;
        break;
      default:
        break;
    }
    Serial.printf("Set HomeKit fan rotation speed to: %.3f \n", ch_fan_rotation_speed.value.float_value); 
    homekit_characteristic_notify(&ch_fan_rotation_speed, ch_fan_rotation_speed.value);
}

void ch_fan_active_state(bool is_active) {
      // Change Homekit fan active state
    ch_fan_active.value.bool_value = is_active; 
    homekit_characteristic_notify(&ch_fan_active, ch_fan_active.value);
}

void transition(Action action) {
    State next_state = get_next_state(current_state, action);

    LOG_D("Transition. Current state: '%s', action: '%s', next state: '%s'", 
      state_to_string[current_state], 
      action_to_string[action], 
      state_to_string[next_state]); 

    if (next_state == StateIgnore || current_state == next_state) {
      LOG_D("Already good. Skipping transition.");
      ch_fan_rotation_for_state(current_state);
      ch_fan_active_state(current_state != StateLow);
      return;
    }

    // Change internal state
    current_state = next_state;
    if (next_state >= 1 && next_state <= 5) { // only set "speed" states
      last_speed = current_state;
    }
    
    ch_fan_active_state(current_state != StateLow);
    
    // Set Homekit timer state
    ch_timer_short_on.value.bool_value = current_state == StateTimerShort;
    ch_timer_long_on.value.bool_value = current_state == StateTimerLong; 
    homekit_characteristic_notify(&ch_timer_short_on, ch_timer_short_on.value);
    homekit_characteristic_notify(&ch_timer_long_on, ch_timer_long_on.value);
    
    // Send the current state to IthoFan
    switch (current_state) { 
      case StateLow:
        sendLowSpeed();
        break;
      case StateMedium:
      case StateTimerShort:
        sendMediumSpeed();
        break;
      case StateHigh:
      case StateTimerLong:
        sendHighSpeed();
        break;
      default:
        break;
    }

    ch_fan_rotation_for_state(current_state);
    
    // Start or restart timers
    switch (action)
    {
      case ActionTimerShortOn:
        timer_short.in(timer_short_ms, time_short_expired);
        break;
      case ActionTimerLongOn:
        timer_long.in(timer_long_ms, time_long_expired);
        break;
      default:
        break;
    }
}

void homekit_setup() {
  ch_fan_active.setter = ch_fan_active_setter;
  ch_fan_rotation_speed.setter = ch_fan_rotation_speed_setter;
  ch_timer_short_on.setter = ch_timer_short_setter;
  ch_timer_long_on.setter = ch_timer_long_setter;
  
  arduino_homekit_setup(&config);
}

void rf_setup() {
  rf.init();
  delay(100);
  rf.initReceive();
  delay(100);
  //pinMode(ITHO_IRQ_PIN, INPUT_PULLUP);
  //attachInterrupt(ITHO_IRQ_PIN, ITHOinterrupt, CHANGE);
  //rf.sendCommand(IthoJoin); //the ID inside ithoCC1101.cpp at this->outIthoPacket.deviceId2 is used!
  //rf.sendCommand(IthoLeave); //the ID inside ithoCC1101.cpp at this->outIthoPacket.deviceId2 is used!
  //delay(100);
  rf.sendCommand(IthoLow);
}

void homekit_loop() {
  arduino_homekit_loop();
  const uint32_t t = millis();
  if (t > next_heap_millis) {
    // show heap info every 30 seconds
    next_heap_millis = t + 30 * 1000;
    LOG_D("Free heap: %d, HomeKit clients: %d", ESP.getFreeHeap(), arduino_homekit_connected_clients_count());
  }
}

void sendRegister() {
  Serial.println("sending join...");
  rf.sendCommand(IthoJoin);
  Serial.println("sending join done.");
}

void sendStandbySpeed() {
  Serial.println("sending standby...");
  rf.sendCommand(IthoStandby);
  Serial.println("sending standby done.");
}

void sendLowSpeed() {
  Serial.println("sending low...");
  rf.sendCommand(IthoLow);
  Serial.println("sending low done.");
}

void sendMediumSpeed() {
  Serial.println("sending medium...");
  rf.sendCommand(IthoMedium);
  Serial.println("sending medium done.");
}

void sendHighSpeed() {
  Serial.println("sending high...");
  rf.sendCommand(IthoHigh);
  Serial.println("sending high done.");
}

void sendFullSpeed() {
  Serial.println("sending FullSpeed...");
  rf.sendCommand(IthoFull);
  Serial.println("sending FullSpeed done.");
}

int compare_float(float f1, float f2)
 {
  float precision = 0.00001;
  if (((f1 - precision) < f2) && 
      ((f1 + precision) > f2))
   {
    return 1;
   }
  else
   {
    return 0;
   }
 }
