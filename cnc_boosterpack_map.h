/*

  cnc_boosterpack_map.h - pin mapping configuration file for CNC BoosterPack

  - on Texas Instruments MSP432P401R LaunchPad

  Part of grblHAL

  Copyright (c) 2017-2024 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#if TRINAMIC_ENABLE
#define BOARD_NAME "CNC BoosterPack (Trinamic)";
#if TRINAMIC_ENABLE
#ifdef TRINAMIC_MIXED_DRIVERS
#undef TRINAMIC_MIXED_DRIVERS
#endif
#define TRINAMIC_MIXED_DRIVERS 0
#ifdef TRINAMIC_I2C
#undef TRINAMIC_I2C
#endif
#define TRINAMIC_I2C 1
#endif
#else
#define BOARD_NAME "CNC BoosterPack"
#endif

#ifdef CNC_BOOSTERPACK
#undef CNC_BOOSTERPACK
#endif

#undef I2C_ENABLE
#undef EEPROM_ENABLE

#define CNC_BOOSTERPACK  1 // Do not change!
#define I2C_ENABLE       1 // Only change if BoosterPack does not have EEPROM mounted
#define EEPROM_ENABLE    1 // Only change if BoosterPack does not have EEPROM mounted

// Define step pulse output pins. NOTE: All step bit pins must be on the same port.

#define STEP_PORT               port(B)
#define X_STEP_PIN              8
#define Y_STEP_PIN              10
#define Z_STEP_PIN              12
#define STEP_OUTMODE            GPIO_MAP
//#define STEP_OUTMODE GPIO_SHIFT3
//#define STEP_OUTMODE GPIO_BITBAND

// Define step direction output pins. NOTE: All direction pins must be on the same port.

#define DIRECTION_PORT          port(A)
#define X_DIRECTION_PIN         6
#define Y_DIRECTION_PIN         7
#define Z_DIRECTION_PIN         5
#define DIRECTION_OUTMODE       GPIO_MAP
//#define DIRECTION_OUTMODE GPIO_BITBAND

// Define stepper driver enable/disable output pin.

#define Z_ENABLE_PORT           port(C)
#define Z_ENABLE_PIN            7
#define XY_ENABLE_PORT          port(B)
#define XY_ENABLE_PIN           13

// Trinamic drivers in I2C mode uses XY_ENABLE_PIN as interrupt input for DIAG1 signal
#if TRINAMIC_ENABLE == 2130 && TRINAMIC_I2C
#define TRINAMIC_DIAG_IRQ_PORT   port(B)
#define TRINAMIC_DIAG_IRQ_PIN    13
#define TRINAMIC_WARN_IRQ_PORT   port(C)
#define TRINAMIC_WARN_IRQ_PIN    7
#endif

#if CNC_BOOSTERPACK_A4998

// Stepper driver VDD/VIO supply

#define STEPPERS_VDD_PORT       port(B)
#define STEPPERS_VDD_PIN        11

#endif

// Define homing/hard limit switch input pins
// NOTE: All limit bit pins must be on the same port

#if CNC_BOOSTERPACK_SHORTS

#define LIMIT_PN                2
#define LIMIT_PORT              port(LIMIT_PN)
#define LIMIT_GPIO              portGpio(LIMIT_PN)
#define LIMIT_INT               portINT(LIMIT_PN)
#define LIMIT_IRQHandler        portHANDLER(LIMIT_PN)

#define X_LIMIT_PIN             3
#define Y_LIMIT_PIN             6
#define Z_LIMIT_PIN             7
//#define LIMIT_INMODE GPIO_BITBAND

#else

#define LIMIT_PORT_X            port(B)
#define LIMIT_PORT_Y            port(A)
#define LIMIT_PORT_Z            port(A)
#define X_LIMIT_PIN             0
#define Y_LIMIT_PIN             14
#define Z_LIMIT_PIN             15
#define LIMIT_INMODE            GPIO_BITBAND

#endif

// Define auxiliary output pins
#define AUXOUTPUT0_PORT         port(B)
#define AUXOUTPUT0_PIN          7
#define AUXOUTPUT1_PORT         port(C)
#define AUXOUTPUT1_PIN          0
#define AUXOUTPUT2_PORT         port(A) // Spindle PWM
#define AUXOUTPUT2_PIN          5
#define AUXOUTPUT3_PORT         port(C) // Spindle direction
#define AUXOUTPUT3_PIN          4
#define AUXOUTPUT4_PORT         port(B) // Spindle enable
#define AUXOUTPUT4_PIN          15
#define AUXOUTPUT5_PORT         port(C) // Coolant flood
#define AUXOUTPUT5_PIN          1
#define AUXOUTPUT6_PORT         port(B) // Coolant mist
#define AUXOUTPUT6_PIN          5

// Define driver spindle pins
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PORT     AUXOUTPUT4_PORT
#define SPINDLE_ENABLE_PIN      AUXOUTPUT4_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PORT        AUXOUTPUT2_PORT
#define SPINDLE_PWM_PIN         AUXOUTPUT2_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PORT  AUXOUTPUT3_PORT
#define SPINDLE_DIRECTION_PIN   AUXOUTPUT3_PIN
#endif

#define SPINDLE_PID_SAMPLE_RATE 5 // ms

// Define flood and mist coolant enable output pins.
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PORT      AUXOUTPUT5_PORT
#define COOLANT_FLOOD_PIN       AUXOUTPUT5_PIN
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PORT       AUXOUTPUT6_PORT
#define COOLANT_MIST_PIN        AUXOUTPUT6_PIN
#endif

// Define user-control controls (cycle start, reset, feed hold) input pins.

#if CNC_BOOSTERPACK_SHORTS

#define RESET_PORT              port(C)
#define RESET_PIN               15
#define CYCLE_START_PORT        port(C)
#define CYCLE_START_PIN         8
#define FEED_HOLD_PORT          port(C)
#define FEED_HOLD_PIN           14

#else

#define RESET_PORT              port(A)
#define RESET_PIN               12
#define CYCLE_START_PORT        port(C)
#define CYCLE_START_PIN         15
#define FEED_HOLD_PORT          port(C)
#define FEED_HOLD_PIN           14

#define CONTROL_INMODE          GPIO_BITBAND

#endif

// Normally used as MPG mode input
#define GPIO2_PN                5
#define GPIO2_PORT              port(C)
#define GPIO2_GPIO              portGpio(GPIO2_PN)
#define GPIO2_INT               portINT(GPIO2_PN)
#define GPIO2_PIN               2

// Normally used as limit switches override input
#define GPIO3_PN                3
#define GPIO3_PORT              port(GPIO3_PN)
#define GPIO3_GPIO              portGpio(GPIO3_PN)
#define GPIO3_INT               portINT(GPIO3_PN)
#define GPIO3_PIN               6

// Normally used as keypad strobe input
#define GPIO6_PN                4
#define GPIO6_PORT              port(GPIO6_PN)
#define GPIO6_GPIO              portGpio(GPIO6_PN)
#define GPIO6_INT               portINT(GPIO6_PN)
#define GPIO6_PIN               1

#if !(MPG_ENABLE == 1 || MODBUS_ENABLE)
#define AUXINPUT0_PORT          port(B)
#define AUXINPUT0_PIN           2 // GPIO4

#define AUXINPUT1_PORT          port(B)
#define AUXINPUT1_PIN           3 // GPIO5
#endif

#if CNC_BOOSTERPACK_SHORTS
#define AUXINPUT2_PORT          port(C)
#define AUXINPUT2_PIN           9
#else
#define AUXINPUT2_PORT          port(C)
#define AUXINPUT2_PIN           6
#endif

#define AUXINPUT3_PORT          port(B) // GPIO3
#define AUXINPUT3_PIN           6
#define AUXINPUT4_PORT          port(C) // GPIO2
#define AUXINPUT4_PIN           2
#define AUXINPUT5_PORT          port(B) // GPIO6
#define AUXINPUT5_PIN           9
#define AUXINPUT6_PORT          port(B) // Probe
#define AUXINPUT6_PIN           14

#if PROBE_ENABLE
#define PROBE_PORT              AUXINPUT6_PORT
#define PROBE_PIN               AUXINPUT6_PIN
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PORT        AUXINPUT2_PORT
#define SAFETY_DOOR_PIN         AUXINPUT2_PIN
#endif

#if MOTOR_FAULT_ENABLE
#define MOTOR_FAULT_PORT        AUXINPUT1_PORT
#define MOTOR_FAULT_PIN         AUXINPUT1_PIN
#endif

#if MPG_ENABLE == 1
#define MPG_MODE_PORT           AUXINPUT4_PORT
#define MPG_MODE_PIN            AUXINPUT4_PIN
#endif

#if I2C_STROBE_ENABLE
#define I2C_STROBE_PORT         AUXINPUT5_PORT
#define I2C_STROBE_PIN          AUXINPUT5_PIN
#endif

#define I2C_PN                  B1
#define I2C_PORT                I2Cport(I2C_PN)
#define I2C_INT                 I2CportINT(I2C_PN)
#define I2C_IRQHandler          I2CportHANDLER(I2C_PN)

#if SPINDLE_ENCODER_ENABLE
#define SPINDLE_PULSE_PN        7
#define SPINDLE_PULSE_PORT      port(SPINDLE_PULSE_PN)
#define SPINDLE_PULSE_PIN       2
#if SPINDLE_SYNC_ENABLE
#define SPINDLE_INDEX_PORT      port(C)
#define SPINDLE_INDEX_PIN       11
#endif
#endif

//#define RTS_PORT                port(B)
//#define RTS_PIN                 4

/*EOF*/
