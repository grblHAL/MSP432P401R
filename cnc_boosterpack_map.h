/*

  cnc_boosterpack_map.h - pin mapping configuration file for CNC BoosterPack

  - on Texas Instruments MSP432P401R LaunchPad

  Part of grblHAL

  Copyright (c) 2017-2021 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

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

#ifdef EEPROM_ENABLE
#undef EEPROM_ENABLE
#endif

#ifdef CNC_BOOSTERPACK
#undef CNC_BOOSTERPACK
#endif

#define CNC_BOOSTERPACK  1 // Do not change!
#define EEPROM_ENABLE    1 // Only change if BoosterPack does not have EEPROM mounted

#define HAS_IOPORTS

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

// Define flood and mist coolant output pins.

#define COOLANT_FLOOD_PORT      port(C)
#define COOLANT_FLOOD_PIN       1
#define COOLANT_MIST_PORT       port(B)
#define COOLANT_MIST_PIN        5

// Define user-control controls (cycle start, reset, feed hold) input pins.

#if CNC_BOOSTERPACK_SHORTS

#define RESET_PORT              port(C)
#define RESET_PIN               15
#define CYCLE_START_PORT        port(C)
#define CYCLE_START_PIN         8
#define FEED_HOLD_PORT          port(C)
#define FEED_HOLD_PIN           14
#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PORT        port(C)
#define SAFETY_DOOR_PIN         9
#endif

#else

#define RESET_PORT              port(A)
#define RESET_PIN               12
#define CYCLE_START_PORT        port(C)
#define CYCLE_START_PIN         15
#define FEED_HOLD_PORT          port(C)
#define FEED_HOLD_PIN           14
#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PORT        port(C)
#define SAFETY_DOOR_PIN         6
#endif

#define CONTROL_INMODE          GPIO_BITBAND

#endif

// Define probe switch input pin.

#define PROBE_PORT              port(B)
#define PROBE_GPIO              portGpio(PROBE_PN)
#define PROBE_PIN               14

// Define spindle enable, spindle direction and PWM output pins.

#define SPINDLE_ENABLE_PORT     port(B)
#define SPINDLE_ENABLE_PIN      15

#define SPINDLE_DIRECTION_PORT  port(C)
#define SPINDLE_DIRECTION_PIN   4

#define SPINDLE_PWM_PORT        P2
#define SPINDLE_PWM_PIN         5

#define SPINDLE_PID_SAMPLE_RATE 5 // ms

/*
 * CNC Boosterpack GPIO assignments
 */

#define AUXOUTPUT0_PORT         port(B)
#define AUXOUTPUT0_PIN          7

#define AUXOUTPUT1_PORT         port(C)
#define AUXOUTPUT1_PIN          0

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

#if !(MPG_MODE_ENABLE || MODBUS_ENABLE)
#define AUXINPUT0_PORT          port(B)
#define AUXINPUT0_PIN           2 // GPIO4

#define AUXINPUT1_PORT          port(B)
#define AUXINPUT1_PIN           3 // GPIO5
#endif

// Define MPG mode input (for selecting secondary UART input)

#if MPG_MODE_ENABLE // GPIO2
#define MODE_PORT               port(C)
#define MODE_SWITCH_PIN         2
#endif

// Define limit switches override input

#if LIMITS_OVERRIDE_ENABLE // GPIO3
#define LIMITS_OVERRIDE_PORT    port(B)
#define LIMITS_OVERRIDE_PIN     6
#endif

#if KEYPAD_ENABLE // GPIO6
#define KEYPAD_PORT             port(B)
#define KEYPAD_IRQ_PIN          9
#endif

#define I2C_PN                  B1
#define I2C_PORT                I2Cport(I2C_PN)
#define I2C_INT                 I2CportINT(I2C_PN)
#define I2C_IRQHandler          I2CportHANDLER(I2C_PN)

/*EOF*/
