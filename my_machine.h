/*
  my_machine.h - configuration for MSP432 processor

  Part of grblHAL

  Copyright (c) 2020-2022 Terje Io

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

// NOTE: Only one board may be enabled!
#define BOARD_CNC_BOOSTERPACK
//#define BOARD_MY_MACHINE // Add my_machine_map.h before enabling this!

// Configuration
// Uncomment to enable, for some a value > 1 may be assigned, if so the default value is shown.

#define SAFETY_DOOR_ENABLE 1 // Enable safety door input.
//#define PLASMA_ENABLE      1 // Plasma/THC plugin.
// Spindle selection:
// Up to four specific spindle drivers can be instantiated at a time
// depending on N_SPINDLE and N_SYS_SPINDLE definitions in grbl/config.h.
// If none are specified the default PWM spindle is instantiated.
// Spindle definitions can be found in grbl/spindle_control.h.
// More here https://github.com/grblHAL/Plugins_spindle
//#define SPINDLE0_ENABLE        SPINDLE_HUANYANG1
//#define SPINDLE1_ENABLE        SPINDLE_PWM0
//#define SPINDLE2_ENABLE        SPINDLE_NONE 
//#define SPINDLE2_ENABLE        SPINDLE_NONE
// **********************
//#define MODBUS_ENABLE      1 // Set to 1 for auto direction, 2 for direction signal on auxillary output pin.
//#define MPG_ENABLE         1 // Enable MPG interface. Requires serial port and one handshake pin unless
                               // KEYPAD_ENABLE is set to 2 when mode switching is done by the CMD_MPG_MODE_TOGGLE (0x8B)
                               // command character. Set both MPG_ENABLE and KEYPAD_ENABLE to 2 to use a handshake pin anyway.
//#define KEYPAD_ENABLE      1 // Set to 1 for I2C keypad, 2 for other input such as serial data. If KEYPAD_ENABLE is set to 2
                               // and MPG_ENABLE is uncommented then a serial stream is shared with the MPG.
//#define TRINAMIC_ENABLE 2130 // Trinamic TMC2130 stepper driver support. NOTE: work in progress.
//#define TRINAMIC_ENABLE 5160 // Trinamic TMC5160 stepper driver support. NOTE: work in progress.
//#define TRINAMIC_I2C       0 // Trinamic I2C - SPI bridge interface.
//#define ODOMETER_ENABLE    1 // Odometer plugin. To be completed.
//#define EEPROM_ENABLE      1 // I2C EEPROM support. Set to 1 for 24LC16 (2K), 3 for 24C32 (4K - 32 byte page) and 2 for other sizes. Uses eeprom plugin.
//#define EEPROM_IS_FRAM     1 // Uncomment when EEPROM is enabled and chip is FRAM, this to remove write delay.
//#define LIMITS_OVERRIDE_ENABLE 1

//#define ESTOP_ENABLE       0 // When enabled only real-time report requests will be executed when the reset pin is asserted.
                               // Note: if left commented out the default setting is determined from COMPATIBILITY_LEVEL.

#ifdef BOARD_CNC_BOOSTERPACK
#define CNC_BOOSTERPACK_SHORTS 0 // Shorts added to BoosterPack for some signals (for faster and simpler driver)
#define CNC_BOOSTERPACK_A4998  1 // Using Polulu A4998 drivers - for suppying VDD via GPIO (PE5)
#endif
