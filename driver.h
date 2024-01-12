/*

  driver.h - configuration file for Texas Instruments MSP432 ARM processor

  Part of grblHAL

  Copyright (c) 2017-2022 Terje Io

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

/*****************************************************************************************
 *                                                                                       *
 * NOTE: any assingment changes related to interrupt generating peripherals              *
 *       must be synchronized with entries in the msp432_startup_ccs.c IRQ vector table  *
 *                                                                                       *
 *****************************************************************************************/

#ifndef __DRIVER_H__
#define __DRIVER_H__

#include <stdbool.h>
#include <stdint.h>

#include "msp.h"

#include "grbl/hal.h"
#include "grbl/nuts_bolts.h"

#ifndef OVERRIDE_MY_MACHINE
#include "my_machine.h"
#endif

#include "grbl/driver_opts.h"

#define NO_MSP_CLASSIC_DEFINES

// Configuration

#ifndef CNC_BOOSTERPACK_SHORTS
#define CNC_BOOSTERPACK_SHORTS  0
#endif
#ifndef CNC_BOOSTERPACK_A4998
#define CNC_BOOSTERPACK_A4998   0
#endif

#define CNC_BOOSTERPACK         0

// Define GPIO I/O mode options

#define GPIO_SHIFT0   0
#define GPIO_SHIFT1   1
#define GPIO_SHIFT2   2
#define GPIO_SHIFT3   3
#define GPIO_SHIFT4   4
#define GPIO_SHIFT5   5
#define GPIO_MAP      8
#define GPIO_BITBAND  9
#define GPIO_MASKED  10

#ifdef BOARD_CNC_BOOSTERPACK
#include "cnc_boosterpack_map.h"
#elif defined(BOARD_MY_MACHINE)
#include "my_machine_map.h"
#else
#error "No board!"
#endif

// Adjust STEP_PULSE_LATENCY to get accurate step pulse length when required, e.g if using high step rates.
// The default value is calibrated for 10 microseconds length.
// NOTE: step output mode, number of axes and compiler optimization settings may all affect this value.
#ifndef STEP_PULSE_LATENCY
#define STEP_PULSE_LATENCY 1.8f // microseconds
#endif

// End configuration

#if TRINAMIC_ENABLE && CNC_BOOSTERPACK_A4998 == 0
#undef CNC_BOOSTERPACK_A4998
#define CNC_BOOSTERPACK_A4998 1
#endif

#if TRINAMIC_ENABLE
#ifndef TRINAMIC_MIXED_DRIVERS
#define TRINAMIC_MIXED_DRIVERS 1
#endif
#include "motors/trinamic.h"
#include "trinamic/common.h"
#endif

#if PLASMA_ENABLE
#include "plasma/thc.h"
#endif

#if ATC_ENABLE
#undef I2C_ENABLE
#define I2C_ENABLE 1
#endif

#define port(p) portI(p)
#define portI(p) P ## p
#define portGpio(p) portG(p)
#define portG(p) GPIO_PORT_P ## p
#define portINT(p) portQ(p)
#define portQ(p) PORT ## p ## _IRQn
#define portHANDLER(p) portH(p)
#define portH(p) PORT ## p ## _IRQHandler

#define I2Cport(p) I2CportI(p)
#define I2CportI(p) EUSCI_ ## p
#define I2CportINT(p) I2CportQ(p)
#define I2CportQ(p) EUSCI ## p ## _IRQn
#define I2CportHANDLER(p) I2CportH(p)
#define I2CportH(p) EUSCI ## p ## _IRQHandler

#define timer(p) timerN(p)
#define timerN(p) TIMER_ ## p
#define timerINT0(p) timerI0(p)
#define timerI0(p) T ## p ## _0_IRQn
#define timerINTN(p) timerIN(p)
#define timerIN(p) T ## p ## _N_IRQn
#define timerHANDLER0(p) timerH(p)
#define timerH(p) T ## p ## _0_IRQHandler
#define timerHANDLERN(p) timerHN(p)
#define timerHN(p) T ## p ## _N_IRQHandler

#define timer32(p) timer32N(p)
#define timer32N(p) TIMER32_ ## p
#define timer32INT(p) timer32I(p)
#define timer32I(p) T32_INT ## p ## _IRQn
#define timer32HANDLER(p) timer32H(p)
#define timer32H(p) T32_INT ## p ## _IRQHandler

// Define timer registers

#define STEPPER_TIM        1
#define STEPPER_TIMER      timer32(STEPPER_TIM)
#define STEPPER_TIMER_INT  timer32INT(STEPPER_TIM)
#define STEPPER_IRQHandler timer32HANDLER(STEPPER_TIM)

#define PULSE_TIM              A2
#define PULSE_TIMER            timer(PULSE_TIM)
#define PULSE_TIMER_INT0       timerINT0(PULSE_TIM)
#define PULSE_TIMER_INTN       timerINTN(PULSE_TIM)
#define STEPPULSE_0_IRQHandler timerHANDLER0(PULSE_TIM)
#define STEPPULSE_N_IRQHandler timerHANDLERN(PULSE_TIM)

#define SPINDLE_PWM_TIM   A0
#define SPINDLE_PWM_TIMER timer(SPINDLE_PWM_TIM)

#define DEBOUNCE_TIM        A3
#define DEBOUNCE_TIMER      timer(DEBOUNCE_TIM)
#define DEBOUNCE_TIMER_INT0 timerINT0(DEBOUNCE_TIM)
#define DEBOUNCE_IRQHandler timerHANDLER0(DEBOUNCE_TIM)

#define RPM_CNT               A1
#define RPM_COUNTER           timer(RPM_CNT)
#define RPM_COUNTER_INT0      timerINT0(RPM_CNT)
#define RPMCOUNTER_IRQHandler timerHANDLER0(RPM_CNT)
#define RPM_COUNTER_PN        7
#define RPM_COUNTER_PORT      port(RPM_COUNTER_PN)
#define RPM_COUNTER_PIN       2
#define RPM_COUNTER_BIT       (1<<RPM_COUNTER_PIN)

#define RPM_TIM       2
#define RPM_TIMER     timer32(RPM_TIM)
#define RPM_TIMER_INT timer32INT(RPM_TIM)

#define RPM_INDEX_PORT  port(C)
#define RPM_INDEX_PIN   11

#if KEYPAD_ENABLE == 1 && !defined(I2C_STROBE_PORT)
#error Keypad plugin not supported!
#elif I2C_STROBE_ENABLE && !defined(I2C_STROBE_PORT)
#error I2C strobe not supported!
#endif

typedef struct {
    pin_function_t id;
    DIO_PORT_Interruptable_Type *port;
    uint8_t pin;
    uint16_t bit;
    pin_group_t group;
    volatile bool active;
    volatile bool debounce;
    pin_irq_mode_t irq_mode;
    pin_mode_t cap;
    ioport_interrupt_callback_ptr interrupt_callback;
    aux_ctrl_t *aux_ctrl;
    const char *description;
} input_signal_t;

typedef struct {
    pin_function_t id;
    DIO_PORT_Interruptable_Type *port;
    uint8_t pin;
    pin_group_t group;
    pin_mode_t mode;
    const char *description;
} output_signal_t;

typedef struct {
    uint8_t n_pins;
    union {
        input_signal_t *inputs;
        output_signal_t *outputs;
    } pins;
} pin_group_pins_t;

// Driver initialization entry point

bool driver_init (void);
uint32_t xTaskGetTickCount();

void ioports_init(pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs);
void ioports_event (input_signal_t *input);

#endif // __DRIVER_H__
