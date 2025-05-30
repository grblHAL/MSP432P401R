/*

  driver.c - driver code for Texas Instruments MSP432P401R ARM processor

  Part of grblHAL

  Copyright (c) 2017-2025 Terje Io

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

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver.h"
#include "serial.h"

#include "grbl/task.h"
#include "grbl/machine_limits.h"
#include "grbl/spindle_sync.h"
#include "grbl/state_machine.h"
#include "grbl/motor_pins.h"
#include "grbl/pin_bits_masks.h"

#if I2C_ENABLE
#include "i2c.h"
#endif

#if EEPROM_ENABLE
#include "eeprom/eeprom.h"
#endif

#if ATC_ENABLE
#include "atc.h"
#endif

#if DRIVER_SPINDLE_ENABLE
static spindle_id_t spindle_id = -1;
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
static spindle_pwm_t spindle_pwm;
#endif // DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#endif // DRIVER_SPINDLE_ENABLE

#ifdef SPINDLE_RPM_CONTROLLED

typedef enum {
    PIDState_Disabled = 0,
    PIDState_Pending,
    PIDState_Active,
} pid_state_t;

// PID data for closed loop spindle RPM control
typedef struct {
    pid_state_t pid_state;
    pidf_t pid;
    bool pid_enabled;
    float rpm;
} spindle_control_t;

static volatile uint32_t pid_count = 0;
static spindle_control_t spindle_control = { .pid_state = PIDState_Disabled, .pid = {0}};

#endif // SPINDLE_RPM_CONTROLLED

static void aux_irq_handler (uint8_t port, bool state);

static pin_debounce_t debounce;
static periph_signal_t *periph_pins = NULL;

static input_signal_t inputpin[] = {
#if ESTOP_ENABLE
    { .id = Input_EStop,          .port = RESET_PORT,         .pin = RESET_PIN,           .group = PinGroup_Control },
#else
    { .id = Input_Reset,          .port = RESET_PORT,         .pin = RESET_PIN,           .group = PinGroup_Control },
#endif
    { .id = Input_FeedHold,       .port = FEED_HOLD_PORT,     .pin = FEED_HOLD_PIN,       .group = PinGroup_Control },
    { .id = Input_CycleStart,     .port = CYCLE_START_PORT,   .pin = CYCLE_START_PIN,     .group = PinGroup_Control },
// Limit input pins must be consecutive in this array
    { .id = Input_LimitX,         .port = LIMIT_PORT_X,       .pin = X_LIMIT_PIN,         .group = PinGroup_Limit },
    { .id = Input_LimitY,         .port = LIMIT_PORT_Y,       .pin = Y_LIMIT_PIN,         .group = PinGroup_Limit },
    { .id = Input_LimitZ,         .port = LIMIT_PORT_Z,       .pin = Z_LIMIT_PIN,         .group = PinGroup_Limit }
#ifdef A_LIMIT_PIN
  , { .id = Input_LimitA,         .port = LIMIT_PORT_A,       .pin = A_LIMIT_PIN,         .group = PinGroup_Limit }
#endif
#ifdef B_LIMIT_PIN
  , { .id = Input_LimitB,         .port = LIMIT_PORT_B,       .pin = B_LIMIT_PIN,         .group = PinGroup_Limit }
#endif
#ifdef C_LIMIT_PIN
  , { .id = Input_LimitC,         .port = LIMIT_PORT_C,       .pin = C_LIMIT_PIN,         .group = PinGroup_Limit }
#endif
#if LIMITS_OVERRIDE_BIT
  , { .id = Input_LimitsOverride, .port = LIMITS_OVERRIDE_PORT, .pin = LIMITS_OVERRIDE_PIN, .group = PinGroup_Limit }
#endif
#if SPINDLE_ENCODER_ENABLE
  , { .id = Input_SpindleIndex,   .port = SPINDLE_INDEX_PORT,   .pin = SPINDLE_INDEX_PIN,   .group = PinGroup_QEI_Index }
#endif
// Aux input pins must be consecutive in this array
#ifdef AUXINPUT0_PIN
  , { .id = Input_Aux0,           .port = AUXINPUT0_PORT,     .pin = AUXINPUT0_PIN,       .group = PinGroup_AuxInput }
#endif
#ifdef AUXINPUT1_PIN
  , { .id = Input_Aux1,           .port = AUXINPUT1_PORT,     .pin = AUXINPUT1_PIN,       .group = PinGroup_AuxInput }
#endif
#ifdef AUXINPUT2_PIN
  , { .id = Input_Aux2,           .port = AUXINPUT2_PORT,     .pin = AUXINPUT2_PIN,       .group = PinGroup_AuxInput }
#endif
#ifdef AUXINPUT3_PIN
  , { .id = Input_Aux3,           .port = AUXINPUT3_PORT,     .pin = AUXINPUT3_PIN,       .group = PinGroup_AuxInput }
#endif
#ifdef AUXINPUT4_PIN
  , { .id = Input_Aux4,           .port = AUXINPUT4_PORT,     .pin = AUXINPUT4_PIN,       .group = PinGroup_AuxInput }
#endif
#ifdef AUXINPUT5_PIN
  , { .id = Input_Aux5,           .port = AUXINPUT5_PORT,     .pin = AUXINPUT5_PIN,       .group = PinGroup_AuxInput }
#endif
#ifdef AUXINPUT6_PIN
  , { .id = Input_Aux6,           .port = AUXINPUT6_PORT,     .pin = AUXINPUT6_PIN,       .group = PinGroup_AuxInput }
#endif
#ifdef AUXINPUT7_PIN
  , { .id = Input_Aux7,           .port = AUXINPUT7_PORT,     .pin = AUXINPUT7_PIN,       .group = PinGroup_AuxInput }
#endif
#ifdef AUXINPUT8_PIN
  , { .id = Input_Aux8,           .port = AUXINPUT8_PORT,     .pin = AUXINPUT8_PIN,       .group = PinGroup_AuxInput }
#endif
#ifdef AUXINPUT9_PIN
  , { .id = Input_Aux9,           .port = AUXINPUT9_PORT,     .pin = AUXINPUT9_PIN,       .group = PinGroup_AuxInput }
#endif
#ifdef AUXINPUT10_PIN
  , { .id = Input_Aux10,          .port = AUXINPUT10_PORT,    .pin = AUXINPUT10_PIN,      .group = PinGroup_AuxInput }
#endif
#ifdef AUXINPUT11_PIN
  , { .id = Input_Aux11,          .port = AUXINPUT11_PORT,    .pin = AUXINPUT11_PIN,      .group = PinGroup_AuxInput }
#endif
};

static output_signal_t outputpin[] = {
    { .id = Output_StepX,           .port = X_STEP_PORT,            .pin = X_STEP_PIN,              .group = PinGroup_StepperStep },
    { .id = Output_StepY,           .port = Y_STEP_PORT,            .pin = Y_STEP_PIN,              .group = PinGroup_StepperStep },
    { .id = Output_StepZ,           .port = Z_STEP_PORT,            .pin = Z_STEP_PIN,              .group = PinGroup_StepperStep },
#ifdef A_AXIS
    { .id = Output_StepA,           .port = A_STEP_PORT,            .pin = A_STEP_PIN,              .group = PinGroup_StepperStep },
#endif
#ifdef B_AXIS
    { .id = Output_StepB,           .port = B_STEP_PORT,            .pin = B_STEP_PIN,              .group = PinGroup_StepperStep },
#endif
#ifdef C_AXIS
    { .id = Output_StepC,           .port = B_STEP_PORT,            .pin = C_STEP_PIN,              .group = PinGroup_StepperStep },
#endif
    { .id = Output_DirX,            .port = X_DIRECTION_PORT,       .pin = X_DIRECTION_PIN,         .group = PinGroup_StepperDir },
    { .id = Output_DirY,            .port = Y_DIRECTION_PORT,       .pin = Y_DIRECTION_PIN,         .group = PinGroup_StepperDir },
    { .id = Output_DirZ,            .port = Z_DIRECTION_PORT,       .pin = Z_DIRECTION_PIN,         .group = PinGroup_StepperDir },
#ifdef A_AXIS
    { .id = Output_DirA,            .port = A_DIRECTION_PORT,       .pin = A_DIRECTION_PIN,         .group = PinGroup_StepperDir },
#endif
#ifdef B_AXIS
    { .id = Output_DirB,            .port = B_DIRECTION_PORT,       .pin = B_DIRECTION_PIN,         .group = PinGroup_StepperDir },
#endif
#ifdef C_AXIS
    { .id = Output_DirC,            .port = C_DIRECTION_PORT,       .pin = C_DIRECTION_PIN,         .group = PinGroup_StepperDir },
#endif
#if CNC_BOOSTERPACK_A4998
    { .id = Output_StepperPower,    .port = STEPPERS_VDD_PORT,      .pin = STEPPERS_VDD_PIN,        .group = PinGroup_StepperPower },
#endif
#if !TRINAMIC_MOTOR_ENABLE
#ifdef STEPPERS_ENABLE_PORT
    { .id = Output_StepperEnable,   .port = STEPPERS_ENABLE_PORT,   .pin = STEPPERS_ENABLE_PIN,     .group = PinGroup_StepperEnable },
#endif
#ifdef XY_ENABLE_PORT
    { .id = Output_StepperEnableXY, .port = XY_ENABLE_PORT,         .pin = XY_ENABLE_PIN,           .group = PinGroup_StepperEnable },
#endif
#ifdef Z_ENABLE_PORT
    { .id = Output_StepperEnableZ,  .port = Z_ENABLE_PORT,          .pin = Z_ENABLE_PIN,            .group = PinGroup_StepperEnable },
#endif
#ifdef A_ENABLE_PORT
    { .id = Output_StepperEnableA,  .port = A_ENABLE_PORT,          .pin = A_ENABLE_PIN,            .group = PinGroup_StepperEnable, },
#endif
#ifdef B_ENABLE_PORT
    { .id = Output_StepperEnableB,  .port = B_ENABLE_PORT,          .pin = B_ENABLE_PIN,            .group = PinGroup_StepperEnable, },
#endif
#ifdef C_ENABLE_PORT
    { .id = Output_StepperEnableC,  .port = C_ENABLE_PORT,          .pin = C_ENABLE_PIN,            .group = PinGroup_StepperEnable, },
#endif
#endif // !TRINAMIC_MOTOR_ENABLE
#ifdef RTS_PIN
    { .id = Output_RTS,             .port = RTS_PORT,               .pin = RTS_PIN,                 .group = PinGroup_UART },
#endif
#ifdef AUXOUTPUT0_PORT
    { .id = Output_Aux0,            .port = AUXOUTPUT0_PORT,        .pin = AUXOUTPUT0_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT1_PORT
    { .id = Output_Aux1,            .port = AUXOUTPUT1_PORT,        .pin = AUXOUTPUT1_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT2_PORT
    { .id = Output_Aux2,            .port = AUXOUTPUT2_PORT,        .pin = AUXOUTPUT2_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT3_PORT
    { .id = Output_Aux3,            .port = AUXOUTPUT3_PORT,        .pin = AUXOUTPUT3_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT4_PORT
    { .id = Output_Aux4,            .port = AUXOUTPUT4_PORT,        .pin = AUXOUTPUT4_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT5_PORT
    { .id = Output_Aux5,            .port = AUXOUTPUT5_PORT,        .pin = AUXOUTPUT5_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT6_PORT
    { .id = Output_Aux6,            .port = AUXOUTPUT6_PORT,        .pin = AUXOUTPUT6_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT7_PORT
    { .id = Output_Aux7,            .port = AUXOUTPUT7_PORT,        .pin = AUXOUTPUT7_PIN,          .group = PinGroup_AuxOutput }
#endif
#ifdef AUXOUTPUT0_ANALOG_PORT
    { .id = Output_Analog_Aux0,     .port = AUXOUTPUT0_ANALOG_PORT, .pin = AUXOUTPUT0_ANALOG_PIN,   .group = PinGroup_AuxOutputAnalog },
#elif defined(AUXOUTPUT0_PWM_PORT)
    { .id = Output_Analog_Aux0,     .port = AUXOUTPUT0_PWM_PORT,    .pin = AUXOUTPUT0_PWM_PIN,      .group = PinGroup_AuxOutputAnalog, .mode = { PINMODE_PWM } },
#endif
#ifdef AUXOUTPUT1_ANALOG_PORT
    { .id = Output_Analog_Aux1,     .port = AUXOUTPUT1_ANALOG_PORT, .pin = AUXOUTPUT1_ANALOG_PIN,   .group = PinGroup_AuxOutputAnalog }
#elif defined(AUXOUTPUT1_PWM_PORT)
    { .id = Output_Analog_Aux1,     .port = AUXOUTPUT1_PWM_PORT,    .pin = AUXOUTPUT1_PWM_PIN,      .group = PinGroup_AuxOutputAnalog, .mode = { PINMODE_PWM } }
#endif
};

static pin_group_pins_t limit_inputs = {0};
static input_signal_t *p1_pins[9], *p2_pins[9], *p3_pins[9], *p4_pins[9], *p5_pins[9], *p6_pins[9];
static volatile bool spindleLock = false;
static bool IOInitDone = false;
static uint16_t pulse_length;
static volatile uint32_t elapsed_tics = 0;
static axes_signals_t next_step_out;

#if SPINDLE_ENCODER_ENABLE

static spindle_data_t spindle_data;
static spindle_encoder_t spindle_encoder = {
    .tics_per_irq = 4
};
static on_spindle_programmed_ptr on_spindle_programmed = NULL;

#ifndef SPINDLE_PULSE_BIT
#define SPINDLE_PULSE_BIT (1<<SPINDLE_PULSE_PIN)
#endif

#endif

static delay_t delay = { .ms = 1, .callback = NULL }; // NOTE: initial ms set to 1 for "resetting" systick timer on startup

#include "grbl/stepdir_map.h"

#if SPINDLE_ENCODER_ENABLE
static void stepperPulseStartSynchronized (stepper_t *stepper);
#endif

#if I2C_STROBE_ENABLE

static driver_irq_handler_t i2c_strobe = { .type = IRQ_I2C_Strobe };

static bool irq_claim (irq_type_t irq, uint_fast8_t id, irq_callback_ptr handler)
{
    bool ok;

    if((ok = irq == IRQ_I2C_Strobe && i2c_strobe.callback == NULL))
        i2c_strobe.callback = handler;

    return ok;
}

#endif

// LinuxCNC example settings
// MAX_OUTPUT = 300 DEADBAND = 0.0 P = 3 I = 1.0 D = 0.1 FF0 = 0.0 FF1 = 0.1 FF2 = 0.0 BIAS = 0.0 MAXI = 20.0 MAXD = 20.0 MAXERROR = 250.0
//
// You will always get oscillation on a PID system if you increase any P,I,D term too high
// I would try using less P (say 2) and then see how high an I term you can have and stay stable
// D term should not be needed

static void driver_delay_ms (uint32_t ms, void (*callback)(void))
{
//    while(delay.callback);

    if((delay.ms = ms) > 0) {
        if(!(delay.callback = callback)) {
            while(delay.ms)
                grbl.on_execute_delay(state_get());
        }
    } else if(callback)
        callback();
}

// Set stepper pulse output pins
// NOTE: step_outbits are: bit0 -> X, bit1 -> Y, bit2 -> Z...
// Mapping to registers can be done by
// 1. bitbanding. Pros: can assign pins to different ports, no RMW needed. Cons: overhead, pin changes not synchronous
// 2. bit shift. Pros: fast, Cons: bits must be consecutive
// 3. lookup table. Pros: signal inversions done at setup, Cons: slower than bit shift
inline __attribute__((always_inline)) static void set_step_outputs (axes_signals_t step_outbits)
{
#if STEP_OUTMODE == GPIO_BITBAND
    step_outbits.value ^= settings.steppers.step_invert.mask;
    BITBAND_PERI(STEP_PORT->OUT, X_STEP_PIN) = step_outbits.x;
    BITBAND_PERI(STEP_PORT->OUT, Y_STEP_PIN) = step_outbits.y;
    BITBAND_PERI(STEP_PORT->OUT, Z_STEP_PIN) = step_outbits.z;
#elif STEP_OUTMODE == GPIO_MAP
    STEP_PORT->OUT = (STEP_PORT->OUT & ~STEP_MASK) | step_outmap[step_outbits.value];
#else
    STEP_PORT->OUT = (STEP_PORT->OUT & ~STEP_MASK) | ((step_outbits.value << STEP_OUTMODE) ^ settings.steppers.step_invert.mask);
#endif
}

// Set stepper direction output pins
// NOTE: see note for set_step_outputs()
/*inline __attribute__((always_inline))*/ static void set_dir_outputs (axes_signals_t dir_outbits)
{
#if DIRECTION_OUTMODE == GPIO_BITBAND
    dir_outbits.value ^= settings.steppers.dir_invert.mask;
    BITBAND_PERI(DIRECTION_PORT->OUT, X_DIRECTION_PIN) = dir_outbits.x;
    BITBAND_PERI(DIRECTION_PORT->OUT, Y_DIRECTION_PIN) = dir_outbits.y;
    BITBAND_PERI(DIRECTION_PORT->OUT, Z_DIRECTION_PIN) = dir_outbits.z;
#elif DIRECTION_OUTMODE == GPIO_MAP
    DIRECTION_PORT->OUT = (DIRECTION_PORT->OUT & ~DIRECTION_MASK) | dir_outmap[dir_outbits.value];
#else
    DIRECTION_PORT->OUT = (DIRECTION_PORT->OUT & ~DIRECTION_MASK) | ((dir_outbits.value << DIRECTION_OUTMODE) ^ settings.steppers.dir_invert.mask);
#endif
}

// Enable/disable stepper motors
static void stepperEnable (axes_signals_t enable, bool hold)
{
#if !TRINAMIC_MOTOR_ENABLE
    enable.value ^= settings.steppers.enable_invert.mask;
    BITBAND_PERI(Z_ENABLE_PORT->OUT, Z_ENABLE_PIN) = enable.z;
    BITBAND_PERI(XY_ENABLE_PORT->OUT, XY_ENABLE_PIN) = enable.x;
#endif
}

// Starts stepper driver ISR timer and forces a stepper driver interrupt callback
static void stepperWakeUp (void)
{
    hal.stepper.enable((axes_signals_t){AXES_BITMASK}, false);
    STEPPER_TIMER->LOAD = hal.f_step_timer / 500; // ~2ms delay to allow drivers time to wake up.
    STEPPER_TIMER->CONTROL |= TIMER32_CONTROL_ENABLE|TIMER32_CONTROL_IE;
}

// Disables stepper driver interrupts
static void stepperGoIdle (bool clear_signals)
{
    STEPPER_TIMER->CONTROL &= ~(TIMER32_CONTROL_ENABLE|TIMER32_CONTROL_IE);
    STEPPER_TIMER->INTCLR = 0;

    if(clear_signals) {
        set_step_outputs((axes_signals_t){0});
        set_dir_outputs((axes_signals_t){0});
    }
}
static void stepperPulseStartDelayed (stepper_t *stepper);
// Sets up stepper driver interrupt timeout, limiting the slowest speed
static void stepperCyclesPerTick (uint32_t cycles_per_tick)
{
    STEPPER_TIMER->LOAD = cycles_per_tick < (1UL << 20) ? cycles_per_tick : 0x000FFFFFUL;
}

// "Normal" version: Sets stepper direction and pulse pins and starts a step pulse a few nanoseconds later.
// If spindle synchronized motion switch to PID version.
static void stepperPulseStart (stepper_t *stepper)
{
    if(stepper->dir_changed.bits) {
        stepper->dir_changed.bits = 0;
        set_dir_outputs(stepper->dir_out);
    }

    if(stepper->step_out.bits) {
        set_step_outputs(stepper->step_out);
        PULSE_TIMER->CTL |= TIMER_A_CTL_CLR|TIMER_A_CTL_MC1;
    }
}

// Delayed pulse version: sets stepper direction and pulse pins and starts a step pulse with an initial delay.
// If spindle synchronized motion switch to PID version.
static void stepperPulseStartDelayed (stepper_t *stepper)
{
    if(stepper->dir_changed.bits) {

        set_dir_outputs(stepper->dir_out);

        if(stepper->step_out.bits) {

            if(stepper->step_out.bits & stepper->dir_changed.bits) {
                next_step_out = stepper->step_out;              // Store out_bits
                PULSE_TIMER->CCR[0] = 0;
                PULSE_TIMER->CCTL[1] &= ~TIMER_A_CCTLN_CCIFG;   // Clear and
                PULSE_TIMER->CCTL[1] |= TIMER_A_CCTLN_CCIE;     // enable CCR1 interrupt
                PULSE_TIMER->CTL |= TIMER_A_CTL_CLR|TIMER_A_CTL_MC1;
            } else {
                set_step_outputs(stepper->step_out);
                PULSE_TIMER->CTL |= TIMER_A_CTL_CLR|TIMER_A_CTL_MC1;
            }
        }

        stepper->dir_changed.bits = 0;

        return;
    }

    if(stepper->step_out.bits) {
        set_step_outputs(stepper->step_out);
        PULSE_TIMER->CTL |= TIMER_A_CTL_CLR|TIMER_A_CTL_MC1;
    }
}

// Enable/disable limit pins interrupt
static void limitsEnable (bool on, axes_signals_t homing_cycle)
{
    bool disable = !on;
    axes_signals_t pin;
    input_signal_t *limit;
    uint_fast8_t idx = limit_inputs.n_pins;
    limit_signals_t homing_source = xbar_get_homing_source_from_cycle(homing_cycle);

    do {
        limit = &limit_inputs.pins.inputs[--idx];
        if(limit->group & (PinGroup_Limit|PinGroup_LimitMax)) {
            if(on && homing_cycle.mask) {
                pin = xbar_fn_to_axismask(limit->id);
                disable = limit->group == PinGroup_Limit ? (pin.mask & homing_source.min.mask) : (pin.mask & homing_source.max.mask);
            }
            BITBAND_PERI(limit->port->IFG, limit->pin) = 0;
            BITBAND_PERI(limit->port->IE, limit->pin) = !disable;
        }
    } while(idx);
}

// Returns limit state as an axes_signals_t variable.
// Each bitfield bit indicates an axis limit, where triggered is 1 and not triggered is 0.
inline static limit_signals_t limitsGetState (void)
{
    limit_signals_t signals = {0};

    signals.min.x = BITBAND_PERI(LIMIT_PORT_X->IN, X_LIMIT_PIN);
    signals.min.y = BITBAND_PERI(LIMIT_PORT_Y->IN, Y_LIMIT_PIN);
    signals.min.z = BITBAND_PERI(LIMIT_PORT_Z->IN, Z_LIMIT_PIN);

    if (settings.limits.invert.mask)
        signals.min.value ^= settings.limits.invert.mask;

    return signals;
}

// Returns system state as a control_signals_t variable.
// Each bitfield bit indicates a control signal, where triggered is 1 and not triggered is 0.
static control_signals_t systemGetState (void)
{
    control_signals_t signals;

    signals.value = settings.control_invert.mask;

  #if ESTOP_ENABLE
    signals.e_stop = BITBAND_PERI(RESET_PORT->IN, RESET_PIN);
  #else
    signals.reset = BITBAND_PERI(RESET_PORT->IN, RESET_PIN);
  #endif
    signals.feed_hold = BITBAND_PERI(FEED_HOLD_PORT->IN, FEED_HOLD_PIN);
    signals.cycle_start = BITBAND_PERI(CYCLE_START_PORT->IN, CYCLE_START_PIN);

#ifdef SAFETY_DOOR_PIN
    if(debounce.safety_door)
        signals.safety_door_ajar = !settings.control_invert.safety_door_ajar;
    else
        signals.safety_door_ajar = BITBAND_PERI(SAFETY_DOOR_PORT->IN, SAFETY_DOOR_PIN);
#endif
#ifdef MOTOR_FAULT_PIN
    signals.motor_fault = BITBAND_PERI(MOTOR_FAULT_PORT->IN, MOTOR_FAULT_PIN);
#endif
#ifdef MOTOR_WARNING_PIN
    signals.motor_warning = BITBAND_PERI(MOTOR_WARNING_PORT->IN, MOTOR_WARNING_PIN);
#endif

    if(settings.control_invert.mask)
        signals.value ^= settings.control_invert.mask;

    return aux_ctrl_scan_status(signals);
}

#if DRIVER_PROBES

static probe_state_t probe_state = { .connected = On };
static probe_t probes[DRIVER_PROBES], *probe = &probes[0];

// Toggle probe connected status. Used when no input pin is available.
static void probeConnectedToggle (void)
{
    probe->flags.connected = !probe_state.connected;
}

// Sets up the probe pin invert mask to
// appropriately set the pin logic according to setting for normal-high/normal-low operation
// and the probing cycle modes for toward-workpiece/away-from-workpiece.
static void probeConfigure (bool is_probe_away, bool probing)
{
    bool invert;

    switch((probe_id_t)probe->probe_id) {
#if TOOLSETTER_ENABLE
        case Probe_Toolsetter:
            invert = settings.probe.invert_toolsetter_input;
            break;
#endif
#if PROBE2_ENABLE
        case Probe_2:
            invert = settings.probe.invert_probe2_input;
            break;
#endif
        default: // Probe_Default
            invert = settings.probe.invert_probe_pin;
            break;
    }

    probe_state.inverted = is_probe_away ? !invert : invert;

    if(probe->flags.latchable) {
        probe_state.is_probing = Off;
        probe_state.triggered = hal.probe.get_state().triggered;
        pin_irq_mode_t irq_mode = probing && !probe_state.triggered ? (probe_state.inverted ? IRQ_Mode_Falling : IRQ_Mode_Rising) : IRQ_Mode_None;
        probe_state.irq_enabled = ioport_enable_irq(probe->port, irq_mode, aux_irq_handler) && irq_mode != IRQ_Mode_None;
    }

    if(!probe_state.irq_enabled)
        probe_state.triggered = Off;

    probe_state.is_probing = probing;
}

// Returns the probe connected and triggered pin states.
static probe_state_t probeGetState (void)
{
    probe_state_t state = {};

    state.probe_id  = probe->probe_id;
    state.connected = probe->flags.connected;

    if(probe_state.is_probing && probe_state.irq_enabled)
        state.triggered = probe_state.triggered;
    else
        state.triggered = BITBAND_PERI(((input_signal_t *)probe->input)->port->IN, ((input_signal_t *)probe->input)->pin) ^ probe_state.inverted;

    return state;
}

static bool probeSelect (probe_id_t probe_id)
{
    bool ok = false;
    uint_fast8_t i = sizeof(probes) / sizeof(probe_t);

    if(!probe_state.is_probing) do {
        i--;
        if((ok = probes[i].probe_id == probe_id && probes[i].input)) {
            probe = &probes[i];
            hal.probe.configure(false, false);
            break;
        }
    } while(i);

    return ok;
}

static bool probe_add (probe_id_t probe_id, uint8_t port, pin_irq_mode_t irq_mode, void *input)
{
    static uint_fast8_t i = 0;

    if(i >= sizeof(probes) / sizeof(probe_t))
        return false;

    bool can_latch;

    if(!(can_latch = (irq_mode & IRQ_Mode_RisingFalling) == IRQ_Mode_RisingFalling))
        hal.signals_cap.probe_triggered = Off;
    else if(i == 0)
        hal.signals_cap.probe_triggered = On;

    probes[i].probe_id = probe_id;
    probes[i].port = port;
    probes[i].flags.connected = probe_state.connected;
    probes[i].flags.latchable = can_latch;
    probes[i].flags.watchable = !!(irq_mode & IRQ_Mode_Change);
    probes[i++].input = input;

    hal.driver_cap.probe_pull_up = On;
    hal.probe.get_state = probeGetState;
    hal.probe.configure = probeConfigure;
    hal.probe.connected_toggle = probeConnectedToggle;

    if(i == 1)
        hal.probe.select = probeSelect;

    return true;
}

#endif // DRIVER_PROBES

#if MPG_ENABLE == 1

static void mpg_select (void *data)
{
    stream_mpg_enable(BITBAND_PERI(MPG_MODE_PORT->IN, MPG_MODE_PIN) == 0);

    BITBAND_PERI(MPG_MODE_PORT->IES, MPG_MODE_PIN) = !sys.mpg_mode;
    BITBAND_PERI(MPG_MODE_PORT->IFG, MPG_MODE_PIN) = 0;
    BITBAND_PERI(MPG_MODE_PORT->IE, MPG_MODE_PIN) = 1;
}

static void mpg_enable (void *data)
{
    bool on = BITBAND_PERI(MPG_MODE_PORT->IN, MPG_MODE_PIN) == 0;

    if(sys.mpg_mode == (BITBAND_PERI(MPG_MODE_PORT->IN, MPG_MODE_PIN) == 0))
        mpg_select(data);

#if I2C_STROBE_ENABLE
    BITBAND_PERI(I2C_STROBE_PORT->IE, I2C_STROBE_PIN) = 1;
#endif
}

#endif //  MPG_ENABLE == 1

static void aux_irq_handler (uint8_t port, bool state)
{
    aux_ctrl_t *pin;
    control_signals_t signals = {0};

    if((pin = aux_ctrl_get_pin(port))) {
        switch(pin->function) {
#if DRIVER_PROBES
  #if PROBE_ENABLE
            case Input_Probe:
  #endif
  #if PROBE2_ENABLE
            case Input_Probe2:
  #endif
  #if TOOLSETTER_ENABLE
            case Input_Toolsetter:
  #endif
                if(probe_state.is_probing) {
                    probe_state.triggered = On;
                    return;
                } else
                    signals.probe_triggered = On;
                break;
#endif
#ifdef I2C_STROBE_PIN
            case Input_I2CStrobe:
                if(i2c_strobe.callback)
                    i2c_strobe.callback(0, DIGITAL_IN(I2C_STROBE_PORT, I2C_STROBE_PIN) == 0);
                break;
#endif
#ifdef MPG_MODE_PIN
            case Input_MPGSelect:
                task_add_immediate(mpg_select, NULL);
                break;
#endif
            default:
                break;
        }
        signals.mask |= pin->cap.mask;
        if(!signals.probe_triggered && pin->irq_mode == IRQ_Mode_Change)
            signals.deasserted = hal.port.wait_on_input(Port_Digital, pin->aux_port, WaitMode_Immediate, 0.0f) == 0;
    }

    if(signals.mask) {
        if(!signals.deasserted)
            signals.mask |= systemGetState().mask;
        hal.control.interrupt_callback(signals);
    }
}

static bool aux_claim_explicit (aux_ctrl_t *aux_ctrl)
{
    xbar_t *pin;

    if(aux_ctrl->input == NULL) {

        uint_fast8_t i = sizeof(inputpin) / sizeof(input_signal_t);

        do {
            --i;
            if(inputpin[i].group == PinGroup_AuxInput && inputpin[i].user_port == aux_ctrl->aux_port)
                aux_ctrl->input = &inputpin[i];
        } while(i && aux_ctrl->input == NULL);
    }

    if(aux_ctrl->input &&(pin = ioport_claim(Port_Digital, Port_Input, &aux_ctrl->aux_port, NULL))) {

        ioport_set_function(pin, aux_ctrl->function, &aux_ctrl->cap);

        switch(aux_ctrl->function) {
#if PROBE_ENABLE
            case Input_Probe:
                hal.driver_cap.probe = probe_add(Probe_Default, aux_ctrl->aux_port, (pin_irq_mode_t)pin->cap.irq_mode, aux_ctrl->input);
                break;
#endif
#if PROBE2_ENABLE
            case Input_Probe2:
                hal.driver_cap.probe2 = probe_add(Probe_2, aux_ctrl->aux_port, (pin_irq_mode_t)pin->cap.irq_mode, aux_ctrl->input);
                break;

#endif
#if TOOLSETTER_ENABLE
            case Input_Toolsetter:
                hal.driver_cap.toolsetter = probe_add(Probe_Toolsetter, aux_ctrl->aux_port, (pin_irq_mode_t)pin->cap.irq_mode, aux_ctrl->input);
                break;
#endif
#if SAFETY_DOOR_ENABLE
            case Input_SafetyDoor:
                ((input_signal_t *)aux_ctrl->input)->mode.debounce = ((input_signal_t *)aux_ctrl->input)->cap.debounce = On;
                break;
#endif
            default: break;
        }
    } else
        aux_ctrl->aux_port = 0xFF;

    return aux_ctrl->aux_port != 0xFF;
}

bool aux_out_claim_explicit (aux_ctrl_out_t *aux_ctrl)
{
    xbar_t *pin;

    if((pin = ioport_claim(Port_Digital, Port_Output, &aux_ctrl->aux_port, NULL)))
        ioport_set_function(pin, aux_ctrl->function, NULL);
    else
        aux_ctrl->aux_port = 0xFF;

    return aux_ctrl->aux_port != 0xFF;
}

#if SPINDLE_ENCODER_ENABLE

inline static float spindle_calc_rpm (uint32_t tpp)
{
    return spindle_encoder.rpm_factor / (float)tpp;
}

#endif

#if DRIVER_SPINDLE_ENABLE

// Static spindle (off, on cw & on ccw)

inline static void spindle_off (spindle_ptrs_t *spindle)
{
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
    spindle->context.pwm->flags.enable_out = Off;
  #ifdef SPINDLE_DIRECTION_PIN
    if(spindle->context.pwm->flags.cloned) {
        BITBAND_PERI(SPINDLE_DIRECTION_PORT->OUT, SPINDLE_DIRECTION_PIN) = settings.pwm_spindle.invert.ccw;
    } else {
        BITBAND_PERI(SPINDLE_ENABLE_PORT->OUT, SPINDLE_ENABLE_PIN) = settings.pwm_spindle.invert.on;
        BITBAND_PERI(SPINDLE_DIRECTION_PORT->OUT, SPINDLE_DIRECTION_PIN) = settings.pwm_spindle.invert.ccw;
    }
  #elif defined(SPINDLE_ENABLE_PIN)
    BITBAND_PERI(SPINDLE_ENABLE_PORT->OUT, SPINDLE_ENABLE_PIN) = settings.pwm_spindle.invert.on;
    BITBAND_PERI(SPINDLE_DIRECTION_PORT->OUT, SPINDLE_DIRECTION_PIN) = settings.pwm_spindle.invert.ccw;
  #endif
#else
    BITBAND_PERI(SPINDLE_ENABLE_PORT->OUT, SPINDLE_ENABLE_PIN) = settings.pwm_spindle.invert.on;
    BITBAND_PERI(SPINDLE_DIRECTION_PORT->OUT, SPINDLE_DIRECTION_PIN) = settings.pwm_spindle.invert.ccw;
#endif
}

inline static void spindle_on (spindle_ptrs_t *spindle)
{
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
  #ifdef SPINDLE_DIRECTION_PIN
    if(spindle->context.pwm->flags.cloned) {
        BITBAND_PERI(SPINDLE_DIRECTION_PORT->OUT, SPINDLE_DIRECTION_PIN) = !settings.pwm_spindle.invert.ccw;
    } else {
        BITBAND_PERI(SPINDLE_ENABLE_PORT->OUT, SPINDLE_ENABLE_PIN) = !settings.pwm_spindle.invert.on;
    }
  #elif defined(SPINDLE_ENABLE_PIN)
    BITBAND_PERI(SPINDLE_ENABLE_PORT->OUT, SPINDLE_ENABLE_PIN) = !settings.pwm_spindle.invert.on;
  #endif
  #if SPINDLE_ENCODER_ENABLE
    if(!spindle->context.pwm->flags.enable_out && spindle->reset_data)
        spindle->reset_data();
  #endif
    spindle->context.pwm->flags.enable_out = On;
#else
    BITBAND_PERI(SPINDLE_ENABLE_PORT->OUT, SPINDLE_ENABLE_PIN) = !settings.pwm_spindle.invert.on;
#endif
}

inline static void spindle_dir (bool ccw)
{
    BITBAND_PERI(SPINDLE_DIRECTION_PORT->OUT, SPINDLE_DIRECTION_PIN) = ccw ^ settings.pwm_spindle.invert.ccw;
}

// Start or stop spindle
static void spindleSetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    if(!state.on)
        spindle_off(spindle);
    else {
        spindle_dir(state.ccw);
        spindle_on(spindle);
    }
}

#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM

// Variable spindle control functions

static void pwm_off (spindle_ptrs_t *spindle)
{
    if(spindle->context.pwm->flags.always_on) {
        SPINDLE_PWM_TIMER->CCR[2] = spindle->context.pwm->off_value;
        SPINDLE_PWM_TIMER->CCTL[2] = settings.pwm_spindle.invert.pwm ? TIMER_A_CCTLN_OUTMOD_6 : TIMER_A_CCTLN_OUTMOD_2;
    } else
        SPINDLE_PWM_TIMER->CCTL[2] = settings.pwm_spindle.invert.pwm ? TIMER_A_CCTLN_OUT : 0; // Set PWM output according to invert setting
}

// Sets spindle speed
static void spindleSetSpeed (spindle_ptrs_t *spindle, uint_fast16_t pwm_value)
{
    while(spindleLock); // wait for PID

    if(pwm_value == spindle->context.pwm->off_value) {

        if(spindle->context.pwm->flags.rpm_controlled) {
            spindle_off(spindle);
            if(spindle->context.pwm->flags.laser_off_overdrive)
                SPINDLE_PWM_TIMER->CCR[2] = spindle->context.pwm->pwm_overdrive;
        } else
            pwm_off(spindle);
#ifdef SPINDLE_RPM_CONTROLLED
        spindle_control.pid.error = 0.0f;
#endif
    } else {

        if(!spindle->context.pwm->flags.enable_out && spindle->context.pwm->flags.rpm_controlled)
            spindle_on(spindle);

        SPINDLE_PWM_TIMER->CCR[2] = pwm_value;
        SPINDLE_PWM_TIMER->CCTL[2] = settings.pwm_spindle.invert.pwm ? TIMER_A_CCTLN_OUTMOD_6 : TIMER_A_CCTLN_OUTMOD_2;
    }
}

#ifdef SPINDLE_RPM_CONTROLLED
// todo: remove?
static void spindleUpdateRPM (spindle_ptrs_t *spindle, float rpm)
{
    while(spindleLock); // wait for PID

    spindleSetSpeed(spindle->context.pwm->compute_value(spindle->context.pwm, rpm + spindle_control.pid.error, spindle_control.pid.error != 0.0f));
    if(spindle->context.pwm->settings->at_speed_tolerance > 0.0f) {
        spindle_data.rpm_low_limit = rpm / (1.0f + spindle->context.pwm->settings->at_speed_tolerance);
        spindle_data.rpm_high_limit = rpm * (1.0f + spindle->context.pwm->settings->at_speed_tolerance);
    }
    spindle_data.rpm_programmed = spindle_data.rpm = rpm;
}

#else

static uint_fast16_t spindleGetPWM (spindle_ptrs_t *spindle, float rpm)
{
    return spindle->context.pwm->compute_value(spindle->context.pwm, rpm, false);
}

#endif

// Start or stop spindle
static void spindleSetStateVariable (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    if(!(spindle->context.pwm->flags.cloned ? state.ccw : state.on)) {
        spindle_off(spindle);
        pwm_off(spindle);
    } else {
#ifdef SPINDLE_DIRECTION_PIN
        if(!spindle->context.pwm->flags.cloned)
            spindle_dir(state.ccw);
#endif
        if(rpm == 0.0f && spindle->context.pwm->flags.rpm_controlled)
            spindle_off(spindle);
        else {
            spindle_on(spindle);
            spindleSetSpeed(spindle, spindle->context.pwm->compute_value(spindle->context.pwm, rpm, false));
        }
    }

#ifdef SPINDLE_RPM_CONTROLLED
    if (!state.on || rpm == 0.0f) {
        spindleSetSpeed(spindle, spindle->context.pwm->off_value);
        spindle_control.rpm = 0.0f;
        spindle_control.pid_state = PIDState_Disabled;
        pidf_reset(&spindle_control.pid);
        spindle_control.pid.sample_rate_prev = 1.0f;
    } else {
        if(spindle_data.rpm_programmed == 0.0f) {
            if(spindle_control.pid_enabled) {
                pid_count = 0;
                spindle_control.pid_state = PIDState_Pending;
            }
        }
  #ifdef sPID_LOG
        sys.pid_log.idx = 0;
  #endif
        spindleSetSpeed(spindle, spindle->context.pwm->compute_value(spindle->context.pwm, rpm + spindle_control.pid.error, spindle_control.pid.error != 0.0f));
    }
#endif

#if SPINDLE_ENCODER_ENABLE
    if(spindle->at_speed_tolerance > 0.0f) {
        float tolerance = rpm * spindle->at_speed_tolerance / 100.0f;
        spindle_data.rpm_low_limit = rpm - tolerance;
        spindle_data.rpm_high_limit = rpm + tolerance;
    }
    spindle_data.state_programmed.on = state.on;
    spindle_data.state_programmed.ccw = state.ccw;
    spindle_data.rpm_programmed = spindle_data.rpm = rpm;
#endif
}

#endif // DRIVER_SPINDLE_ENABLE & SPINDLE_PWM

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (spindle_ptrs_t *spindle)
{
    spindle_state_t state = { settings.pwm_spindle.invert.mask };

//    state.on = (SPINDLE_ENABLE_PORT->IN & SPINDLE_ENABLE_BIT) != 0;
    state.on = BITBAND_PERI(SPINDLE_ENABLE_PORT->IN, SPINDLE_ENABLE_PIN);
    state.ccw = BITBAND_PERI(SPINDLE_DIRECTION_PORT->IN, SPINDLE_DIRECTION_PIN);
    state.value ^= settings.pwm_spindle.invert.mask;
#ifdef SPINDLE_PWM_PIN
    state.on |= spindle->param->state.on;
#endif

#if SPINDLE_ENCODER_ENABLE
    if(spindle && spindle->get_data) {
        spindle_data_t *spindle_data = spindle->get_data(SpindleData_AtSpeed);
        state.at_speed = spindle_data->state_programmed.at_speed;
        state.encoder_error = spindle_data->error_count > 0;
    }
#else
    UNUSED(spindle);
#endif

    return state;
}

#ifdef SPINDLE_RPM_CONTROLLED

inline static void spindle_rpm_pid (uint32_t tpp)
{
    spindleLock = true;

    spindle_control.rpm = spindle_calc_rpm(tpp);
    float error = pidf(&spindle_control.pid, spindle_data.rpm_programmed, spindle_control.rpm, 1.0);

#ifdef sPID_LOG
    if(sys.pid_log.idx < PID_LOG) {
        sys.pid_log.target[sys.pid_log.idx] = error;
        sys.pid_log.actual[sys.pid_log.idx] = rpm;
        sys.pid_log.idx++;
    }
#endif

    SPINDLE_PWM_TIMER->CCR[2] = spindle_pwm.compute_value(&spindle_pwm, spindle_data.rpm_programmed + error, error != 0.0f);

    spindleLock = false;
}

#endif

bool spindleConfig (spindle_ptrs_t *spindle)
{
    if(spindle == NULL)
        return false;

    if(spindle_precompute_pwm_values(spindle, &spindle_pwm, &settings.pwm_spindle, 12000000UL / (settings.pwm_spindle.pwm_freq > 200.0f ? 2 : 16))) {

        spindle->set_state = spindleSetStateVariable;

        if(settings.pwm_spindle.pwm_freq > 200.0f)
            SPINDLE_PWM_TIMER->CTL &= ~TIMER_A_CTL_ID__8;
        else
            SPINDLE_PWM_TIMER->CTL |= TIMER_A_CTL_ID__8;

        SPINDLE_PWM_TIMER->CCR[0] = spindle_pwm.period;
        SPINDLE_PWM_TIMER->CCTL[2] = settings.pwm_spindle.invert.pwm ? TIMER_A_CCTLN_OUT : 0;   // Set PWM output according to invert setting and
        SPINDLE_PWM_TIMER->CTL |= TIMER_A_CTL_CLR|TIMER_A_CTL_MC0|TIMER_A_CTL_MC1;              // start PWM timer (with no pulse output)

        spindle->set_state = spindleSetStateVariable;
    } else {
        if(spindle->param->state.on)
            spindle->set_state(spindle, (spindle_state_t){0}, 0.0f);
        spindle->set_state = spindleSetState;
    }

    spindle->cap.at_speed = spindle->cap.variable && settings.spindle.ppr > 0;

    spindle_update_caps(spindle, spindle->cap.variable ? &spindle_pwm : NULL);

  #ifdef SPINDLE_RPM_CONTROLLED

    if((spindle_control.pid_enabled = spindle->get_data && (settings.spindle.pid.p_gain != 0.0) || pidf_config_changed(&spindle_control.pid, &settings.spindle.pid))) {
        spindle->set_state((spindle_state_t){0}, 0.0f);
        pidf_init(&spindle_control.pid, &settings.spindle.pid);
  //      spindle_encoder.pid.cfg.i_max_error = spindle_encoder.pid.cfg.i_max_error / settings->spindle.pid.i_gain; // Makes max value sensible?
    } else
        spindle_control.pid_state = PIDState_Disabled;

  #endif

    return true;
}

#endif // DRIVER_SPINDLE_ENABLE & SPINDLE_PWM

#if SPINDLE_ENCODER_ENABLE

static spindle_data_t *spindleGetData (spindle_data_request_t request)
{
    bool stopped;
    uint32_t pulse_length, rpm_timer_delta;
    spindle_encoder_counter_t encoder;

    while(spindle_encoder.spin_lock);

    __disable_irq();

    memcpy(&encoder, &spindle_encoder.counter, sizeof(spindle_encoder_counter_t));

    pulse_length = spindle_encoder.timer.pulse_length / spindle_encoder.tics_per_irq;
    rpm_timer_delta = spindle_encoder.timer.last_pulse - RPM_TIMER->VALUE; // NOTE: timer is counting down!

    __enable_irq();

    // If no (4) spindle pulses during last 250mS assume RPM is 0
    if((stopped = ((pulse_length == 0) || (rpm_timer_delta > spindle_encoder.maximum_tt)))) {
        spindle_data.rpm = 0.0f;
        rpm_timer_delta = (RPM_COUNTER->R - (uint16_t)spindle_encoder.counter.last_count) * pulse_length;
    }

    switch(request) {

        case SpindleData_Counters:
            spindle_data.index_count = encoder.index_count;
            spindle_data.pulse_count = encoder.pulse_count + (uint32_t)((uint16_t)RPM_COUNTER->R - (uint16_t)encoder.last_count);
            spindle_data.error_count = spindle_encoder.error_count;
            break;

        case SpindleData_RPM:
            if(!stopped)
#ifdef SPINDLE_RPM_CONTROLLED
                spindle_data.rpm = spindle_control.pid_enabled ? spindle_control.rpm : spindle_calc_rpm(pulse_length);
#else
                spindle_data.rpm = spindle_calc_rpm(pulse_length);
#endif
            break;

        case SpindleData_AtSpeed:
#ifdef SPINDLE_RPM_CONTROLLED
            spindle_validate_at_speed(spindle_data, stopped ? 0.0f : (spindle_control.pid_enabled ? spindle_control.rpm : spindle_calc_rpm(pulse_length)));
#else
            spindle_validate_at_speed(spindle_data, stopped ? 0.0f : spindle_calc_rpm(pulse_length));
#endif
            spindle_data.state_programmed.encoder_error = spindle_encoder.error_count > 0;
            break;

        case SpindleData_AngularPosition:;
            int32_t d = (uint16_t)((uint16_t)encoder.last_count - (uint16_t)encoder.last_index);
            if(d < 0)
                d++;
            spindle_data.angular_position = (float)encoder.index_count +
                    ((float)((uint16_t)encoder.last_count - (uint16_t)encoder.last_index) +
                              (pulse_length == 0 ? 0.0f : (float)rpm_timer_delta / (float)pulse_length)) *
                                spindle_encoder.pulse_distance;
            if(spindle_data.angular_position < (float)encoder.index_count)
                d++;
            break;
    }

    return &spindle_data;
}

static void spindleDataReset (void)
{
    while(spindleLock);

    uint32_t systick_state = SysTick->CTRL;

    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
/*
    uint32_t index_count = spindle_data.index_count + 2, timeout = elapsed_tics + 1000;
    if(spindleGetData(SpindleData_RPM).rpm > 0.0f) { // wait for index pulse if running

        while(index_count != spindle_data.index_count && elapsed_tics <= timeout);

//        if(uwTick > timeout)
//            alarm?
    }
*/

#ifdef SPINDLE_RPM_CONTROLLED
    if(spindle_control.pid_enabled)
        spindle_control.pid_state = PIDState_Pending;
#endif

    RPM_TIMER->LOAD = (uint32_t)-1L; // Reload RPM timer
    RPM_COUNTER->CTL = 0;

    spindle_encoder.timer.last_pulse =
    spindle_encoder.timer.last_index = RPM_TIMER->VALUE;

    spindle_encoder.timer.pulse_length =
    spindle_encoder.counter.last_count =
    spindle_encoder.counter.last_index =
    spindle_encoder.counter.pulse_count =
    spindle_encoder.counter.index_count =
    spindle_encoder.error_count = 0;

    RPM_COUNTER->CCR[0] = spindle_encoder.tics_per_irq;
    RPM_COUNTER->CTL = TIMER_A_CTL_MC__CONTINUOUS|TIMER_A_CTL_CLR;

    spindle_encoder.counter.last_count = RPM_COUNTER->R;

    if(systick_state & SysTick_CTRL_ENABLE_Msk)
        SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

static void onSpindleProgrammed (spindle_ptrs_t *spindle, spindle_state_t state, float rpm, spindle_rpm_mode_t mode)
{
    if(on_spindle_programmed)
        on_spindle_programmed(spindle, state, rpm, mode);

    if(spindle->get_data == spindleGetData) {
        if((spindle_data.at_speed_enabled = spindle->at_speed_tolerance > 0.0f)) {
            float tolerance = rpm * spindle->at_speed_tolerance / 100.0f;
            spindle_data.rpm_low_limit = rpm - tolerance;
            spindle_data.rpm_high_limit = rpm + tolerance;
        }
        spindle_data.state_programmed.on = state.on;
        spindle_data.state_programmed.ccw = state.ccw;
        spindle_data.rpm_programmed = spindle_data.rpm = rpm;
    }
}

#endif // SPINDLE_ENCODER_ENABLE

// Start/stop coolant (and mist if enabled)
static void coolantSetState (coolant_state_t mode)
{
    mode.value ^= settings.coolant.invert.mask;
    BITBAND_PERI(COOLANT_FLOOD_PORT->OUT, COOLANT_FLOOD_PIN) = mode.flood;
    BITBAND_PERI(COOLANT_MIST_PORT->OUT, COOLANT_MIST_PIN) = mode.mist;
}

// Returns coolant state in a coolant_state_t variable
static coolant_state_t coolantGetState (void)
{
    coolant_state_t state = {0};

    state.flood = BITBAND_PERI(COOLANT_FLOOD_PORT->IN, COOLANT_FLOOD_PIN);
    state.mist  = BITBAND_PERI(COOLANT_MIST_PORT->IN, COOLANT_MIST_PIN);
    state.value ^= settings.coolant.invert.mask;

    return state;
}

static void enable_irq (void)
{
    __enable_irq();
}

static void disable_irq (void)
{
    __disable_irq();
}

// Helper functions for setting/clearing/inverting individual bits atomically (uninterruptable)
static void bitsSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    __disable_irq();
    *ptr |= bits;
    __enable_irq();
}

static uint_fast16_t bitsClearAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    __disable_irq();
    uint_fast16_t prev = *ptr;
    *ptr &= ~bits;
    __enable_irq();
    return prev;
}

static uint_fast16_t valueSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t value)
{
    __disable_irq();
    uint_fast16_t prev = *ptr;
    *ptr = value;
    __enable_irq();
    return prev;
}

uint32_t getElapsedTicks (void)
{
    return elapsed_tics;
}

// Configure perhipherals when settings are initialized or changed
void settings_changed (settings_t *settings, settings_changed_flags_t changed)
{
    spindle_ptrs_t *spindle = spindle_get(spindle_id);

#if USE_STEPDIR_MAP
    stepdirmap_init(settings);
#endif

    if(IOInitDone) {

#if SPINDLE_ENCODER_ENABLE

        static const spindle_data_ptrs_t encoder_data = {
            .get = spindleGetData,
            .reset = spindleDataReset
        };

        static bool event_claimed = false;

        if((hal.spindle_data.get = settings->spindle.ppr > 0 ? spindleGetData : NULL)) {
            if(spindle_encoder.ppr != settings->spindle.ppr) {

                spindle_ptrs_t *spindle;

                hal.spindle_data.reset = spindleDataReset;
                if((spindle = spindle_get(0)))
                    spindle->set_state(spindle, (spindle_state_t){0}, 0.0f);

                if(!event_claimed) {
                    event_claimed = true;
                    on_spindle_programmed = grbl.on_spindle_programmed;
                    grbl.on_spindle_programmed = onSpindleProgrammed;
                }

                float timer_resolution = 1.0f / (float)(SystemCoreClock / 16);

                spindle_encoder.ppr = settings->spindle.ppr;
                spindle_encoder.tics_per_irq = 4;
                spindle_encoder.pulse_distance = 1.0f / spindle_encoder.ppr;
                spindle_encoder.maximum_tt = (uint32_t)(0.25f / timer_resolution) * spindle_encoder.tics_per_irq; // 250 mS
                spindle_encoder.rpm_factor = 60.0f / ((timer_resolution * (float)spindle_encoder.ppr));
                BITBAND_PERI(SPINDLE_INDEX_PORT->IES, SPINDLE_INDEX_PIN) = 1;
                BITBAND_PERI(SPINDLE_INDEX_PORT->IE, SPINDLE_INDEX_PIN) = 1;
                spindleDataReset();
            }
        } else {
            spindle_encoder.ppr = 0;
            hal.spindle_data.reset = NULL;
        }

        spindle_bind_encoder(spindle_encoder.ppr ? &encoder_data : NULL);

#endif // SPINDLE_ENCODER_ENABLE

#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
        if(changed.spindle) {
            spindleConfig(spindle_get_hal(spindle_id, SpindleHAL_Configured));
            if(spindle_id == spindle_get_default())
                spindle_select(spindle_id);
        }
#endif
#if SPINDLE_ENCODER_ENABLE
        if(!hal.spindle_data.get)
            BITBAND_PERI(SPINDLE_INDEX_PORT->IE, SPINDLE_INDEX_PIN) = 0;
#endif

        hal.stepper.go_idle(true);

        pulse_length = (uint16_t)(12.0f * (settings->steppers.pulse_microseconds - STEP_PULSE_LATENCY));

        if(hal.driver_cap.step_pulse_delay && settings->steppers.pulse_delay_microseconds > 0.0f) {
            int16_t pulse_delay = (uint16_t)(12.0f * (settings->steppers.pulse_delay_microseconds - 1.2f));
            PULSE_TIMER->CCR[1] = pulse_delay < 2 ? 2 : pulse_delay;
            hal.stepper.pulse_start = stepperPulseStartDelayed;
        } else
            hal.stepper.pulse_start = stepperPulseStart;

        PULSE_TIMER->CCTL[1] &= ~TIMER_A_CCTLN_CCIE; // Disable CCR1 (step delay) interrupt
        PULSE_TIMER->CCR[0] = pulse_length;

        /*************************
         *  Control pins config  *
         *************************/

        uint32_t i = sizeof(inputpin) / sizeof(input_signal_t), p1_count = 0, p2_count = 0, p3_count = 0, p4_count = 0, p5_count = 0, p6_count = 0;
        input_signal_t *input;

        control_signals_t control_fei;
        control_fei.mask = settings->control_disable_pullup.mask ^ settings->control_invert.mask;

        axes_signals_t limit_fei;
        limit_fei.mask = settings->limits.disable_pullup.mask ^ settings->limits.invert.mask;

        do {

            input = &inputpin[--i];
            if(input->group != PinGroup_AuxInput)
                input->mode.irq_mode = IRQ_Mode_None;

            BITBAND_PERI(input->port->IE, input->pin) = 0;

            switch(input->id) {
#if ESTOP_ENABLE
                case Input_EStop:
                    input->mode.pull_mode = settings->control_disable_pullup.e_stop ? PullMode_Down : PullMode_Up;
                    input->mode.irq_mode = control_fei.e_stop ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;
#else
                case Input_Reset:
                    input->mode.pull_mode = settings->control_disable_pullup.reset ? PullMode_Down : PullMode_Up;
                    input->mode.irq_mode = control_fei.reset ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;
#endif
                case Input_FeedHold:
                    input->mode.pull_mode = settings->control_disable_pullup.feed_hold ? PullMode_Down : PullMode_Up;
                    input->mode.irq_mode = control_fei.feed_hold ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_CycleStart:
                    input->mode.pull_mode = settings->control_disable_pullup.cycle_start ? PullMode_Down : PullMode_Up;
                    input->mode.irq_mode = control_fei.cycle_start ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_LimitX:
                case Input_LimitX_2:
                case Input_LimitX_Max:
                    input->mode.pull_mode = settings->limits.disable_pullup.x ? PullMode_Down : PullMode_Up;
                    input->mode.irq_mode = limit_fei.x ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_LimitY:
                case Input_LimitY_2:
                case Input_LimitY_Max:
                    input->mode.pull_mode = settings->limits.disable_pullup.y ? PullMode_Down : PullMode_Up;
                    input->mode.irq_mode = limit_fei.y ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_LimitZ:
                case Input_LimitZ_2:
                case Input_LimitZ_Max:
                    input->mode.pull_mode = settings->limits.disable_pullup.z ? PullMode_Down : PullMode_Up;
                    input->mode.irq_mode = limit_fei.z ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_LimitA:
                case Input_LimitA_Max:
                    input->mode.pull_mode = settings->limits.disable_pullup.a ? PullMode_Down : PullMode_Up;
                    input->mode.irq_mode = limit_fei.a ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_LimitB:
                case Input_LimitB_Max:
                    input->mode.pull_mode = settings->limits.disable_pullup.b ? PullMode_Down : PullMode_Up;
                    input->mode.irq_mode = limit_fei.b ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_LimitC:
                case Input_LimitC_Max:
                    input->mode.pull_mode = settings->limits.disable_pullup.c ? PullMode_Down : PullMode_Up;
                    input->mode.irq_mode = limit_fei.c ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_LimitsOverride:
                    input->mode.pull_mode = PullMode_Up;
                    break;

                case Input_SpindleIndex:
                    input->mode.pull_mode = PullMode_Up;
                    input->mode.irq_mode = IRQ_Mode_Falling;
                    break;

                default:
                    break;
            }

            BITBAND_PERI(input->port->DIR, input->pin) = 0;
            BITBAND_PERI(input->port->REN, input->pin) = 1;
            BITBAND_PERI(input->port->OUT, input->pin) = input->mode.pull_mode == PullMode_Up;

            if(input->mode.irq_mode != IRQ_Mode_None || input->group == PinGroup_AuxInput) {

                switch((uint32_t)input->port) {

                    case((uint32_t)PA):
                        if(input->pin < 8)
                            p1_pins[p1_count++] = input;
                        else
                            p2_pins[p2_count++] = input;
                        break;

                    case((uint32_t)PB):
                        if(input->pin < 8)
                            p3_pins[p3_count++] = input;
                        else
                            p4_pins[p4_count++] = input;
                        break;

                    case((uint32_t)PC):
                        if(input->pin < 8)
                            p5_pins[p5_count++] = input;
                        else
                            p6_pins[p6_count++] = input;
                        break;
                }

                if(input->mode.irq_mode != IRQ_Mode_None) {
                    BITBAND_PERI(input->port->IES, input->pin) = input->mode.irq_mode == IRQ_Mode_Change
                                                                  ? BITBAND_PERI(input->port->IN, input->pin)
                                                                  : input->mode.irq_mode == IRQ_Mode_Falling;
                    BITBAND_PERI(input->port->IFG, input->pin) = 0;
                    if(input->group != PinGroup_Limit)
                        BITBAND_PERI(input->port->IE, input->pin) = 1;
                }
            }

        } while(i);

        if(p1_count)
            NVIC_EnableIRQ(PORT1_IRQn);
        if(p2_count)
            NVIC_EnableIRQ(PORT2_IRQn);
        if(p3_count)
            NVIC_EnableIRQ(PORT3_IRQn);
        if(p4_count)
            NVIC_EnableIRQ(PORT4_IRQn);
        if(p5_count)
            NVIC_EnableIRQ(PORT5_IRQn);
        if(p6_count)
            NVIC_EnableIRQ(PORT6_IRQn);

        hal.limits.enable(settings->limits.flags.hard_enabled, (axes_signals_t){0});
        aux_ctrl_irq_enable(settings, aux_irq_handler);

        /***************************
         *  MPG mode input enable  *
         ***************************/

#if MPG_ENABLE == 1
        if(hal.driver_cap.mpg_mode) {
            // Enable pullup and switch to input
            BITBAND_PERI(MPG_MODE_PORT->OUT, MPG_MODE_PIN) = 1;
            BITBAND_PERI(MPG_MODE_PORT->REN, MPG_MODE_PIN) = 1;
            BITBAND_PERI(MPG_MODE_PORT->DIR, MPG_MODE_PIN) = 0;
        }
#endif
    }
}

static char *port2char (void *port, uint8_t pin)
{
    switch((uint32_t)port) {

        case((uint32_t)PA):
            return pin <= 7 ? "P1." : "P2.";

        case((uint32_t)PB):
            return pin <= 7 ? "P3." : "P4.";

        case((uint32_t)PC):
            return pin <= 7 ? "P5." : "P6.";

        case((uint32_t)PD):
            return pin <= 7 ? "P7." : "P8.";
    }

    return "?";
}

static void enumeratePins (bool low_level, pin_info_ptr pin_info, void *data)
{
    static xbar_t pin = {};
    uint32_t i, id = 0;

    pin.mode.input = On;

    for(i = 0; i < sizeof(inputpin) / sizeof(input_signal_t); i++) {
        pin.id = id++;
        pin.pin = inputpin[i].pin > 7 ? inputpin[i].pin - 8 : inputpin[i].pin;
        pin.function = inputpin[i].id;
        pin.group = inputpin[i].group;
        pin.port = low_level ? (void *)inputpin[i].port : (void *)port2char(inputpin[i].port, inputpin[i].pin);
        pin.mode.pwm = pin.group == PinGroup_SpindlePWM;
        pin.description = inputpin[i].description;

        pin_info(&pin, data);
    };

    pin.mode.mask = 0;
    pin.mode.output = On;

    for(i = 0; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
        pin.id = id++;
        pin.pin = outputpin[i].pin > 7 ? outputpin[i].pin - 8 : outputpin[i].pin;
        pin.function = outputpin[i].id;
        pin.group = outputpin[i].group;
        pin.port = low_level ? (void *)outputpin[i].port : (void *)port2char(outputpin[i].port, outputpin[i].pin);
        pin.description = outputpin[i].description;

        pin_info(&pin, data);
    };

    periph_signal_t *ppin = periph_pins;

    if(ppin) do {
        pin.id = id++;
        pin.pin = ppin->pin.pin > 7 ? ppin->pin.pin - 8 : ppin->pin.pin;
        pin.function = ppin->pin.function;
        pin.group = ppin->pin.group;
        pin.port = low_level ? ppin->pin.port : (void *)port2char(ppin->pin.port, ppin->pin.pin);
        pin.mode = ppin->pin.mode;
        pin.description = ppin->pin.description;

        pin_info(&pin, data);
    } while(ppin = ppin->next);
}

void registerPeriphPin (const periph_pin_t *pin)
{
    periph_signal_t *add_pin = malloc(sizeof(periph_signal_t));

    if(!add_pin)
        return;

    memcpy(&add_pin->pin, pin, sizeof(periph_pin_t));
    add_pin->next = NULL;

    if(periph_pins == NULL) {
        periph_pins = add_pin;
    } else {
        periph_signal_t *last = periph_pins;
        while(last->next)
            last = last->next;
        last->next = add_pin;
    }
}

void setPeriphPinDescription (const pin_function_t function, const pin_group_t group, const char *description)
{
    periph_signal_t *ppin = periph_pins;

    if(ppin) do {
        if(ppin->pin.function == function && ppin->pin.group == group) {
            ppin->pin.description = description;
            ppin = NULL;
        } else
            ppin = ppin->next;
    } while(ppin);
}

// Initializes MCU peripherals for grblHAL use
static bool driver_setup (settings_t *settings)
{
    /*************************
     *  Output signals init  *
     *************************/

    uint32_t i;
    output_signal_t *output;
    for(i = 0 ; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
        output = &outputpin[i];
        BITBAND_PERI(output->port->DIR, output->pin) = 1;
        if(output->group == PinGroup_StepperPower) {
            BITBAND_PERI(output->port->DS, output->pin) = 1;
            BITBAND_PERI(output->port->OUT, output->pin) = 1;
        }
    }

 // Stepper init

    STEPPER_TIMER->CONTROL = TIMER32_CONTROL_SIZE|TIMER32_CONTROL_MODE;

    PULSE_TIMER->CTL = TIMER_A_CTL_SSEL__SMCLK|TIMER_A_CTL_CLR; // CLK: 12Mhz = 833.33... ns
    PULSE_TIMER->CCTL[0] |= TIMER_A_CCTLN_CCIE;

    NVIC_EnableIRQ(STEPPER_TIMER_INT);  // Enable stepper interrupt and
    NVIC_EnableIRQ(PULSE_TIMER_INT0);   // step pulse interrupts
    NVIC_EnableIRQ(PULSE_TIMER_INTN);   // ...

    NVIC_SetPriority(PULSE_TIMER_INT0, 1);
    NVIC_SetPriority(PULSE_TIMER_INTN, 1);
    NVIC_SetPriority(STEPPER_TIMER_INT, 2);

 // Spindle init

#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM

    ((DIO_PORT_Even_Interruptable_Type *)SPINDLE_PWM_PORT)->DIR |= (1 << SPINDLE_PWM_PIN);
    ((DIO_PORT_Even_Interruptable_Type *)SPINDLE_PWM_PORT)->SEL1 &= ~(1 << SPINDLE_PWM_PIN);
    ((DIO_PORT_Even_Interruptable_Type *)SPINDLE_PWM_PORT)->SEL0 |= (1 << SPINDLE_PWM_PIN);
    SPINDLE_PWM_TIMER->CTL = TIMER_A_CTL_SSEL__SMCLK;
    SPINDLE_PWM_TIMER->EX0 = 0;

#endif // DRIVER_SPINDLE_ENABLE & SPINDLE_PWM

#if SPINDLE_ENCODER_ENABLE

    memset(&spindle_encoder, 0, sizeof(spindle_encoder_t));
    memset(&spindle_data, 0, sizeof(spindle_data));

    SPINDLE_PULSE_PORT->SEL0 |= SPINDLE_PULSE_BIT; // Set as counter input
    RPM_COUNTER->CTL = TIMER_A_CTL_MC__CONTINUOUS|TIMER_A_CTL_CLR;
    RPM_COUNTER->CCTL[0] = TIMER_A_CCTLN_CCIE;
    RPM_COUNTER->CCR[0] = spindle_encoder.tics_per_irq;

    NVIC_EnableIRQ(SPINDLE_PULSE_INT0);   // Enable RPM timer interrupt

    RPM_TIMER->CONTROL = TIMER32_CONTROL_SIZE|TIMER32_CONTROL_ENABLE|TIMER32_CONTROL_PRESCALE_1; // rolls over after ~23 minutes

#endif // SPINDLE_ENCODER_ENABLE

// Set defaults

#if ATC_ENABLE
    atc_init();
#endif

    IOInitDone = settings->version.id == 23;

    hal.settings_changed(settings, (settings_changed_flags_t){0});

    return IOInitDone;
}

#ifdef ENABLE_SPINDLE_LINEARIZATION
static void driver_rt_report (stream_write_ptr stream_write, report_tracking_flags_t report)
{
    if(report.pwm) {
        char sbuf[20];
        sprintf(sbuf, "|PWM:%d", settings.pwm_spindle.invert.pwm ? spindle_pwm.period - SPINDLE_PWM_TIMER->CCR[2] - 1 : SPINDLE_PWM_TIMER->CCR[2]);
        stream_write(sbuf);
    }
}
#endif

// Initialize HAL pointers, setup serial comms and enable EEPROM
// NOTE: Grbl is not yet configured (from EEPROM data), driver_setup() will be called when done

bool driver_init (void)
{
    // Enable EEPROM and serial port here for Grbl to be able to configure itself and report any errors

    SystemInit();

    CS->KEY = CS_KEY_VAL;   // Unlock CS module for register access

 /* Change clock source to high frequency crystal */

    PJ->SEL0 |= BIT2|BIT3;
    PJ->SEL1 &= ~(BIT2|BIT3);

    BITBAND_PERI(CS->CTL2, CS_CTL2_HFXTDRIVE_OFS) = 1;
    CS->CTL2 = (CS->CTL2 & (~CS_CTL2_HFXTFREQ_MASK)) | (CS_CTL2_HFXTFREQ_5);
    BITBAND_PERI(CS->CTL2, CS_CTL2_HFXTBYPASS_OFS) = 0;

    BITBAND_PERI(CS->CTL2, CS_CTL2_HFXT_EN_OFS) = 1;
    while (BITBAND_PERI(CS->IFG, CS_IFG_HFXTIFG_OFS))
        BITBAND_PERI(CS->CLRIFG,CS_CLRIFG_CLR_HFXTIFG_OFS) = 1;

    BITBAND_PERI(CS->CTL2, CS_CTL2_HFXTDRIVE_OFS) = 1;

 /* set MCLK to 48 MHz */

    while (!BITBAND_PERI(CS->STAT, CS_STAT_MCLK_READY_OFS));
    CS->CTL1 = CS_CTL1_DIVM_0 | CS_CTL1_SELM__HFXTCLK | (CS->CTL1 & ~(CS_CTL1_SELM_MASK | CS_CTL1_DIVM_MASK));
    while (!BITBAND_PERI(CS->STAT, CS_STAT_MCLK_READY_OFS));

 /* set SMCLK to 12 MHz */

    while (!BITBAND_PERI(CS->STAT, CS_STAT_SMCLK_READY_OFS));
    CS->CTL1 = CS_CTL1_DIVS_2 | CS_CTL1_SELS__HFXTCLK | (CS->CTL1 & ~(CS_CTL1_DIVS_MASK | CS_CTL1_SELS_MASK));
    while (!BITBAND_PERI(CS->STAT, CS_STAT_SMCLK_READY_OFS));

    CS->KEY = 0;    // Lock CS module for register access

//    Interrupt_disableSleepOnIsrExit();

 /* enable lazy stacking of FPU registers */

    FPU->FPCCR = (FPU->FPCCR & ~FPU_FPCCR_LSPEN_Msk) | FPU_FPCCR_ASPEN_Msk;

 /* Set SysTick IRQ to lowest priority */
    NVIC_SetPriority(SysTick_IRQn, (1 << __NVIC_PRIO_BITS) - 1);

    SysTick->LOAD = (SystemCoreClock / 1000) - 1;
    SysTick->VAL = 0;
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk|SysTick_CTRL_TICKINT_Msk|SysTick_CTRL_ENABLE_Msk;

#if MPG_ENABLE == 1
    // Drive MPG mode input pin low until setup complete
    BITBAND_PERI(MPG_MODE_PORT->DIR, MPG_MODE_PIN) = 1;
    BITBAND_PERI(MPG_MODE_PORT->OUT, MPG_MODE_PIN) = 0;
#endif

    hal.info = "MSP432";
    hal.driver_version = "250528";
    hal.driver_url = GRBL_URL "/MSP432P401R";
#ifdef BOARD_NAME
    hal.board = BOARD_NAME;
#endif
#ifdef BOARD_URL
    hal.board_url = BOARD_URL;
#endif
    hal.driver_setup = driver_setup;
    hal.f_step_timer = SystemCoreClock;
    hal.rx_buffer_size = RX_BUFFER_SIZE;
    hal.delay_ms = driver_delay_ms;
    hal.settings_changed = settings_changed;

    hal.stepper.wake_up = stepperWakeUp;
    hal.stepper.go_idle = stepperGoIdle;
    hal.stepper.enable = stepperEnable;
    hal.stepper.cycles_per_tick = stepperCyclesPerTick;
    hal.stepper.pulse_start = stepperPulseStart;

    hal.limits.enable = limitsEnable;
    hal.limits.get_state = limitsGetState;

    hal.coolant.set_state = coolantSetState;
    hal.coolant.get_state = coolantGetState;

    hal.control.get_state = systemGetState;

    hal.irq_enable = enable_irq;
    hal.irq_disable = disable_irq;
#if I2C_STROBE_ENABLE
    hal.irq_claim = irq_claim;
#endif
    hal.set_bits_atomic = bitsSetAtomic;
    hal.clear_bits_atomic = bitsClearAtomic;
    hal.set_value_atomic = valueSetAtomic;
    hal.get_elapsed_ticks = getElapsedTicks;
    hal.enumerate_pins = enumeratePins;
    hal.periph_port.register_pin = registerPeriphPin;
    hal.periph_port.set_pin_description = setPeriphPinDescription;

    serialRegisterStreams();
    if(!stream_connect_instance(SERIAL_STREAM, BAUD_RATE))
        while(true); // Cannot boot if no communication channel is available!

#if EEPROM_ENABLE
    i2c_eeprom_init();
#else
    hal.nvs.type = NVS_None;
#endif

#ifdef ENABLE_SPINDLE_LINEARIZATION
    grbl.on_realtime_report = driver_rt_report;
#endif

#if DRIVER_SPINDLE_ENABLE

 #if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM

   static const spindle_ptrs_t spindle = {
       .type = SpindleType_PWM,
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
       .ref_id = SPINDLE_PWM0,
#else
       .ref_id = SPINDLE_PWM0_NODIR,
#endif
       .config = spindleConfig,
       .set_state = spindleSetStateVariable,
       .get_state = spindleGetState,
       .get_pwm = spindleGetPWM,
  #ifdef SPINDLE_RPM_CONTROLLED
       .update_rpm = spindleUpdateRPM,
  #else
       .get_pwm = spindleGetPWM,
       .update_pwm = spindleSetSpeed,
  #endif
       .cap = {
           .gpio_controlled = On,
           .variable = On,
           .laser = On,
           .pwm_invert = On,
   #if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
           .direction = On
   #endif
       }
   };

 #else

   static const spindle_ptrs_t spindle = {
       .type = SpindleType_Basic,
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
       .ref_id = SPINDLE_ONOFF0_DIR,
#else
       .ref_id = SPINDLE_ONOFF0,
#endif
       .set_state = spindleSetState,
       .get_state = spindleGetState,
       .cap = {
           .gpio_controlled = On,
  #if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
           .direction = On
  #endif
       }
   };

 #endif

   spindle_id = spindle_register(&spindle, DRIVER_SPINDLE_NAME);

#endif // DRIVER_SPINDLE_ENABLE

  // driver capabilities, used for announcing and negotiating (with the core) driver functionality

    hal.limits_cap = get_limits_cap();
    hal.home_cap = get_home_cap();
    hal.driver_cap.spindle_sync = On;
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM && defined(SPINDLE_RPM_CONTROLLED)
    hal.driver_cap.spindle_pid = On;
#endif
#if SPINDLE_ENCODER_ENABLE
    hal.driver_cap.spindle_encoder = On;
#endif
    hal.coolant_cap.bits = COOLANT_ENABLE;
    hal.driver_cap.software_debounce = On;
    hal.driver_cap.step_pulse_delay = On;
    hal.driver_cap.amass_level = 3;
    hal.driver_cap.control_pull_up = On;
    hal.driver_cap.limits_pull_up = On;

    static pin_group_pins_t aux_inputs = {0}, aux_outputs = {0}, aux_ainputs = {0}, aux_aoutputs = {0};

    uint32_t i;
    input_signal_t *input;

    for(i = 0; i < sizeof(inputpin) / sizeof(input_signal_t); i++) {
        input = &inputpin[i];
        input->bit = 1 << input->pin;
        input->mode.input = input->cap.input = On;
        if(input->group == PinGroup_AuxInput) {
            if(aux_inputs.pins.inputs == NULL)
                aux_inputs.pins.inputs = input;

            input->user_port = aux_inputs.n_pins++;
            input->id = (pin_function_t)(Input_Aux0 + input->user_port);
            input->mode.pull_mode = input->cap.pull_mode = PullMode_Up;
            input->cap.debounce = hal.driver_cap.software_debounce;
            input->cap.irq_mode = IRQ_Mode_RisingFalling;

            aux_ctrl_t *aux_remap;
            if((aux_remap = aux_ctrl_remap_explicit(input->port, input->pin, input->user_port, input))) {
                if(aux_remap->function == Input_Probe)
                    aux_remap->irq_mode = IRQ_Mode_RisingFalling;
            }
        } else if(input->group == PinGroup_Limit) {
            if(limit_inputs.pins.inputs == NULL)
                limit_inputs.pins.inputs = input;
            limit_inputs.n_pins++;
            input->mode.debounce = hal.driver_cap.software_debounce;
        } else if(input->group == PinGroup_Control)
            input->mode.debounce = hal.driver_cap.software_debounce;
    }

    output_signal_t *output;

    for(i = 0; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
        output = &outputpin[i];
        output->mode.output = On;
        if(output->group == PinGroup_AuxOutput) {
            if(aux_outputs.pins.outputs == NULL)
                aux_outputs.pins.outputs = output;
            output->id = (pin_function_t)(Output_Aux0 + aux_outputs.n_pins);

            aux_out_remap_explicit(output->port, output->pin, aux_outputs.n_pins, output);

            aux_outputs.n_pins++;
        }
        if(output->group == PinGroup_AuxOutputAnalog) {
            if(aux_aoutputs.pins.outputs == NULL)
                aux_aoutputs.pins.outputs = output;
            output->id = (pin_function_t)(Output_Analog_Aux0 + aux_aoutputs.n_pins);
            aux_aoutputs.n_pins++;
        }
    }

    ioports_init(&aux_inputs, &aux_outputs);
    ioports_init_analog(&aux_ainputs, &aux_aoutputs);
    aux_ctrl_claim_ports(aux_claim_explicit, NULL);
    aux_ctrl_claim_out_ports(aux_out_claim_explicit, NULL);

#include "grbl/plugins_init.h"

#if MPG_ENABLE == 1
    if(!hal.driver_cap.mpg_mode)
        hal.driver_cap.mpg_mode = stream_mpg_register(stream_open_instance(MPG_STREAM, 115200, NULL, NULL), false, NULL);
    if(hal.driver_cap.mpg_mode)
        task_run_on_startup(mpg_enable, NULL);
#elif MPG_ENABLE == 2
    if(!hal.driver_cap.mpg_mode)
        hal.driver_cap.mpg_mode = stream_mpg_register(stream_open_instance(MPG_STREAM, 115200, NULL, NULL), false, stream_mpg_check_enable);
#endif

    // no need to move version check before init - compiler will fail any signature mismatch for existing entries
    return hal.version == 10;
}

/* interrupt handlers */

// Main stepper driver
void STEPPER_IRQHandler (void)
{
    STEPPER_TIMER->INTCLR = 0;
//    spindle_encoder.timer_value_step = RPM_TIMER->VALUE; // to be used for spindle synchronized motion?
    hal.stepper.interrupt_callback();
}

/* The Stepper Port Reset Interrupt: This interrupt handles the falling edge of the step
   pulse. This should always trigger before the next general stepper driver interrupt and independently
   finish, if stepper driver interrupts is disabled after completing a move.
   NOTE: Interrupt collisions between the serial and stepper interrupts can cause delays by
   a few microseconds, if they execute right before one another. Not a big deal, but can
   cause issues at high step rates if another high frequency asynchronous interrupt is
   added to Grbl.
*/

// This interrupt is used only when STEP_PULSE_DELAY is enabled. Here, the step pulse is
// initiated after the STEP_PULSE_DELAY time period has elapsed.
void STEPPULSE_N_IRQHandler (void)
{
    if(PULSE_TIMER->IV == 0x02) {                               // CCR1 - IV read clears interrupt
        set_step_outputs(next_step_out);                    // Begin step pulse
        PULSE_TIMER->CCTL[1] &= ~TIMER_A_CCTLN_CCIE;            // Disable CCR1 interrupt
        PULSE_TIMER->CCR[0] = pulse_length;                     // Set pulse length
        PULSE_TIMER->CTL |= TIMER_A_CTL_CLR|TIMER_A_CTL_MC1;    // and restart timer
    }
}

// This interrupt is enabled when Grbl sets the motor port bits to execute
// a step. This ISR resets the motor port after a short period (settings.pulse_microseconds)
// completing one step cycle.
void STEPPULSE_0_IRQHandler (void)
{
    set_step_outputs((axes_signals_t){0});
    PULSE_TIMER->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;
    PULSE_TIMER->CTL &= ~(TIMER_A_CTL_MC0|TIMER_A_CTL_MC1); // Disable Timer0 to prevent re-entering this interrupt when it's not needed.
}

#if SPINDLE_ENCODER_ENABLE

void RPMCOUNTER_IRQHandler (void)
{
    spindle_encoder.spin_lock = true;

    __disable_irq();
    uint32_t tval = RPM_TIMER->VALUE;
    uint16_t cval = RPM_COUNTER->R;
    __enable_irq();

    RPM_COUNTER->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;
    RPM_COUNTER->CCR[0] += spindle_encoder.tics_per_irq;

    spindle_encoder.counter.pulse_count += (uint16_t)(cval - (uint16_t)spindle_encoder.counter.last_count);
    spindle_encoder.counter.last_count = cval;
    spindle_encoder.timer.pulse_length = spindle_encoder.timer.last_pulse - tval;
    spindle_encoder.timer.last_pulse = tval;

    spindle_encoder.spin_lock = false;
}

#endif // SPINDLE_ENCODER_ENABLE

static void pin_debounce (void *input)
{
    input_signal_t *signal = (input_signal_t *)input;

    BITBAND_PERI(signal->port->IE, signal->pin) = 1;

    if(signal->id == Input_SafetyDoor)
        debounce.safety_door = Off;

    if(BITBAND_PERI(signal->port->IN, signal->pin) == (signal->mode.irq_mode == IRQ_Mode_Falling ? 0 : 1))
      switch(signal->group) {

        case PinGroup_Limit:
            {
                limit_signals_t state = limitsGetState();

                if(limit_signals_merge(state).mask)
                    hal.limits.interrupt_callback(state);
            }
            break;

        case PinGroup_Control:
            hal.control.interrupt_callback(systemGetState());
            break;

        case PinGroup_AuxInput:
            ioports_event(signal);
            break;

        default:
            break;
    }
}

static inline __attribute__((always_inline)) IRQHandler (input_signal_t **inputs, uint16_t iflags)
{
    uint32_t groups = 0, i = 0;

    input_signal_t *input;

    while((input = inputs[i++])) {
        if(iflags & input->bit) {

            if(input->mode.debounce && task_add_delayed(pin_debounce, input, 32)) {
                BITBAND_PERI(input->port->IE, input->pin) = 0;

                if(input->id == Input_SafetyDoor)
                    debounce.safety_door = On;

            } else switch(input->group) {

#if SPINDLE_ENCODER_ENABLE
                case PinGroup_QEI_Index:
                    if(spindle_encoder.counter.index_count && (uint16_t)(RPM_COUNTER->R - (uint16_t)spindle_encoder.counter.last_index) != spindle_encoder.ppr)
                        spindle_encoder.error_count++;

                    spindle_encoder.timer.last_index = RPM_TIMER->VALUE;
                    spindle_encoder.counter.last_index = RPM_COUNTER->R;
                    spindle_encoder.counter.index_count++;
                    break;
#endif

#if MPG_ENABLE == 1
                case PinGroup_MPG:
                    BITBAND_PERI(MPG_MODE_PORT->IE, MPG_MODE_PIN) = 0;
                    task_add_immediate(mpg_select, NULL);
                    break;
#endif

#if I2C_STROBE_ENABLE
                case PinGroup_Keypad:
                    if(i2c_strobe.callback)
                        i2c_strobe.callback(0, !BITBAND_PERI(I2C_STROBE_PORT->IN, I2C_STROBE_PIN));
                    break;
#endif

                case PinGroup_AuxInput:
                    ioports_event(input);
                    break;

                default:
                    groups |= input->group;
                    break;
            }
        };
    }

    if(groups & PinGroup_Limit) {
        limit_signals_t state = limitsGetState();
        if(limit_signals_merge(state).value)
            hal.limits.interrupt_callback(state);
    }

    if(groups & PinGroup_Control)
        hal.control.interrupt_callback(systemGetState());
}

void PORT1_IRQHandler (void)
{
    uint8_t iflags = P1->IFG;

    P1->IFG &= ~iflags;

    IRQHandler(p1_pins, iflags);
}

void PORT2_IRQHandler (void)
{
    uint8_t iflags = P2->IFG;

    P2->IFG &= ~iflags;

    IRQHandler(p2_pins, iflags << 8);
}

void PORT3_IRQHandler (void)
{
    uint8_t iflags = P3->IFG;

    P3->IFG &= ~iflags;

    IRQHandler(p3_pins, iflags);
}

void PORT4_IRQHandler (void)
{
    uint8_t iflags = P4->IFG;

    P4->IFG &= ~iflags;

    IRQHandler(p4_pins, iflags << 8);
}

void PORT5_IRQHandler (void)
{
    uint8_t iflags = P5->IFG;

    P5->IFG &= ~iflags;

    IRQHandler(p5_pins, iflags);
}

void PORT6_IRQHandler (void)
{
    uint8_t iflags = P6->IFG;

    P6->IFG &= ~iflags;

    IRQHandler(p6_pins, iflags << 8);
}

// Interrupt handler for 1 ms interval timer
void SysTick_Handler (void)
{
    elapsed_tics++;

#if defined(SPINDLE_RPM_CONTROLLED)

    static uint32_t spid = SPINDLE_PID_SAMPLE_RATE, tpp = 0;

    switch(spindle_control.pid_state) {

        case PIDState_Pending:
            if(pid_count == 0) {
                tpp = 0;
                spid = SPINDLE_PID_SAMPLE_RATE;
            }

            if(pid_count < 500)
                pid_count++;
            else if(spindle_data.index_count > 2)
                spindle_control.pid_state = PIDState_Active;

            tpp += spindle_encoder.timer.pulse_length / spindle_encoder.tics_per_irq;

            if(--spid == 0) {
                spindle_control.rpm = spindle_calc_rpm(tpp / SPINDLE_PID_SAMPLE_RATE);
                tpp = 0;
                spid = SPINDLE_PID_SAMPLE_RATE;
            }
            break;

        case PIDState_Active:
            tpp += spindle_encoder.timer.pulse_length / spindle_encoder.tics_per_irq;
            if(--spid == 0) {
                spindle_rpm_pid(tpp / SPINDLE_PID_SAMPLE_RATE);
                tpp = 0;
                spid = SPINDLE_PID_SAMPLE_RATE;
            }
            break;
    }

#endif

    if(delay.ms && !(--delay.ms) && delay.callback) {
        delay.callback();
        delay.callback = NULL;
    }
}
