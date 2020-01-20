
#if 0
#include "Marlin.h"
#include "stepper.h"
#include "endstops.h"
#include "planner.h"
#include "temperature.h"
#include "ultralcd.h"
#include "language.h"
#include "cardreader.h"
#include "speed_lookuptable.h"




#if ENABLED(RESIN)
  #include <SPI.h>
#endif

Stepper stepper; // Singleton

// public:

block_t* Stepper::current_block = NULL;  // A pointer to the block currently being traced


// private:

uint8_t Stepper::last_direction_bits = 0;        // The next stepping-bits to be output
int16_t Stepper::cleaning_buffer_counter = 0;

long Stepper::counter_X = 0,
     Stepper::counter_Y = 0,
     Stepper::counter_Z = 0,
     Stepper::counter_E = 0;

#if ENABLED(RESIN)
  uint8_t Stepper::resin_spi_part1 = 0;
  uint8_t Stepper::resin_spi_part2 = 0;
  uint16_t Stepper::y_dac_position = 0;
  uint16_t Stepper::x_dac_position = 0;
#endif


volatile uint32_t Stepper::step_events_completed = 0; // The number of step events executed in the current block


long Stepper::acceleration_time, Stepper::deceleration_time;

volatile long Stepper::count_position[NUM_AXIS] = { 0 };
volatile signed char Stepper::count_direction[NUM_AXIS] = { 1, 1, 1, 1 };


uint8_t Stepper::step_loops, Stepper::step_loops_nominal;

uint16_t Stepper::OCR1A_nominal,
         Stepper::acc_step_rate; // needed for deceleration start point

volatile long Stepper::endstops_trigsteps[XYZ];


  #define X_APPLY_DIR(v,Q) X_DIR_WRITE(v)
  #define X_APPLY_STEP(v,Q) X_STEP_WRITE(v)


  #define Y_APPLY_DIR(v,Q) Y_DIR_WRITE(v)
  #define Y_APPLY_STEP(v,Q) Y_STEP_WRITE(v)


  #define Z_APPLY_DIR(v,Q) Z_DIR_WRITE(v)
  #define Z_APPLY_STEP(v,Q) Z_STEP_WRITE(v)


  #define E_APPLY_STEP(v,Q) E_STEP_WRITE(v)


// intRes = longIn1 * longIn2 >> 24
// uses:
// r26 to store 0
// r27 to store bits 16-23 of the 48bit result. The top bit is used to round the two byte result.
// note that the lower two bytes and the upper byte of the 48bit result are not calculated.
// this can cause the result to be out by one as the lower bytes may cause carries into the upper ones.
// B0 A0 are bits 24-39 and are the returned value
// C1 B1 A1 is longIn1
// D2 C2 B2 A2 is longIn2
//
#define MultiU24X32toH16(intRes, longIn1, longIn2) \
  asm volatile ( \
                 "clr r26 \n\t" \
                 "mul %A1, %B2 \n\t" \
                 "mov r27, r1 \n\t" \
                 "mul %B1, %C2 \n\t" \
                 "movw %A0, r0 \n\t" \
                 "mul %C1, %C2 \n\t" \
                 "add %B0, r0 \n\t" \
                 "mul %C1, %B2 \n\t" \
                 "add %A0, r0 \n\t" \
                 "adc %B0, r1 \n\t" \
                 "mul %A1, %C2 \n\t" \
                 "add r27, r0 \n\t" \
                 "adc %A0, r1 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "mul %B1, %B2 \n\t" \
                 "add r27, r0 \n\t" \
                 "adc %A0, r1 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "mul %C1, %A2 \n\t" \
                 "add r27, r0 \n\t" \
                 "adc %A0, r1 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "mul %B1, %A2 \n\t" \
                 "add r27, r1 \n\t" \
                 "adc %A0, r26 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "lsr r27 \n\t" \
                 "adc %A0, r26 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "mul %D2, %A1 \n\t" \
                 "add %A0, r0 \n\t" \
                 "adc %B0, r1 \n\t" \
                 "mul %D2, %B1 \n\t" \
                 "add %B0, r0 \n\t" \
                 "clr r1 \n\t" \
                 : \
                 "=&r" (intRes) \
                 : \
                 "d" (longIn1), \
                 "d" (longIn2) \
                 : \
                 "r26" , "r27" \
               )

// Some useful constants

/**
 *         __________________________
 *        /|                        |\     _________________         ^
 *       / |                        | \   /|               |\        |
 *      /  |                        |  \ / |               | \       s
 *     /   |                        |   |  |               |  \      p
 *    /    |                        |   |  |               |   \     e
 *   +-----+------------------------+---+--+---------------+----+    e
 *   |               BLOCK 1            |      BLOCK 2          |    d
 *
 *                           time ----->
 *
 *  The trapezoid is the shape the speed curve over time. It starts at block->initial_rate, accelerates
 *  first block->accelerate_until step_events_completed, then keeps going at constant speed until
 *  step_events_completed reaches block->decelerate_after after which it decelerates until the trapezoid generator is reset.
 *  The slope of acceleration is calculated using v = u + at where t is the accumulated timer values of the steps so far.
 */
void Stepper::wake_up() {
  // TCNT1 = 0;
  ENABLE_STEPPER_DRIVER_INTERRUPT();
}

/**
 * Set the stepper direction of each axis
 *
 *   COREXY: X_AXIS=A_AXIS and Y_AXIS=B_AXIS
 *   COREXZ: X_AXIS=A_AXIS and Z_AXIS=C_AXIS
 *   COREYZ: Y_AXIS=B_AXIS and Z_AXIS=C_AXIS
 */
void Stepper::set_directions() {

  #define SET_STEP_DIR(AXIS) \
    if (motor_direction(AXIS ##_AXIS)) { \
      AXIS ##_APPLY_DIR(INVERT_## AXIS ##_DIR, false); \
      count_direction[AXIS ##_AXIS] = -1; \
    } \
    else { \
      AXIS ##_APPLY_DIR(!INVERT_## AXIS ##_DIR, false); \
      count_direction[AXIS ##_AXIS] = 1; \
    }

  #if HAS_X_DIR
    SET_STEP_DIR(X); // A
  #endif
  #if HAS_Y_DIR
    SET_STEP_DIR(Y); // B
  #endif
  #if HAS_Z_DIR
    SET_STEP_DIR(Z); // C
  #endif

  #if DISABLED(LIN_ADVANCE)
    if (motor_direction(E_AXIS)) {
      REV_E_DIR();
      count_direction[E_AXIS] = -1;
    }
    else {
      NORM_E_DIR();
      count_direction[E_AXIS] = 1;
    }
  #endif // !LIN_ADVANCE
}

#if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
  extern volatile uint8_t e_hit;
#endif

/**
 * Stepper Driver Interrupt
 *
 * Directly pulses the stepper motors at high frequency.
 * Timer 1 runs at a base frequency of 2MHz, with this ISR using OCR1A compare mode.
 *
 * OCR1A   Frequency
 *     1     2 MHz
 *    50    40 KHz
 *   100    20 KHz - capped max rate
 *   200    10 KHz - nominal max rate
 *  2000     1 KHz - sleep rate
 *  4000   500  Hz - init rate
 */
ISR(TIMER1_COMPA_vect) {

    Stepper::isr();

}

#define _ENABLE_ISRs() do { cli(); if (thermalManager.in_temp_isr) CBI(TIMSK0, OCIE0B); else SBI(TIMSK0, OCIE0B); ENABLE_STEPPER_DRIVER_INTERRUPT(); } while(0)

void Stepper::isr() {

  uint16_t ocr_val;

  #define ENDSTOP_NOMINAL_OCR_VAL 3000 // Check endstops every 1.5ms to guarantee two stepper ISRs within 5ms for BLTouch
  #define OCR_VAL_TOLERANCE       1000 // First max delay is 2.0ms, last min delay is 0.5ms, all others 1.5ms

  #if DISABLED(LIN_ADVANCE)
    // Disable Timer0 ISRs and enable global ISR again to capture UART events (incoming chars)
    CBI(TIMSK0, OCIE0B); // Temperature ISR
    DISABLE_STEPPER_DRIVER_INTERRUPT();
    sei();
  #endif

  #define _SPLIT(L) (ocr_val = (uint16_t)L)
  #if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)

    #define SPLIT(L) _SPLIT(L)

  #else // !ENDSTOP_INTERRUPTS_FEATURE : Sample endstops between stepping ISRs

    static uint32_t step_remaining = 0;

    #define SPLIT(L) do { \
      _SPLIT(L); \
      if (ENDSTOPS_ENABLED && L > ENDSTOP_NOMINAL_OCR_VAL) { \
        const uint16_t remainder = (uint16_t)L % (ENDSTOP_NOMINAL_OCR_VAL); \
        ocr_val = (remainder < OCR_VAL_TOLERANCE) ? ENDSTOP_NOMINAL_OCR_VAL + remainder : ENDSTOP_NOMINAL_OCR_VAL; \
        step_remaining = (uint16_t)L - ocr_val; \
      } \
    }while(0)

    if (step_remaining && ENDSTOPS_ENABLED) {   // Just check endstops - not yet time for a step
      endstops.update();

      // Next ISR either for endstops or stepping
      ocr_val = step_remaining <= ENDSTOP_NOMINAL_OCR_VAL ? step_remaining : ENDSTOP_NOMINAL_OCR_VAL;
      step_remaining -= ocr_val;
      _NEXT_ISR(ocr_val);
      NOLESS(OCR1A, TCNT1 + 16);
      _ENABLE_ISRs(); // re-enable ISRs
      return;
    }

  #endif // !ENDSTOP_INTERRUPTS_FEATURE

  //
  // When cleaning, discard the current block and run fast
  //
  if (cleaning_buffer_counter) {
    if (cleaning_buffer_counter < 0) {          // Count up for endstop hit
      if (current_block) planner.discard_current_block(); // Discard the active block that led to the trigger
      if (!planner.discard_continued_block())   // Discard next CONTINUED block
        cleaning_buffer_counter = 0;            // Keep discarding until non-CONTINUED
    }
    else {
      planner.discard_current_block();
      --cleaning_buffer_counter;                // Count down for abort print
      #if ENABLED(SD_FINISHED_STEPPERRELEASE) && defined(SD_FINISHED_RELEASECOMMAND)
        if (!cleaning_buffer_counter) enqueue_and_echo_commands_P(PSTR(SD_FINISHED_RELEASECOMMAND));
      #endif
    }
    current_block = NULL;                       // Prep to get a new block after cleaning
    _NEXT_ISR(200);                             // Run at max speed - 10 KHz
    _ENABLE_ISRs();
    return;
  }

  // If there is no current block, attempt to pop one from the buffer
  if (!current_block) {
    // Anything in the buffer?
    if ((current_block = planner.get_current_block())) {
      trapezoid_generator_reset();

      // Initialize Bresenham counters to 1/2 the ceiling
      counter_X = counter_Y = counter_Z = counter_E = -(current_block->step_event_count >> 1);

      #if ENABLED(MIXING_EXTRUDER)
        MIXING_STEPPERS_LOOP(i)
          counter_m[i] = -(current_block->mix_event_count[i] >> 1);
      #endif

      step_events_completed = 0;

      #if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
        e_hit = 2; // Needed for the case an endstop is already triggered before the new move begins.
                   // No 'change' can be detected.
      #endif

      #if ENABLED(Z_LATE_ENABLE)
        if (current_block->steps[Z_AXIS] > 0) {
          enable_Z();
          _NEXT_ISR(2000); // Run at slow speed - 1 KHz
          _ENABLE_ISRs(); // re-enable ISRs
          return;
        }
      #endif




#define _COUNTER(AXIS) counter_## AXIS
#define _APPLY_STEP(AXIS) AXIS ##_APPLY_STEP
#define _INVERT_STEP_PIN(AXIS) INVERT_## AXIS ##_STEP_PIN


  

    }
    else {
      _NEXT_ISR(2000); // Run at slow speed - 1 KHz
      _ENABLE_ISRs(); // re-enable ISRs
      return;
    }
  }

  // Update endstops state, if enabled
  #if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
    if (e_hit && ENDSTOPS_ENABLED) {
      endstops.update();
      e_hit--;
    }
  #else
    if (ENDSTOPS_ENABLED) endstops.update();
  #endif

  #if ENABLED(RESIN)
      if (current_block->laser_on)
        //
        if(thermalManager.target_temperature[HOTEND_INDEX] > 254) {
          analogWrite(LASER_FIRING_PIN, 255);
          //analogWrite(LASER_ENABLE_PIN, 255);
        }
        else {
          analogWrite(LASER_FIRING_PIN, thermalManager.target_temperature[HOTEND_INDEX]);
          //analogWrite(LASER_ENABLE_PIN, thermalManager.target_temperature[HOTEND_INDEX]);
        }
      else
      {
        analogWrite(LASER_FIRING_PIN, 0);
        //analogWrite(LASER_ENABLE_PIN, 0);
      }

      //x_dac_position = count_position[_AXIS(X)] + 0x8000;
      x_dac_position = count_position[_AXIS(X)] + 0x8000;

      resin_spi_part1 = (uint8_t)((x_dac_position >> 8) & 0xFF);
      resin_spi_part2 = (uint8_t)(x_dac_position & 0xFF);

      WRITE(GALVO_SS_PIN, LOW);

      SPI.transfer(0x30);
      SPI.transfer(resin_spi_part1);
      SPI.transfer(resin_spi_part2);
      WRITE(GALVO_SS_PIN, HIGH);
    
      y_dac_position = count_position[_AXIS(Y)] + 0x8000;

      resin_spi_part1 = (uint8_t)((y_dac_position >> 8) & 0xFF);
      resin_spi_part2 = (uint8_t)(y_dac_position & 0xFF);

      WRITE(GALVO_SS_PIN, LOW);

      SPI.transfer(0x31);
      SPI.transfer(resin_spi_part1);
      SPI.transfer(resin_spi_part2);
      WRITE(GALVO_SS_PIN, HIGH);
    
#endif

  // Take multiple steps per interrupt (For high speed moves)
  bool all_steps_done = false;

  #if ENABLED(RESIN)

  #endif
  for (uint8_t i = step_loops; i--;) {
    #if ENABLED(LIN_ADVANCE)

      counter_E += current_block->steps[E_AXIS];
      if (counter_E > 0) {
        counter_E -= current_block->step_event_count;
        #if DISABLED(MIXING_EXTRUDER)
          // Don't step E here for mixing extruder
          count_position[E_AXIS] += count_direction[E_AXIS];
          motor_direction(E_AXIS) ? --e_steps[TOOL_E_INDEX] : ++e_steps[TOOL_E_INDEX];
        #endif
      }

      #if ENABLED(MIXING_EXTRUDER)
        // Step mixing steppers proportionally
        const bool dir = motor_direction(E_AXIS);
        MIXING_STEPPERS_LOOP(j) {
          counter_m[j] += current_block->steps[E_AXIS];
          if (counter_m[j] > 0) {
            counter_m[j] -= current_block->mix_event_count[j];
            dir ? --e_steps[j] : ++e_steps[j];
          }
        }
      #endif

    #endif // LIN_ADVANCE

    #define _COUNTER(AXIS) counter_## AXIS
    #define _APPLY_STEP(AXIS) AXIS ##_APPLY_STEP
    #define _INVERT_STEP_PIN(AXIS) INVERT_## AXIS ##_STEP_PIN

    // Advance the Bresenham counter; start a pulse if the axis needs a step
    #define PULSE_START(AXIS) \
      _COUNTER(AXIS) += current_block->steps[_AXIS(AXIS)]; \
      if (_COUNTER(AXIS) > 0) { _APPLY_STEP(AXIS)(!_INVERT_STEP_PIN(AXIS),0); }

    // Stop an active pulse, reset the Bresenham counter, update the position
    #define PULSE_STOP(AXIS) \
      if (_COUNTER(AXIS) > 0) { \
        _COUNTER(AXIS) -= current_block->step_event_count; \
        count_position[_AXIS(AXIS)] += count_direction[_AXIS(AXIS)]; \
        _APPLY_STEP(AXIS)(_INVERT_STEP_PIN(AXIS),0); \
      }

    /**
     * Estimate the number of cycles that the stepper logic already takes
     * up between the start and stop of the X stepper pulse.
     *
     * Currently this uses very modest estimates of around 5 cycles.
     * True values may be derived by careful testing.
     *
     * Once any delay is added, the cost of the delay code itself
     * may be subtracted from this value to get a more accurate delay.
     * Delays under 20 cycles (1.25µs) will be very accurate, using NOPs.
     * Longer delays use a loop. The resolution is 8 cycles.
     */
    #if HAS_X_STEP
      #define _CYCLE_APPROX_1 5
    #else
      #define _CYCLE_APPROX_1 0
    #endif
    #if ENABLED(X_DUAL_STEPPER_DRIVERS)
      #define _CYCLE_APPROX_2 _CYCLE_APPROX_1 + 4
    #else
      #define _CYCLE_APPROX_2 _CYCLE_APPROX_1
    #endif
    #if HAS_Y_STEP
      #define _CYCLE_APPROX_3 _CYCLE_APPROX_2 + 5
    #else
      #define _CYCLE_APPROX_3 _CYCLE_APPROX_2
    #endif
    #if ENABLED(Y_DUAL_STEPPER_DRIVERS)
      #define _CYCLE_APPROX_4 _CYCLE_APPROX_3 + 4
    #else
      #define _CYCLE_APPROX_4 _CYCLE_APPROX_3
    #endif
    #if HAS_Z_STEP
      #define _CYCLE_APPROX_5 _CYCLE_APPROX_4 + 5
    #else
      #define _CYCLE_APPROX_5 _CYCLE_APPROX_4
    #endif
    #if ENABLED(Z_DUAL_STEPPER_DRIVERS)
      #define _CYCLE_APPROX_6 _CYCLE_APPROX_5 + 4
    #else
      #define _CYCLE_APPROX_6 _CYCLE_APPROX_5
    #endif
    #if DISABLED(LIN_ADVANCE)
      #if ENABLED(MIXING_EXTRUDER)
        #define _CYCLE_APPROX_7 _CYCLE_APPROX_6 + (MIXING_STEPPERS) * 6
      #else
        #define _CYCLE_APPROX_7 _CYCLE_APPROX_6 + 5
      #endif
    #else
      #define _CYCLE_APPROX_7 _CYCLE_APPROX_6
    #endif

    #define CYCLES_EATEN_XYZE _CYCLE_APPROX_7
    #define EXTRA_CYCLES_XYZE (STEP_PULSE_CYCLES - (CYCLES_EATEN_XYZE))

    /**
     * If a minimum pulse time was specified get the timer 0 value.
     *
     * TCNT0 has an 8x prescaler, so it increments every 8 cycles.
     * That's every 0.5µs on 16MHz and every 0.4µs on 20MHz.
     * 20 counts of TCNT0 -by itself- is a good pulse delay.
     * 10µs = 160 or 200 cycles.
     */
    #if EXTRA_CYCLES_XYZE > 20
      uint32_t pulse_start = TCNT0;
    #endif

    #if DISABLED(RESIN1)

      #if HAS_X_STEP
        PULSE_START(X);
      #endif
      #if HAS_Y_STEP
        PULSE_START(Y);
      #endif

    #endif

    #if HAS_Z_STEP
      PULSE_START(Z);
    #endif

    // For non-advance use linear interpolation for E also

     
        PULSE_START(E);
 


    // For minimum pulse time wait before stopping pulses
    #if EXTRA_CYCLES_XYZE > 20
      while (EXTRA_CYCLES_XYZE > (uint32_t)(TCNT0 - pulse_start) * (INT0_PRESCALER)) { /* nada */ }
      pulse_start = TCNT0;
    #elif EXTRA_CYCLES_XYZE > 0
      DELAY_NOPS(EXTRA_CYCLES_XYZE);
    #endif

    #if DISABLED(RESIN1)
      #if HAS_X_STEP
        PULSE_STOP(X);
      #endif
      #if HAS_Y_STEP
        PULSE_STOP(Y);
      #endif
    #endif

    #if HAS_Z_STEP
      PULSE_STOP(Z);
    #endif

    #if DISABLED(LIN_ADVANCE)
      
        PULSE_STOP(E);
      #endif


    if (++step_events_completed >= current_block->step_event_count) {
      all_steps_done = true;
      break;
    }

    // For minimum pulse time wait after stopping pulses also
    #if EXTRA_CYCLES_XYZE > 20
      if (i) while (EXTRA_CYCLES_XYZE > (uint32_t)(TCNT0 - pulse_start) * (INT0_PRESCALER)) { /* nada */ }
    #elif EXTRA_CYCLES_XYZE > 0
      if (i) DELAY_NOPS(EXTRA_CYCLES_XYZE);
    #endif

  } // steps_loop



  // Calculate new timer value
  if (step_events_completed <= (uint32_t)current_block->accelerate_until) {

    MultiU24X32toH16(acc_step_rate, acceleration_time, current_block->acceleration_rate);
    acc_step_rate += current_block->initial_rate;

    // upper limit
    NOMORE(acc_step_rate, current_block->nominal_rate);

    // step_rate to timer interval
    const uint16_t interval = calc_timer_interval(acc_step_rate);

    SPLIT(interval);  // split step into multiple ISRs if larger than ENDSTOP_NOMINAL_OCR_VAL
    _NEXT_ISR(ocr_val);

    acceleration_time += interval;

  }
  else if (step_events_completed > (uint32_t)current_block->decelerate_after) {
    uint16_t step_rate;
    MultiU24X32toH16(step_rate, deceleration_time, current_block->acceleration_rate);

    if (step_rate < acc_step_rate) { // Still decelerating?
      step_rate = acc_step_rate - step_rate;
      NOLESS(step_rate, current_block->final_rate);
    }
    else
      step_rate = current_block->final_rate;

    // step_rate to timer interval
    const uint16_t interval = calc_timer_interval(step_rate);

    SPLIT(interval);  // split step into multiple ISRs if larger than ENDSTOP_NOMINAL_OCR_VAL
    _NEXT_ISR(ocr_val);

    deceleration_time += interval;


  }
  else {



    SPLIT(OCR1A_nominal);  // split step into multiple ISRs if larger than ENDSTOP_NOMINAL_OCR_VAL
    _NEXT_ISR(ocr_val);

    // ensure we're running at the correct step rate, even if we just came off an acceleration
    step_loops = step_loops_nominal;
  }

  #if DISABLED(LIN_ADVANCE)
    NOLESS(OCR1A, TCNT1 + 16);
  #endif

  // If current block is finished, reset pointer
  if (all_steps_done) {
    current_block = NULL;
    planner.discard_current_block();
  }
  #if DISABLED(LIN_ADVANCE)
    _ENABLE_ISRs(); // re-enable ISRs
  #endif
}



void Stepper::init() {

 

  // Init Microstepping Pins
  #if HAS_MICROSTEPS
    microstep_init();
  #endif

  #if ENABLED(RESIN)
    //resin_init();
  #endif
  
  // Init Dir Pins
  #if HAS_X_DIR
    X_DIR_INIT;
  #endif
  #if HAS_X2_DIR
    X2_DIR_INIT;
  #endif
  #if HAS_Y_DIR
    Y_DIR_INIT;
    #if ENABLED(Y_DUAL_STEPPER_DRIVERS) && HAS_Y2_DIR
      Y2_DIR_INIT;
    #endif
  #endif
  #if HAS_Z_DIR
    Z_DIR_INIT;
    #if ENABLED(Z_DUAL_STEPPER_DRIVERS) && HAS_Z2_DIR
      Z2_DIR_INIT;
    #endif
  #endif
  #if HAS_E0_DIR
    E0_DIR_INIT;
  #endif
  #if HAS_E1_DIR
    E1_DIR_INIT;
  #endif
  #if HAS_E2_DIR
    E2_DIR_INIT;
  #endif
  #if HAS_E3_DIR
    E3_DIR_INIT;
  #endif
  #if HAS_E4_DIR
    E4_DIR_INIT;
  #endif

  // Init Enable Pins - steppers default to disabled.
  #if HAS_X_ENABLE
    X_ENABLE_INIT;
    if (!X_ENABLE_ON) X_ENABLE_WRITE(HIGH);
    #if ENABLED(DUAL_X_CARRIAGE) && HAS_X2_ENABLE
      X2_ENABLE_INIT;
      if (!X_ENABLE_ON) X2_ENABLE_WRITE(HIGH);
    #endif
  #endif
  #if HAS_Y_ENABLE
    Y_ENABLE_INIT;
    if (!Y_ENABLE_ON) Y_ENABLE_WRITE(HIGH);
    #if ENABLED(Y_DUAL_STEPPER_DRIVERS) && HAS_Y2_ENABLE
      Y2_ENABLE_INIT;
      if (!Y_ENABLE_ON) Y2_ENABLE_WRITE(HIGH);
    #endif
  #endif
  #if HAS_Z_ENABLE
    Z_ENABLE_INIT;
    if (!Z_ENABLE_ON) Z_ENABLE_WRITE(HIGH);
    #if ENABLED(Z_DUAL_STEPPER_DRIVERS) && HAS_Z2_ENABLE
      Z2_ENABLE_INIT;
      if (!Z_ENABLE_ON) Z2_ENABLE_WRITE(HIGH);
    #endif
  #endif
  #if HAS_E0_ENABLE
    E0_ENABLE_INIT;
    if (!E_ENABLE_ON) E0_ENABLE_WRITE(HIGH);
  #endif
  #if HAS_E1_ENABLE
    E1_ENABLE_INIT;
    if (!E_ENABLE_ON) E1_ENABLE_WRITE(HIGH);
  #endif
  #if HAS_E2_ENABLE
    E2_ENABLE_INIT;
    if (!E_ENABLE_ON) E2_ENABLE_WRITE(HIGH);
  #endif
  #if HAS_E3_ENABLE
    E3_ENABLE_INIT;
    if (!E_ENABLE_ON) E3_ENABLE_WRITE(HIGH);
  #endif
  #if HAS_E4_ENABLE
    E4_ENABLE_INIT;
    if (!E_ENABLE_ON) E4_ENABLE_WRITE(HIGH);
  #endif

  // Init endstops and pullups
  endstops.init();

  #define _STEP_INIT(AXIS) AXIS ##_STEP_INIT
  #define _WRITE_STEP(AXIS, HIGHLOW) AXIS ##_STEP_WRITE(HIGHLOW)
  #define _DISABLE(AXIS) disable_## AXIS()

  #define AXIS_INIT(AXIS, PIN) \
    _STEP_INIT(AXIS); \
    _WRITE_STEP(AXIS, _INVERT_STEP_PIN(PIN)); \
    _DISABLE(AXIS)

  #define E_AXIS_INIT(NUM) AXIS_INIT(E## NUM, E)

  // Init Step Pins
  #if HAS_X_STEP
    #if ENABLED(X_DUAL_STEPPER_DRIVERS) || ENABLED(DUAL_X_CARRIAGE)
      X2_STEP_INIT;
      X2_STEP_WRITE(INVERT_X_STEP_PIN);
    #endif
    AXIS_INIT(X, X);
  #endif

  #if HAS_Y_STEP
    #if ENABLED(Y_DUAL_STEPPER_DRIVERS)
      Y2_STEP_INIT;
      Y2_STEP_WRITE(INVERT_Y_STEP_PIN);
    #endif
    AXIS_INIT(Y, Y);
  #endif

  #if HAS_Z_STEP
    #if ENABLED(Z_DUAL_STEPPER_DRIVERS)
      Z2_STEP_INIT;
      Z2_STEP_WRITE(INVERT_Z_STEP_PIN);
    #endif
    AXIS_INIT(Z, Z);
  #endif

  #if HAS_E0_STEP
    E_AXIS_INIT(0);
  #endif
  #if HAS_E1_STEP
    E_AXIS_INIT(1);
  #endif
  #if HAS_E2_STEP
    E_AXIS_INIT(2);
  #endif
  #if HAS_E3_STEP
    E_AXIS_INIT(3);
  #endif
  #if HAS_E4_STEP
    E_AXIS_INIT(4);
  #endif

  // waveform generation = 0100 = CTC
  SET_WGM(1, CTC_OCRnA);

  // output mode = 00 (disconnected)
  SET_COMA(1, NORMAL);

  // Set the timer pre-scaler
  // Generally we use a divider of 8, resulting in a 2MHz timer
  // frequency on a 16MHz MCU. If you are going to change this, be
  // sure to regenerate speed_lookuptable.h with
  // create_speed_lookuptable.py
  SET_CS(1, PRESCALER_8);  //  CS 2 = 1/8 prescaler

  // Init Stepper ISR to 122 Hz for quick starting
  OCR1A = 0x4000;
  TCNT1 = 0;
  ENABLE_STEPPER_DRIVER_INTERRUPT();



  endstops.enable(true); // Start with endstops active. After homing they can be disabled
  sei();

  set_directions(); // Init directions to last_direction_bits = 0
}


/**
 * Block until all buffered steps are executed / cleaned
 */
void Stepper::synchronize() { while (planner.blocks_queued() || cleaning_buffer_counter) idle(); }

/**
 * Set the stepper positions directly in steps
 *
 * The input is based on the typical per-axis XYZ steps.
 * For CORE machines XYZ needs to be translated to ABC.
 *
 * This allows get_axis_position_mm to correctly
 * derive the current XYZ position later on.
 */
void Stepper::set_position(const long &a, const long &b, const long &c, const long &e) {

  synchronize(); // Bad to set stepper counts in the middle of a move

  CRITICAL_SECTION_START;

    // default non-h-bot planning
    count_position[X_AXIS] = a;
    count_position[Y_AXIS] = b;
    count_position[Z_AXIS] = c;


  count_position[E_AXIS] = e;
  CRITICAL_SECTION_END;
}

void Stepper::set_position(const AxisEnum &axis, const long &v) {
  CRITICAL_SECTION_START;
  count_position[axis] = v;
  CRITICAL_SECTION_END;
}

void Stepper::set_e_position(const long &e) {
  CRITICAL_SECTION_START;
  count_position[E_AXIS] = e;
  CRITICAL_SECTION_END;
}

/**
 * Get a stepper's position in steps.
 */
long Stepper::position(const AxisEnum axis) {
  CRITICAL_SECTION_START;
  const long count_pos = count_position[axis];
  CRITICAL_SECTION_END;
  return count_pos;
}

/**
 * Get an axis position according to stepper position(s)
 * For CORE machines apply translation from ABC to XYZ.
 */
float Stepper::get_axis_position_mm(const AxisEnum axis) {
  float axis_steps;
  #if IS_CORE
    // Requesting one of the "core" axes?
    if (axis == CORE_AXIS_1 || axis == CORE_AXIS_2) {
      CRITICAL_SECTION_START;
      // ((a1+a2)+(a1-a2))/2 -> (a1+a2+a1-a2)/2 -> (a1+a1)/2 -> a1
      // ((a1+a2)-(a1-a2))/2 -> (a1+a2-a1+a2)/2 -> (a2+a2)/2 -> a2
      axis_steps = 0.5f * (
        axis == CORE_AXIS_2 ? CORESIGN(count_position[CORE_AXIS_1] - count_position[CORE_AXIS_2])
                            : count_position[CORE_AXIS_1] + count_position[CORE_AXIS_2]
      );
      CRITICAL_SECTION_END;
    }
    else
      axis_steps = position(axis);
  #else
    axis_steps = position(axis);
  #endif
  return axis_steps * planner.steps_to_mm[axis];
}

void Stepper::finish_and_disable() {
  synchronize();
  disable_all_steppers();
}

void Stepper::quick_stop() {
  cleaning_buffer_counter = 5000;
  DISABLE_STEPPER_DRIVER_INTERRUPT();
  while (planner.blocks_queued()) planner.discard_current_block();
  current_block = NULL;
  ENABLE_STEPPER_DRIVER_INTERRUPT();
  #if ENABLED(ULTRA_LCD)
    planner.clear_block_buffer_runtime();
  #endif
}

void Stepper::endstop_triggered(AxisEnum axis) {



    endstops_trigsteps[axis] = count_position[axis];


  kill_current_block();
  cleaning_buffer_counter = -1; // Discard the rest of the move
}

void Stepper::report_positions() {
  CRITICAL_SECTION_START;
  const long xpos = count_position[X_AXIS],
             ypos = count_position[Y_AXIS],
             zpos = count_position[Z_AXIS];
  CRITICAL_SECTION_END;

  #if CORE_IS_XY || CORE_IS_XZ || IS_SCARA
    SERIAL_PROTOCOLPGM(MSG_COUNT_A);
  #else
    SERIAL_PROTOCOLPGM(MSG_COUNT_X);
  #endif
  SERIAL_PROTOCOL(xpos);

  #if CORE_IS_XY || CORE_IS_YZ || IS_SCARA
    SERIAL_PROTOCOLPGM(" B:");
  #else
    SERIAL_PROTOCOLPGM(" Y:");
  #endif
  SERIAL_PROTOCOL(ypos);

  #if CORE_IS_XZ || CORE_IS_YZ
    SERIAL_PROTOCOLPGM(" C:");
  #else
    SERIAL_PROTOCOLPGM(" Z:");
  #endif
  SERIAL_PROTOCOL(zpos);

  SERIAL_EOL();
}


#endif