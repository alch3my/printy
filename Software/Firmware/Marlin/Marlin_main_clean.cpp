#if 0


/*
 * M898 - RESIN turn off laser manually
 * M899 - RESIN turn on laser manually
 *
 * ************ Custom codes - This can change to suit future G-code regulations
 * M928 - Start SD logging: "M928 filename.gco". Stop with M29. (Requires SDSUPPORT)
 * M999 - Restart after being stopped by error
 *
 * "T" Codes
 *
 * T0-T3 - Select an extruder (tool) by index: "T<n> F<units/min>"
 *
 */

#include "Marlin.h"

#include "ultralcd.h"
#include "planner.h"
#include "stepper.h"
#include "endstops.h"
#include "temperature.h"
#include "cardreader.h"
#include "configuration_store.h"
#include "language.h"
#include "pins_arduino.h"
#include "math.h"
#include "nozzle.h"
#include "duration_t.h"
#include "types.h"
#include "gcode.h"


#if ENABLED(RESIN)
  #include <SPI.h>
#endif



bool Running = true;

uint8_t marlin_debug_flags = DEBUG_NONE;

/**
 * Cartesian Current Position
 *   Used to track the native machine position as moves are queued.
 *   Used by 'buffer_line_to_current_position' to do a move after changing it.
 *   Used by 'SYNC_PLAN_POSITION_KINEMATIC' to update 'planner.position'.
 */
float current_position[XYZE] = { 0.0 };

/**
 * Cartesian Destination
 *   The destination for a move, filled in by G-code movement commands,
 *   and expected by functions like 'prepare_move_to_destination'.
 *   Set with 'gcode_get_destination' or 'set_destination_from_current'.
 */
float destination[XYZE] = { 0.0 };

/**
 * axis_homed
 *   Flags that each linear axis was homed.
 *   XYZ on cartesian, ABC on delta, ABZ on SCARA.
 *
 * axis_known_position
 *   Flags that the position is known in each linear axis. Set when homed.
 *   Cleared whenever a stepper powers off, potentially losing its position.
 */
bool axis_homed[XYZ] = { false }, axis_known_position[XYZ] = { false };

/**
 * GCode line number handling. Hosts may opt to include line numbers when
 * sending commands to Marlin, and lines will be checked for sequentiality.
 * M110 N<int> sets the current line number.
 */
static long gcode_N, gcode_LastN, Stopped_gcode_LastN = 0;

/**
 * GCode Command Queue
 * A simple ring buffer of BUFSIZE command strings.
 *
 * Commands are copied into this buffer by the command injectors
 * (immediate, serial, sd card) and they are processed sequentially by
 * the main loop. The process_next_command function parses the next
 * command and hands off execution to individual handler functions.
 */
uint8_t commands_in_queue = 0; // Count of commands in the queue
static uint8_t cmd_queue_index_r = 0, // Ring buffer read position
               cmd_queue_index_w = 0; // Ring buffer write position
#if ENABLED(M100_FREE_MEMORY_WATCHER)
  char command_queue[BUFSIZE][MAX_CMD_SIZE];  // Necessary so M100 Free Memory Dumper can show us the commands and any corruption
#else                                         // This can be collapsed back to the way it was soon.
static char command_queue[BUFSIZE][MAX_CMD_SIZE];
#endif

/**
 * Next Injected Command pointer. NULL if no commands are being injected.
 * Used by Marlin internally to ensure that commands initiated from within
 * are enqueued ahead of any pending serial or sd card commands.
 */
static const char *injected_commands_P = NULL;

/**
 * Feed rates are often configured with mm/m
 * but the planner and stepper like mm/s units.
 */
static const float homing_feedrate_mm_s[] PROGMEM = {
  #if ENABLED(DELTA)
    MMM_TO_MMS(HOMING_FEEDRATE_Z), MMM_TO_MMS(HOMING_FEEDRATE_Z),
  #else
    MMM_TO_MMS(HOMING_FEEDRATE_XY), MMM_TO_MMS(HOMING_FEEDRATE_XY),
  #endif
  MMM_TO_MMS(HOMING_FEEDRATE_Z), 0
};
FORCE_INLINE float homing_feedrate(const AxisEnum a) { return pgm_read_float(&homing_feedrate_mm_s[a]); }

float feedrate_mm_s = MMM_TO_MMS(1500.0);
static float saved_feedrate_mm_s;
int16_t feedrate_percentage = 100, saved_feedrate_percentage;

// Initialized by settings.load()
bool axis_relative_modes[] = AXIS_RELATIVE_MODES;


// Software Endstops are based on the configured limits.
float soft_endstop_min[XYZ] = { X_MIN_BED, Y_MIN_BED, Z_MIN_POS },
      soft_endstop_max[XYZ] = { X_MAX_BED, Y_MAX_BED, Z_MAX_POS };


#if FAN_COUNT > 0
  int16_t fanSpeeds[FAN_COUNT] = { 0 };
  #if ENABLED(EXTRA_FAN_SPEED)
    int16_t old_fanSpeeds[FAN_COUNT],
            new_fanSpeeds[FAN_COUNT];
  #endif
  #if ENABLED(PROBING_FANS_OFF)
    bool fans_paused = false;
    int16_t paused_fanSpeeds[FAN_COUNT] = { 0 };
  #endif
#endif

// The active extruder (tool). Set with T<extruder> command.
uint8_t active_extruder = 0;

// Relative Mode. Enable with G91, disable with G90.
static bool relative_mode = false;

// For M109 and M190, this flag may be cleared (by M108) to exit the wait loop
volatile bool wait_for_heatup = true;



const char axis_codes[XYZE] = { 'X', 'Y', 'Z', 'E' };

// Number of characters read in the current line of serial input
static int serial_count = 0;

// Inactivity shutdown
millis_t previous_cmd_ms = 0;
static millis_t max_inactive_time = 0;
static millis_t stepper_inactive_time = (DEFAULT_STEPPER_DEACTIVE_TIME) * 1000UL;

// Print Job Timer
#if ENABLED(PRINTCOUNTER)
  PrintCounter print_job_timer = PrintCounter();
#else
  Stopwatch print_job_timer = Stopwatch();
#endif

// Buzzer - I2C on the LCD or a BEEPER_PIN
#if ENABLED(LCD_USE_I2C_BUZZER)
  #define BUZZ(d,f) lcd_buzz(d, f)
#elif PIN_EXISTS(BEEPER)
  Buzzer buzzer;
  #define BUZZ(d,f) buzzer.tone(d, f)
#else
  #define BUZZ(d,f) NOOP
#endif

uint8_t target_extruder;


#if HAS_ABL
  float xy_probe_feedrate_mm_s = MMM_TO_MMS(XY_PROBE_SPEED);
  #define XY_PROBE_FEEDRATE_MM_S xy_probe_feedrate_mm_s
#elif defined(XY_PROBE_SPEED)
  #define XY_PROBE_FEEDRATE_MM_S MMM_TO_MMS(XY_PROBE_SPEED)
#else
  #define XY_PROBE_FEEDRATE_MM_S PLANNER_XY_FEEDRATE()
#endif

#if ENABLED(AUTO_BED_LEVELING_BILINEAR)
  #if ENABLED(DELTA)
    #define ADJUST_DELTA(V) \
      if (planner.leveling_active) { \
        const float zadj = bilinear_z_offset(V); \
        delta[A_AXIS] += zadj; \
        delta[B_AXIS] += zadj; \
        delta[C_AXIS] += zadj; \
      }
  #else
    #define ADJUST_DELTA(V) if (planner.leveling_active) { delta[Z_AXIS] += bilinear_z_offset(V); }
  #endif
#elif IS_KINEMATIC
  #define ADJUST_DELTA(V) NOOP
#endif




#if ENABLED(RESIN)

  uint16_t resin_segments_per_second = RESIN_SEGMENTS_PER_SECOND;
  float resin[XYZE];
  const float z0 = Z0_RESIN;
  const float r = R_RESIN;
  const float size_2_angle = SIZE_2_ANGLE_RESIN;
  const float rad_to_deg = RAD_TO_DEG_RESIN;

  void calculate_resin(const float logical[XYZ]);

#endif


float cartes[XYZ] = { 0 };


static bool send_ok[BUFSIZE];

  #define host_keepalive() NOOP




FORCE_INLINE float pgm_read_any(const float *p) { return pgm_read_float_near(p); }
FORCE_INLINE signed char pgm_read_any(const signed char *p) { return pgm_read_byte_near(p); }

#define XYZ_CONSTS_FROM_CONFIG(type, array, CONFIG) \
  static const PROGMEM type array##_P[XYZ] = { X_##CONFIG, Y_##CONFIG, Z_##CONFIG }; \
  static inline type array(const AxisEnum axis) { return pgm_read_any(&array##_P[axis]); } \
  typedef void __void_##CONFIG##__

XYZ_CONSTS_FROM_CONFIG(float, base_min_pos,   MIN_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_max_pos,   MAX_POS);
XYZ_CONSTS_FROM_CONFIG(float, base_home_pos,  HOME_POS);
XYZ_CONSTS_FROM_CONFIG(float, max_length,     MAX_LENGTH);
XYZ_CONSTS_FROM_CONFIG(float, home_bump_mm,   HOME_BUMP_MM);
XYZ_CONSTS_FROM_CONFIG(signed char, home_dir, HOME_DIR);

/**
 * ***************************************************************************
 * ******************************** FUNCTIONS ********************************
 * ***************************************************************************
 */

void stop();

void get_available_commands();
void process_next_command();
void process_parsed_command();

void get_cartesian_from_steppers();
void set_current_from_steppers_for_axis(const AxisEnum axis);

void tool_change(const uint8_t tmp_extruder, const float fr_mm_s=0.0, bool no_move=false);
void report_current_position();
void report_current_position_detail();

/**
 * sync_plan_position
 *
 * Set the planner/stepper positions directly from current_position with
 * no kinematic translation. Used for homing axes and cartesian/core syncing.
 */
void sync_plan_position() {

  planner.set_position_mm(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
}
inline void sync_plan_position_e() { planner.set_e_position_mm(current_position[E_AXIS]); }


  #define SYNC_PLAN_POSITION_KINEMATIC() sync_plan_position()

extern "C" {
  extern char __bss_end;
  extern char __heap_start;
  extern void* __brkval;

  int freeMemory() {
    int free_memory;
    if ((int)__brkval == 0)
      free_memory = ((int)&free_memory) - ((int)&__bss_end);
    else
      free_memory = ((int)&free_memory) - ((int)__brkval);
    return free_memory;
  }
}


/**
 * Inject the next "immediate" command, when possible, onto the front of the queue.
 * Return true if any immediate commands remain to inject.
 */
static bool drain_injected_commands_P() {
  if (injected_commands_P != NULL) {
    size_t i = 0;
    char c, cmd[30];
    strncpy_P(cmd, injected_commands_P, sizeof(cmd) - 1);
    cmd[sizeof(cmd) - 1] = '\0';
    while ((c = cmd[i]) && c != '\n') i++; // find the end of this gcode command
    cmd[i] = '\0';
    if (enqueue_and_echo_command(cmd))     // success?
      injected_commands_P = c ? injected_commands_P + i + 1 : NULL; // next command or done
  }
  return (injected_commands_P != NULL);    // return whether any more remain
}

/**
 * Record one or many commands to run from program memory.
 * Aborts the current queue, if any.
 * Note: drain_injected_commands_P() must be called repeatedly to drain the commands afterwards
 */
void enqueue_and_echo_commands_P(const char * const pgcode) {
  injected_commands_P = pgcode;
  drain_injected_commands_P(); // first command executed asap (when possible)
}

/**
 * Clear the Marlin command queue
 */
void clear_command_queue() {
  cmd_queue_index_r = cmd_queue_index_w;
  commands_in_queue = 0;
}

/**
 * Once a new command is in the ring buffer, call this to commit it
 */
inline void _commit_command(bool say_ok) {
  send_ok[cmd_queue_index_w] = say_ok;
  if (++cmd_queue_index_w >= BUFSIZE) cmd_queue_index_w = 0;
  commands_in_queue++;
}

/**
 * Copy a command from RAM into the main command buffer.
 * Return true if the command was successfully added.
 * Return false for a full buffer, or if the 'command' is a comment.
 */
inline bool _enqueuecommand(const char* cmd, bool say_ok=false) {
  if (*cmd == ';' || commands_in_queue >= BUFSIZE) return false;
  strcpy(command_queue[cmd_queue_index_w], cmd);
  _commit_command(say_ok);
  return true;
}

/**
 * Enqueue with Serial Echo
 */
bool enqueue_and_echo_command(const char* cmd, bool say_ok/*=false*/) {
  if (_enqueuecommand(cmd, say_ok)) {
    SERIAL_ECHO_START();
    SERIAL_ECHOPAIR(MSG_ENQUEUEING, cmd);
    SERIAL_CHAR('"');
    SERIAL_EOL();
    return true;
  }
  return false;
}

void setup_killpin() {
  #if HAS_KILL
    SET_INPUT_PULLUP(KILL_PIN);
  #endif
}

#if ENABLED(FILAMENT_RUNOUT_SENSOR)

  void setup_filrunoutpin() {
    #if ENABLED(ENDSTOPPULLUP_FIL_RUNOUT)
      SET_INPUT_PULLUP(FIL_RUNOUT_PIN);
    #else
      SET_INPUT(FIL_RUNOUT_PIN);
    #endif
  }

#endif

void setup_powerhold() {
  #if HAS_SUICIDE
    OUT_WRITE(SUICIDE_PIN, HIGH);
  #endif
  #if HAS_POWER_SWITCH
    #if ENABLED(PS_DEFAULT_OFF)
      OUT_WRITE(PS_ON_PIN, PS_ON_ASLEEP);
    #else
      OUT_WRITE(PS_ON_PIN, PS_ON_AWAKE);
    #endif
  #endif
}

void suicide() {
  #if HAS_SUICIDE
    OUT_WRITE(SUICIDE_PIN, LOW);
  #endif
}

void servo_init() {
  #if NUM_SERVOS >= 1 && HAS_SERVO_0
    servo[0].attach(SERVO0_PIN);
    servo[0].detach(); // Just set up the pin. We don't have a position yet. Don't move to a random position.
  #endif
  #if NUM_SERVOS >= 2 && HAS_SERVO_1
    servo[1].attach(SERVO1_PIN);
    servo[1].detach();
  #endif
  #if NUM_SERVOS >= 3 && HAS_SERVO_2
    servo[2].attach(SERVO2_PIN);
    servo[2].detach();
  #endif
  #if NUM_SERVOS >= 4 && HAS_SERVO_3
    servo[3].attach(SERVO3_PIN);
    servo[3].detach();
  #endif


}


void gcode_line_error(const char* err, bool doFlush = true) {
  SERIAL_ERROR_START();
  serialprintPGM(err);
  SERIAL_ERRORLN(gcode_LastN);
  //Serial.println(gcode_N);
  if (doFlush) FlushSerialRequestResend();
  serial_count = 0;
}

/**
 * Get all commands waiting on the serial port and queue them.
 * Exit when the buffer is full or when no more characters are
 * left on the serial port.
 */
inline void get_serial_commands() {
  static char serial_line_buffer[MAX_CMD_SIZE];
  static bool serial_comment_mode = false;



  /**
   * Loop while serial characters are incoming and the queue is not full
   */
  int c;
  while (commands_in_queue < BUFSIZE && (c = MYSERIAL.read()) >= 0) {

    char serial_char = c;

    /**
     * If the character ends the line
     */
    if (serial_char == '\n' || serial_char == '\r') {

      serial_comment_mode = false;                      // end of line == end of comment

      if (!serial_count) continue;                      // Skip empty lines

      serial_line_buffer[serial_count] = 0;             // Terminate string
      serial_count = 0;                                 // Reset buffer

      char* command = serial_line_buffer;

      while (*command == ' ') command++;                // Skip leading spaces
      char *npos = (*command == 'N') ? command : NULL;  // Require the N parameter to start the line

      if (npos) {

        bool M110 = strstr_P(command, PSTR("M110")) != NULL;

        if (M110) {
          char* n2pos = strchr(command + 4, 'N');
          if (n2pos) npos = n2pos;
        }

        gcode_N = strtol(npos + 1, NULL, 10);

        if (gcode_N != gcode_LastN + 1 && !M110) {
          gcode_line_error(PSTR(MSG_ERR_LINE_NO));
          return;
        }

        char *apos = strrchr(command, '*');
        if (apos) {
          uint8_t checksum = 0, count = uint8_t(apos - command);
          while (count) checksum ^= command[--count];
          if (strtol(apos + 1, NULL, 10) != checksum) {
            gcode_line_error(PSTR(MSG_ERR_CHECKSUM_MISMATCH));
            return;
          }
        }
        else {
          gcode_line_error(PSTR(MSG_ERR_NO_CHECKSUM));
          return;
        }

        gcode_LastN = gcode_N;
      }

      // Movement commands alert when stopped
      if (IsStopped()) {
        char* gpos = strchr(command, 'G');
        if (gpos) {
          const int codenum = strtol(gpos + 1, NULL, 10);
          switch (codenum) {
            case 0:
            case 1:
            case 2:
            case 3:
              SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
              LCD_MESSAGEPGM(MSG_STOPPED);
              break;
          }
        }
      }

      #if DISABLED(EMERGENCY_PARSER)
        // If command was e-stop process now
        if (strcmp(command, "M108") == 0) {
          wait_for_heatup = false;
          #if ENABLED(ULTIPANEL)
            wait_for_user = false;
          #endif
        }
        if (strcmp(command, "M112") == 0) kill(PSTR(MSG_KILLED));
        if (strcmp(command, "M410") == 0) { quickstop_stepper(); }
      #endif

      #if defined(NO_TIMEOUTS) && NO_TIMEOUTS > 0
        last_command_time = ms;
      #endif

      // Add the command to the queue
      _enqueuecommand(serial_line_buffer, true);
    }
    else if (serial_count >= MAX_CMD_SIZE - 1) {
      // Keep fetching, but ignore normal characters beyond the max length
      // The command will be injected when EOL is reached
    }
    else if (serial_char == '\\') {  // Handle escapes
      if ((c = MYSERIAL.read()) >= 0) {
        // if we have one more character, copy it over
        serial_char = c;
        if (!serial_comment_mode) serial_line_buffer[serial_count++] = serial_char;
      }
      // otherwise do nothing
    }
    else { // it's not a newline, carriage return or escape char
      if (serial_char == ';') serial_comment_mode = true;
      if (!serial_comment_mode) serial_line_buffer[serial_count++] = serial_char;
    }

  } // queue has space, serial has data
}



/**
 * Add to the circular command queue the next command from:
 *  - The command-injection queue (injected_commands_P)
 *  - The active serial input (usually USB)
 *  - The SD card file being actively printed
 */
void get_available_commands() {

  // if any immediate commands remain, don't get other commands yet
  if (drain_injected_commands_P()) return;

  get_serial_commands();


}

/**
 * Set target_extruder from the T parameter or the active_extruder
 *
 * Returns TRUE if the target is invalid
 */
bool get_target_extruder_from_command(const uint16_t code) {
  if (parser.seenval('T')) {
    const int8_t e = parser.value_byte();
    if (e >= EXTRUDERS) {
      SERIAL_ECHO_START();
      SERIAL_CHAR('M');
      SERIAL_ECHO(code);
      SERIAL_ECHOLNPAIR(" " MSG_INVALID_EXTRUDER " ", e);
      return true;
    }
    target_extruder = e;
  }
  else
    target_extruder = active_extruder;

  return false;
}



/**
 * Set an axis' current position to its home position (after homing).
 *
 * For Core and Cartesian robots this applies one-to-one when an
 * individual axis has been homed.
 *
 * DELTA should wait until all homing is done before setting the XYZ
 * current_position to home, because homing is a single operation.
 * In the case where the axis positions are already known and previously
 * homed, DELTA could home to X or Y individually by moving either one
 * to the center. However, homing Z always homes XY and Z.
 *
 * SCARA should wait until all XY homing is done before setting the XY
 * current_position to home, because neither X nor Y is at home until
 * both are at home. Z can however be homed individually.
 *
 * Callers must sync the planner position after calling this!
 */
static void set_axis_is_at_home(const AxisEnum axis) {


  axis_known_position[axis] = axis_homed[axis] = true;

  {
    current_position[axis] = base_home_pos(axis);
  }

}

/**
 * Some planner shorthand inline functions
 */
inline float get_homing_bump_feedrate(const AxisEnum axis) {
  static const uint8_t homing_bump_divisor[] PROGMEM = HOMING_BUMP_DIVISOR;
  uint8_t hbd = pgm_read_byte(&homing_bump_divisor[axis]);
  if (hbd < 1) {
    hbd = 10;
    SERIAL_ECHO_START();
    SERIAL_ECHOLNPGM("Warning: Homing Bump Divisor < 1");
  }
  return homing_feedrate(axis) / hbd;
}

/**
 * Move the planner to the current position from wherever it last moved
 * (or from wherever it has been told it is located).
 */
inline void buffer_line_to_current_position() {
  planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate_mm_s, active_extruder);
}

/**
 * Move the planner to the position stored in the destination array, which is
 * used by G0/G1/G2/G3/G5 and many other functions to set a destination.
 */
inline void buffer_line_to_destination(const float fr_mm_s) {
  planner.buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], fr_mm_s, active_extruder);
}


/**
 *  Plan a move to (X, Y, Z) and set the current_position
 *  The final current_position may not be the one that was requested
 */
void do_blocking_move_to(const float &rx, const float &ry, const float &rz, const float &fr_mm_s/*=0.0*/) {
  const float old_feedrate_mm_s = feedrate_mm_s;



  const float z_feedrate = fr_mm_s ? fr_mm_s : homing_feedrate(Z_AXIS);



    // If Z needs to raise, do it before moving XY
    if (current_position[Z_AXIS] < rz) {
      feedrate_mm_s = z_feedrate;
      current_position[Z_AXIS] = rz;
      buffer_line_to_current_position();
    }

    feedrate_mm_s = fr_mm_s ? fr_mm_s : XY_PROBE_FEEDRATE_MM_S;
    current_position[X_AXIS] = rx;
    current_position[Y_AXIS] = ry;
    buffer_line_to_current_position();

    // If Z needs to lower, do it after moving XY
    if (current_position[Z_AXIS] > rz) {
      feedrate_mm_s = z_feedrate;
      current_position[Z_AXIS] = rz;
      buffer_line_to_current_position();
    }


  stepper.synchronize();

  feedrate_mm_s = old_feedrate_mm_s;


}
void do_blocking_move_to_x(const float &rx, const float &fr_mm_s/*=0.0*/) {
  do_blocking_move_to(rx, current_position[Y_AXIS], current_position[Z_AXIS], fr_mm_s);
}
void do_blocking_move_to_z(const float &rz, const float &fr_mm_s/*=0.0*/) {
  do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], rz, fr_mm_s);
}
void do_blocking_move_to_xy(const float &rx, const float &ry, const float &fr_mm_s/*=0.0*/) {
  do_blocking_move_to(rx, ry, current_position[Z_AXIS], fr_mm_s);
}

//
// Prepare to do endstop or probe moves
// with custom feedrates.
//
//  - Save current feedrates
//  - Reset the rate multiplier
//  - Reset the command timeout
//  - Enable the endstops (for endstop moves)
//
static void setup_for_endstop_or_probe_move() {

  saved_feedrate_mm_s = feedrate_mm_s;
  saved_feedrate_percentage = feedrate_percentage;
  feedrate_percentage = 100;
  refresh_cmd_timeout();
}

static void clean_up_after_endstop_or_probe_move() {

  feedrate_mm_s = saved_feedrate_mm_s;
  feedrate_percentage = saved_feedrate_percentage;
  refresh_cmd_timeout();
}



/**
 * Home an individual linear axis
 */
static void do_homing_move(const AxisEnum axis, const float distance, const float fr_mm_s=0.0) {



  // Tell the planner the axis is at 0
  current_position[axis] = 0;


    sync_plan_position();
    current_position[axis] = distance;
    planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], fr_mm_s ? fr_mm_s : homing_feedrate(axis), active_extruder);


  stepper.synchronize();


  endstops.hit_on_purpose();


}

/**
 * Home an individual "raw axis" to its endstop.
 * This applies to XYZ on Cartesian and Core robots, and
 * to the individual ABC steppers on DELTA and SCARA.
 *
 * At the end of the procedure the axis is marked as
 * homed and the current position of that axis is updated.
 * Kinematic robots should wait till all axes are homed
 * before updating the current position.
 */

#define HOMEAXIS(LETTER) homeaxis(LETTER##_AXIS)

static void homeaxis(const AxisEnum axis) {

  #if IS_SCARA
    // Only Z homing (with probe) is permitted
    if (axis != Z_AXIS) { BUZZ(100, 880); return; }
  #else
    #define CAN_HOME(A) \
      (axis == A##_AXIS && ((A##_MIN_PIN > -1 && A##_HOME_DIR < 0) || (A##_MAX_PIN > -1 && A##_HOME_DIR > 0)))
    if (!CAN_HOME(X) && !CAN_HOME(Y) && !CAN_HOME(Z)) return;
  #endif

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      SERIAL_ECHOPAIR(">>> homeaxis(", axis_codes[axis]);
      SERIAL_CHAR(')');
      SERIAL_EOL();
    }
  #endif

  const int axis_home_dir =
    #if ENABLED(DUAL_X_CARRIAGE)
      (axis == X_AXIS) ? x_home_dir(active_extruder) :
    #endif
    home_dir(axis);

  // Homing Z towards the bed? Deploy the Z probe or endstop.
  #if HOMING_Z_WITH_PROBE
    if (axis == Z_AXIS && DEPLOY_PROBE()) return;
  #endif

  // Set flags for X, Y, Z motor locking
  #if ENABLED(X_DUAL_ENDSTOPS)
    if (axis == X_AXIS) stepper.set_homing_flag_x(true);
  #endif
  #if ENABLED(Y_DUAL_ENDSTOPS)
    if (axis == Y_AXIS) stepper.set_homing_flag_y(true);
  #endif
  #if ENABLED(Z_DUAL_ENDSTOPS)
    if (axis == Z_AXIS) stepper.set_homing_flag_z(true);
  #endif

  // Disable stealthChop if used. Enable diag1 pin on driver.
  #if ENABLED(SENSORLESS_HOMING)
    #if ENABLED(X_IS_TMC2130)
      if (axis == X_AXIS) tmc_sensorless_homing(stepperX);
    #endif
    #if ENABLED(Y_IS_TMC2130)
      if (axis == Y_AXIS) tmc_sensorless_homing(stepperY);
    #endif
  #endif

  // Fast move towards endstop until triggered
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("Home 1 Fast:");
  #endif
  do_homing_move(axis, 1.5 * max_length(axis) * axis_home_dir);

  // When homing Z with probe respect probe clearance
  const float bump = axis_home_dir * (
    #if HOMING_Z_WITH_PROBE
      (axis == Z_AXIS) ? max(Z_CLEARANCE_BETWEEN_PROBES, home_bump_mm(Z_AXIS)) :
    #endif
    home_bump_mm(axis)
  );

  // If a second homing move is configured...
  if (bump) {
    // Move away from the endstop by the axis HOME_BUMP_MM
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("Move Away:");
    #endif
    do_homing_move(axis, -bump);

    // Slow move towards endstop until triggered
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("Home 2 Slow:");
    #endif
    do_homing_move(axis, 2 * bump, get_homing_bump_feedrate(axis));
  }

  /**
   * Home axes that have dual endstops... differently
   */
  #if ENABLED(X_DUAL_ENDSTOPS) || ENABLED(Y_DUAL_ENDSTOPS) || ENABLED(Z_DUAL_ENDSTOPS)
    const bool pos_dir = axis_home_dir > 0;
    #if ENABLED(X_DUAL_ENDSTOPS)
      if (axis == X_AXIS) {
        const bool lock_x1 = pos_dir ? (x_endstop_adj > 0) : (x_endstop_adj < 0);
        const float adj = FABS(x_endstop_adj);
        if (lock_x1) stepper.set_x_lock(true); else stepper.set_x2_lock(true);
        do_homing_move(axis, pos_dir ? -adj : adj);
        if (lock_x1) stepper.set_x_lock(false); else stepper.set_x2_lock(false);
        stepper.set_homing_flag_x(false);
      }
    #endif
    #if ENABLED(Y_DUAL_ENDSTOPS)
      if (axis == Y_AXIS) {
        const bool lock_y1 = pos_dir ? (y_endstop_adj > 0) : (y_endstop_adj < 0);
        const float adj = FABS(y_endstop_adj);
        if (lock_y1) stepper.set_y_lock(true); else stepper.set_y2_lock(true);
        do_homing_move(axis, pos_dir ? -adj : adj);
        if (lock_y1) stepper.set_y_lock(false); else stepper.set_y2_lock(false);
        stepper.set_homing_flag_y(false);
      }
    #endif
    #if ENABLED(Z_DUAL_ENDSTOPS)
      if (axis == Z_AXIS) {
        const bool lock_z1 = pos_dir ? (z_endstop_adj > 0) : (z_endstop_adj < 0);
        const float adj = FABS(z_endstop_adj);
        if (lock_z1) stepper.set_z_lock(true); else stepper.set_z2_lock(true);
        do_homing_move(axis, pos_dir ? -adj : adj);
        if (lock_z1) stepper.set_z_lock(false); else stepper.set_z2_lock(false);
        stepper.set_homing_flag_z(false);
      }
    #endif
  #endif

  #if IS_SCARA

    set_axis_is_at_home(axis);
    SYNC_PLAN_POSITION_KINEMATIC();

  #elif ENABLED(DELTA)

    // Delta has already moved all three towers up in G28
    // so here it re-homes each tower in turn.
    // Delta homing treats the axes as normal linear axes.

    // retrace by the amount specified in delta_endstop_adj + additional 0.1mm in order to have minimum steps
    if (delta_endstop_adj[axis] * Z_HOME_DIR <= 0) {
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("delta_endstop_adj:");
      #endif
      do_homing_move(axis, delta_endstop_adj[axis] - 0.1 * Z_HOME_DIR);
    }

  #else

    // For cartesian/core machines,
    // set the axis to its home position
    set_axis_is_at_home(axis);
    sync_plan_position();

    destination[axis] = current_position[axis];

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS("> AFTER set_axis_is_at_home", current_position);
    #endif

  #endif

  // Re-enable stealthChop if used. Disable diag1 pin on driver.
  #if ENABLED(SENSORLESS_HOMING)
    #if ENABLED(X_IS_TMC2130)
      if (axis == X_AXIS) tmc_sensorless_homing(stepperX, false);
    #endif
    #if ENABLED(Y_IS_TMC2130)
      if (axis == Y_AXIS) tmc_sensorless_homing(stepperY, false);
    #endif
  #endif

  // Put away the Z probe
  #if HOMING_Z_WITH_PROBE
    if (axis == Z_AXIS && STOW_PROBE()) return;
  #endif

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      SERIAL_ECHOPAIR("<<< homeaxis(", axis_codes[axis]);
      SERIAL_CHAR(')');
      SERIAL_EOL();
    }
  #endif
} // homeaxis()


/**
 * ***************************************************************************
 * ***************************** G-CODE HANDLING *****************************
 * ***************************************************************************
 */

/**
 * Set XYZE destination and feedrate from the current GCode command
 *
 *  - Set destination from included axis codes
 *  - Set to current for missing axis codes
 *  - Set the feedrate, if included
 */
void gcode_get_destination() {
  LOOP_XYZE(i) {
    if (parser.seen(axis_codes[i])) {
      const float v = parser.value_axis_units((AxisEnum)i) + (axis_relative_modes[i] || relative_mode ? current_position[i] : 0);
      destination[i] = i == E_AXIS ? v : LOGICAL_TO_NATIVE(v, i);
    }
    else
      destination[i] = current_position[i];
  }

  if (parser.linearval('F') > 0.0)
    feedrate_mm_s = MMM_TO_MMS(parser.value_feedrate());


}



/**************************************************
 ***************** GCode Handlers *****************
 **************************************************/


  #define G0_G1_CONDITION true


/**
 * G0, G1: Coordinated movement of X Y Z E axes
 */
inline void gcode_G0_G1(

) {
  if (IsRunning() && G0_G1_CONDITION) {
    gcode_get_destination(); // For X Y Z E F
      prepare_move_to_destination();
  }
}



void dwell(millis_t time) {
  refresh_cmd_timeout();
  time += previous_cmd_ms;
  while (PENDING(millis(), time)) idle();
}

/**
 * G4: Dwell S<seconds> or P<milliseconds>
 */
inline void gcode_G4() {
  millis_t dwell_ms = 0;

  if (parser.seenval('P')) dwell_ms = parser.value_millis(); // milliseconds to wait
  if (parser.seenval('S')) dwell_ms = parser.value_millis_from_seconds(); // seconds to wait

  stepper.synchronize();
  #if ENABLED(NANODLP_Z_SYNC)
    SERIAL_ECHOLNPGM(MSG_Z_MOVE_COMP);
  #endif

  if (!lcd_hasstatus()) LCD_MESSAGEPGM(MSG_DWELL);

  dwell(dwell_ms);
}



/**
 * G28: Home all axes according to settings
 *
 * Parameters
 *
 *  None  Home to all axes with no parameters.
 *        With QUICK_HOME enabled XY will home together, then Z.
 *
 * Cartesian parameters
 *
 *  X   Home to the X endstop
 *  Y   Home to the Y endstop
 *  Z   Home to the Z endstop
 *
 */
inline void gcode_G28(const bool always_home_all) {


  // Wait for planner moves to finish!
  stepper.synchronize();

  setup_for_endstop_or_probe_move();
  
  endstops.enable(true); // Enable endstops for next homing move

  #if ENABLED(DELTA)

    home_delta();
    UNUSED(always_home_all);

  #else // NOT DELTA

    const bool homeX = always_home_all || parser.seen('X'),
               homeY = always_home_all || parser.seen('Y'),
               homeZ = always_home_all || parser.seen('Z'),
               home_all = (!homeX && !homeY && !homeZ) || (homeX && homeY && homeZ);

    set_destination_from_current();

    #if Z_HOME_DIR > 0  // If homing away from BED do Z first

      if (home_all || homeZ) {
        HOMEAXIS(Z);
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("> HOMEAXIS(Z)", current_position);
        #endif
      }

    #endif

    if (home_all || homeX || homeY) {
      // Raise Z before homing any other axes and z is not already high enough (never lower z)
      destination[Z_AXIS] = Z_HOMING_HEIGHT;
      if (destination[Z_AXIS] > current_position[Z_AXIS]) {

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING))
            SERIAL_ECHOLNPAIR("Raise Z (before homing) to ", destination[Z_AXIS]);
        #endif

        do_blocking_move_to_z(destination[Z_AXIS]);
      }
    }



    // Home X
    if (home_all || homeX) {



        HOMEAXIS(X);


    }

    #if DISABLED(HOME_Y_BEFORE_X)
      // Home Y
      if (home_all || homeY) {
        HOMEAXIS(Y);

      }
    #endif

    // Home Z last if homing towards the bed
    #if Z_HOME_DIR < 0
      if (home_all || homeZ) {
        #if ENABLED(Z_SAFE_HOMING)
          home_z_safely();
        #else
          HOMEAXIS(Z);
        #endif
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("> (home_all || homeZ) > final", current_position);
        #endif
      } // home_all || homeZ
    #endif // Z_HOME_DIR < 0

    SYNC_PLAN_POSITION_KINEMATIC();

  #endif // !DELTA (gcode_G28)

  endstops.not_homing();

 
  clean_up_after_endstop_or_probe_move();

  // Restore the active tool after homing
  

#ifdef RESIN
	set_axis_is_at_home(X_AXIS);
	set_axis_is_at_home(Y_AXIS);
#endif

  lcd_refresh();

  report_current_position();


} // G28

void home_all_axes() { gcode_G28(true); }



/**
 * G92: Set current position to given X Y Z E
 */
inline void gcode_G92() {

  stepper.synchronize();



  #if ENABLED(CNC_COORDINATE_SYSTEMS)
    #define IS_G92_0 (parser.subcode == 0)
  #else
    #define IS_G92_0 true
  #endif

  bool didE = false;
  #if IS_SCARA || !HAS_POSITION_SHIFT
    bool didXYZ = false;
  #else
    constexpr bool didXYZ = false;
  #endif

  if (IS_G92_0) LOOP_XYZE(i) {
    if (parser.seenval(axis_codes[i])) {
      const float l = parser.value_axis_units((AxisEnum)i),
                  v = i == E_AXIS ? l : LOGICAL_TO_NATIVE(l, i),
                  d = v - current_position[i];
      if (!NEAR_ZERO(d)) {
        #if IS_SCARA || !HAS_POSITION_SHIFT
          if (i == E_AXIS) didE = true; else didXYZ = true;
          current_position[i] = v;        // Without workspaces revert to Marlin 1.0 behavior
        #elif HAS_POSITION_SHIFT
          if (i == E_AXIS) {
            didE = true;
            current_position[E_AXIS] = v; // When using coordinate spaces, only E is set directly
          }
          else {
            position_shift[i] += d;       // Other axes simply offset the coordinate space
            update_software_endstops((AxisEnum)i);
          }
        #endif
      }
    }
  }


  if (didXYZ)
    SYNC_PLAN_POSITION_KINEMATIC();
  else if (didE)
    sync_plan_position_e();

  report_current_position();
}


#if ENABLED(RESIN)
    inline void gcode_M899() {
      SERIAL_ECHOLNPGM("laser ON");
      analogWrite(LASER_FIRING_PIN, 32);
    }

    inline void gcode_M898() {
      SERIAL_ECHOLNPGM("laser OFF");
      analogWrite(LASER_FIRING_PIN, 0);
    }

#endif



/**
 * M17: Enable power on all stepper motors
 */
inline void gcode_M17() {
  LCD_MESSAGEPGM(MSG_NO_MOVE);
  enable_all_steppers();
}



/**
 * M31: Get the time since the start of SD Print (or last M109)
 */
inline void gcode_M31() {
  char buffer[21];
  duration_t elapsed = print_job_timer.duration();
  elapsed.toString(buffer);
  lcd_setstatus(buffer);

  SERIAL_ECHO_START();
  SERIAL_ECHOLNPAIR("Print time: ", buffer);
}


/**
 * Sensitive pin test for M42, M226
 */
static bool pin_is_protected(const int8_t pin) {
  static const int8_t sensitive_pins[] PROGMEM = SENSITIVE_PINS;
  for (uint8_t i = 0; i < COUNT(sensitive_pins); i++)
    if (pin == (int8_t)pgm_read_byte(&sensitive_pins[i])) return true;
  return false;
}

/**
 * M42: Change pin status via GCode
 *
 *  P<pin>  Pin number (LED if omitted)
 *  S<byte> Pin status from 0 - 255
 */
inline void gcode_M42() {
  if (!parser.seenval('S')) return;
  const byte pin_status = parser.value_byte();

  const int pin_number = parser.intval('P', LED_PIN);
  if (pin_number < 0) return;

  if (pin_is_protected(pin_number)) {
    SERIAL_ERROR_START();
    SERIAL_ERRORLNPGM(MSG_ERR_PROTECTED_PIN);
    return;
  }

  pinMode(pin_number, OUTPUT);
  digitalWrite(pin_number, pin_status);
  analogWrite(pin_number, pin_status);

  #if FAN_COUNT > 0
    switch (pin_number) {
      #if HAS_FAN0
        case FAN_PIN: fanSpeeds[0] = pin_status; break;
      #endif
      #if HAS_FAN1
        case FAN1_PIN: fanSpeeds[1] = pin_status; break;
      #endif
      #if HAS_FAN2
        case FAN2_PIN: fanSpeeds[2] = pin_status; break;
      #endif
    }
  #endif
}


/**
 * M75: Start print timer
 */
inline void gcode_M75() { print_job_timer.start(); }

/**
 * M76: Pause print timer
 */
inline void gcode_M76() { print_job_timer.pause(); }

/**
 * M77: Stop print timer
 */
inline void gcode_M77() { print_job_timer.stop(); }


/**
 * M104: Set hot end temperature
 */
inline void gcode_M104() {
  if (get_target_extruder_from_command(104)) return;
  if (DEBUGGING(DRYRUN)) return;



  if (parser.seenval('S')) {
    const int16_t temp = parser.value_celsius();
    thermalManager.setTargetHotend(temp, target_extruder);



    if (parser.value_celsius() > thermalManager.degHotend(target_extruder))
      lcd_status_printf_P(0, PSTR("E%i %s"), target_extruder + 1, MSG_HEATING);
  }


}

/**
 * M105: Read hot end and bed temperature
 */
inline void gcode_M105() {
  if (get_target_extruder_from_command(105)) return;

  #if HAS_TEMP_HOTEND || HAS_TEMP_BED
    SERIAL_PROTOCOLPGM(MSG_OK);
    thermalManager.print_heaterstates();
  #else // !HAS_TEMP_HOTEND && !HAS_TEMP_BED
    SERIAL_ERROR_START();
    SERIAL_ERRORLNPGM(MSG_ERR_NO_THERMISTORS);
  #endif

  SERIAL_EOL();
}

/**
 * M109: Sxxx Wait for extruder(s) to reach temperature. Waits only when heating.
 *       Rxxx Wait for extruder(s) to reach temperature. Waits when heating and cooling.
 */

#ifndef MIN_COOLING_SLOPE_DEG
  #define MIN_COOLING_SLOPE_DEG 1.50
#endif
#ifndef MIN_COOLING_SLOPE_TIME
  #define MIN_COOLING_SLOPE_TIME 60
#endif

inline void gcode_M109() {

  if (get_target_extruder_from_command(109)) return;
  if (DEBUGGING(DRYRUN)) return;

  #if ENABLED(SINGLENOZZLE)
    if (target_extruder != active_extruder) return;
  #endif

  const bool no_wait_for_cooling = parser.seenval('S');
  if (no_wait_for_cooling || parser.seenval('R')) {
    const int16_t temp = parser.value_celsius();
    thermalManager.setTargetHotend(temp, target_extruder);

    

    if (thermalManager.isHeatingHotend(target_extruder)) lcd_status_printf_P(0, PSTR("E%i %s"), target_extruder + 1, MSG_HEATING);
  }
  else return;


  #if TEMP_RESIDENCY_TIME > 0
    millis_t residency_start_ms = 0;
    // Loop until the temperature has stabilized
    #define TEMP_CONDITIONS (!residency_start_ms || PENDING(now, residency_start_ms + (TEMP_RESIDENCY_TIME) * 1000UL))
  #else
    // Loop until the temperature is very close target
    #define TEMP_CONDITIONS (wants_to_cool ? thermalManager.isCoolingHotend(target_extruder) : thermalManager.isHeatingHotend(target_extruder))
  #endif

  float target_temp = -1.0, old_temp = 9999.0;
  bool wants_to_cool = false;
  wait_for_heatup = true;
  millis_t now, next_temp_ms = 0, next_cool_check_ms = 0;

  do {
    // Target temperature might be changed during the loop
    if (target_temp != thermalManager.degTargetHotend(target_extruder)) {
      wants_to_cool = thermalManager.isCoolingHotend(target_extruder);
      target_temp = thermalManager.degTargetHotend(target_extruder);

      // Exit if S<lower>, continue if S<higher>, R<lower>, or R<higher>
      if (no_wait_for_cooling && wants_to_cool) break;
    }

    now = millis();
    if (ELAPSED(now, next_temp_ms)) { //Print temp & remaining time every 1s while waiting
      next_temp_ms = now + 1000UL;
      thermalManager.print_heaterstates();
      #if TEMP_RESIDENCY_TIME > 0
        SERIAL_PROTOCOLPGM(" W:");
        if (residency_start_ms)
          SERIAL_PROTOCOL(long((((TEMP_RESIDENCY_TIME) * 1000UL) - (now - residency_start_ms)) / 1000UL));
        else
          SERIAL_PROTOCOLCHAR('?');
      #endif
      SERIAL_EOL();
    }

    idle();
    refresh_cmd_timeout(); // to prevent stepper_inactive_time from running out

    const float temp = thermalManager.degHotend(target_extruder);



    // Prevent a wait-forever situation if R is misused i.e. M109 R0
    if (wants_to_cool) {
      // break after MIN_COOLING_SLOPE_TIME seconds
      // if the temperature did not drop at least MIN_COOLING_SLOPE_DEG
      if (!next_cool_check_ms || ELAPSED(now, next_cool_check_ms)) {
        if (old_temp - temp < MIN_COOLING_SLOPE_DEG) break;
        next_cool_check_ms = now + 1000UL * MIN_COOLING_SLOPE_TIME;
        old_temp = temp;
      }
    }

  } while (wait_for_heatup && TEMP_CONDITIONS);

  if (wait_for_heatup) {
    LCD_MESSAGEPGM(MSG_HEATING_COMPLETE);

  }


}

/**
 * M110: Set Current Line Number
 */
inline void gcode_M110() {
  if (parser.seenval('N')) gcode_LastN = parser.value_long();
}

/**
 * M111: Set the debug level
 */
inline void gcode_M111() {
  if (parser.seen('S')) marlin_debug_flags = parser.byteval('S');

  const static char str_debug_1[] PROGMEM = MSG_DEBUG_ECHO,
                    str_debug_2[] PROGMEM = MSG_DEBUG_INFO,
                    str_debug_4[] PROGMEM = MSG_DEBUG_ERRORS,
                    str_debug_8[] PROGMEM = MSG_DEBUG_DRYRUN,
                    str_debug_16[] PROGMEM = MSG_DEBUG_COMMUNICATION
                    #if ENABLED(DEBUG_LEVELING_FEATURE)
                      , str_debug_32[] PROGMEM = MSG_DEBUG_LEVELING
                    #endif
                    ;

  const static char* const debug_strings[] PROGMEM = {
    str_debug_1, str_debug_2, str_debug_4, str_debug_8, str_debug_16
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      , str_debug_32
    #endif
  };

  SERIAL_ECHO_START();
  SERIAL_ECHOPGM(MSG_DEBUG_PREFIX);
  if (marlin_debug_flags) {
    uint8_t comma = 0;
    for (uint8_t i = 0; i < COUNT(debug_strings); i++) {
      if (TEST(marlin_debug_flags, i)) {
        if (comma++) SERIAL_CHAR(',');
        serialprintPGM((char*)pgm_read_word(&debug_strings[i]));
      }
    }
  }
  else {
    SERIAL_ECHOPGM(MSG_DEBUG_OFF);
  }
  SERIAL_EOL();
}



/**
 * M140: Set bed temperature
 */
inline void gcode_M140() {
  if (DEBUGGING(DRYRUN)) return;
  if (parser.seenval('S')) thermalManager.setTargetBed(parser.value_celsius());
}



/**
 * M81: Turn off Power, including Power Supply, if there is one.
 *
 *      This code should ALWAYS be available for EMERGENCY SHUTDOWN!
 */
inline void gcode_M81() {
  thermalManager.disable_all_heaters();
  stepper.finish_and_disable();

  safe_delay(1000); // Wait 1 second before switching off


}

/**
 * M82: Set E codes absolute (default)
 */
inline void gcode_M82() { axis_relative_modes[E_AXIS] = false; }

/**
 * M83: Set E codes relative while in Absolute Coordinates (G90) mode
 */
inline void gcode_M83() { axis_relative_modes[E_AXIS] = true; }

/**
 * M18, M84: Disable stepper motors
 */
inline void gcode_M18_M84() {
  if (parser.seenval('S')) {
    stepper_inactive_time = parser.value_millis_from_seconds();
  }
  else {
    bool all_axis = !((parser.seen('X')) || (parser.seen('Y')) || (parser.seen('Z')) || (parser.seen('E')));
    if (all_axis) {
      stepper.finish_and_disable();
    }
    else {
      stepper.synchronize();
      if (parser.seen('X')) disable_X();
      if (parser.seen('Y')) disable_Y();
      if (parser.seen('Z')) disable_Z();
      #if E0_ENABLE_PIN != X_ENABLE_PIN && E1_ENABLE_PIN != Y_ENABLE_PIN // Only enable on boards that have separate ENABLE_PINS
        if (parser.seen('E')) disable_e_steppers();
      #endif
    }


  }
}

/**
 * M85: Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
 */
inline void gcode_M85() {
  if (parser.seen('S')) max_inactive_time = parser.value_millis_from_seconds();
}

/**
 * Multi-stepper support for M92, M201, M203
 */
#if ENABLED(DISTINCT_E_FACTORS)
  #define GET_TARGET_EXTRUDER(CMD) if (get_target_extruder_from_command(CMD)) return
  #define TARGET_EXTRUDER target_extruder
#else
  #define GET_TARGET_EXTRUDER(CMD) NOOP
  #define TARGET_EXTRUDER 0
#endif

/**
 * M92: Set axis steps-per-unit for one or more axes, X, Y, Z, and E.
 *      (Follows the same syntax as G92)
 *
 *      With multiple extruders use T to specify which one.
 */
inline void gcode_M92() {

  GET_TARGET_EXTRUDER(92);

  LOOP_XYZE(i) {
    if (parser.seen(axis_codes[i])) {
      if (i == E_AXIS) {
        const float value = parser.value_per_axis_unit((AxisEnum)(E_AXIS + TARGET_EXTRUDER));
        if (value < 20.0) {
          float factor = planner.axis_steps_per_mm[E_AXIS + TARGET_EXTRUDER] / value; // increase e constants if M92 E14 is given for netfab.
          planner.max_jerk[E_AXIS] *= factor;
          planner.max_feedrate_mm_s[E_AXIS + TARGET_EXTRUDER] *= factor;
          planner.max_acceleration_steps_per_s2[E_AXIS + TARGET_EXTRUDER] *= factor;
        }
        planner.axis_steps_per_mm[E_AXIS + TARGET_EXTRUDER] = value;
      }
      else {
        planner.axis_steps_per_mm[i] = parser.value_per_axis_unit((AxisEnum)i);
      }
    }
  }
  planner.refresh_positioning();
}

/**
 * Output the current position to serial
 */
void report_current_position() {
  SERIAL_PROTOCOLPGM("X:");
  SERIAL_PROTOCOL(LOGICAL_X_POSITION(current_position[X_AXIS]));
  SERIAL_PROTOCOLPGM(" Y:");
  SERIAL_PROTOCOL(LOGICAL_Y_POSITION(current_position[Y_AXIS]));
  SERIAL_PROTOCOLPGM(" Z:");
  SERIAL_PROTOCOL(LOGICAL_Z_POSITION(current_position[Z_AXIS]));
  SERIAL_PROTOCOLPGM(" E:");
  SERIAL_PROTOCOL(current_position[E_AXIS]);

  stepper.report_positions();


}



/**
 * M114: Report current position to host
 */
inline void gcode_M114() {



  stepper.synchronize();
  report_current_position();
}



inline void gcode_M115() {
  SERIAL_PROTOCOLLNPGM(MSG_M115_REPORT);

}

/**
 * M117: Set LCD Status Message
 */
inline void gcode_M117() { lcd_setstatus(parser.string_arg); }

/**
 * M118: Display a message in the host console.
 *
 *  A1  Append '// ' for an action command, as in OctoPrint
 *  E1  Have the host 'echo:' the text
 */
inline void gcode_M118() {
  if (parser.boolval('E')) SERIAL_ECHO_START();
  if (parser.boolval('A')) SERIAL_ECHOPGM("// ");
  SERIAL_ECHOLN(parser.string_arg);
}

/**
 * M119: Output endstop states to serial output
 */
inline void gcode_M119() { endstops.M119(); }

/**
 * M120: Enable endstops and set non-homing endstop state to "enabled"
 */
inline void gcode_M120() { endstops.enable_globally(true); }

/**
 * M121: Disable endstops and set non-homing endstop state to "disabled"
 */
inline void gcode_M121() { endstops.enable_globally(false); }



/**
 * M201: Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
 *
 *       With multiple extruders use T to specify which one.
 */
inline void gcode_M201() {

  GET_TARGET_EXTRUDER(201);

  LOOP_XYZE(i) {
    if (parser.seen(axis_codes[i])) {
      const uint8_t a = i + (i == E_AXIS ? TARGET_EXTRUDER : 0);
      planner.max_acceleration_mm_per_s2[a] = parser.value_axis_units((AxisEnum)a);
    }
  }
  // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
  planner.reset_acceleration_rates();
}



/**
 * M203: Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in units/sec
 *
 *       With multiple extruders use T to specify which one.
 */
inline void gcode_M203() {

  GET_TARGET_EXTRUDER(203);

  LOOP_XYZE(i)
    if (parser.seen(axis_codes[i])) {
      const uint8_t a = i + (i == E_AXIS ? TARGET_EXTRUDER : 0);
      planner.max_feedrate_mm_s[a] = parser.value_axis_units((AxisEnum)a);
    }
}

/**
 * M204: Set Accelerations in units/sec^2 (M204 P1200 R3000 T3000)
 *
 *    P = Printing moves
 *    R = Retract only (no X, Y, Z) moves
 *    T = Travel (non printing) moves
 *
 *  Also sets minimum segment time in ms (B20000) to prevent buffer under-runs and M20 minimum feedrate
 */
inline void gcode_M204() {
  if (parser.seen('S')) {  // Kept for legacy compatibility. Should NOT BE USED for new developments.
    planner.travel_acceleration = planner.acceleration = parser.value_linear_units();
    SERIAL_ECHOLNPAIR("Setting Print and Travel Acceleration: ", planner.acceleration);
  }
  if (parser.seen('P')) {
    planner.acceleration = parser.value_linear_units();
    SERIAL_ECHOLNPAIR("Setting Print Acceleration: ", planner.acceleration);
  }
  if (parser.seen('R')) {
    planner.retract_acceleration = parser.value_linear_units();
    SERIAL_ECHOLNPAIR("Setting Retract Acceleration: ", planner.retract_acceleration);
  }
  if (parser.seen('T')) {
    planner.travel_acceleration = parser.value_linear_units();
    SERIAL_ECHOLNPAIR("Setting Travel Acceleration: ", planner.travel_acceleration);
  }
}

/**
 * M205: Set Advanced Settings
 *
 *    S = Min Feed Rate (units/s)
 *    T = Min Travel Feed Rate (units/s)
 *    B = Min Segment Time (µs)
 *    X = Max X Jerk (units/sec^2)
 *    Y = Max Y Jerk (units/sec^2)
 *    Z = Max Z Jerk (units/sec^2)
 *    E = Max E Jerk (units/sec^2)
 */
inline void gcode_M205() {
  if (parser.seen('S')) planner.min_feedrate_mm_s = parser.value_linear_units();
  if (parser.seen('T')) planner.min_travel_feedrate_mm_s = parser.value_linear_units();
  if (parser.seen('B')) planner.min_segment_time_us = parser.value_ulong();
  if (parser.seen('X')) planner.max_jerk[X_AXIS] = parser.value_linear_units();
  if (parser.seen('Y')) planner.max_jerk[Y_AXIS] = parser.value_linear_units();
  if (parser.seen('Z')) planner.max_jerk[Z_AXIS] = parser.value_linear_units();
  if (parser.seen('E')) planner.max_jerk[E_AXIS] = parser.value_linear_units();
}



/**
 * M211: Enable, Disable, and/or Report software endstops
 *
 * Usage: M211 S1 to enable, M211 S0 to disable, M211 alone for report
 */
inline void gcode_M211() {
  SERIAL_ECHO_START();
  #if HAS_SOFTWARE_ENDSTOPS
    if (parser.seen('S')) soft_endstops_enabled = parser.value_bool();
    SERIAL_ECHOPGM(MSG_SOFT_ENDSTOPS);
    serialprintPGM(soft_endstops_enabled ? PSTR(MSG_ON) : PSTR(MSG_OFF));
  #else
    SERIAL_ECHOPGM(MSG_SOFT_ENDSTOPS);
    SERIAL_ECHOPGM(MSG_OFF);
  #endif
  SERIAL_ECHOPGM(MSG_SOFT_MIN);
  SERIAL_ECHOPAIR(    MSG_X, LOGICAL_X_POSITION(soft_endstop_min[X_AXIS]));
  SERIAL_ECHOPAIR(" " MSG_Y, LOGICAL_Y_POSITION(soft_endstop_min[Y_AXIS]));
  SERIAL_ECHOPAIR(" " MSG_Z, LOGICAL_Z_POSITION(soft_endstop_min[Z_AXIS]));
  SERIAL_ECHOPGM(MSG_SOFT_MAX);
  SERIAL_ECHOPAIR(    MSG_X, LOGICAL_X_POSITION(soft_endstop_max[X_AXIS]));
  SERIAL_ECHOPAIR(" " MSG_Y, LOGICAL_Y_POSITION(soft_endstop_max[Y_AXIS]));
  SERIAL_ECHOLNPAIR(" " MSG_Z, LOGICAL_Z_POSITION(soft_endstop_max[Z_AXIS]));
}


/**
 * M220: Set speed percentage factor, aka "Feed Rate" (M220 S95)
 */
inline void gcode_M220() {
  if (parser.seenval('S')) feedrate_percentage = parser.value_int();
}

/**
 * M221: Set extrusion percentage (M221 T0 S95)
 */
inline void gcode_M221() {
  if (get_target_extruder_from_command(221)) return;
  if (parser.seenval('S')) {
    planner.flow_percentage[target_extruder] = parser.value_int();
    planner.refresh_e_factor(target_extruder);
  }
}

/**
 * M226: Wait until the specified pin reaches the state required (M226 P<pin> S<state>)
 */
inline void gcode_M226() {
  if (parser.seen('P')) {
    const int pin_number = parser.value_int(),
              pin_state = parser.intval('S', -1); // required pin state - default is inverted

    if (WITHIN(pin_state, -1, 1) && pin_number > -1 && !pin_is_protected(pin_number)) {

      int target = LOW;

      stepper.synchronize();

      pinMode(pin_number, INPUT);
      switch (pin_state) {
        case 1:
          target = HIGH;
          break;
        case 0:
          target = LOW;
          break;
        case -1:
          target = !digitalRead(pin_number);
          break;
      }

      while (digitalRead(pin_number) != target) idle();

    } // pin_state -1 0 1 && pin_number > -1
  } // parser.seen('P')
}



/**
 * M303: PID relay autotune
 *
 *       S<temperature> sets the target temperature. (default 150C)
 *       E<extruder> (-1 for the bed) (default 0)
 *       C<cycles>
 *       U<bool> with a non-zero value will apply the result to current settings
 */
inline void gcode_M303() {

    SERIAL_ERROR_START();
    SERIAL_ERRORLNPGM(MSG_ERR_M303_DISABLED);

}



/**
 * M400: Finish all moves
 */
inline void gcode_M400() { stepper.synchronize(); }



void quickstop_stepper() {
  stepper.quick_stop();
  stepper.synchronize();
  set_current_from_steppers_for_axis(ALL_AXES);
  SYNC_PLAN_POSITION_KINEMATIC();
}




/**
 * M500: Store settings in EEPROM
 */
inline void gcode_M500() {
  (void)settings.save();
}

/**
 * M501: Read settings from EEPROM
 */
inline void gcode_M501() {
  (void)settings.load();
}

/**
 * M502: Revert to default settings
 */
inline void gcode_M502() {
  (void)settings.reset();
}



/**
 * M907: Set digital trimpot motor current using axis codes X, Y, Z, E, B, S
 */
inline void gcode_M907() {

}



/**
 * M355: Turn case light on/off and set brightness
 *
 *   P<byte>  Set case light brightness (PWM pin required - ignored otherwise)
 *
 *   S<bool>  Set case light on/off
 *
 *   When S turns on the light on a PWM pin then the current brightness level is used/restored
 *
 *   M355 P200 S0 turns off the light & sets the brightness level
 *   M355 S1 turns on the light with a brightness of 200 (assuming a PWM pin)
 */
inline void gcode_M355() {
  #if HAS_CASE_LIGHT
    uint8_t args = 0;
    if (parser.seenval('P')) ++args, case_light_brightness = parser.value_byte();
    if (parser.seenval('S')) ++args, case_light_on = parser.value_bool();
    if (args) update_case_light();

    // always report case light status
    SERIAL_ECHO_START();
    if (!case_light_on) {
      SERIAL_ECHOLN("Case light: off");
    }
    else {
      if (!USEABLE_HARDWARE_PWM(CASE_LIGHT_PIN)) SERIAL_ECHOLN("Case light: on");
      else SERIAL_ECHOLNPAIR("Case light: ", (int)case_light_brightness);
    }

  #else
    SERIAL_ERROR_START();
    SERIAL_ERRORLNPGM(MSG_ERR_M355_NONE);
  #endif // HAS_CASE_LIGHT
}


/**
 * M999: Restart after being stopped
 *
 * Default behaviour is to flush the serial buffer and request
 * a resend to the host starting on the last N line received.
 *
 * Sending "M999 S1" will resume printing without flushing the
 * existing command buffer.
 *
 */
inline void gcode_M999() {
  Running = true;
  lcd_reset_alert_level();

  if (parser.boolval('S')) return;

  // gcode_LastN = Stopped_gcode_LastN;
  FlushSerialRequestResend();
}

inline void invalid_extruder_error(const uint8_t e) {
  SERIAL_ECHO_START();
  SERIAL_CHAR('T');
  SERIAL_ECHO_F(e, DEC);
  SERIAL_CHAR(' ');
  SERIAL_ECHOLN(MSG_INVALID_EXTRUDER);
}



/**
 * Perform a tool-change, which may result in moving the
 * previous tool out of the way and the new tool into place.
 */
void tool_change(const uint8_t tmp_extruder, const float fr_mm_s/*=0.0*/, bool no_move/*=false*/) {
  #if ENABLED(MIXING_EXTRUDER) && MIXING_VIRTUAL_TOOLS > 1

    if (tmp_extruder >= MIXING_VIRTUAL_TOOLS)
      return invalid_extruder_error(tmp_extruder);

    // T0-Tnnn: Switch virtual tool by changing the mix
    for (uint8_t j = 0; j < MIXING_STEPPERS; j++)
      mixing_factor[j] = mixing_virtual_tool_mix[tmp_extruder][j];

  #else // !MIXING_EXTRUDER || MIXING_VIRTUAL_TOOLS <= 1

    if (tmp_extruder >= EXTRUDERS)
      return invalid_extruder_error(tmp_extruder);

    #if HOTENDS > 1

      const float old_feedrate_mm_s = fr_mm_s > 0.0 ? fr_mm_s : feedrate_mm_s;

      feedrate_mm_s = fr_mm_s > 0.0 ? fr_mm_s : XY_PROBE_FEEDRATE_MM_S;

      if (tmp_extruder != active_extruder) {
        if (!no_move && axis_unhomed_error()) {
          no_move = true;
          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("No move on toolchange");
          #endif
        }

        // Save current position to destination, for use later
        set_destination_from_current();

        #if ENABLED(DUAL_X_CARRIAGE)

          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) {
              SERIAL_ECHOPGM("Dual X Carriage Mode ");
              switch (dual_x_carriage_mode) {
                case DXC_FULL_CONTROL_MODE: SERIAL_ECHOLNPGM("DXC_FULL_CONTROL_MODE"); break;
                case DXC_AUTO_PARK_MODE: SERIAL_ECHOLNPGM("DXC_AUTO_PARK_MODE"); break;
                case DXC_DUPLICATION_MODE: SERIAL_ECHOLNPGM("DXC_DUPLICATION_MODE"); break;
              }
            }
          #endif

          const float xhome = x_home_pos(active_extruder);
          if (dual_x_carriage_mode == DXC_AUTO_PARK_MODE
              && IsRunning()
              && (delayed_move_time || current_position[X_AXIS] != xhome)
          ) {
            float raised_z = current_position[Z_AXIS] + TOOLCHANGE_PARK_ZLIFT;
            #if ENABLED(MAX_SOFTWARE_ENDSTOPS)
              NOMORE(raised_z, soft_endstop_max[Z_AXIS]);
            #endif
            #if ENABLED(DEBUG_LEVELING_FEATURE)
              if (DEBUGGING(LEVELING)) {
                SERIAL_ECHOLNPAIR("Raise to ", raised_z);
                SERIAL_ECHOLNPAIR("MoveX to ", xhome);
                SERIAL_ECHOLNPAIR("Lower to ", current_position[Z_AXIS]);
              }
            #endif
            // Park old head: 1) raise 2) move to park position 3) lower
            for (uint8_t i = 0; i < 3; i++)
              planner.buffer_line(
                i == 0 ? current_position[X_AXIS] : xhome,
                current_position[Y_AXIS],
                i == 2 ? current_position[Z_AXIS] : raised_z,
                current_position[E_AXIS],
                planner.max_feedrate_mm_s[i == 1 ? X_AXIS : Z_AXIS],
                active_extruder
              );
            stepper.synchronize();
          }

          // Apply Y & Z extruder offset (X offset is used as home pos with Dual X)
          current_position[Y_AXIS] -= hotend_offset[Y_AXIS][active_extruder] - hotend_offset[Y_AXIS][tmp_extruder];
          current_position[Z_AXIS] -= hotend_offset[Z_AXIS][active_extruder] - hotend_offset[Z_AXIS][tmp_extruder];

          // Activate the new extruder ahead of calling set_axis_is_at_home!
          active_extruder = tmp_extruder;

          // This function resets the max/min values - the current position may be overwritten below.
          set_axis_is_at_home(X_AXIS);

          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) DEBUG_POS("New Extruder", current_position);
          #endif

          // Only when auto-parking are carriages safe to move
          if (dual_x_carriage_mode != DXC_AUTO_PARK_MODE) no_move = true;

          switch (dual_x_carriage_mode) {
            case DXC_FULL_CONTROL_MODE:
              // New current position is the position of the activated extruder
              current_position[X_AXIS] = inactive_extruder_x_pos;
              // Save the inactive extruder's position (from the old current_position)
              inactive_extruder_x_pos = destination[X_AXIS];
              break;
            case DXC_AUTO_PARK_MODE:
              // record raised toolhead position for use by unpark
              COPY(raised_parked_position, current_position);
              raised_parked_position[Z_AXIS] += TOOLCHANGE_UNPARK_ZLIFT;
              #if ENABLED(MAX_SOFTWARE_ENDSTOPS)
                NOMORE(raised_parked_position[Z_AXIS], soft_endstop_max[Z_AXIS]);
              #endif
              active_extruder_parked = true;
              delayed_move_time = 0;
              break;
            case DXC_DUPLICATION_MODE:
              // If the new extruder is the left one, set it "parked"
              // This triggers the second extruder to move into the duplication position
              active_extruder_parked = (active_extruder == 0);

              if (active_extruder_parked)
                current_position[X_AXIS] = inactive_extruder_x_pos;
              else
                current_position[X_AXIS] = destination[X_AXIS] + duplicate_extruder_x_offset;
              inactive_extruder_x_pos = destination[X_AXIS];
              extruder_duplication_enabled = false;
              #if ENABLED(DEBUG_LEVELING_FEATURE)
                if (DEBUGGING(LEVELING)) {
                  SERIAL_ECHOLNPAIR("Set inactive_extruder_x_pos=", inactive_extruder_x_pos);
                  SERIAL_ECHOLNPGM("Clear extruder_duplication_enabled");
                }
              #endif
              break;
          }

          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) {
              SERIAL_ECHOLNPAIR("Active extruder parked: ", active_extruder_parked ? "yes" : "no");
              DEBUG_POS("New extruder (parked)", current_position);
            }
          #endif

          // No extra case for HAS_ABL in DUAL_X_CARRIAGE. Does that mean they don't work together?

        #else // !DUAL_X_CARRIAGE

          #if ENABLED(PARKING_EXTRUDER) // Dual Parking extruder
            const float z_diff = hotend_offset[Z_AXIS][active_extruder] - hotend_offset[Z_AXIS][tmp_extruder];
            float z_raise = PARKING_EXTRUDER_SECURITY_RAISE;
            if (!no_move) {

              const float parkingposx[] = PARKING_EXTRUDER_PARKING_X,
                          midpos = (parkingposx[0] + parkingposx[1]) * 0.5 + hotend_offset[X_AXIS][active_extruder],
                          grabpos = parkingposx[tmp_extruder] + hotend_offset[X_AXIS][active_extruder]
                                    + (tmp_extruder == 0 ? -(PARKING_EXTRUDER_GRAB_DISTANCE) : PARKING_EXTRUDER_GRAB_DISTANCE);
              /**
               *  Steps:
               *    1. Raise Z-Axis to give enough clearance
               *    2. Move to park position of old extruder
               *    3. Disengage magnetic field, wait for delay
               *    4. Move near new extruder
               *    5. Engage magnetic field for new extruder
               *    6. Move to parking incl. offset of new extruder
               *    7. Lower Z-Axis
               */

              // STEP 1
              #if ENABLED(DEBUG_LEVELING_FEATURE)
                SERIAL_ECHOLNPGM("Starting Autopark");
                if (DEBUGGING(LEVELING)) DEBUG_POS("current position:", current_position);
              #endif
              current_position[Z_AXIS] += z_raise;
              #if ENABLED(DEBUG_LEVELING_FEATURE)
                SERIAL_ECHOLNPGM("(1) Raise Z-Axis ");
                if (DEBUGGING(LEVELING)) DEBUG_POS("Moving to Raised Z-Position", current_position);
              #endif
              planner.buffer_line_kinematic(current_position, planner.max_feedrate_mm_s[Z_AXIS], active_extruder);
              stepper.synchronize();

              // STEP 2
              current_position[X_AXIS] = parkingposx[active_extruder] + hotend_offset[X_AXIS][active_extruder];
              #if ENABLED(DEBUG_LEVELING_FEATURE)
                SERIAL_ECHOLNPAIR("(2) Park extruder ", active_extruder);
                if (DEBUGGING(LEVELING)) DEBUG_POS("Moving ParkPos", current_position);
              #endif
              planner.buffer_line_kinematic(current_position, planner.max_feedrate_mm_s[X_AXIS], active_extruder);
              stepper.synchronize();

              // STEP 3
              #if ENABLED(DEBUG_LEVELING_FEATURE)
                SERIAL_ECHOLNPGM("(3) Disengage magnet ");
              #endif
              pe_deactivate_magnet(active_extruder);

              // STEP 4
              #if ENABLED(DEBUG_LEVELING_FEATURE)
                SERIAL_ECHOLNPGM("(4) Move to position near new extruder");
              #endif
              current_position[X_AXIS] += (active_extruder == 0 ? 10 : -10); // move 10mm away from parked extruder

              #if ENABLED(DEBUG_LEVELING_FEATURE)
                if (DEBUGGING(LEVELING)) DEBUG_POS("Moving away from parked extruder", current_position);
              #endif
              planner.buffer_line_kinematic(current_position, planner.max_feedrate_mm_s[X_AXIS], active_extruder);
              stepper.synchronize();

              // STEP 5
              #if ENABLED(DEBUG_LEVELING_FEATURE)
                SERIAL_ECHOLNPGM("(5) Engage magnetic field");
              #endif

              #if ENABLED(PARKING_EXTRUDER_SOLENOIDS_INVERT)
                pe_activate_magnet(active_extruder); //just save power for inverted magnets
              #endif
              pe_activate_magnet(tmp_extruder);

              // STEP 6
              current_position[X_AXIS] = grabpos + (tmp_extruder == 0 ? (+10) : (-10));
              planner.buffer_line_kinematic(current_position, planner.max_feedrate_mm_s[X_AXIS], active_extruder);
              current_position[X_AXIS] = grabpos;
              #if ENABLED(DEBUG_LEVELING_FEATURE)
                SERIAL_ECHOLNPAIR("(6) Unpark extruder ", tmp_extruder);
                if (DEBUGGING(LEVELING)) DEBUG_POS("Move UnparkPos", current_position);
              #endif
              planner.buffer_line_kinematic(current_position, planner.max_feedrate_mm_s[X_AXIS]/2, active_extruder);
              stepper.synchronize();

              // Step 7
              current_position[X_AXIS] = midpos - hotend_offset[X_AXIS][tmp_extruder];
              #if ENABLED(DEBUG_LEVELING_FEATURE)
                SERIAL_ECHOLNPGM("(7) Move midway between hotends");
                if (DEBUGGING(LEVELING)) DEBUG_POS("Move midway to new extruder", current_position);
              #endif
              planner.buffer_line_kinematic(current_position, planner.max_feedrate_mm_s[X_AXIS], active_extruder);
              stepper.synchronize();
              #if ENABLED(DEBUG_LEVELING_FEATURE)
                SERIAL_ECHOLNPGM("Autopark done.");
              #endif
            }
            else { // nomove == true
              // Only engage magnetic field for new extruder
              pe_activate_magnet(tmp_extruder);
              #if ENABLED(PARKING_EXTRUDER_SOLENOIDS_INVERT)
                pe_activate_magnet(active_extruder); // Just save power for inverted magnets
              #endif
            }
            current_position[Z_AXIS] -= hotend_offset[Z_AXIS][tmp_extruder] - hotend_offset[Z_AXIS][active_extruder]; // Apply Zoffset

            #if ENABLED(DEBUG_LEVELING_FEATURE)
              if (DEBUGGING(LEVELING)) DEBUG_POS("Applying Z-offset", current_position);
            #endif

          #endif // dualParking extruder

          #if ENABLED(SWITCHING_NOZZLE)
            #define DONT_SWITCH (SWITCHING_EXTRUDER_SERVO_NR == SWITCHING_NOZZLE_SERVO_NR)
            // <0 if the new nozzle is higher, >0 if lower. A bigger raise when lower.
            const float z_diff = hotend_offset[Z_AXIS][active_extruder] - hotend_offset[Z_AXIS][tmp_extruder],
                        z_raise = 0.3 + (z_diff > 0.0 ? z_diff : 0.0);

            // Always raise by some amount (destination copied from current_position earlier)
            current_position[Z_AXIS] += z_raise;
            planner.buffer_line_kinematic(current_position, planner.max_feedrate_mm_s[Z_AXIS], active_extruder);
            move_nozzle_servo(tmp_extruder);
          #endif

          /**
           * Set current_position to the position of the new nozzle.
           * Offsets are based on linear distance, so we need to get
           * the resulting position in coordinate space.
           *
           * - With grid or 3-point leveling, offset XYZ by a tilted vector
           * - With mesh leveling, update Z for the new position
           * - Otherwise, just use the raw linear distance
           *
           * Software endstops are altered here too. Consider a case where:
           *   E0 at X=0 ... E1 at X=10
           * When we switch to E1 now X=10, but E1 can't move left.
           * To express this we apply the change in XY to the software endstops.
           * E1 can move farther right than E0, so the right limit is extended.
           *
           * Note that we don't adjust the Z software endstops. Why not?
           * Consider a case where Z=0 (here) and switching to E1 makes Z=1
           * because the bed is 1mm lower at the new position. As long as
           * the first nozzle is out of the way, the carriage should be
           * allowed to move 1mm lower. This technically "breaks" the
           * Z software endstop. But this is technically correct (and
           * there is no viable alternative).
           */
          #if ABL_PLANAR
            // Offset extruder, make sure to apply the bed level rotation matrix
            vector_3 tmp_offset_vec = vector_3(hotend_offset[X_AXIS][tmp_extruder],
                                               hotend_offset[Y_AXIS][tmp_extruder],
                                               0),
                     act_offset_vec = vector_3(hotend_offset[X_AXIS][active_extruder],
                                               hotend_offset[Y_AXIS][active_extruder],
                                               0),
                     offset_vec = tmp_offset_vec - act_offset_vec;

            #if ENABLED(DEBUG_LEVELING_FEATURE)
              if (DEBUGGING(LEVELING)) {
                tmp_offset_vec.debug(PSTR("tmp_offset_vec"));
                act_offset_vec.debug(PSTR("act_offset_vec"));
                offset_vec.debug(PSTR("offset_vec (BEFORE)"));
              }
            #endif

            offset_vec.apply_rotation(planner.bed_level_matrix.transpose(planner.bed_level_matrix));

            #if ENABLED(DEBUG_LEVELING_FEATURE)
              if (DEBUGGING(LEVELING)) offset_vec.debug(PSTR("offset_vec (AFTER)"));
            #endif

            // Adjustments to the current position
            const float xydiff[2] = { offset_vec.x, offset_vec.y };
            current_position[Z_AXIS] += offset_vec.z;

          #else // !ABL_PLANAR

            const float xydiff[2] = {
              hotend_offset[X_AXIS][tmp_extruder] - hotend_offset[X_AXIS][active_extruder],
              hotend_offset[Y_AXIS][tmp_extruder] - hotend_offset[Y_AXIS][active_extruder]
            };

            #if ENABLED(MESH_BED_LEVELING)

              if (planner.leveling_active) {
                #if ENABLED(DEBUG_LEVELING_FEATURE)
                  if (DEBUGGING(LEVELING)) SERIAL_ECHOPAIR("Z before MBL: ", current_position[Z_AXIS]);
                #endif
                float x2 = current_position[X_AXIS] + xydiff[X_AXIS],
                      y2 = current_position[Y_AXIS] + xydiff[Y_AXIS],
                      z1 = current_position[Z_AXIS], z2 = z1;
                planner.apply_leveling(current_position[X_AXIS], current_position[Y_AXIS], z1);
                planner.apply_leveling(x2, y2, z2);
                current_position[Z_AXIS] += z2 - z1;
                #if ENABLED(DEBUG_LEVELING_FEATURE)
                  if (DEBUGGING(LEVELING))
                    SERIAL_ECHOLNPAIR(" after: ", current_position[Z_AXIS]);
                #endif
              }

            #endif // MESH_BED_LEVELING

          #endif // !HAS_ABL

          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) {
              SERIAL_ECHOPAIR("Offset Tool XY by { ", xydiff[X_AXIS]);
              SERIAL_ECHOPAIR(", ", xydiff[Y_AXIS]);
              SERIAL_ECHOLNPGM(" }");
            }
          #endif

          // The newly-selected extruder XY is actually at...
          current_position[X_AXIS] += xydiff[X_AXIS];
          current_position[Y_AXIS] += xydiff[Y_AXIS];

          // Set the new active extruder
          active_extruder = tmp_extruder;

        #endif // !DUAL_X_CARRIAGE

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("Sync After Toolchange", current_position);
        #endif

        // Tell the planner the new "current position"
        SYNC_PLAN_POSITION_KINEMATIC();

        // Move to the "old position" (move the extruder into place)
        #if ENABLED(SWITCHING_NOZZLE)
          destination[Z_AXIS] += z_diff;  // Include the Z restore with the "move back"
        #endif
        if (!no_move && IsRunning()) {
          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) DEBUG_POS("Move back", destination);
          #endif
          // Move back to the original (or tweaked) position
          do_blocking_move_to(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS]);
        }
        #if ENABLED(SWITCHING_NOZZLE)
          else {
            // Move back down. (Including when the new tool is higher.)
            do_blocking_move_to_z(destination[Z_AXIS], planner.max_feedrate_mm_s[Z_AXIS]);
          }
        #endif
      } // (tmp_extruder != active_extruder)

      stepper.synchronize();

      #if ENABLED(EXT_SOLENOID) && !ENABLED(PARKING_EXTRUDER)
        disable_all_solenoids();
        enable_solenoid_on_active_extruder();
      #endif // EXT_SOLENOID

      feedrate_mm_s = old_feedrate_mm_s;

    #else // HOTENDS <= 1

      UNUSED(fr_mm_s);
      UNUSED(no_move);

      #if ENABLED(MK2_MULTIPLEXER)
        if (tmp_extruder >= E_STEPPERS)
          return invalid_extruder_error(tmp_extruder);

        select_multiplexed_stepper(tmp_extruder);
      #endif

      // Set the new active extruder
      active_extruder = tmp_extruder;

    #endif // HOTENDS <= 1

    #if ENABLED(SWITCHING_EXTRUDER) && !DONT_SWITCH
      stepper.synchronize();
      move_extruder_servo(active_extruder);
    #endif

    #if HAS_FANMUX
      fanmux_switch(active_extruder);
    #endif

    SERIAL_ECHO_START();
    SERIAL_ECHOLNPAIR(MSG_ACTIVE_EXTRUDER, (int)active_extruder);

  #endif // !MIXING_EXTRUDER || MIXING_VIRTUAL_TOOLS <= 1
}

/**
 * T0-T3: Switch tool, usually switching extruders
 *
 *   F[units/min] Set the movement feedrate
 *   S1           Don't move the tool in XY after change
 */
inline void gcode_T(const uint8_t tmp_extruder) {



  #if HOTENDS == 1 || (ENABLED(MIXING_EXTRUDER) && MIXING_VIRTUAL_TOOLS > 1)

    tool_change(tmp_extruder);

  #elif HOTENDS > 1

    tool_change(
      tmp_extruder,
      MMM_TO_MMS(parser.linearval('F')),
      (tmp_extruder == active_extruder) || parser.boolval('S')
    );

  #endif

}

/**
 * Process the parsed command and dispatch it to its handler
 */
void process_parsed_command() {
  KEEPALIVE_STATE(IN_HANDLER);

  #if ENABLED(RESIN)

    int temp_laser_state = digitalRead(LASER_ENABLE_PIN);

    while (READ(CASE_OPEN_PIN) || READ(CASE_OPEN2_PIN)) {
      KEEPALIVE_STATE(DOOR_OPEN);
      digitalWrite(LASER_ENABLE_PIN, LOW);
      idle();
    }
    digitalWrite(LASER_ENABLE_PIN, temp_laser_state);
    KEEPALIVE_STATE(IN_HANDLER);
    
  #endif

  // Handle a known G, M, or T
  switch (parser.command_letter) {
    case 'G': switch (parser.codenum) {

      // G0, G1
      case 0:
      case 1:

          gcode_G0_G1();

        break;



      // G4 Dwell
      case 4:
        gcode_G4();
        break;

      case 28: // G28: Home all axes, one at a time
        gcode_G28(false);
        break;


      case 90: // G90
        relative_mode = false;
        break;
      case 91: // G91
        relative_mode = true;
        break;

      case 92: // G92
        gcode_G92();
        break;

    }
    break;

    case 'M': switch (parser.codenum) {

      case 17: // M17: Enable all stepper motors
        gcode_M17();
        break;


      case 31: // M31: Report time since the start of SD print or last M109
        gcode_M31(); break;

      case 42: // M42: Change pin state
        gcode_M42(); break;


      case 75: // M75: Start print timer
        gcode_M75(); break;
      case 76: // M76: Pause print timer
        gcode_M76(); break;
      case 77: // M77: Stop print timer
        gcode_M77(); break;

      case 104: // M104: Set hot end temperature
        gcode_M104();
        break;

      case 110: // M110: Set Current Line Number
        gcode_M110();
        break;

      case 111: // M111: Set debug level
        gcode_M111();
        break;

      #if DISABLED(EMERGENCY_PARSER)

        case 108: // M108: Cancel Waiting
          gcode_M108();
          break;

        case 112: // M112: Emergency Stop
          gcode_M112();
          break;

        case 410: // M410 quickstop - Abort all the planned moves.
          gcode_M410();
          break;

      #endif

      case 140: // M140: Set bed temperature
        gcode_M140();
        break;

      case 105: // M105: Report current temperature
        gcode_M105();
        KEEPALIVE_STATE(NOT_BUSY);
        return; // "ok" already printed


      case 109: // M109: Wait for hotend temperature to reach target
        gcode_M109();
        break;


      case 81: // M81: Turn off Power, including Power Supply, if possible
        gcode_M81();
        break;

      case 82: // M82: Set E axis normal mode (same as other axes)
        gcode_M82();
        break;
      case 83: // M83: Set E axis relative mode
        gcode_M83();
        break;
      case 18: // M18 => M84
      case 84: // M84: Disable all steppers or set timeout
        gcode_M18_M84();
        break;
      case 85: // M85: Set inactivity stepper shutdown timeout
        gcode_M85();
        break;
      case 92: // M92: Set the steps-per-unit for one or more axes
        gcode_M92();
        break;
      case 114: // M114: Report current position
        gcode_M114();
        break;
      case 115: // M115: Report capabilities
        gcode_M115();
        break;
      case 117: // M117: Set LCD message text, if possible
        gcode_M117();
        break;
      case 118: // M118: Display a message in the host console
        gcode_M118();
        break;
      case 119: // M119: Report endstop states
        gcode_M119();
        break;
      case 120: // M120: Enable endstops
        gcode_M120();
        break;
      case 121: // M121: Disable endstops
        gcode_M121();
        break;


      #if DISABLED(NO_VOLUMETRICS)
        case 200: // M200: Set filament diameter, E to cubic units
          gcode_M200();
          break;
      #endif

      case 201: // M201: Set max acceleration for print moves (units/s^2)
        gcode_M201();
        break;

      case 203: // M203: Set max feedrate (units/sec)
        gcode_M203();
        break;
      case 204: // M204: Set acceleration
        gcode_M204();
        break;
      case 205: // M205: Set advanced settings
        gcode_M205();
        break;


      case 211: // M211: Enable, Disable, and/or Report software endstops
        gcode_M211();
        break;



      case 220: // M220: Set Feedrate Percentage: S<percent> ("FR" on your LCD)
        gcode_M220();
        break;

      case 221: // M221: Set Flow Percentage
        gcode_M221();
        break;

      case 226: // M226: Wait until a pin reaches a state
        gcode_M226();
        break;



      case 303: // M303: PID autotune
        gcode_M303();
        break;

      case 400: // M400: Finish all moves
        gcode_M400();
        break;

      case 500: // M500: Store settings in EEPROM
        gcode_M500();
        break;
      case 501: // M501: Read settings from EEPROM
        gcode_M501();
        break;
      case 502: // M502: Revert to default settings
        gcode_M502();
        break;


      #if ENABLED(RESIN)
        case 898:
          gcode_M898(); // M898: turn laser off
          break;  
        case 899:
          gcode_M899(); // M899: turn laser on
          break;
      #endif  

      #if ENABLED(LIN_ADVANCE)
        case 900: // M900: Set advance K factor.
          gcode_M900();
          break;
      #endif

      case 907: // M907: Set digital trimpot motor current using axis codes.
        gcode_M907();
        break;



      case 355: // M355 set case light brightness
        gcode_M355();
        break;

      case 999: // M999: Restart after being Stopped
        gcode_M999();
        break;
    }
    break;

    case 'T':
      gcode_T(parser.codenum);
      break;

    default: parser.unknown_command_error();
  }

  KEEPALIVE_STATE(NOT_BUSY);

  ok_to_send();
}

void process_next_command() {
  char * const current_command = command_queue[cmd_queue_index_r];

  if (DEBUGGING(ECHO)) {
    SERIAL_ECHO_START();
    SERIAL_ECHOLN(current_command);

  }

  // Parse the next command in the queue
  parser.parse(current_command);
  process_parsed_command();
}

/**
 * Send a "Resend: nnn" message to the host to
 * indicate that a command needs to be re-sent.
 */
void FlushSerialRequestResend() {
  //char command_queue[cmd_queue_index_r][100]="Resend:";
  MYSERIAL.flush();
  SERIAL_PROTOCOLPGM(MSG_RESEND);
  SERIAL_PROTOCOLLN(gcode_LastN + 1);
  ok_to_send();
}

/**
 * Send an "ok" message to the host, indicating
 * that a command was successfully processed.
 *
 * If ADVANCED_OK is enabled also include:
 *   N<int>  Line number of the command, if any
 *   P<int>  Planner space remaining
 *   B<int>  Block queue space remaining
 */
void ok_to_send() {
  refresh_cmd_timeout();
  if (!send_ok[cmd_queue_index_r]) return;
  SERIAL_PROTOCOLPGM(MSG_OK);
 
  SERIAL_EOL();
}



/**
 * Get the stepper positions in the cartes[] array.
 * Forward kinematics are applied for DELTA and SCARA.
 *
 * The result is in the current coordinate space with
 * leveling applied. The coordinates need to be run through
 * unapply_leveling to obtain machine coordinates suitable
 * for current_position, etc.
 */
void get_cartesian_from_steppers() {


      cartes[X_AXIS] = stepper.get_axis_position_mm(X_AXIS);
      cartes[Y_AXIS] = stepper.get_axis_position_mm(Y_AXIS);

    cartes[Z_AXIS] = stepper.get_axis_position_mm(Z_AXIS);

}

/**
 * Set the current_position for an axis based on
 * the stepper positions, removing any leveling that
 * may have been applied.
 *
 * To prevent small shifts in axis position always call
 * SYNC_PLAN_POSITION_KINEMATIC after updating axes with this.
 *
 * To keep hosts in sync, always call report_current_position
 * after updating the current_position.
 */
void set_current_from_steppers_for_axis(const AxisEnum axis) {
  get_cartesian_from_steppers();
 
  if (axis == ALL_AXES)
    COPY(current_position, cartes);
  else
    current_position[axis] = cartes[axis];
}




#if ENABLED(RESIN)

  bool prepare_resin_move_to(const float (&rtarget)[XYZE]) {
    planner.laser_status = false;

    // Get the top feedrate of the move in the XY plane
    const float _feedrate_mm_s = MMS_SCALED(feedrate_mm_s);

    const float xdiff = rtarget[X_AXIS] - current_position[X_AXIS],
                ydiff = rtarget[Y_AXIS] - current_position[Y_AXIS];

    // If the move is only in Z/E don't split up the move
    //if (!xdiff && !ydiff) {
      //planner.buffer_line_kinematic(rtarget, _feedrate_mm_s, active_extruder);
      //return false; // caller will update current_position
    //}
    
    // Fail if attempting move outside printable radius
    if (!position_is_reachable(rtarget[X_AXIS], rtarget[Y_AXIS])) return true;

    // Remaining cartesian distances
    const float zdiff = rtarget[Z_AXIS] - current_position[Z_AXIS],
                ediff = rtarget[E_AXIS] - current_position[E_AXIS];
    
    if (current_position[E_AXIS] < destination[E_AXIS])
      planner.laser_status = true;
    
    // Get the linear distance in XYZ
    // If the move is very short, check the E move distance
    // No E move either? Game over.
    float cartesian_mm = SQRT(sq(xdiff) + sq(ydiff) + sq(zdiff));
    //if (UNEAR_ZERO(cartesian_mm)) cartesian_mm = FABS(ediff);
    //if (UNEAR_ZERO(cartesian_mm)) return true;

    // Minimum number of seconds to move the given distance
    const float seconds = cartesian_mm / _feedrate_mm_s;

    // The number of segments-per-second times the duration
    // gives the number of segments
    uint16_t segments = resin_segments_per_second * seconds;

    // Minimum segment size is 0.25mm
    NOMORE(segments, cartesian_mm * 4);
 
    // At least one segment is required
    NOLESS(segments, 1);

    // The approximate length of each segment
    const float inv_segments = 1.0 / float(segments),
                segment_distance[XYZE] = {
                  xdiff * inv_segments,
                  ydiff * inv_segments,
                  zdiff * inv_segments,
                  ediff * inv_segments
                };

    //SERIAL_ECHOPAIR("mm=", cartesian_mm);
    //SERIAL_ECHOPAIR(" seconds=", seconds);
    //SERIAL_ECHOPAIR(" laser=", planner.laser_status);
    //SERIAL_ECHOLNPAIR(" segments=", segments);

    // Get the current position as starting point
    float raw[XYZE];
    COPY(raw, current_position);

    // Calculate and execute the segments
    
    while (--segments) {
      LOOP_XYZE(i) raw[i] += segment_distance[i];

      calculate_resin(raw);
      
      planner.dac_X = uint16_t(LROUND(resin[X_AXIS] * planner.axis_steps_per_mm[X_AXIS])) + 0x8000;
      planner.dac_Y = uint16_t(LROUND(resin[Y_AXIS] * planner.axis_steps_per_mm[Y_AXIS])) + 0x8000;
      planner.buffer_segment(resin[X_AXIS], resin[Y_AXIS], resin[Z_AXIS], 0.0, _feedrate_mm_s, active_extruder);
      //planner.buffer_segment(0, 0, resin[Z_AXIS], 0.0, _feedrate_mm_s, active_extruder);
    }

    // Ensure last segment arrives at target location.
    calculate_resin(rtarget);
    planner.dac_X = uint16_t(LROUND(resin[X_AXIS] * planner.axis_steps_per_mm[X_AXIS])) + 0x8000;
    planner.dac_Y = uint16_t(LROUND(resin[Y_AXIS] * planner.axis_steps_per_mm[Y_AXIS])) + 0x8000;
    planner.buffer_segment(resin[X_AXIS], resin[Y_AXIS], resin[Z_AXIS], 0.0, _feedrate_mm_s, active_extruder);
    //planner.buffer_segment(0, 0, resin[Z_AXIS], 0.0, _feedrate_mm_s, active_extruder);
    planner.laser_status = false;
    //planner.buffer_segment(resin[X_AXIS], resin[Y_AXIS], resin[Z_AXIS], 0.0, _feedrate_mm_s, active_extruder);


    return false; // caller will update current_position

  }

#endif

/**
 * Resin forward kinematics math
 **/

#if ENABLED(RESIN)
  
  //upper left, upper right, lower left, lower right mm offset
  const float cal_mapx[4] = {0,0,0,0};//{-5.0, 5.0, 0.0, 5.0};
  const float cal_mapy[4] = {0,0,0,0};//{-7.0, -10.0, -10.0, -5.0};

  const float pi_div_4 = 0.7853981634;

  const float z0_squared = z0*z0;
  const float inv_z0 = 1.0/z0;
  //float abs_x = 0;

  
  inline float fast_atan(float x) {
    //return atan(x);
    float abs_x = fabs(x);
    return pi_div_4*x - x*(abs_x-1)*(0.2447 + 0.0663*abs_x);
  }

  /*
  inline float fast_atan(float x) {
    return pi_div_4*x - 0.273*x*(1-fabs(x));
  }
  */
 
  inline float fast_sqrt(float x) {
    //return SQRT(x);
    float xhalf = 0.5*x;
    union {
        float x;
        long i;
    } u;
    
    u.x = x;
    u.i = 0x5F3759DF - (u.i>>1);
    return x*u.x*(1.5-xhalf*sq(u.x));
  }


  void calculate_resin(const float logical[XYZ]){
    

    
    digitalWrite(35, HIGH);
    resin[X_AXIS] = atan((logical[X_AXIS])/(r+SQRT(logical[Y_AXIS]*logical[Y_AXIS] + z0_squared)))*RESIN_RAD_TO_MM;
    resin[Y_AXIS] = atan((logical[Y_AXIS])*inv_z0)*RESIN_RAD_TO_MM;    
    resin[Z_AXIS] = logical[Z_AXIS];
    digitalWrite(35, LOW);



  }

#endif


/**
 * Prepare a single move and get ready for the next one
 *
 * This may result in several calls to planner.buffer_line to
 * do smaller moves for DELTA, SCARA, mesh moves, etc.
 *
 * Make sure current_position[E] and destination[E] are good
 * before calling or cold/lengthy extrusion may get missed.
 */
void prepare_move_to_destination() {
  clamp_to_software_endstops(destination);
  refresh_cmd_timeout();


  if (
    #if UBL_SEGMENTED
      ubl.prepare_segmented_line_to(destination, MMS_SCALED(feedrate_mm_s))
    #elif IS_KINEMATIC
      prepare_kinematic_move_to(destination)
    #elif ENABLED(RESIN)
      prepare_resin_move_to(destination)
    #else
      prepare_move_to_destination_cartesian()
    #endif
  ) return;

  set_current_from_destination();
}



void enable_all_steppers() {
  enable_X();
  enable_Y();
  enable_Z();
  enable_E0();
  enable_E1();
  enable_E2();
  enable_E3();
  enable_E4();
}

void disable_e_steppers() {
  disable_E0();
  disable_E1();
  disable_E2();
  disable_E3();
  disable_E4();
}

void disable_all_steppers() {
  disable_X();
  disable_Y();
  disable_Z();
  disable_e_steppers();
}


/**
 * Manage several activities:
 *  - Check for Filament Runout
 *  - Keep the command buffer full
 *  - Check for maximum inactive time between commands
 *  - Check for maximum inactive time between stepper commands
 *  - Check if pin CHDK needs to go LOW
 *  - Check for KILL button held down
 *  - Check for HOME button held down
 *  - Check if cooling fan needs to be switched on
 *  - Check if an idle but hot extruder needs filament extruded (EXTRUDER_RUNOUT_PREVENT)
 */
void manage_inactivity(bool ignore_stepper_queue/*=false*/) {



  if (commands_in_queue < BUFSIZE) get_available_commands();

  const millis_t ms = millis();

  if (max_inactive_time && ELAPSED(ms, previous_cmd_ms + max_inactive_time)) {
    SERIAL_ERROR_START();
    SERIAL_ECHOLNPAIR(MSG_KILL_INACTIVE_TIME, parser.command_ptr);
    kill(PSTR(MSG_KILLED));
  }

  // Prevent steppers timing-out in the middle of M600
  #if ENABLED(ADVANCED_PAUSE_FEATURE) && ENABLED(PAUSE_PARK_NO_STEPPER_TIMEOUT)
    #define MOVE_AWAY_TEST !move_away_flag
  #else
    #define MOVE_AWAY_TEST true
  #endif

  if (MOVE_AWAY_TEST && stepper_inactive_time && ELAPSED(ms, previous_cmd_ms + stepper_inactive_time)
      && !ignore_stepper_queue && !planner.blocks_queued()) {
    #if ENABLED(DISABLE_INACTIVE_X)
      disable_X();
    #endif
    #if ENABLED(DISABLE_INACTIVE_Y)
      disable_Y();
    #endif
    #if ENABLED(DISABLE_INACTIVE_Z)
      disable_Z();
    #endif
    #if ENABLED(DISABLE_INACTIVE_E)
      disable_e_steppers();
    #endif
    #if ENABLED(AUTO_BED_LEVELING_UBL) && ENABLED(ULTIPANEL)  // Only needed with an LCD
      ubl.lcd_map_control = defer_return_to_status = false;
    #endif
  }

  #ifdef CHDK // Check if pin should be set to LOW after M240 set it to HIGH
    if (chdkActive && ELAPSED(ms, chdkHigh + CHDK_DELAY)) {
      chdkActive = false;
      WRITE(CHDK, LOW);
    }
  #endif




  planner.check_axes_activity();
}

/**
 * Standard idle routine keeps the machine alive
 */
void idle(

) {
 

  lcd_update();

  host_keepalive();


  manage_inactivity(
 
  );

  thermalManager.manage_heater();

}

/**
 * Kill all activity and lock the machine.
 * After this the machine will need to be reset.
 */
void kill(const char* lcd_msg) {
  SERIAL_ERROR_START();
  SERIAL_ERRORLNPGM(MSG_ERR_KILLED);

  thermalManager.disable_all_heaters();
  disable_all_steppers();

  #if ENABLED(ULTRA_LCD)
    kill_screen(lcd_msg);
  #else
    UNUSED(lcd_msg);
  #endif

  _delay_ms(600); // Wait a short time (allows messages to get out before shutting down.
  cli(); // Stop interrupts

  _delay_ms(250); //Wait to ensure all interrupts routines stopped
  thermalManager.disable_all_heaters(); //turn off heaters again

  #ifdef ACTION_ON_KILL
    SERIAL_ECHOLNPGM("//action:" ACTION_ON_KILL);
  #endif

  #if HAS_POWER_SWITCH
    SET_INPUT(PS_ON_PIN);
  #endif

  suicide();
  while (1) {
    #if ENABLED(USE_WATCHDOG)
      watchdog_reset();
    #endif
  } // Wait for reset
}

/**
 * Turn off heaters and stop the print in progress
 * After a stop the machine may be resumed with M999
 */
void stop() {
  thermalManager.disable_all_heaters(); // 'unpause' taken care of in here



  if (IsRunning()) {
    Stopped_gcode_LastN = gcode_LastN; // Save last g_code for restart
    SERIAL_ERROR_START();
    SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
    LCD_MESSAGEPGM(MSG_STOPPED);
    safe_delay(350);       // allow enough time for messages to get out before stopping
    Running = false;
  }
}

/**
 * Marlin entry-point: Set up before the program loop
 *  - Set up the kill pin, filament runout, power hold
 *  - Start the serial port
 *  - Print startup messages and diagnostics
 *  - Get EEPROM or default settings
 *  - Initialize managers for:
 *    • temperature
 *    • planner
 *    • watchdog
 *    • stepper
 *    • photo pin
 *    • servos
 *    • LCD controller
 *    • Digipot I2C
 *    • Z probe sled
 *    • status LEDs
 */
void setup() {

  setup_killpin();

  setup_powerhold();



  MYSERIAL.begin(BAUDRATE);
  SERIAL_PROTOCOLLNPGM("start");
  SERIAL_ECHO_START();


  // Check startup - does nothing if bootloader sets MCUSR to 0
  byte mcu = MCUSR;
  if (mcu &  1) SERIAL_ECHOLNPGM(MSG_POWERUP);
  if (mcu &  2) SERIAL_ECHOLNPGM(MSG_EXTERNAL_RESET);
  if (mcu &  4) SERIAL_ECHOLNPGM(MSG_BROWNOUT_RESET);
  if (mcu &  8) SERIAL_ECHOLNPGM(MSG_WATCHDOG_RESET);
  if (mcu & 32) SERIAL_ECHOLNPGM(MSG_SOFTWARE_RESET);
  MCUSR = 0;

  SERIAL_ECHOPGM(MSG_MARLIN);
  SERIAL_CHAR(' ');
  SERIAL_ECHOLNPGM(SHORT_BUILD_VERSION);
  SERIAL_EOL();


  SERIAL_ECHO_START();
  SERIAL_ECHOPAIR(MSG_FREE_MEMORY, freeMemory());
  SERIAL_ECHOLNPAIR(MSG_PLANNER_BUFFER_BYTES, (int)sizeof(block_t)*BLOCK_BUFFER_SIZE);

  // Send "ok" after commands by default
  for (int8_t i = 0; i < BUFSIZE; i++) send_ok[i] = true;

  // Load data from EEPROM if available (or use defaults)
  // This also updates variables in the planner, elsewhere
  (void)settings.load();

  #if HAS_M206_COMMAND
    // Initialize current position based on home_offset
    COPY(current_position, home_offset);
  #else
    ZERO(current_position);
  #endif

  // Vital to init stepper/planner equivalent for current_position
  SYNC_PLAN_POSITION_KINEMATIC();

  thermalManager.init();    // Initialize temperature loop


  stepper.init();    // Initialize stepper, this enables interrupts!
  servo_init();



  lcd_init();


#if ENABLED(RESIN)
  //void Stepper::resin_init() {

    pinMode(LASER_FIRING_PIN, OUTPUT);
    digitalWrite(LASER_FIRING_PIN, LOW); 
    
    pinMode(LASER_ENABLE_PIN, OUTPUT);
    digitalWrite(LASER_ENABLE_PIN, LOW);
/*
    int myEraser = 7;             // this is 111 in binary and is used as an eraser
    TCCR3B &= ~myEraser;
    int myPrescaler = 2;         // this could be a number in [1 , 6]. In this case, 3 corresponds in binary to 011.   
    TCCR3B |= myPrescaler;
*/
    
    pinMode(CASE_OPEN_PIN, INPUT);
    //digitalWrite(CASE_OPEN_PIN, HIGH);
    
    pinMode(CASE_OPEN2_PIN, INPUT);
    //digitalWrite(CASE_OPEN2_PIN, HIGH);
    
    pinMode(GALVO_SS_PIN, OUTPUT);
    WRITE(GALVO_SS_PIN, HIGH);
    
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV2); // Run at 8 MHz 
    // start the SPI library:
    SPI.begin();
  //}
#endif
}

/**
 * The main Marlin program loop
 *
 *  - Save or log commands to SD
 *  - Process available commands (if not saving)
 *  - Call heater manager
 *  - Call inactivity manager
 *  - Call endstop manager
 *  - Call LCD update
 */
void loop() {

  if (commands_in_queue < BUFSIZE) get_available_commands();

  if (commands_in_queue) {



    process_next_command();



    // The queue may be reset by a command handler or by code invoked by idle() within a handler
    if (commands_in_queue) {
      --commands_in_queue;
      if (++cmd_queue_index_r >= BUFSIZE) cmd_queue_index_r = 0;
    }
  }
  endstops.report_state();
  idle();
}


#endif