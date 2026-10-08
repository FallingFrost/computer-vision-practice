#include "Controller.h"

const unsigned long Controller_c::UPDATE_INTERVAL_MS;
const unsigned long Controller_c::TELEMETRY_INTERVAL_MS;
const unsigned long Controller_c::RECOVERY_TIMEOUT_MS;
const uint16_t Controller_c::LINE_THRESHOLD;
constexpr float Controller_c::BASE_PWM;
constexpr float Controller_c::LINE_FOLLOW_GAIN;
constexpr float Controller_c::RECOVERY_LEFT_PWM;
constexpr float Controller_c::RECOVERY_RIGHT_PWM;
const int Controller_c::TEST_DURATION_CALLS;
const int Controller_c::RAMP_STEP_INTERVAL;
constexpr float Controller_c::RAMP_PWM_STEP;
constexpr float Controller_c::MAX_RAMP_PWM;
constexpr float Controller_c::ENCODER_COUNTS_PER_REV;
constexpr float Controller_c::WHEEL_DIAMETER_MM;
const int Controller_c::SPEED_SAMPLE_INTERVAL;
const int Controller_c::SAMPLES_PER_TRIAL;
constexpr float Controller_c::TEST_PWM;
const int Controller_c::DEADBAND_WHEEL;
const int Controller_c::DEADBAND_DIRECTION;
constexpr float Controller_c::DEADBAND_PWM_MIN;
constexpr float Controller_c::DEADBAND_PWM_MAX;
constexpr float Controller_c::DEADBAND_PWM_STEP;
const int Controller_c::DEADBAND_REPEATS;
const int Controller_c::DEADBAND_SAMPLE_CALLS;
const int Controller_c::DEADBAND_SAMPLES_PER_TRIAL;
const int Controller_c::DEADBAND_REST_MIN_CALLS;
const int Controller_c::DEADBAND_REST_QUIET_CALLS;
const int Controller_c::DEADBAND_REST_MAX_CALLS;
const int Controller_c::SATURATION_WHEEL;
const int Controller_c::SATURATION_DIRECTION;
const bool Controller_c::SATURATION_OPPOSITE_PAIR;
constexpr float Controller_c::SATURATION_PWM_MIN;
constexpr float Controller_c::SATURATION_PWM_MAX;
constexpr float Controller_c::SATURATION_PWM_STEP;
constexpr float Controller_c::SATURATION_PWM_LIMIT;
const int Controller_c::SATURATION_REPEATS;
const int Controller_c::SATURATION_COOL_CALLS;
const int Controller_c::SATURATION_SETTLE_CALLS;
const int Controller_c::SATURATION_SAMPLE_CALLS;
const int Controller_c::SATURATION_SAMPLES_PER_TRIAL;
const int Controller_c::GAIN_LEVEL_COUNT;
constexpr float Controller_c::GAIN_PWM_LEVEL_0;
constexpr float Controller_c::GAIN_PWM_LEVEL_1;
const int Controller_c::STEP_WHEEL;
constexpr float Controller_c::STEP_PWM_LOW;
constexpr float Controller_c::STEP_PWM_HIGH;
const unsigned long Controller_c::STEP_GAP_MS;
const unsigned long Controller_c::STEP_INITIAL_MS;
const unsigned long Controller_c::STEP_TRANSIENT_MS;
const unsigned long Controller_c::STEP_FINAL_MS;

namespace {
/** @brief One step-response trial: hold one command, then change it once. */
struct StepTrial_c {
  const char *id;    // reported as trial_id
  const char *split; // "fit" or "held back"
  float pwm_initial;
  float pwm_final;
};

// Rising steps are repeated, so the models are fitted to more than one trace
// and the repeat-to-repeat spread is visible. Falling steps are held back from
// every fit, so their prediction is a test rather than a restatement of the
// fit - and a falling response is a genuinely new transition, not the same one
// played again.
const StepTrial_c STEP_TRIALS[] = {
  {"rise_1", "fit",       Controller_c::STEP_PWM_LOW,  Controller_c::STEP_PWM_HIGH},
  {"rise_2", "fit",       Controller_c::STEP_PWM_LOW,  Controller_c::STEP_PWM_HIGH},
  {"rise_3", "fit",       Controller_c::STEP_PWM_LOW,  Controller_c::STEP_PWM_HIGH},
  {"rise_4", "fit",       Controller_c::STEP_PWM_LOW,  Controller_c::STEP_PWM_HIGH},
  {"rise_5", "fit",       Controller_c::STEP_PWM_LOW,  Controller_c::STEP_PWM_HIGH},
  {"fall_1", "held back", Controller_c::STEP_PWM_HIGH, Controller_c::STEP_PWM_LOW},
  {"fall_2", "held back", Controller_c::STEP_PWM_HIGH, Controller_c::STEP_PWM_LOW},
  {"fall_3", "held back", Controller_c::STEP_PWM_HIGH, Controller_c::STEP_PWM_LOW},
  {"fall_4", "held back", Controller_c::STEP_PWM_HIGH, Controller_c::STEP_PWM_LOW},
  {"fall_5", "held back", Controller_c::STEP_PWM_HIGH, Controller_c::STEP_PWM_LOW},
};
const int STEP_TRIAL_COUNT = (int)(sizeof(STEP_TRIALS) / sizeof(STEP_TRIALS[0]));
}  // namespace

Controller_c::Controller_c()
  : update_timer(UPDATE_INTERVAL_MS),
    telemetry_timer(TELEMETRY_INTERVAL_MS) {
  // Set before reset() because reset() deliberately leaves these alone: the
  // header must print once per session, and the trial number must keep
  // counting across trials rather than restarting at 1 each time.
  csv_header_done = false;
  db_header_done = false;
  sat_header_done = false;
  step_header_done = false;
  trial_num = 0;
  reset();
}

void Controller_c::reset() {
  signal = 0;
  mode = WAITING;
  recovery_start_ms = 0;

  // Reset the exercise 1 test routine so a new button press starts a
  // fresh 3-second trial rather than continuing from the previous one.
  call_count = 0;
  ramp_pwm = 0.0f;
  ramp_count = 0;
  test_start_ms = 0;

  // Reset the exercise 2 speed measurement. Note that trial_num is NOT reset
  // here: it must keep counting upwards across trials so that each trial in
  // the CSV keeps its own label. It is initialised in the constructor.
  last_time_ms = 0;
  last_count_left = 0;
  last_count_right = 0;
  speed_left_cps = 0.0f;
  speed_right_cps = 0.0f;
  speed_left_mm_s = 0.0f;
  speed_right_mm_s = 0.0f;
  speed_sample_count = 0;
  speed_baseline_set = false;
  sample_num = 0;

  // Reset the exercise 3 sweep so the next button press starts a fresh ladder
  // from the first level. Reaching here mid-sweep would be a bug: the sweep is
  // only supposed to return to waiting once it has finished or been aborted,
  // and nothing inside the sweep calls setSignal(0) for that reason.
  // db_header_done is deliberately NOT cleared: the header belongs at the top
  // of the capture, once per session, not once per sweep.
  db_rep = 0;
  db_pwm_index = 0;
  db_sample = 0;
  db_in_run = false;
  db_rest_calls = 0;
  db_rest_quiet = 0;
  db_run_calls = 0;
  db_rest_settled = false;
  db_rest_anchor = 0;
  db_rest_prev = 0;
  db_rest_residual = 0;
  db_last_count = 0;
  db_last_ms = 0;
  db_complete = false;

  // Same rewinding for the exercise 4 sweep: a new button press starts a fresh
  // ladder rather than continuing the previous one. sat_header_done is again
  // left alone so the header still appears once per session.
  sweep_uses_gain_levels = false;
  sat_phase = SAT_COOL;
  sat_rep = 0;
  sat_pwm_index = 0;
  sat_sample = 0;
  sat_phase_calls = 0;
  sat_last_left = 0;
  sat_last_right = 0;
  sat_last_ms = 0;
  sat_complete = false;

  // Exercise 7 rewinds to the first trial of the step-response list, so a new
  // button press replays the whole sequence rather than resuming it.
  step_trial_index = 0;
  step_phase = STEP_GAP;
  step_sample = 0;
  step_phase_start_ms = 0;
  step_step_ms = 0;
  step_command_ok = false;
  step_complete = false;

  // Unknown until update() performs the first encoder refresh, so treat it as
  // failed rather than assuming a good read.
  last_encoder_read_ok = false;
}

uint8_t Controller_c::getSignal() const {
  return signal;
}

void Controller_c::setSignal(uint8_t requested_signal) {
  if (requested_signal == 0) {
    reset();
    return;
  }

  if (signal == 0) {
    signal = 1;
    mode = FOLLOWING_LINE;
    recovery_start_ms = 0;
  }
}

void Controller_c::update(Robot_c &robot, RobotWifiAP_c &server) {

  // Get the current count of milliseconds since the StampC3 was powered on.
  unsigned long now = robot.getMillis();

  // Check if our TaskTimer_c for the general controller update indicates
  // that it is the correct time to operate.  Note, we give the TaskTimer_c
  // the current count in milliseconds.
  // We do this because we don't want to query the robot faster than at 10ms
  // intervals.
  if (!update_timer.isReady(now)) {

    // TaskTimer_c says it is not ready, so we simply end this call to
    // the controller here by calling return.
    return;
  }

  // The TaskTimer_c was ready, so we now reset the TaskTimer_c so it
  // begins counting again.  We now run the remaining controller code.
  update_timer.resetTimer(now);

  // Ask the robot for latest information on surface reflectance sensors.
  robot.getSurfaceSensors();

  // Ask the robot for the latest encoder information, and then use this
  // to update the odometry on board the StampC3 (see OdometryModel.h).
  // The result is kept, not just tested: a failed read leaves the previous
  // counts in the cache, and a stalled cache is indistinguishable from a wheel
  // that did not move. The deadband sweep leans on "the count did not change"
  // as evidence that the wheel stopped, so it has to be able to tell the two
  // apart.
  last_encoder_read_ok = robot.getEncoders();
  if (last_encoder_read_ok) {
    odometry.update(robot.getLeftEncoderCount(), robot.getRightEncoderCount());
  }

  // Pose estimation from the Pololu 3Pi robot itself.
  robot.getPose();

  // While waiting, a button press sets signal to 1 and starts the physical
  // robot's line follower, even without Processing connected. The transmitted
  // signal also starts an armed Digital Twin live comparison. This is a start
  // control, not a start/stop toggle; further presses are ignored while signal is 1.
  if (signal == 0) {
    if (robot.isButtonPressed()) {
      setSignal(1);
    }
  }

  // The user has started the demonstration, so we run the selected test
  // routine. Exactly one routine should be called here: comment out the one
  // you are not using rather than calling both at the same time.
  if (signal == 1) {
    testStepResponse(robot, now);
    // testGain(robot, now);
    // testSaturation(robot, now);
    // testDeadband(robot, now);
    // measureWheelSpeed(robot, now);
    // motorStartStopTest(robot, now);
    // runLineFollower(robot, now);

  }

  // Use another TaskTimer_c to limit how often we transmit telemetry data
  // on WiFi and Serial.
  //
  // Disabled for the wheel-speed measurement: publishTelemetry writes a
  // 14-column record every 50ms, which would interleave with the 9-column
  // measurement rows and leave a CSV that no longer parses. Re-enable it when
  // you want the Digital Twin feed back instead of a clean measurement file.
  if (telemetry_timer.isReady(now)) {
    telemetry_timer.resetTimer(now);
    // publishTelemetry(robot, server, now);

  }
}

bool Controller_c::lineDetected(const Robot_c &robot) const {
  // Note: only using the central sensor to detect a line.
  if (robot.surface.reading[2] >= LINE_THRESHOLD) {
    return true;
  }

  return false;
}

void Controller_c::runLineFollower(Robot_c &robot, unsigned long now) {
  bool line_detected = lineDetected(robot);

  if (!line_detected && mode == FOLLOWING_LINE) {
    mode = SEARCHING_FOR_LINE;
    recovery_start_ms = now;
  }

  if (mode == SEARCHING_FOR_LINE) {
    if (line_detected) {
      mode = FOLLOWING_LINE;
    } else if (now - recovery_start_ms >= RECOVERY_TIMEOUT_MS) {
      mode = STOPPED;
    }
  }

  if (mode == STOPPED) {
    robot.setMotorPWM(0, 0);
    return;
  }

  if (mode == SEARCHING_FOR_LINE) {
    robot.setMotorPWM(RECOVERY_LEFT_PWM, RECOVERY_RIGHT_PWM);
    return;
  }

  float error = (float)robot.surface.reading[1] - (float)robot.surface.reading[3];
  float turn = error * LINE_FOLLOW_GAIN;
  robot.setMotorPWM(BASE_PWM - turn, BASE_PWM + turn);
}

void Controller_c::motorStartStopTest(Robot_c &robot, unsigned long now) {

  // Has the routine already run for its full duration? If so, command the
  // motors off explicitly, release the start signal so that the button can
  // begin a fresh trial, and do no further work this call.
  if (call_count >= TEST_DURATION_CALLS) {
    robot.setMotorPWM(0, 0);
    if (Serial) {
      Serial.print("[test] stop_ms=");
      Serial.print(now);
      Serial.print(" duration_ms=");
      Serial.print(now - test_start_ms);
      Serial.print(" calls=");
      Serial.println(call_count);
    }
    setSignal(0);
    return;
  }

  // First call of a trial: remember the real start time so the reported
  // duration is measured, not assumed.
  if (call_count == 0) {
    test_start_ms = now;
    if (Serial) {
      Serial.print("[test] start_ms=");
      Serial.println(test_start_ms);
    }
  }

  // Otherwise this call is one more 10ms interval of the test.
  call_count = call_count + 1;

  // The PWM value changes less often than the function is called, so
  // ramp_count divides the test into RAMP_STEP_INTERVAL-call slices.
  ramp_count = ramp_count + 1;
  if (ramp_count >= RAMP_STEP_INTERVAL) {
    ramp_count = 0;

    // First half of the test ramps up, second half ramps back down, so the
    // routine reaches zero on its own instead of stopping abruptly.
    if (call_count <= TEST_DURATION_CALLS / 2) {
      ramp_pwm = ramp_pwm + RAMP_PWM_STEP;
    } else {
      ramp_pwm = ramp_pwm - RAMP_PWM_STEP;
    }

    // Never request more than the chosen maximum, and never go negative.
    ramp_pwm = constrain(ramp_pwm, 0.0f, MAX_RAMP_PWM);
  }

  // Opposite signs on the two wheels rotate the chassis on the spot rather
  // than driving it away. This matches the sign convention already used by
  // RECOVERY_LEFT_PWM and RECOVERY_RIGHT_PWM above.
  robot.setMotorPWM(-ramp_pwm, ramp_pwm);
}

void Controller_c::measureWheelSpeed(Robot_c &robot, unsigned long now) {

  // One trial is a fixed number of samples. When it is complete, command the
  // motors off, release the start signal so the button can begin a new trial,
  // and do no further work this call.
  if (sample_num >= SAMPLES_PER_TRIAL) {
    robot.setMotorPWM(0, 0);
    setSignal(0);
    return;
  }

  // Drive both motors at the configured test PWM. With TEST_PWM still at 0 the
  // motors stay off, which is the by-hand trial the exercise asks for first.
  robot.setMotorPWM(TEST_PWM, TEST_PWM);

  // Slow the measurement down. Sampling every cycle at 10ms would leave too few
  // whole encoder counts in each interval at low wheel speed, and the remainder
  // is discarded because counts are integers.
  speed_sample_count = speed_sample_count + 1;
  if (speed_sample_count < SPEED_SAMPLE_INTERVAL) {
    return;
  }
  speed_sample_count = 0;

  // Read the cached encoder snapshot. update() already refreshed it once this
  // cycle, and getLeftEncoderCount()/getRightEncoderCount() only read the
  // cache, so both wheels are reported from the same instant. Do not call
  // getEncoders() again here, or the two wheels would come from two different
  // snapshots. A failed read leaves the previous values in the cache, which
  // appears as a zero count change rather than an error.
  int32_t count_left = robot.getLeftEncoderCount();
  int32_t count_right = robot.getRightEncoderCount();

  // The first sample of a trial only establishes the baseline: there is no
  // previous reference to subtract from yet.
  if (!speed_baseline_set) {
    last_count_left = count_left;
    last_count_right = count_right;
    last_time_ms = now;
    speed_baseline_set = true;
    trial_num = trial_num + 1;
    sample_num = 0;
    return;
  }

  // Elapsed time is measured from the clock, not assumed to be 10ms.
  float delta_ms = (float)(now - last_time_ms);

  // Guard against a zero or near-zero interval: dividing by it would produce
  // an infinite speed estimate.
  if (delta_ms > 1.0f) {

    // Current count minus previous count, so a forward-turning wheel reports a
    // positive speed. The exercise page's example uses the opposite order;
    // either convention works, but pick one and check the sign physically.
    float delta_left = (float)(count_left - last_count_left);
    float delta_right = (float)(count_right - last_count_right);

    speed_left_cps = delta_left / delta_ms * 1000.0f;
    speed_right_cps = delta_right / delta_ms * 1000.0f;

    // Convert counts per second into millimetres per second, so the numbers
    // mean something without knowing this robot's encoder resolution.
    float mm_per_count = (PI * WHEEL_DIAMETER_MM) / ENCODER_COUNTS_PER_REV;
    speed_left_mm_s = speed_left_cps * mm_per_count;
    speed_right_mm_s = speed_right_cps * mm_per_count;
  }

  // Remember this snapshot as the reference for the next accepted sample.
  last_count_left = count_left;
  last_count_right = count_right;
  last_time_ms = now;

  // Emit one CSV record. The first seven columns are exactly the set the Colab
  // notebook expects; the last two are derived values added for reporting.
  if (Serial) {
    if (!csv_header_done) {
      Serial.println("trial_num,sample_num,timestamp_ms,requested_left_pwm,requested_right_pwm,left_encoder_count,right_encoder_count,left_speed_mm_s,right_speed_mm_s");
      csv_header_done = true;
    }
    Serial.print(trial_num);
    Serial.print(',');
    Serial.print(sample_num);
    Serial.print(',');
    Serial.print(now);
    Serial.print(',');
    Serial.print(robot.getLeftMotorPWM());
    Serial.print(',');
    Serial.print(robot.getRightMotorPWM());
    Serial.print(',');
    Serial.print((long)count_left);
    Serial.print(',');
    Serial.print((long)count_right);
    Serial.print(',');
    Serial.print(speed_left_mm_s, 3);
    Serial.print(',');
    Serial.println(speed_right_mm_s, 3);
  }

  sample_num = sample_num + 1;
}

// ---------------------------------------------------------------------------
// Exercise 3: deadband sweep.
//
// One wheel is driven from a verified stop through a ladder of PWM magnitudes.
// Every trial stands alone: rest, register, command, sample. The report is one
// row per sample, labelled with the command, the repeat and the sample number,
// so the analysis can ask how often a given command produced motion rather
// than only how much.
// ---------------------------------------------------------------------------

int32_t Controller_c::deadbandWheelCount(Robot_c &robot) {
  // Read the snapshot update() already refreshed this cycle. Do not call
  // getEncoders() here: that refreshes the cache, which would place the two
  // wheels at different instants and spend more of the 10ms budget on I2C.
  if (DEADBAND_WHEEL == 0) {
    return robot.getLeftEncoderCount();
  }
  return robot.getRightEncoderCount();
}

int32_t Controller_c::deadbandOtherCount(Robot_c &robot) {
  if (DEADBAND_WHEEL == 0) {
    return robot.getRightEncoderCount();
  }
  return robot.getLeftEncoderCount();
}

int Controller_c::deadbandLevelCount() const {
  return (int)((DEADBAND_PWM_MAX - DEADBAND_PWM_MIN) / DEADBAND_PWM_STEP) + 1;
}

float Controller_c::deadbandPwm() const {
  float magnitude = DEADBAND_PWM_MIN + (float)db_pwm_index * DEADBAND_PWM_STEP;
  if (DEADBAND_DIRECTION < 0) {
    return -magnitude;
  }
  return magnitude;
}

void Controller_c::deadbandApplyCommand(Robot_c &robot) {
  float pwm = deadbandPwm();

  // Only the driven wheel is commanded. The other channel stays at zero on
  // purpose: the free wheel is not what this test characterises, and its
  // encoder is reported separately as evidence about the chassis instead.
  if (DEADBAND_WHEEL == 0) {
    robot.setMotorPWM(pwm, 0.0f);
  } else {
    robot.setMotorPWM(0.0f, pwm);
  }
}

bool Controller_c::deadbandStepRest(Robot_c &robot) {

  // Hold the motors off for the whole rest so the next trial really does start
  // from rest. A wheel that is already turning needs less torque than one
  // breaking away from a stop, so the two are different measurements and only
  // the second one answers this exercise.
  robot.setMotorPWM(0, 0);

  int32_t count = deadbandWheelCount(robot);

  if (db_rest_calls == 0) {
    // Anchored right after the command went to zero, so the residual below
    // includes the coast-down. That coast is worth keeping: it is how far the
    // wheel travels after power is removed.
    db_rest_anchor = count;
    db_rest_prev = count;
    db_rest_quiet = 0;
  }

  // "The count did not change" only proves a stopped wheel if the read
  // actually succeeded. A failed refresh leaves the cache frozen, which is
  // otherwise indistinguishable from perfect stillness.
  if (!last_encoder_read_ok) {
    db_rest_quiet = 0;
  } else if (count == db_rest_prev) {
    db_rest_quiet = db_rest_quiet + 1;
  } else {
    db_rest_quiet = 0;
  }
  db_rest_prev = count;
  db_rest_calls = db_rest_calls + 1;
  db_rest_residual = count - db_rest_anchor;

  bool long_enough = (db_rest_calls >= DEADBAND_REST_MIN_CALLS);
  bool quiet_enough = (db_rest_quiet >= DEADBAND_REST_QUIET_CALLS);
  bool out_of_patience = (db_rest_calls >= DEADBAND_REST_MAX_CALLS);

  if ((long_enough && quiet_enough) || out_of_patience) {
    // Reported per trial, so "T_rest was long enough" ends up as a measurement
    // in the CSV rather than an assumption about the robot.
    db_rest_settled = quiet_enough;
    return true;
  }
  return false;
}

void Controller_c::deadbandStepRun(Robot_c &robot, unsigned long now) {

  db_run_calls = db_run_calls + 1;
  if (db_run_calls < DEADBAND_SAMPLE_CALLS) {
    return;
  }
  db_run_calls = 0;

  // Re-sent on each sample boundary rather than every cycle. The value does
  // not change within a trial, so this is only insurance against a dropped I2C
  // write, and once per 50ms is soon enough for that.
  deadbandApplyCommand(robot);

  int32_t count = deadbandWheelCount(robot);
  int32_t other = deadbandOtherCount(robot);

  // The interval is measured, never assumed to be DEADBAND_SAMPLE_CALLS x 10ms.
  float delta_ms = (float)(now - db_last_ms);
  float speed_cps = 0.0f;
  if (delta_ms > 1.0f) {
    speed_cps = (float)(count - db_last_count) / delta_ms * 1000.0f;
  }

  db_sample = db_sample + 1;

  if (Serial) {
    if (!db_header_done) {
      Serial.println("wheel,direction,PWM,trial_num,sample_num,speed_cps,speed_mm_s,dt_ms,time_ms,encoder_count,other_encoder_count,rest_residual_counts,rest_settled");
      db_header_done = true;
    }

    float mm_per_count = (PI * WHEEL_DIAMETER_MM) / ENCODER_COUNTS_PER_REV;
    float pwm = deadbandPwm();

    // Both labels follow from the command actually being sent, so neither can
    // drift out of agreement with the PWM column.
    const char *wheel_label = (DEADBAND_WHEEL == 0) ? "left" : "right";
    const char *direction_label = (pwm < 0.0f) ? "reverse" : "forward";

    Serial.print(wheel_label);
    Serial.print(',');
    Serial.print(direction_label);
    Serial.print(',');
    Serial.print((int)pwm);
    Serial.print(',');
    Serial.print(db_rep + 1);
    Serial.print(',');
    Serial.print(db_sample);
    Serial.print(',');
    Serial.print(speed_cps, 3);
    Serial.print(',');
    Serial.print(speed_cps * mm_per_count, 3);
    Serial.print(',');
    Serial.print(delta_ms, 1);
    Serial.print(',');
    Serial.print(now);
    Serial.print(',');
    Serial.print((long)count);
    Serial.print(',');
    Serial.print((long)other);
    Serial.print(',');
    Serial.print((long)db_rest_residual);
    Serial.print(',');
    Serial.println(db_rest_settled ? 1 : 0);
  }

  db_last_count = count;
  db_last_ms = now;

  if (db_sample >= DEADBAND_SAMPLES_PER_TRIAL) {
    // Trial over. Hand back to the rest phase, whose first act is to command
    // zero, so the motors are not stopped explicitly here.
    db_in_run = false;
    db_rest_calls = 0;
    db_rest_quiet = 0;
    db_rest_settled = false;
    deadbandAdvance();
  }
}

void Controller_c::deadbandAdvance() {

  // Walk the ladder to its end before starting a new pass. Keeping the repeat
  // outside the level loop is what turns drift over the session into a
  // difference between repeats instead of a fake trend along the PWM axis.
  db_pwm_index = db_pwm_index + 1;

  if (db_pwm_index < deadbandLevelCount()) {
    return;
  }

  db_pwm_index = 0;
  db_rep = db_rep + 1;

  if (db_rep >= DEADBAND_REPEATS) {
    db_complete = true;
  }
}

void Controller_c::testDeadband(Robot_c &robot, unsigned long now) {

  // The ladder has been walked its full number of passes. Stop explicitly, on
  // the driven wheel and the free one, then release the signal so the button
  // can start a fresh sweep. setSignal(0) calls reset(), which is also what
  // rewinds the sweep counters - so nothing inside the sweep may call it, or a
  // run in progress would be wiped back to its first level.
  if (db_complete) {
    robot.setMotorPWM(0, 0);
    if (Serial) {
      Serial.println("[db] sweep complete");
    }
    setSignal(0);
    return;
  }

  // A press during a powered run aborts the sweep. The button starts a test
  // while signal is zero; once a sweep is under way it is repurposed as a stop,
  // which is the only way to halt a multi-minute ladder without pulling power.
  // The first trial of the first pass is exempt, so the press that started the
  // sweep can never be read as the press that stops it. isButtonPressed()
  // repeats while held, so press and release rather than holding it down.
  bool past_first_trial = (db_rep > 0) || (db_pwm_index > 0);
  if (db_in_run && past_first_trial && robot.isButtonPressed()) {
    robot.setMotorPWM(0, 0);
    if (Serial) {
      Serial.println("[db] aborted");
    }
    setSignal(0);
    return;
  }

  if (!db_in_run) {
    if (deadbandStepRest(robot)) {
      // The rest is over. Registering the initial count and applying the test
      // command are the last two steps of the exercise's timeline, so they
      // happen here on the transition rather than on the call after it.
      db_last_count = deadbandWheelCount(robot);
      db_last_ms = now;
      db_run_calls = 0;
      db_sample = 0;
      db_in_run = true;
      deadbandApplyCommand(robot);
    }
    return;
  }

  deadbandStepRun(robot, now);
}

// ---------------------------------------------------------------------------
// Exercise 4: saturation sweep.
//
// The same ladder idea aimed at the top of the range. One thing is different:
// the wheel does not have to stop between commands. What it needs instead is a
// settling window after each new command, so the reported numbers describe a
// speed that has finished changing rather than one still on its way there.
// Cooling windows keep each powered burst short, because a hot winding does not
// respond the way a cool one does, and the response is the thing being measured.
// ---------------------------------------------------------------------------

int Controller_c::sweepLevelCount() const {
  if (sweep_uses_gain_levels) {
    return GAIN_LEVEL_COUNT;
  }
  return (int)((SATURATION_PWM_MAX - SATURATION_PWM_MIN) / SATURATION_PWM_STEP) + 1;
}

float Controller_c::sweepPwm() const {
  float magnitude;

  if (sweep_uses_gain_levels) {
    // A short explicit list rather than a ladder: these are the commands held
    // back from the gain fit, and there is no useful ordering between them.
    switch (sat_pwm_index) {
      case 0:  magnitude = GAIN_PWM_LEVEL_0; break;
      case 1:  magnitude = GAIN_PWM_LEVEL_1; break;
      default: magnitude = GAIN_PWM_LEVEL_0; break;
    }
  } else {
    magnitude = SATURATION_PWM_MIN + (float)sat_pwm_index * SATURATION_PWM_STEP;
  }

  // Clamped before it is ever sent. The course caps the supported command
  // magnitude at 200, and a typo in the constants above must not drive past it.
  magnitude = constrain(magnitude, 0.0f, SATURATION_PWM_LIMIT);

  if (SATURATION_DIRECTION < 0) {
    return -magnitude;
  }
  return magnitude;
}

void Controller_c::saturationApplyCommand(Robot_c &robot) {
  float pwm = sweepPwm();

  if (SATURATION_OPPOSITE_PAIR) {
    // Opposite directions cancel the two reaction torques, which is what stops
    // the chassis trying to rotate when both wheels are commanded hard.
    robot.setMotorPWM(pwm, -pwm);
    return;
  }

  if (SATURATION_WHEEL == 0) {
    robot.setMotorPWM(pwm, 0.0f);
  } else {
    robot.setMotorPWM(0.0f, pwm);
  }
}

void Controller_c::saturationEmitRows(int32_t left, int32_t right,
                                      float delta_ms, unsigned long now) {

  if (Serial && !sat_header_done) {
    Serial.println("wheel,direction,PWM,trial_num,sample_num,settled_speed_cps,speed_mm_s,dt_ms,time_ms,encoder_count,other_encoder_count");
    sat_header_done = true;
  }

  float pwm = sweepPwm();
  float mm_per_count = (PI * WHEEL_DIAMETER_MM) / ENCODER_COUNTS_PER_REV;

  // Which channels carry a command decides which channels get a row. An
  // uncommanded wheel is not a measurement of that wheel, so it is left out
  // rather than reported as a row of zeros that looks like a stalled motor.
  bool report_left = SATURATION_OPPOSITE_PAIR || (SATURATION_WHEEL == 0);
  bool report_right = SATURATION_OPPOSITE_PAIR || (SATURATION_WHEEL != 0);

  struct Row {
    const char *wheel;
    float command;
    int32_t count;     // this wheel's count at this instant
    int32_t previous;  // this wheel's count one sample ago
    int32_t other;     // the other wheel's count, as evidence about the chassis
  };
  Row rows[2];
  int row_count = 0;
  if (report_left) {
    rows[row_count++] = {"left", pwm, left, sat_last_left, right};
  }
  if (report_right) {
    // In opposite-pair mode the right wheel takes the mirrored command.
    rows[row_count++] = {"right", SATURATION_OPPOSITE_PAIR ? -pwm : pwm,
                         right, sat_last_right, left};
  }

  for (int i = 0; i < row_count; i++) {
    // The interval is measured, never assumed to be 5 x 10ms.
    float speed_cps = 0.0f;
    if (delta_ms > 1.0f) {
      speed_cps = (float)(rows[i].count - rows[i].previous) / delta_ms * 1000.0f;
    }

    // Derived from the command actually sent, so the label cannot disagree
    // with the PWM column.
    const char *direction_label = (rows[i].command < 0.0f) ? "reverse" : "forward";

    Serial.print(rows[i].wheel);
    Serial.print(',');
    Serial.print(direction_label);
    Serial.print(',');
    Serial.print((int)rows[i].command);
    Serial.print(',');
    Serial.print(sat_rep + 1);
    Serial.print(',');
    Serial.print(sat_sample);
    Serial.print(',');
    Serial.print(speed_cps, 3);
    Serial.print(',');
    Serial.print(speed_cps * mm_per_count, 3);
    Serial.print(',');
    Serial.print(delta_ms, 1);
    Serial.print(',');
    Serial.print(now);
    Serial.print(',');
    Serial.print((long)rows[i].count);
    Serial.print(',');
    Serial.println((long)rows[i].other);
  }
}

void Controller_c::saturationAdvance() {

  // Advance the level first and only start a new pass after the whole ladder
  // has been walked, so any drift over the session lands on every level
  // equally instead of accumulating along the PWM axis and faking a plateau.
  sat_pwm_index = sat_pwm_index + 1;

  if (sat_pwm_index < sweepLevelCount()) {
    return;
  }

  sat_pwm_index = 0;
  sat_rep = sat_rep + 1;

  if (sat_rep >= SATURATION_REPEATS) {
    sat_complete = true;
  }
}

void Controller_c::testSaturation(Robot_c &robot, unsigned long now) {
  // Exercise 4: walk the ladder that maps the top of the command range.
  sweep_uses_gain_levels = false;
  sweepStep(robot, now);
}

void Controller_c::testGain(Robot_c &robot, unsigned long now) {
  // Exercise 5: the same sweep, pointed at the held-back commands instead of a
  // ladder. Sharing the phase machine is deliberate - these rows exist to be
  // compared against earlier data, so their settling, sampling and timing have
  // to be identical or a difference in protocol would look like a modelling
  // error.
  sweep_uses_gain_levels = true;
  sweepStep(robot, now);
}

void Controller_c::sweepStep(Robot_c &robot, unsigned long now) {

  // Ladder finished. Stop both channels explicitly, then release the signal so
  // the button can start a fresh sweep. setSignal(0) calls reset(), which is
  // also what rewinds the sweep counters, so nothing inside the sweep may call
  // it while a ladder is still running.
  if (sat_complete) {
    robot.setMotorPWM(0, 0);
    if (Serial) {
      Serial.println(sweep_uses_gain_levels ? "[gain] sweep complete"
                                            : "[sat] sweep complete");
    }
    setSignal(0);
    return;
  }

  // A press while actually measuring ends the sweep. Armed only from the second
  // trial on, so the press that started the sweep can never stop it. The
  // measuring window is the only phase worth interrupting: the other two have
  // the motors off or barely spinning.
  bool past_first_trial = (sat_rep > 0) || (sat_pwm_index > 0);
  if (sat_phase == SAT_MEASURE && past_first_trial && robot.isButtonPressed()) {
    robot.setMotorPWM(0, 0);
    if (Serial) {
      Serial.println(sweep_uses_gain_levels ? "[gain] aborted" : "[sat] aborted");
    }
    setSignal(0);
    return;
  }

  if (sat_phase == SAT_COOL) {
    // Motors off on every call of the cooling window, not just the first, so a
    // dropped I2C write cannot leave the previous command running through it.
    robot.setMotorPWM(0, 0);
    sat_phase_calls = sat_phase_calls + 1;
    if (sat_phase_calls >= SATURATION_COOL_CALLS) {
      sat_phase = SAT_SETTLE;
      sat_phase_calls = 0;
      saturationApplyCommand(robot);
    }
    return;
  }

  if (sat_phase == SAT_SETTLE) {
    // Run-up happens here and none of it is reported. Switching to the
    // measuring phase is what opens the window, and the first sample is a full
    // sampling interval after that, so the settling transient cannot leak into
    // the average.
    sat_phase_calls = sat_phase_calls + 1;

    // Re-sent every 100ms through the settling window as insurance against a
    // dropped I2C write. Without this, one failed write would leave the wheel
    // stationary for the whole window and the trial would quietly report a
    // stationary motor rather than an error.
    if ((sat_phase_calls % 10) == 0) {
      saturationApplyCommand(robot);
    }

    if (sat_phase_calls >= SATURATION_SETTLE_CALLS) {
      sat_phase = SAT_MEASURE;
      sat_phase_calls = 0;
      sat_sample = 0;
      sat_last_left = robot.getLeftEncoderCount();
      sat_last_right = robot.getRightEncoderCount();
      sat_last_ms = now;
    }
    return;
  }

  // SAT_MEASURE.
  sat_phase_calls = sat_phase_calls + 1;
  if (sat_phase_calls < SATURATION_SAMPLE_CALLS) {
    return;
  }
  sat_phase_calls = 0;

  // Re-sent on each sample boundary rather than every cycle: the value does not
  // change within a trial, so this is only insurance against a dropped write.
  saturationApplyCommand(robot);

  // Both wheels come from the same cached snapshot, so the two rows of an
  // opposite-pair sample describe one instant.
  int32_t left = robot.getLeftEncoderCount();
  int32_t right = robot.getRightEncoderCount();

  float delta_ms = (float)(now - sat_last_ms);
  sat_sample = sat_sample + 1;

  saturationEmitRows(left, right, delta_ms, now);

  sat_last_left = left;
  sat_last_right = right;
  sat_last_ms = now;

  if (sat_sample >= SATURATION_SAMPLES_PER_TRIAL) {
    // Trial over. Hand back to the cooling window, whose first act is to
    // command zero, so the motors are not stopped explicitly here.
    sat_phase = SAT_COOL;
    sat_phase_calls = 0;
    saturationAdvance();
  }
}

// ---------------------------------------------------------------------------
// Exercise 7: step response.
// ---------------------------------------------------------------------------

bool Controller_c::stepApplyCommand(Robot_c &robot, float pwm) {
  if (STEP_WHEEL == 0) {
    return robot.setMotorPWM(pwm, 0.0f);
  }
  return robot.setMotorPWM(0.0f, pwm);
}

void Controller_c::stepEmitRow(Robot_c &robot, unsigned long now) {

  if (Serial && !step_header_done) {
    Serial.println("trial_id,split,wheel,direction,sample_num,timestamp_ms,step_timestamp_ms,PWM,encoder_count,phase");
    step_header_done = true;
  }

  const StepTrial_c &trial = STEP_TRIALS[step_trial_index];

  // The command this row was taken under. The row written at the boundary
  // between the initial hold and the transient therefore carries the OLD
  // command, because the change goes out immediately after it; the first row
  // that can show the new command is the next one.
  float commanded;
  const char *phase_label;
  if (step_phase == STEP_INITIAL_HOLD) {
    commanded = trial.pwm_initial;
    phase_label = "initial_hold";
  } else if (step_phase == STEP_TRANSIENT) {
    commanded = trial.pwm_final;
    phase_label = "transient";
  } else {
    commanded = trial.pwm_final;
    phase_label = "final_hold";
  }

  int32_t count = (STEP_WHEEL == 0) ? robot.getLeftEncoderCount()
                                    : robot.getRightEncoderCount();

  // Both labels follow from the command actually in force, so neither can drift
  // out of agreement with the PWM column.
  const char *wheel_label = (STEP_WHEEL == 0) ? "left" : "right";
  const char *direction_label = (commanded < 0.0f) ? "reverse" : "forward";

  Serial.print(trial.id);
  Serial.print(',');
  Serial.print(trial.split);
  Serial.print(',');
  Serial.print(wheel_label);
  Serial.print(',');
  Serial.print(direction_label);
  Serial.print(',');
  Serial.print(step_sample);
  Serial.print(',');
  Serial.print(now);
  Serial.print(',');
  Serial.print(step_step_ms);
  Serial.print(',');
  Serial.print((int)commanded);
  Serial.print(',');
  Serial.print((long)count);
  Serial.print(',');
  Serial.println(phase_label);
}

void Controller_c::testStepResponse(Robot_c &robot, unsigned long now) {

  if (step_complete) {
    robot.setMotorPWM(0, 0);
    if (Serial) {
      Serial.println("[step] test complete");
    }
    setSignal(0);
    return;
  }

  // A press stops the test. Armed only after the first trial has finished, so
  // the press that started the test can never be read as the press that stops
  // it. isButtonPressed() repeats while held, so press and release.
  if (step_trial_index > 0 && robot.isButtonPressed()) {
    robot.setMotorPWM(0, 0);
    if (Serial) {
      Serial.println("[step] aborted");
    }
    setSignal(0);
    return;
  }

  // First call of a sweep: anchor the opening gap at the current time, so it
  // runs its full length instead of ending immediately against a stale zero.
  if (step_phase_start_ms == 0) {
    step_phase_start_ms = now;
  }

  const StepTrial_c &trial = STEP_TRIALS[step_trial_index];

  if (step_phase == STEP_GAP) {
    // Nothing is recorded here. The wheel is still winding down from whatever
    // the previous trial left it at, and a row labelled as a hold would be a
    // description of something that did not happen.
    robot.setMotorPWM(0, 0);
    if (now - step_phase_start_ms >= STEP_GAP_MS) {
      step_phase = STEP_INITIAL_HOLD;
      step_phase_start_ms = now;
      step_sample = 0;
      step_command_ok = false;
      // Fixed here, before any row is written, because a row written during the
      // hold cannot know a time that has not happened yet. The change is
      // actually issued on the first cycle at or after this stamp, so the real
      // command edge lands within one controller tick of what is reported.
      step_step_ms = now + STEP_INITIAL_MS;
    }
    return;
  }

  // The snapshot is taken before the phase transition is evaluated. That
  // ordering is what makes the boundary row carry the command that was actually
  // in force while the counts were read.
  stepEmitRow(robot, now);
  step_sample = step_sample + 1;

  if (step_phase == STEP_INITIAL_HOLD) {
    // Retried until the write is acknowledged rather than re-sent every cycle.
    // This loop has the least room to spare of any in the project, and a retry
    // buys the same guarantee as a duplicate for half the traffic.
    if (!step_command_ok) {
      step_command_ok = stepApplyCommand(robot, trial.pwm_initial);
    }
    if (now - step_phase_start_ms >= STEP_INITIAL_MS) {
      step_phase = STEP_TRANSIENT;
      step_command_ok = false;
    }
    return;
  }

  if (step_phase == STEP_TRANSIENT) {
    if (!step_command_ok) {
      step_command_ok = stepApplyCommand(robot, trial.pwm_final);
    }
    if (now - step_step_ms >= STEP_TRANSIENT_MS) {
      step_phase = STEP_FINAL_HOLD;
      step_phase_start_ms = now;
    }
    return;
  }

  // STEP_FINAL_HOLD: the second command is held long enough that the settled
  // speed can be read off the tail rather than extrapolated.
  if (!step_command_ok) {
    step_command_ok = stepApplyCommand(robot, trial.pwm_final);
  }
  if (now - step_phase_start_ms >= STEP_FINAL_MS) {
    step_trial_index = step_trial_index + 1;
    if (step_trial_index >= STEP_TRIAL_COUNT) {
      step_complete = true;
    }
    step_phase = STEP_GAP;
    step_phase_start_ms = now;
    step_sample = 0;
    step_command_ok = false;
  }
}

void Controller_c::publishTelemetry(Robot_c &robot, RobotWifiAP_c &server, unsigned long timestamp_ms) {
  server.printf(
    "%lu,%.2f,%.2f,%.5f,%d,%d,%ld,%ld,%u,%u,%u,%u,%u,%u\n",
    timestamp_ms,
    odometry.pose.x,
    odometry.pose.y,
    odometry.pose.theta,
    robot.getLeftMotorPWM(),
    robot.getRightMotorPWM(),
    (long)robot.getLeftEncoderCount(),
    (long)robot.getRightEncoderCount(),
    robot.surface.reading[0],
    robot.surface.reading[1],
    robot.surface.reading[2],
    robot.surface.reading[3],
    robot.surface.reading[4],
    signal
  );
  if ( Serial ) { // If serial is connected
    Serial.printf(
      "%lu,%.2f,%.2f,%.5f,%d,%d,%ld,%ld,%u,%u,%u,%u,%u,%u\n",
      timestamp_ms,
      odometry.pose.x,
      odometry.pose.y,
      odometry.pose.theta,
      robot.getLeftMotorPWM(),
      robot.getRightMotorPWM(),
      (long)robot.getLeftEncoderCount(),
      (long)robot.getRightEncoderCount(),
      robot.surface.reading[0],
      robot.surface.reading[1],
      robot.surface.reading[2],
      robot.surface.reading[3],
      robot.surface.reading[4],
      signal
    );
  }
}
