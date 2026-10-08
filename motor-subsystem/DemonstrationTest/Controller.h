#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Robot.h"
#include "RobotWifiAP.h"
#include "TaskTimer.h"
#include "OdometryModel.h"

/**
 * @brief Supplies the nominal line-following controller for the StampC3 example.
 *
 * Controller_c owns the controller state, while Robot_c owns the latest hardware
 * snapshot and RobotWifiAP_c owns optional telemetry transport. Students may read
 * or replace this controller, but should treat its constants as a demonstrator
 * baseline rather than a calibration of every physical robot.
 */
class Controller_c {
  
  public:
    /** @brief Editable local encoder-to-pose model; inspect odometry.pose.
     * @note Controller mode resets do not reset this estimate. Use
     * odometry.reset(...) explicitly to start a new physical trial.
     */
    OdometryModel_c odometry;

    // To keep track of how often a function is called.
    int call_count;
    // Ramp state: current PWM magnitude, and a sub-counter that divides
    // the main test duration into shorter PWM update intervals.
    float ramp_pwm;
    int ramp_count;
    // Timestamp captured on the first call of a trial, used to report the
    // real elapsed duration rather than assuming 10ms per call.
    unsigned long test_start_ms;

    // Exercise 2 speed-measurement state. "last_*" hold the previous accepted
    // snapshot; the difference to the current snapshot is the motion that
    // happened between the two sample times.
    unsigned long last_time_ms;
    int32_t last_count_left;
    int32_t last_count_right;
    float speed_left_cps;
    float speed_right_cps;
    float speed_left_mm_s;
    float speed_right_mm_s;
    // Sub-counter that slows sampling down to SPEED_SAMPLE_INTERVAL calls.
    int speed_sample_count;
    // False until the first snapshot of a trial has been stored as a baseline.
    bool speed_baseline_set;
    // CSV row labels: which trial, and which sample within that trial.
    int trial_num;
    int sample_num;

    // Result of the most recent encoder refresh in update(). A failed I2C read
    // leaves the previous counts in the cache, which looks identical to a wheel
    // that did not move. Anything that treats "the count did not change" as
    // evidence must check this too, or it will mistake a broken read for proof
    // of a stopped wheel.
    bool last_encoder_read_ok;

    // Exercise 3 deadband sweep state. Advanced entirely by the counters below;
    // the sweep never touches signal, because setSignal(0) calls reset() and
    // would wipe the sweep back to its first level mid-run.
    bool db_header_done;
    int db_rep;            // 0..DEADBAND_REPEATS-1, the outer loop; reported
                           // as trial_num, which is why it starts at one
    int db_pwm_index;      // 0..(levels-1) within the current repeat
    int db_sample;         // 1..N_s, the row label within a trial
    bool db_in_run;        // false = resting, true = powered and sampling
    int db_rest_calls;     // update() calls spent in the current rest
    int db_rest_quiet;     // consecutive resting calls with an unchanged count
    int db_run_calls;      // update() calls since the current run began
    bool db_rest_settled;  // did the rest reach the quiet criterion?
    int32_t db_rest_anchor;  // count when the rest began, for residual travel
    int32_t db_rest_prev;    // count at the previous resting call
    int32_t db_rest_residual; // counts travelled while the motors were off
    int32_t db_last_count;   // count at the previous accepted sample
    unsigned long db_last_ms; // time of the previous accepted sample
    bool db_complete;        // set once the last trial of the last repeat ends

    /** @brief The three windows the saturation sweep repeats, in order. */
    enum SatPhase {
      SAT_COOL,     // motors off, letting the winding shed heat
      SAT_SETTLE,   // command applied, speed still changing - never reported
      SAT_MEASURE   // command held, speed settled - everything here is reported
    };

    // Settle-and-measure sweep state, shared by exercises 4 and 5. The two
    // tests differ only in which commands they walk, so they share one phase
    // machine: that is what guarantees the held-back commands are measured
    // under exactly the same conditions as the data they will be compared to.
    bool sweep_uses_gain_levels;  // false = saturation ladder, true = held-back list
    bool sat_header_done;
    SatPhase sat_phase;
    int sat_rep;           // 0..SATURATION_REPEATS-1, the outer loop; this is
                           // reported as trial_num, matching the deadband sweep
    int sat_pwm_index;     // 0..(levels-1) within the current repeat
    int sat_sample;        // 1..N_s inside the measuring window
    int sat_phase_calls;   // update() calls spent in the current phase
    int32_t sat_last_left;   // both wheels tracked, because the opposite-pair
    int32_t sat_last_right;  // mode reports them both from one sample instant
    unsigned long sat_last_ms;
    bool sat_complete;

    /** @brief The four windows of one step-response trial, in order. */
    enum StepPhase {
      STEP_GAP,           // motors off, nothing recorded
      STEP_INITIAL_HOLD,  // first command applied, waiting for a steady speed
      STEP_TRANSIENT,     // the step has happened, the speed is changing
      STEP_FINAL_HOLD     // second command held, measuring where it settled
    };

    // Exercise 7 step-response state.
    bool step_header_done;
    int step_trial_index;          // index into the trial table
    StepPhase step_phase;
    int step_sample;               // row label within the current trial
    unsigned long step_phase_start_ms;
    unsigned long step_step_ms;    // when the command changes, as reported
    bool step_command_ok;          // has the current command been acknowledged?
    bool step_complete;

    /** @brief Fastest interval between controller and I2C acquisition cycles. */
    static const unsigned long UPDATE_INTERVAL_MS = 10;
    /** @brief Interval between telemetry records in milliseconds. */
    static const unsigned long TELEMETRY_INTERVAL_MS = 50;
    /** @brief Maximum line-search duration before the controller stops. */
    static const unsigned long RECOVERY_TIMEOUT_MS = 5000;
    /** @brief Surface-reading threshold used to decide whether a line is present. */
    static const uint16_t LINE_THRESHOLD = 1400;
    /** @brief Nominal forward PWM bias for line following. */
    static constexpr float BASE_PWM = 45.0f;
    /** @brief Proportional gain applied to the DN2-minus-DN4 surface error. */
    static constexpr float LINE_FOLLOW_GAIN = 0.01f;
    /** @brief Left PWM used during the one-direction line-search rotation. */
    static constexpr float RECOVERY_LEFT_PWM = -35.0f;
    /** @brief Right PWM used during the one-direction line-search rotation. */
    static constexpr float RECOVERY_RIGHT_PWM = 35.0f;

    /** @brief update() calls in the motor start/stop test, at 10ms per call.
     * @note 300 calls x 10ms = 3000ms = 3 seconds. */
    static const int TEST_DURATION_CALLS = 300;
    /** @brief update() calls between each change of the ramp PWM value.
     * @note 10 calls x 10ms = 100ms between PWM changes. */
    static const int RAMP_STEP_INTERVAL = 10;
    /** @brief PWM magnitude added or removed at each ramp step. */
    static constexpr float RAMP_PWM_STEP = 4.0f;
    /** @brief Highest PWM magnitude the ramp is allowed to request.
     * @note Conservative on purpose: the middleware motor limits are wider,
     * but the ramp only needs to demonstrate a smooth increase and decrease. */
    static constexpr float MAX_RAMP_PWM = 60.0f;

    /** @brief Nominal encoder counts per full wheel revolution.
     * @note Pololu 3Pi+ Standard Edition: 12 counts per motor-shaft revolution
     * multiplied by the 29.86:1 gearbox gives 358.3 counts per wheel turn. */
    static constexpr float ENCODER_COUNTS_PER_REV = 358.3f;
    /** @brief Nominal wheel diameter in millimetres.
     * @note Used to convert counts per second into millimetres per second.
     * The odometry exercises calibrate the effective value for this robot. */
    static constexpr float WHEEL_DIAMETER_MM = 32.0f;
    /** @brief update() calls between speed samples.
     * @note 5 calls x 10ms = 50ms per sample, i.e. 20 samples per second.
     * Sampling more slowly than every cycle leaves more encoder counts in each
     * interval, which matters at low wheel speed where whole counts are scarce. */
    static const int SPEED_SAMPLE_INTERVAL = 5;
    /** @brief Samples collected in one trial before the routine stops.
     * @note 200 samples at 20 samples/second = 10 seconds per trial. */
    static const int SAMPLES_PER_TRIAL = 200;
    /** @brief PWM requested from both motors during a trial.
     * @note 0 leaves the motors off so the wheel can be turned by hand.
     * Change this value and re-upload to run a powered trial. */
    static constexpr float TEST_PWM = 60.0f;

    // ------------------------------------------------------------------
    // Exercise 3: deadband sweep.
    // Drives ONE wheel, from rest, through a fixed ladder of PWM magnitudes
    // and reports how fast that wheel turned at each command. The point is to
    // find the smallest command that produces sustained motion rather than a
    // twitch, so every trial must start from a genuinely stopped wheel.
    // ------------------------------------------------------------------

    /** @brief Which wheel the sweep drives: 0 = left, 1 = right.
     * @note Only the driven wheel is reported. The other wheel is free in this
     * test, so its encoder is evidence about the chassis, not about itself. */
    static const int DEADBAND_WHEEL = 0;
    /** @brief Which way the sweep drives it: +1 = forward, -1 = reverse.
     * @note The reported direction label is derived from the sign of the
     * command actually sent, so a wrong value here cannot silently mislabel
     * half the dataset. */
    static const int DEADBAND_DIRECTION = 1;

    /** @brief First PWM magnitude tested.
     * @note Deliberately zero. One sweep level at zero command measures the
     * noise floor - desk vibration, cable tug, encoder jitter - that every
     * later "did it move?" decision is judged against. It costs one level and
     * makes the detection threshold a measurement instead of a guess. */
    static constexpr float DEADBAND_PWM_MIN = 0.0f;
    /** @brief Last PWM magnitude tested. */
    static constexpr float DEADBAND_PWM_MAX = 60.0f;
    /** @brief Increment between sweep levels. */
    static constexpr float DEADBAND_PWM_STEP = 5.0f;
    /** @brief Full passes over every level, and also the number of trials each
     * level gets. Total trials = DEADBAND_REPEATS x (number of levels).
     * @note Repeats are the OUTER loop on purpose. Sweeping all levels inside
     * one repeat lets session drift (battery sag, warming rubber, cable
     * tension) accumulate along the PWM axis and masquerade as a PWM effect.
     * With the repeat outermost, drift is shared by every level and shows up
     * as a difference BETWEEN repeats instead. It also means an interrupted
     * run still leaves a complete, comparable set of levels.
     * @note This is why the reported trial_num is the repeat number: the same
     * command is tested once per repeat, exactly as the exercise's example
     * output shows two trials against the same PWM value. */
    static const int DEADBAND_REPEATS = 5;
    /** @brief update() calls between speed samples. 5 x 10ms = 50ms.
     * @note Kept at 50ms rather than the 10ms minimum. Total counts over a
     * fixed window do not change with the sampling interval, so a longer
     * interval costs nothing in average-speed resolution while buying a 5x
     * finer per-sample quantisation step (5.6 mm/s against 28 mm/s). That
     * per-sample step is what separates a twitch from a slow crawl. */
    static const int DEADBAND_SAMPLE_CALLS = 5;
    /** @brief Speed samples per trial (N_s). 20 x 50ms = 1.0s of motion. */
    static const int DEADBAND_SAMPLES_PER_TRIAL = 20;
    /** @brief Shortest rest before a trial. 100 x 10ms = 1.0s. */
    static const int DEADBAND_REST_MIN_CALLS = 100;
    /** @brief Rest keeps extending until the encoder has been still this long.
     * @note 50 x 10ms = 0.5s with zero count change, i.e. the wheel moved less
     * than one count in half a second: below 2 counts/s, or 0.56 mm/s. This is
     * the evidence that T_rest was long enough, rather than an assumption. */
    static const int DEADBAND_REST_QUIET_CALLS = 50;
    /** @brief Rest is abandoned and the trial flagged unsettled at this point.
     * @note 300 x 10ms = 3.0s. Reaching this means the wheel never stopped. */
    static const int DEADBAND_REST_MAX_CALLS = 300;

    // ------------------------------------------------------------------
    // Exercise 4: saturation sweep.
    // Same ladder idea as the deadband sweep, aimed at the top of the range,
    // and with one important difference: the wheel does NOT have to stop
    // between commands. What it does need is a settling period after each new
    // command, long enough for the speed to stabilise, before the measurement
    // window opens. Samples taken during settling must never reach the average,
    // so the two windows are separate phases below.
    // ------------------------------------------------------------------

    /** @brief Which wheel the sweep drives when only one is driven: 0 = left, 1 = right. */
    static const int SATURATION_WHEEL = 0;
    /** @brief Which way it drives it: +1 = forward, -1 = reverse. */
    static const int SATURATION_DIRECTION = 1;
    /** @brief Drive both wheels in opposite directions instead of one.
     * @note The course suggests this for high-speed tests because the two
     * reaction torques cancel. It is not needed with the wheels off the
     * ground, but it is the safer option if the robot is ever tested in
     * contact with a surface. Both wheels are then reported, one row each,
     * so the notebook can still split them by wheel and direction. */
    static const bool SATURATION_OPPOSITE_PAIR = false;

    /** @brief First PWM magnitude tested. Overlaps the deadband sweep so the
     * two datasets share one command value for a direct cross-check. */
    static constexpr float SATURATION_PWM_MIN = 60.0f;
    /** @brief Last PWM magnitude tested: the top of the supported range. */
    static constexpr float SATURATION_PWM_MAX = 200.0f;
    /** @brief Increment between sweep levels. */
    static constexpr float SATURATION_PWM_STEP = 10.0f;
    /** @brief Anything beyond this magnitude is clamped before it is sent. */
    static constexpr float SATURATION_PWM_LIMIT = 200.0f;
    /** @brief Full passes over every level, and the trials each level gets. */
    static const int SATURATION_REPEATS = 5;

    /** @brief update() calls the motors stay off between trials. 100 x 10ms = 1.0s.
     * @note This is the cooling interval the exercise asks for. High commands
     * heat the winding, and heat changes the very response being measured. */
    static const int SATURATION_COOL_CALLS = 100;
    /** @brief update() calls between applying a command and measuring it.
     * @note 60 x 10ms = 0.6s. Nothing here is reported, which is the point:
     * the settled-speed average must not contain any of the run-up. */
    static const int SATURATION_SETTLE_CALLS = 60;
    /** @brief update() calls between speed samples. 5 x 10ms = 50ms. */
    static const int SATURATION_SAMPLE_CALLS = 5;
    /** @brief Speed samples per trial. 20 x 50ms = 1.0s of measured motion. */
    static const int SATURATION_SAMPLES_PER_TRIAL = 20;

    // ------------------------------------------------------------------
    // Exercise 5: gain.
    //
    // The gain is the slope of the response inside the moving, unsaturated
    // region, and the evidence for that region already exists: the deadband
    // sweep bounds it from below and the saturation sweep from above. So this
    // test does not repeat those measurements. It measures only the commands
    // that were deliberately held back from the fit, which is what makes the
    // model's prediction of them a test rather than a restatement.
    //
    // Both conditions below are inside the saturation ladder, so they must be
    // EXCLUDED from the fitting data in the notebook. The course is explicit
    // that the held-back command is one "you have not directly measured" - so
    // the exercise 4 rows at these two commands do not belong in the gain
    // dataset at all.
    // ------------------------------------------------------------------

    /** @brief How many held-back commands the gain test re-measures. */
    static const int GAIN_LEVEL_COUNT = 2;
    /** @brief Held-back command in the middle of the fitted range.
     * @note Tests that the line interpolates, which is the easy case. */
    static constexpr float GAIN_PWM_LEVEL_0 = 100.0f;
    /** @brief Held-back command one step below the saturation knee.
     * @note Tests the edge of the fitted range, where a straight line has the
     * least room to be wrong and still look right. */
    static constexpr float GAIN_PWM_LEVEL_1 = 160.0f;

    // ------------------------------------------------------------------
    // Exercise 7: slew rate and lag.
    //
    // Exercises 5 and 6 described where the speed ends up. This one asks how it
    // gets there: hold a command until the speed is steady, change it once, and
    // keep the whole time series until the new speed settles.
    //
    // Two things are different from every earlier test, and both matter:
    //   1. SAMPLING IS 10ms, not 50ms. The transient is only a few hundred
    //      milliseconds long; at 50ms it would be three or four points and the
    //      shape would be unreadable.
    //   2. The CSV reports RAW COUNTS AND TIMESTAMPS, not a speed. The notebook
    //      differences them itself, so the derivation stays inspectable and the
    //      reporting cannot hide a mistake behind a number we computed.
    // ------------------------------------------------------------------

    /** @brief Which wheel the step test drives: 0 = left, 1 = right. */
    static const int STEP_WHEEL = 0;
    /** @brief Command held before the step. Must sit inside the usable range.
     * @note 60 and 140 are both comfortably inside 40-170, so neither deadband
     * nor saturation is doing the shaping. A step that starts near a limit
     * would measure that limit rather than the transient. */
    static constexpr float STEP_PWM_LOW = 60.0f;
    /** @brief Command applied after the step. */
    static constexpr float STEP_PWM_HIGH = 140.0f;
    /** @brief The trial list itself lives in Controller.cpp, where each entry
     * names its trial_id, whether it fits or is held back, and the two commands.
     * @note The repeats and the held-back split are properties of that list, not
     * of constants here - edit the list to change them, or the two would drift
     * apart and the header would be lying about what the test does. */

    /** @brief Motors off between trials, nothing recorded. */
    static const unsigned long STEP_GAP_MS = 1000;
    /** @brief Hold the first command until the speed is steady. Recorded. */
    static const unsigned long STEP_INITIAL_MS = 1000;
    /** @brief Time after the step that is reported as the transient. */
    static const unsigned long STEP_TRANSIENT_MS = 1400;
    /** @brief Hold the second command to measure where it settled. Recorded. */
    static const unsigned long STEP_FINAL_MS = 1000;

    /** @brief Creates a controller in its waiting state. */
    Controller_c();

    /** @brief Clears signal and mode state, returning the controller to waiting.
     * @note This does not reset the task timers or the local odometry estimate.
     * It sends no motor command: returning to waiting does not stop the motors.
     * Send robot.setMotorPWM(0, 0) separately and check the command result.
     */
    void reset();

    /**
     * @brief Returns whether the controller has received its start signal.
     * @return Zero while waiting; one after the button or caller starts it.
     */
    uint8_t getSignal() const;

    /**
     * @brief Starts the controller or resets it to waiting.
     * @param requested_signal Zero resets to waiting without sending a motor stop;
     * a non-zero value starts line following only if the current signal is zero.
     * @note The signal remains one after a line-search timeout. Reset to zero
     * before requesting another start; a repeated non-zero value has no effect.
     */
    void setSignal(uint8_t requested_signal);

    /**
     * @brief Runs one time-controlled acquisition, control, and telemetry cycle.
     * @param robot Hardware-facing interface and cached measurement snapshot.
     * @param server Optional WiFi/TCP telemetry transport.
     * @note Each accepted update cycle attempts to refresh sensors, encoders,
     * and the middleware pose. Failed reads retain their previous cached values;
     * local odometry is updated only after a successful encoder read. The method
     * returns early between update intervals.
     */
    void update(Robot_c &robot, RobotWifiAP_c &server);

  private:
    /** @brief Internal modes used by the nominal line-following demonstration. */
    enum Mode {
      WAITING,
      FOLLOWING_LINE,
      SEARCHING_FOR_LINE,
      STOPPED
    };

    uint8_t signal;
    Mode mode;
    TaskTimer_c update_timer;
    TaskTimer_c telemetry_timer;
    unsigned long recovery_start_ms;
    /** @brief True once the CSV column header has been printed this session.
     * @note Deliberately not cleared by reset(): the header must appear once at
     * the top of the capture, not once per trial, or the file stops parsing. */
    bool csv_header_done;

    /** @brief Returns whether cached DN3 is at or above LINE_THRESHOLD. */
    bool lineDetected(const Robot_c &robot) const;

    /** @brief Applies the nominal controller behaviour for the current mode. */
    void runLineFollower(Robot_c &robot, unsigned long now);

    /**
     * @brief Exercise 1 test routine: 3-second ramp up and down, then stop.
     * @param robot Hardware-facing interface used to command the motors.
     * @param now Controller-cycle timestamp in milliseconds.
     * @note Non-blocking: it returns immediately on every call and keeps its
     * progress in call_count, so update() is never held up.
     */
    void motorStartStopTest(Robot_c &robot, unsigned long now);

    /**
     * @brief Exercise 2 test routine: estimate wheel speed from encoder counts.
     * @param robot Hardware-facing interface holding the encoder snapshot.
     * @param now Controller-cycle timestamp in milliseconds.
     * @note Non-blocking. Samples every SPEED_SAMPLE_INTERVAL calls, computes
     * counts per second from the change since the previous sample, and prints
     * one CSV record per sample. Runs for SAMPLES_PER_TRIAL samples, then
     * stops the motors and re-arms the button.
     */
    void measureWheelSpeed(Robot_c &robot, unsigned long now);

    /**
     * @brief Exercise 3 test routine: sweep the deadband from rest.
     * @param robot Hardware-facing interface holding the encoder snapshot.
     * @param now Controller-cycle timestamp in milliseconds.
     * @note Non-blocking. Follows the exercise's rest/run timeline: command the
     * motor off, wait for a verified rest, register the initial count, apply the
     * test command, then take N_s samples at a fixed interval. Every trial
     * starts from that rest, which is the whole point - a wheel that is already
     * turning needs less torque than one breaking away, so a continued-motion
     * measurement answers a different question and must not be mixed in.
     * A button press during a run stops the motors and ends the sweep, so a
     * misbehaving robot can be stopped without pulling power.
     */
    void testDeadband(Robot_c &robot, unsigned long now);

    /** @brief Returns the encoder count for whichever wheel the sweep drives. */
    int32_t deadbandWheelCount(Robot_c &robot);

    /** @brief Returns the other wheel's count, for evidence about the chassis. */
    int32_t deadbandOtherCount(Robot_c &robot);

    /** @brief Returns how many PWM magnitudes the ladder contains. */
    int deadbandLevelCount() const;

    /** @brief Returns the signed PWM magnitude for the current sweep level. */
    float deadbandPwm() const;

    /** @brief Sends the current sweep command to the driven wheel only. */
    void deadbandApplyCommand(Robot_c &robot);

    /**
     * @brief Exercise 4 test routine: sweep the top of the PWM range.
     * @param robot Hardware-facing interface holding the encoder snapshot.
     * @param now Controller-cycle timestamp in milliseconds.
     * @note Non-blocking. Each trial is cooling, then settling, then measuring.
     * The wheel does not have to stop between commands, but the command must be
     * held for the settling window before measuring starts, so that the average
     * describes a settled speed rather than a speed still on its way there.
     * Cooling intervals keep high commands short and intermittent, which is
     * what stops the winding overheating and altering the response being
     * measured. A button press during a measuring window ends the sweep.
     */
    void testSaturation(Robot_c &robot, unsigned long now);

    /**
     * @brief Exercise 5 test routine: re-measure the held-back commands.
     * @param robot Hardware-facing interface holding the encoder snapshot.
     * @param now Controller-cycle timestamp in milliseconds.
     * @note Identical protocol to testSaturation, walking GAIN_PWM_LEVEL_*. That
     * matters more than it looks: the whole point of these rows is to be
     * compared against data collected earlier, so any difference in settling,
     * sampling or timing would be indistinguishable from a modelling error.
     */
    void testGain(Robot_c &robot, unsigned long now);

    /** @brief Runs one settle-and-measure sweep step for the selected command list. */
    void sweepStep(Robot_c &robot, unsigned long now);

    /** @brief Returns how many commands the selected list contains. */
    int sweepLevelCount() const;

    /** @brief Returns the signed, clamped PWM magnitude for the current level. */
    float sweepPwm() const;

    /** @brief Sends the current sweep command to the wheel or wheel pair. */
    void saturationApplyCommand(Robot_c &robot);

    /** @brief Reports one measuring sample as one row per driven wheel.
     * @note Both counts are passed in rather than re-read, so the two rows of
     * an opposite-pair sample describe the same instant. */
    void saturationEmitRows(int32_t left, int32_t right, float delta_ms, unsigned long now);

    /** @brief Moves on to the next trial, level or repeat after a measuring window. */
    void saturationAdvance();

    /**
     * @brief Exercise 7 test routine: step the command and record the transient.
     * @param robot Hardware-facing interface holding the encoder snapshot.
     * @param now Controller-cycle timestamp in milliseconds.
     * @note Non-blocking. Each trial is: gap with the motors off, hold the first
     * command until the speed is steady, change the command once, keep the time
     * series through the transient, then hold the second command long enough to
     * measure where it settled.
     *
     * @note Samples are recorded on every controller cycle, so the reporting
     * interval is the same 10ms as the acquisition interval. That is the point:
     * at the 50ms used by the other exercises the whole transient would be a
     * handful of points. It also means the serial link is about half busy while
     * a trial runs, so the record carries only the columns the analysis needs -
     * anything extra would spend bandwidth that the step timing needs.
     */
    void testStepResponse(Robot_c &robot, unsigned long now);

    /** @brief Sends a command to the test wheel only.
     * @return true when the I2C write was acknowledged, so the caller knows
     * whether it still needs to retry. */
    bool stepApplyCommand(Robot_c &robot, float pwm);

    /** @brief Reports one row: the raw snapshot plus its trial labels. */
    void stepEmitRow(Robot_c &robot, unsigned long now);

    /** @brief Advances one rest cycle; returns true when the run may begin. */
    bool deadbandStepRest(Robot_c &robot);

    /** @brief Takes one speed sample and reports it as a CSV row. */
    void deadbandStepRun(Robot_c &robot, unsigned long now);

    /** @brief Moves on to the next trial, level or repeat after a completed run. */
    void deadbandAdvance();

    /**
     * @brief Sends the current controller and robot snapshot as one CSV record.
     * @note Controller.cpp writes these 14 columns, in order: timestamp_ms,
     * x, y, theta, left_pwm, right_pwm, left_count, right_count,
     * DN1, DN2, DN3, DN4, DN5, signal. These labels are not sent as a header.
     * x and y are in mm and theta is in radians, from the local Euler estimate,
     * not robot.pose from I2C. PWM values are cached requests, not measured output.
     * The timestamp is the controller-cycle start time; it does not guarantee
     * that every cached reading was successfully refreshed in that cycle.
     */
    void publishTelemetry(Robot_c &robot, RobotWifiAP_c &server, unsigned long timestamp_ms);
};

#endif
