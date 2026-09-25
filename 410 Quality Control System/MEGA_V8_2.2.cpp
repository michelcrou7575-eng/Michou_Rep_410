#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h> // strncpy_P - copying PROGMEM strings into RAM for the LCD dirty-cell fields
#include <string.h>       // strncmp/strncpy/strcpy
#include <stdio.h>        // snprintf

#define FW_VERSION "      V8.2.2"
#define FW_DATE "2026-09-01"

constexpr uint8_t PIR_PINS[5] = {A0, A1, A2, A3, A4};
constexpr uint8_t BANNER_POINT_A_PIN = A10;
constexpr uint8_t POT_BASELINE_PIN = A8;
constexpr uint8_t POT_REFERENCE_PIN = A9;
constexpr uint8_t BANNER_DIGITAL_PIN = 3;
constexpr uint8_t NSL32_PWM_PIN = 6;
constexpr uint8_t SCREEN_BUTTON_PIN = 11;
constexpr uint8_t WHITE_AUX_SW_PIN = 12;
constexpr uint8_t STATUS_LED_PIN = 13;
constexpr uint8_t LED_CORRECTION_UP_PIN = 40;
constexpr uint8_t LED_STANDBY_PIN = 44;
constexpr uint8_t LED_CORRECTION_DOWN_PIN = 48;

constexpr uint8_t TELEGRAM_LENGTH = 16;
constexpr uint8_t PROTOCOL_VERSION = 0x05; // DB20/DB30 DBB0 - confirmed against FB203
constexpr uint8_t PLC_COMMAND_TYPE = 0x01; // DB20 DBB1 - "01 = PLC command"
constexpr uint8_t MEGA_REPLY_TYPE = 0x02;  // DB30 DBB1 - "Expected 02"
constexpr uint32_t REPLY_DELAY_MS = 50UL;
constexpr uint32_t COMM_TIMEOUT_MS = 1000UL;
constexpr uint32_t RECEIVE_BYTE_TIMEOUT_MS = 20UL; // abort partial telegram if next byte doesn't arrive in time
constexpr uint32_t SLOW_LOOP_WARN_MS = 10UL;       // half of RECEIVE_BYTE_TIMEOUT_MS - flag any single loop() pass eating meaningfully into that budget
constexpr uint32_t SENSOR_SELECTION_MESSAGE_MS = 2000UL;

constexpr uint8_t CONTROL_MACHINE_RUN = 0x01;
constexpr uint8_t CONTROL_AUTO_MODE = 0x02;
constexpr uint8_t CONTROL_RESET_DIAGNOSTICS = 0x04;
constexpr uint8_t CONTROL_PIR_ENABLE = 0x08;
constexpr uint8_t CONTROL_LCD_WAKE = 0x10;
constexpr uint8_t CONTROL_REMOTE_RESET = 0x20; // DB20 DBB3 bit 5 (was Reserved) - PLC-commanded WDT reset
constexpr uint8_t CONTROL_SELECT_A = 0x40;     // DB20 DBB3 bit 6 - Select A
constexpr uint8_t CONTROL_SELECT_B = 0x80;     // DB20 DBB3 bit 7 - Select B

// AGC_Command values (DB20 DBB4). PwmUp/PwmDown are reserved but
// currently stubbed to a no-op - see executeNewPlcCommand(). FB610
// always uses AbsolutePwm.
enum class AgcCommand : uint8_t
{
    Hold = 0,
    PwmUp = 1,
    PwmDown = 2,
    AbsolutePwm = 3
};

constexpr uint8_t STATUS_READY = 0x01;
constexpr uint8_t STATUS_AUTO_RECEIVED = 0x02;
constexpr uint8_t STATUS_RUN_RECEIVED = 0x04;
constexpr uint8_t STATUS_NSL_OUTPUT_ACTIVE = 0x08;
constexpr uint8_t STATUS_LCD_AWAKE = 0x10;
constexpr uint8_t STATUS_PIR_PROCESSING_ACTIVE = 0x20;
constexpr uint8_t STATUS_PROTOCOL_ERROR = 0x40;
constexpr uint8_t STATUS_HARDWARE_FAULT = 0x80;

constexpr uint8_t CAPABILITY_LCD = 0x01;
constexpr uint8_t CAPABILITY_PIR = 0x02;
constexpr uint8_t CAPABILITY_BANNER_AGC = 0x04;
constexpr uint8_t CAPABILITY_POINT_A = 0x08;
constexpr uint8_t MEGA_CAPABILITY_BYTE =
    CAPABILITY_LCD | CAPABILITY_PIR | CAPABILITY_BANNER_AGC | CAPABILITY_POINT_A;

uint8_t receiveTelegram[TELEGRAM_LENGTH] = {};
uint8_t transmitTelegram[TELEGRAM_LENGTH] = {};
uint8_t receiveIndex = 0;
bool receivingTelegram = false;
uint32_t lastReceiveByteMs = 0;
bool replyPending = false;
uint32_t replyDueMs = 0;
uint8_t replySequence = 0;
bool plcTelegramValid = false;
bool protocolErrorActive = false;
bool hardwareFaultActive = false;
bool plcMachineRun = false;
bool plcAutoMode = false;
bool plcPirEnable = false;
bool plcSelectA = false;
bool plcSelectB = false;
bool previousResetDiagnosticsBit = false;
bool previousLcdWakeBit = false;
bool previousRemoteResetBit = false;
// Latched, never cleared once set - the WDT reset that follows wipes
// RAM anyway, so there's no cold-boot state to worry about restoring.
// Latching (vs. re-checking the live bit every loop) means a single
// pulsed Control_Byte bit from the PLC is enough to guarantee the
// reset actually happens even if the PLC drops the bit again before
// the ~1s WDTO_1S timeout elapses.
bool remoteResetRequested = false;
uint32_t lastValidTelegramMs = 0;
uint16_t communicationErrorCount = 0;
AgcCommand receivedCommand = AgcCommand::Hold;
AgcCommand lastExecutedCommand = AgcCommand::Hold;
uint8_t receivedPwmTarget = 0;   // DB20 DBB5 (field name "Absolute_Index" on the PLC side) - raw 0-255 PWM from FB610's AGC_Out, valid when receivedCommand == AbsolutePwm
uint8_t actualVelocity = 0;      // NEW Build 17 - DB20 v5.3 DBB11 (field name "Actual_Velocity"). Already EMA'd PLC-side by FC120 v1.3 (separate filter from FB610's AGC loop) - confirmed on the real machine this never exceeds ~200, so no further scaling/clamping needed here
uint8_t lastLoggedPwmTarget = 0; // last AGC_Out value we printed - lets us log only when the PLC's commanded target actually changes telegram-to-telegram
uint8_t lastExecutedSequence = 0xFF;
uint8_t lastVelocityForSeamLength = 0;

int32_t expWindowOut = 0;
float seamLengthMm = 0.0F;
constexpr uint8_t SEAM_LENGTH_VELOCITY_DELTA_THRESHOLD = 5U; // noticeable velocity move before the display estimate is nudged

constexpr float ENCODER_COUNTS_PER_METER = 4000.0;   // Encoder 300mm Circumference / 1200 PPR = 4000 counts/meter.
constexpr float ENCODER_TUBING_PULLEY_RATIO = 1.025; // The Draw cylinder pulley that drives the Encoder Pulley is
// Smaller than the tubing Cylinder, so the encoder sees 0.975x (2.5% less) of the actual tube travel.

// Local serial terminal override (USB Serial, independent of the PLC
// Serial1 link). While active, PLC AGC commands are ignored entirely -
// this is meant for bench use without a PLC connected.
//   S/s = override ON, X/x = override OFF, A/a = PWM up, Z/z = PWM down
bool manualOverrideActive = false;

constexpr uint8_t LCD_ADDRESS = 0x3F; // PCF8574AT backpack 0x3F
constexpr uint8_t LCD_COLS = 20;
constexpr uint8_t LCD_ROWS = 4;
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLS, LCD_ROWS);

// No frame buffer, no chunked flush state machine needed here - unlike
// the older full-frame display drivers this firmware used to work
// around in prior revisions, LiquidCrystal_I2C writes go out
// character-by-character over I2C as
// individual short transactions. drawFieldIfChanged() below still only
// rewrites cells that actually changed, so a normal frame touches at
// most a handful of characters - cheap enough that no throttling
// machinery is needed to keep it from stalling Serial1 telegram timing.

enum class DisplayScreen : uint8_t
{
    Numerical = 0,
    Drawing = 1,
    DiagnosticsA = 2, // NEW Build 18 - was one Diagnostics screen; split A/B, 8 fields didn't fit in 3 content rows on a 20x4 char LCD (comms/classify half: SEQ, ERR, TGT, A5, PIR, CMD)
    DiagnosticsB = 3, // NEW Build 18 - AGC/pot half: Rex, POS, SET
    PulseWidth = 4    // live Banner pulse width/gap + Exp_Window_Out
    ,
    SelectA = 5 // PLC-selected seam sensor A
    ,
    SelectB = 6 // PLC-selected seam sensor B
};
constexpr uint8_t DISPLAY_SCREEN_COUNT = 7;

DisplayScreen currentScreen = DisplayScreen::Numerical;
bool sensorSelectionMessageActive = false;
DisplayScreen sensorSelectionMessageScreen = DisplayScreen::Numerical;
uint32_t sensorSelectionMessageDueMs = 0;
bool lcdAvailable = false;
bool lcdSleeping = false;
bool forceFullRedraw = true; // moved up here (from near drawFieldIfChanged below) so wakeLCD() can set it too
constexpr uint32_t LCD_UPDATE_MS = 300UL;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 40UL;
constexpr uint32_t LCD_SLEEP_DELAY_MS = 600000UL; // 10 minutes - auto-sleep after this long with no button activity
uint32_t previousDisplayMs = 0;
uint32_t lastMeaningfulActivityMs = 0;
bool previousRawButton = HIGH;
bool stableButton = HIGH;
uint32_t buttonChangeMs = 0;

bool suppressNextAdvance = false;

// NSL-32 PWM state. This is now the ONLY value that represents what's
// being driven - no lookup table, no index. FB610 (PLC) computes the
// target byte and sends it directly (AGC_Command=3); the Mega's job is
// just to apply it, confirm it back, and show it.
constexpr uint8_t STARTUP_PWM = 33U; // idle PWM on boot
uint8_t currentPWM = STARTUP_PWM;

// The one and only path that writes the NSL-32 PWM pin - called from
// both the PLC (AbsolutePwm command) and the serial override (S/A/Z).

float calculateSeamLengthMm()
{
    return static_cast<float>(expWindowOut) * 1000.0F / ENCODER_COUNTS_PER_METER * ENCODER_TUBING_PULLEY_RATIO;
}

void updateSeamLengthMmWithVelocity()
{
    const float rawSeamLengthMm = calculateSeamLengthMm();
    const int16_t velocityDelta = static_cast<int16_t>(actualVelocity) - static_cast<int16_t>(lastVelocityForSeamLength);
    const uint16_t absVelocityDelta = velocityDelta < 0 ? static_cast<uint16_t>(-velocityDelta)
                                                        : static_cast<uint16_t>(velocityDelta);

    // Keep the PLC's raw Exp_Window_Out value as the authoritative floor,
    // but nudge the displayed seam length when the measured velocity makes a
    // noticeable step change. This avoids the display looking stale while
    // still suppressing tiny jitter from the filtered velocity byte.
    if (absVelocityDelta >= SEAM_LENGTH_VELOCITY_DELTA_THRESHOLD)
    {
        const float velocityAdjustmentMm = static_cast<float>(velocityDelta) * 0.35F;
        seamLengthMm = rawSeamLengthMm + velocityAdjustmentMm;
    }
    else
    {
        seamLengthMm = rawSeamLengthMm;
    }

    lastVelocityForSeamLength = actualVelocity;
}

void setPwmOutput(uint8_t newPwm)
{
    if (newPwm != currentPWM)
    {
        Serial.print(F("[PWM] "));
        Serial.print(currentPWM);
        Serial.print(F(" -> "));
        Serial.println(newPwm);
    }
    currentPWM = newPwm;
    analogWrite(NSL32_PWM_PIN, currentPWM);
    seamLengthMm = calculateSeamLengthMm();
}

//======================================================================
// NSL-32 resistance <-> PWM lookup table (diagnostic/display only -
// never used to drive anything).
//
// Bench-calibrated against the old (pre-decoupling) circuit, where Point
// B / A6 was DC-coupled through the NSL-32 branch and its byte value
// tracked NSL-32 resistance directly. Since the 4.7uF now decouples the
// NSL-32 from Point B at DC, A6 no longer carries any NSL-32 resistance
// information - it reads pure operator-pot wiper position instead.
//
// So this is now interpolated from currentPWM (what's actually being
// driven, via analogWrite) rather than an A6 reading: an OPEN-LOOP
// *expected* resistance for the commanded PWM, not a closed-loop
// measurement. Shown on the LCD and mirrored into the reply telegram's
// NSL_Resistance field for the PLC/HMI's benefit. It does not feed back
// into any control decision - PLC/FM350 (via FB610) still owns that
// entirely.
//======================================================================
struct ResistancePwmPoint
{
    float resistanceOhms;
    uint8_t pwmValue;
};

constexpr ResistancePwmPoint RES_PWM_TABLE[] =
    {
        {2000.0F, 255}, {4000.0F, 141}, {6000.0F, 101}, {8000.0F, 79}, {10000.0F, 65}, {12000.0F, 56}, {14000.0F, 49}, {16000.0F, 43}, {18000.0F, 38}, {20000.0F, 35}, {22000.0F, 31}, {24000.0F, 28}, {26000.0F, 26}, {28000.0F, 24}, {30000.0F, 21}, {32000.0F, 19}, {34000.0F, 17}, {36000.0F, 15}, {38000.0F, 14}, {40000.0F, 12}, {42000.0F, 10}, {44000.0F, 8}, {46000.0F, 4}};

constexpr size_t RES_PWM_TABLE_SIZE = sizeof(RES_PWM_TABLE) / sizeof(RES_PWM_TABLE[0]);

// Idle / no-target reference point (bench-measured resting condition)
constexpr float IDLE_RESISTANCE_OHMS = 11600.0F;

// Estimate effective series resistance from a PWM-scaled Point B reading.
// Table is monotonic decreasing (PWM falls as resistance rises); clamps
// to the measured bench range outside the calibrated span.
//
// Interpolation is done in CONDUCTANCE (1/R), not resistance. The 47K
// parallel leg means Reff = (Rnsl*47k)/(Rnsl+47k), i.e. conductances add:
// 1/Reff = 1/Rnsl + 1/47000. That keeps the underlying NSL-32 drive curve's
// shape intact in conductance space (const R^2 = 0.99 vs PWM, measured
// against RES_PWM_TABLE), whereas raw resistance is badly nonlinear
// (R^2 = 0.59) because Reff asymptotes toward 47k as Rnsl grows - lerping
// resistance directly crushes accuracy exactly where the table is sparse
// (low-PWM / high-R end). Same table, just interpolate 1/r instead of r.
float estimateSeriesResistanceOhms(uint8_t pwmValue)
{
    if (pwmValue >= RES_PWM_TABLE[0].pwmValue)
        return RES_PWM_TABLE[0].resistanceOhms;

    if (pwmValue <= RES_PWM_TABLE[RES_PWM_TABLE_SIZE - 1].pwmValue)
        return RES_PWM_TABLE[RES_PWM_TABLE_SIZE - 1].resistanceOhms;

    for (size_t i = 0; i + 1 < RES_PWM_TABLE_SIZE; ++i)
    {
        const uint8_t p0 = RES_PWM_TABLE[i].pwmValue;
        const uint8_t p1 = RES_PWM_TABLE[i + 1].pwmValue;
        if (pwmValue <= p0 && pwmValue >= p1)
        {
            const float g0 = 1.0F / RES_PWM_TABLE[i].resistanceOhms;
            const float g1 = 1.0F / RES_PWM_TABLE[i + 1].resistanceOhms;
            const float frac = static_cast<float>(p0 - pwmValue) / static_cast<float>(p0 - p1);
            const float g = g0 + frac * (g1 - g0);
            return 1.0F / g;
        }
    }
    return IDLE_RESISTANCE_OHMS; // fallback, should not reach here
}

constexpr float ADC_REFERENCE_VOLTAGE = 5.0F;
// Circuit correction: 4.7uF now decouples the wiper from the 22K leg, so
// A8 (POT_BASELINE_PIN) rides on Point A's DC bias.
// Bargraph span follows the bench-observed swing of
// that bias (0.25-1.65V full pot travel).
//
// NEW Build 16 - POT_REFERENCE_PIN (A9) taps Point C (Banner analog GND /
// low pot side), which floats above digital GND by a confirmed nominal
// +0.250V bias, with some optical-dependent variation on top (per bench
// notes). Rather than assume that bias is fixed, A9 is sampled and
// filtered live each cycle and subtracted from the A8/Point A reading
// (see potBaselineCorrected in updateAnalogMeasurements()), so the
// bargraph tracks true pot wiper travel referenced to Point C instead
// of drifting digital GND. The MIN/MAX bounds below are re-referenced
// to the corrected (offset-removed) signal accordingly - confirmed by
// Michou at 0.25V-1.65V.

constexpr float POT_REFERENCE_BIAS_VOLTAGE = 0.250F; // nominal Point C bias above digital GND

constexpr float POT_BARGRAPH_MIN_VOLTAGE = 0.25F;
constexpr float POT_BARGRAPH_MAX_VOLTAGE = 1.65F;
constexpr uint16_t POT_BARGRAPH_MIN_ADC = static_cast<uint16_t>(POT_BARGRAPH_MIN_VOLTAGE / ADC_REFERENCE_VOLTAGE * 1023.0F + 0.5F);
constexpr uint16_t POT_BARGRAPH_MAX_ADC = static_cast<uint16_t>(POT_BARGRAPH_MAX_VOLTAGE / ADC_REFERENCE_VOLTAGE * 1023.0F + 0.5F);
constexpr uint32_t ANALOG_UPDATE_MS = 10UL;
uint16_t potBaselineRaw = 0;
uint16_t potBaselineFiltered = 0;
uint16_t potReferenceRaw = 0;      // NEW Build 16 - A9/Point C, raw
uint16_t potReferenceFiltered = 0; // NEW Build 16 - A9/Point C, EMA-filtered
uint16_t potBaselineCorrected = 0; // NEW Build 16 - potBaselineFiltered minus live Point C offset, clamped >= 0
uint16_t pointAFiltered = 0;
uint32_t previousAnalogMs = 0;
uint8_t pirMask = 0;

// pointBPwmValue: A6/Point B was only ever meant to read pot position and
// movement - see updatePotMovedDetection() below, which owns the actual
// settled/moved determination. estimatedSeriesResistanceOhms is a
// separate, unrelated open-loop figure from currentPWM - see
// estimateSeriesResistanceOhms() above.
uint8_t pointBPwmValue = 0;
float estimatedSeriesResistanceOhms = 0.0F;

//======================================================================
// Pot-movement detection
//
// Point B (A6) is sampled once per seam pulse, a fixed delay after the
// pulse's falling edge - late enough that any transient the pulse
// itself couples onto A6 has settled, early enough it's well before
// the next pulse can arrive. Each quiet-window sample is compared to
// the previous one: a real seam returns A6 to the same resting level,
// so back-to-back quiet samples only differ if the operator actually
// moved the pot in between. Tie-in to pulse edges (rather than a fixed
// polling interval) means the comparison is naturally immune to any
// pulse-coupled noise on A6, since sampling never happens near an edge.
//======================================================================
constexpr uint16_t POT_MOVED_DEADBAND = 4;     // ADC counts - tune on the bench
constexpr uint32_t POT_SETTLE_DELAY_MS = 15UL; // wait after pulse falling edge

// Decimation for jobs with multiple seams per revolution (e.g. two seams
// 180deg apart). Sampling every single pulse compares seam-to-seam against
// its opposed twin, which can differ enough on its own to false-trigger
// Pot_Moved. Only arm the quiet-window sample every SEAM_PULSE_DECIMATION-th
// falling edge, so consecutive compared samples are spaced further apart
// (ideally always the same physical seam pass-to-pass). Set to 1 to sample
// every pulse (single-seam jobs).
constexpr uint8_t SEAM_PULSE_DECIMATION = 4U;
uint8_t seamPulseCounter = 0;

bool previousBannerPulseState = false;
bool potSampleArmed = false;
uint32_t potSampleDueMs = 0;
uint16_t potQuietSample = 0;
uint16_t previousPotQuietSample = 0;
bool potQuietSampleValid = false;
bool potMoved = false;
bool potSettled = false; // inverse of potMoved, valid once potQuietSampleValid - repurposed from the old localTargetIndicated flag, which wrongly treated A6 as a resistance/target proxy

//======================================================================
// NEW Build 10 - Banner pulse width/gap measurement (interrupt-driven)
//
// Bench request after shortening the Banner pulse produced a hardware
// double-trigger (confirmed via PLC-side XPS capture of DB888: a
// normal-width real pulse immediately followed, ~50ms later, by a very
// short spurious second pulse). This needs to be visible directly on
// the bench, in real time, without another PLC-side capture cycle.
//
// Deliberately interrupt-driven (attachInterrupt on BANNER_DIGITAL_PIN,
// CHANGE), not the polled digitalRead() updatePotMovedDetection() already
// uses - polling only samples once per loop() pass, and a pulse short
// enough to hide inside a single pass is exactly the failure mode being
// investigated. bannerEdgeISR() only timestamps edges and does minimal
// integer math - no Serial, no display calls, nothing blocking.
//
// All multi-byte fields the ISR writes are volatile and are only ever
// read on the loop() side inside a noInterrupts()/interrupts() pair -
// a 32-bit read/write is not atomic on an 8-bit AVR, and a torn read
// here would show a bogus pulse width, which would be actively
// misleading for exactly the kind of glitch this exists to catch.
//======================================================================
volatile uint32_t lastPulseWidthUs = 0; // most recent HIGH-time (rising -> falling)
volatile uint32_t lastPulseGapUs = 0;   // most recent LOW-time before that pulse (previous falling -> this rising)
volatile uint32_t minPulseWidthUs = 0xFFFFFFFFUL;
volatile uint32_t maxPulseWidthUs = 0;
volatile uint16_t pulseWidthSampleCount = 0; // lets the display distinguish "no data yet" from a real 0
volatile uint32_t isrLastRisingUs = 0;
volatile uint32_t isrLastFallingUs = 0;
volatile bool isrHaveRisingEdge = false;
volatile bool isrHaveFallingEdge = false;

// Debounced tube/seam pulse counter (Tube/Min, Screen 2). Confirmed
// hardware double-trigger: every real Banner pulse is followed ~50ms
// later by a short spurious second pulse on the same line - counting
// raw falling edges (pulseWidthSampleCount above) reads ~2x actual
// tube rate. Fix: only accept a falling edge as a real tube pulse if
// at least TUBE_PULSE_DEBOUNCE_MS has elapsed since the last accepted
// one - rejects the ~50ms follower without needing to know its exact
// width. 100ms confirmed by Michou against the real machine's max
// production seam rate; retune here if the line ever runs faster.
constexpr uint32_t TUBE_PULSE_DEBOUNCE_MS = 100UL;
volatile uint32_t tubePulseCount = 0;
volatile uint32_t lastAcceptedTubePulseMs = 0;

// Rolling history of accepted-pulse gaps (ms) for the Tube/Min period
// average (v8.2.2 - replaces the old fixed 2000ms count-window). Each
// entry is a precise ISR timestamp delta between consecutive accepted
// pulses - no window-length quantization the way a coarse pulse-count-
// over-a-timer approach had. Small history size on purpose: this is
// already far less noisy per-sample than the old method, so it doesn't
// need many samples to smooth out real mechanical jitter, and a small
// history means the average refreshes on nearly every accepted pulse
// instead of waiting a fixed window - faster AND smoother, no tradeoff
// between the two the way widening a count-window would have been.
// ISR side only ever writes one array slot + bumps an index - same cost
// class as the min/max pulse-width tracking already below, so this adds
// negligible interrupt latency ahead of the upcoming PIR work.
constexpr uint8_t TUBE_PULSE_GAP_HISTORY = 4;
volatile uint32_t tubePulseGapMs[TUBE_PULSE_GAP_HISTORY] = {0};
volatile uint8_t tubePulseGapIndex = 0;
volatile uint8_t tubePulseGapFilled = 0;

// Debounced pulse-counting half of the Tube/Min blend on the Drawing
// screen. Averages the last TUBE_PULSE_GAP_HISTORY accepted-pulse gaps
// (already double-trigger-filtered by the ISR's debounce above) and
// converts the average period into a tubes/min rate. Returns 0.0F until
// at least one gap has been recorded since boot, same "no data yet"
// convention as before.
float readTubePerMinuteFromPulses()
{
    uint32_t gaps[TUBE_PULSE_GAP_HISTORY];
    uint8_t filled;

    noInterrupts();
    filled = tubePulseGapFilled;

    for (uint8_t i = 0; i < filled; ++i)
        gaps[i] = tubePulseGapMs[i];
    interrupts();

    if (filled == 0)
        return 0.0F;

    uint32_t sumMs = 0;
    for (uint8_t i = 0; i < filled; ++i)
        sumMs += gaps[i];

    const float avgGapMs = static_cast<float>(sumMs) / static_cast<float>(filled);

    return (avgGapMs > 0.0F) ? (60000.0F / avgGapMs) : 0.0F;
}

void bannerEdgeISR()
{
    const uint32_t now = micros();

    if (digitalRead(BANNER_DIGITAL_PIN) == HIGH)
    {
        // Rising edge - a new pulse is starting.
        if (isrHaveFallingEdge)
            lastPulseGapUs = now - isrLastFallingUs;
        isrLastRisingUs = now;
        isrHaveRisingEdge = true;
    }
    else
    {
        // Falling edge - the pulse that just started is now complete.
        if (isrHaveRisingEdge)
        {
            const uint32_t width = now - isrLastRisingUs;
            lastPulseWidthUs = width;

            if (width < minPulseWidthUs)
                minPulseWidthUs = width;

            if (width > maxPulseWidthUs)
                maxPulseWidthUs = width;

            if (pulseWidthSampleCount < 0xFFFFU)
                ++pulseWidthSampleCount;

            // Debounced tube-pulse gap recording (Tube/Min) - reject the
            // ~50ms double-trigger follower pulse. millis() during CHANGE
            // ISR is the same pattern already used elsewhere in this ISR's
            // ecosystem (micros() for width, millis() consumed loop-side);
            // an ISR-context millis() read is standard and safe here.
            const uint32_t nowMs = millis();

            if (nowMs - lastAcceptedTubePulseMs >= TUBE_PULSE_DEBOUNCE_MS)
            {
                if (lastAcceptedTubePulseMs != 0)
                {
                    // Skip recording a gap for the very first accepted
                    // pulse since boot - there's no prior accepted pulse
                    // to measure from yet.
                    tubePulseGapMs[tubePulseGapIndex] = nowMs - lastAcceptedTubePulseMs;
                    tubePulseGapIndex = (tubePulseGapIndex + 1) % TUBE_PULSE_GAP_HISTORY;
                    if (tubePulseGapFilled < TUBE_PULSE_GAP_HISTORY)
                        ++tubePulseGapFilled;
                }
                lastAcceptedTubePulseMs = nowMs;
                ++tubePulseCount;
            }
        }
        isrLastFallingUs = now;
        isrHaveFallingEdge = true;
    }
}

// Resets the rolling min/max/sample-count (called on entry to the
// PulseWidth screen) - not the last-width/last-gap values, which stay
// live regardless. Brief critical section: this only touches a handful
// of 16/32-bit variables, not worth the interrupt-latency risk of doing
// it any other way.
void resetPulseWidthStats()
{
    noInterrupts();
    minPulseWidthUs = 0xFFFFFFFFUL;
    maxPulseWidthUs = 0;
    pulseWidthSampleCount = 0;
    interrupts();
}

uint16_t readAveragedAnalog(uint8_t pin)
{
    uint32_t sum = 0;

    for (uint8_t i = 0; i < 8; ++i)
        sum += analogRead(pin);
    return static_cast<uint16_t>(sum / 8UL);
}

void updatePotMovedDetection()
{
    const bool currentPulseState = digitalRead(BANNER_DIGITAL_PIN) == HIGH;

    // Falling edge of a seam pulse arms a delayed quiet-window sample,
    // but only every SEAM_PULSE_DECIMATION-th pulse - see comment above.
    if (previousBannerPulseState && !currentPulseState)
    {
        ++seamPulseCounter;

        if (seamPulseCounter >= SEAM_PULSE_DECIMATION)
        {
            seamPulseCounter = 0;
            potSampleArmed = true;
            potSampleDueMs = millis() + POT_SETTLE_DELAY_MS;
        }
    }
    previousBannerPulseState = currentPulseState;

    if (potSampleArmed && static_cast<int32_t>(millis() - potSampleDueMs) >= 0)
    {
        potSampleArmed = false;
        previousPotQuietSample = potQuietSample;
        potQuietSample = readAveragedAnalog(POT_BASELINE_PIN); // fast average, not the slow display filter

        if (potQuietSampleValid)
        {
            const int16_t delta = static_cast<int16_t>(potQuietSample) -
                                  static_cast<int16_t>(previousPotQuietSample);
            const uint16_t absDelta = delta < 0 ? static_cast<uint16_t>(-delta)
                                                : static_cast<uint16_t>(delta);
            potMoved = absDelta > POT_MOVED_DEADBAND;
            potSettled = !potMoved;
        }
        potQuietSampleValid = true;
    }
}

float adcToVoltage(uint16_t adc)
{
    return static_cast<float>(adc) * ADC_REFERENCE_VOLTAGE / 1023.0F;
}

uint8_t scaleAdcToByte(uint16_t adc)
{
    adc = constrain(adc, 0U, 1023U);
    return static_cast<uint8_t>((static_cast<uint32_t>(adc) * 255UL + 511UL) / 1023UL);
}

void updateAnalogMeasurements()
{
    const uint32_t now = millis();

    if (now - previousAnalogMs < ANALOG_UPDATE_MS)
        return;
    previousAnalogMs = now;

    potBaselineRaw = readAveragedAnalog(POT_BASELINE_PIN);
    const uint16_t pointARaw = readAveragedAnalog(BANNER_POINT_A_PIN);
    potReferenceRaw = readAveragedAnalog(POT_REFERENCE_PIN); // NEW Build 16 - Point C live sample

    if (potBaselineFiltered == 0)
        potBaselineFiltered = potBaselineRaw;
    else
        potBaselineFiltered = static_cast<uint16_t>((static_cast<uint32_t>(potBaselineFiltered) * 15UL + potBaselineRaw) / 16UL);

    if (pointAFiltered == 0)
        pointAFiltered = pointARaw;
    else
        pointAFiltered = static_cast<uint16_t>((static_cast<uint32_t>(pointAFiltered) * 7UL + pointARaw) / 8UL);

    // NEW Build 16 - same slow EMA as Point A (Point C is a DC reference,
    // not expected to move quickly; matching Point A's filter constant
    // keeps both tracked at comparable settling speed).
    if (potReferenceFiltered == 0)
        potReferenceFiltered = potReferenceRaw;
    else
        potReferenceFiltered = static_cast<uint16_t>((static_cast<uint32_t>(potReferenceFiltered) * 7UL + potReferenceRaw) / 8UL);

    // NEW Build 16 - subtract Point C's live offset from the Point A/A8
    // reading so the bargraph reflects true pot wiper travel rather than
    // digital-GND-referenced drift. Clamped so a momentary crossover
    // (e.g. at power-up before both filters have settled) can't underflow
    // the unsigned result.
    potBaselineCorrected = (potBaselineFiltered > potReferenceFiltered)
                               ? (potBaselineFiltered - potReferenceFiltered)
                               : 0U;

    // pointBPwmValue: still a valid diagnostic of Point B / A6 itself
    // (pot wiper position, post-decoupling) - used for the POS readout
    // and pot-moved detection, NOT for resistance anymore.
    pointBPwmValue = scaleAdcToByte(potBaselineFiltered);
    // estimatedSeriesResistanceOhms: open-loop expected NSL-32 resistance
    // for the currently-driven PWM (currentPWM), interpolated from the
    // bench LUT. A6 no longer carries NSL-32 branch info post-decoupling,
    // so this is no longer a closed-loop measurement - see header comment
    // above RES_PWM_TABLE.
    estimatedSeriesResistanceOhms = estimateSeriesResistanceOhms(currentPWM);
}

void readPIRInputs()
{
    if (!plcPirEnable)
    {
        pirMask = 0;
        return;
    }

    uint8_t newMask = 0;

    for (uint8_t i = 0; i < 5; ++i)

        if (digitalRead(PIR_PINS[i]) == HIGH)
            newMask |= static_cast<uint8_t>(1U << i);
    pirMask = newMask;
}

void wakeLCD()
{
    if (!lcdAvailable)
        return;
    lastMeaningfulActivityMs = millis();

    if (lcdSleeping)
    {
        lcdSleeping = false;
        lcd.backlight();        // character LCD's sleep analog - PCF8574 backpack still drives the panel, backlight is what actually goes dark
        forceFullRedraw = true; // safety net - guarantees synced content after the sleep/wake toggle
    }
}

void noteMeaningfulActivity() { wakeLCD(); }

// Only ever called with lastExecutedCommand, which is only ever
// Hold/PwmUp/PwmDown (direction is derived, never passed through) -
// no AbsolutePwm case needed here.
const __FlashStringHelper *commandText(AgcCommand command)
{
    switch (command)
    {
    case AgcCommand::PwmUp:
        return F("PWM UP");
    case AgcCommand::PwmDown:
        return F("PWM DN");
    default:
        return F("HOLD");
    }
}

void updateCorrectionLEDs()
{
    digitalWrite(LED_CORRECTION_UP_PIN, lastExecutedCommand == AgcCommand::PwmUp ? HIGH : LOW);
    digitalWrite(LED_CORRECTION_DOWN_PIN, lastExecutedCommand == AgcCommand::PwmDown ? HIGH : LOW);
    digitalWrite(LED_STANDBY_PIN, lastExecutedCommand == AgcCommand::Hold ? HIGH : LOW);
}

void executeNewPlcCommand(uint8_t sequence)
{
    if (sequence == lastExecutedSequence)
        return;
    lastExecutedSequence = sequence;

    if (manualOverrideActive)
    {
        // Serial terminal owns the actuator right now - ignore this PLC
        // command entirely (still ACKed via the reply telegram, just not
        // acted on) so the two control sources can't fight each other.
        if (receivedCommand == AgcCommand::AbsolutePwm && receivedPwmTarget != currentPWM)
        {
            Serial.print(F("[SUPPRESSED] AGC_Out="));
            Serial.print(receivedPwmTarget);
            Serial.println(F(" ignored - manual override active"));
        }
        return;
    }

    lastExecutedCommand = AgcCommand::Hold;

    if (!plcAutoMode || !plcMachineRun)
    {
        if (receivedCommand == AgcCommand::AbsolutePwm && receivedPwmTarget != currentPWM)
        {
            Serial.print(F("[SUPPRESSED] AGC_Out="));
            Serial.print(receivedPwmTarget);
            Serial.print(F(" ignored - "));
            Serial.println(!plcAutoMode ? F("Auto not set") : F("Machine_Run not set"));
        }
        updateCorrectionLEDs();
        return;
    }

    switch (receivedCommand)
    {
    case AgcCommand::AbsolutePwm:
    {
        // receivedPwmTarget is FB610's AGC_Out: a raw 0-255 PWM byte.
        // Direction (for the correction LEDs and the Last_Command
        // confirmation byte) is derived from comparing it to what's
        // currently driven - FB610 already did the AGC thinking on the
        // PLC side, the Mega just applies + confirms.
        const uint8_t targetPwm = receivedPwmTarget;

        if (targetPwm > currentPWM)
            lastExecutedCommand = AgcCommand::PwmUp;

        else if (targetPwm < currentPWM)
            lastExecutedCommand = AgcCommand::PwmDown;

        else
            lastExecutedCommand = AgcCommand::Hold; // not moving
        setPwmOutput(targetPwm);
        noteMeaningfulActivity();
        break;
    }

    case AgcCommand::PwmUp:
    case AgcCommand::PwmDown:
        // Reserved but not implemented - FB610 always drives via
        // AbsolutePwm, so these never arrive in normal operation.
        // Treated as a no-op (Hold) rather than removed from the
        // protocol, in case step-based control is wanted again later.
        lastExecutedCommand = AgcCommand::Hold;
        break;

    default:
        lastExecutedCommand = AgcCommand::Hold;
        break;
    }

    updateCorrectionLEDs();
}

//======================================================================
// Dirty-cell LCD redraw. The previous dirty-rect display system existed
// to work around a previous display driver's per-frame rasterization cost.
// A character LCD has no rasterization
// step at all - lcd.print() just shifts bytes out over I2C to the
// PCF8574 backpack - so the only remaining reason to track "did this
// field actually change" is to avoid pointless I2C traffic and visible
// flicker, not to avoid a CPU stall. Same self-erase requirement as
// before: callers must pad newText to a FIXED width so a shorter
// replacement doesn't leave stale characters from a longer previous
// string sitting past the end of the reprint.
// forceFullRedraw (declared above, near lcdSleeping) handles
// boot/screen-switch/wake (lcd.clear() + every field drawn once);

//======================================================================

// Reprints text at (col,row) only if it differs from what was last
// drawn there (or forceFullRedraw is set). previous must point at a
// persistent buffer of at least bufSize bytes.
void drawFieldIfChanged(uint8_t col, uint8_t row, char *previous, uint8_t bufSize, const char *newText)
{
    if (!forceFullRedraw && strncmp(previous, newText, bufSize) == 0)
        return;
    lcd.setCursor(col, row);
    lcd.print(newText);
    strncpy(previous, newText, bufSize - 1);
    previous[bufSize - 1] = '\0';
}

char prevHeaderAutoMan[5] = "";
char prevHeaderRunStop[5] = "";
char prevHeaderCom[7] = "";
char prevHeaderVelocity[8] = "";     // Actual Velocity, shown in the persistent header row on every screen (see drawHeader())
char prevHeaderScreen[3] = "";       // screen number at the far-right end of the header row
char prevHeaderOverrideText[2] = ""; // '*' = manual override ON, ' ' = OFF - single char, character-LCD equivalent of the old filled/outlined pixel box

constexpr const char *OHM_UNIT = "Ohm";

// Persistent status row (row 0), drawn on every screen. A 20x4 char LCD
// has 4 physical rows total .

void drawHeader() // Screen 1,2,3,4,5 Top Line
{
    drawFieldIfChanged(0, 0, prevHeaderAutoMan, sizeof(prevHeaderAutoMan), plcAutoMode ? "AUTO" : "MAN ");
    drawFieldIfChanged(6, 0, prevHeaderRunStop, sizeof(prevHeaderRunStop), plcMachineRun ? "RUN " : "STOP");

    {
        char velHeaderBuf[8];
        snprintf(velHeaderBuf, sizeof(velHeaderBuf), "%3u m/m", actualVelocity);
        drawFieldIfChanged(13, 0, prevHeaderVelocity, sizeof(prevHeaderVelocity), velHeaderBuf);
    }
    {
        char screenLabel[3];
        snprintf(screenLabel, sizeof(screenLabel), "%d", static_cast<int>(currentScreen) + 1);
        drawFieldIfChanged(19, 3, prevHeaderScreen, sizeof(prevHeaderScreen), screenLabel);
    }
}

char prevDrawSeamMm[14] = "";
char prevNumPot[21] = "";
char prevNumPwm[16] = "";
char prevNumRes[18] = ""; // "NSL16.60kS999.9mm" - resistance + seam combined on one row now, see below

void drawNumericalScreen_S1() // Screen 1
{
    drawHeader();
    char fieldBuf[21];
    char potNumStr[8];
    char NSLnumStr[8];
    char numStr[8];

    // BASE X.XXV NNNN - voltage always 0.00-1.60 post-decoupling
    // (POT_BARGRAPH_MAX_VOLTAGE), so width=4 is always exactly hit.
    dtostrf(estimatedSeriesResistanceOhms / 1000.0F, 5, 1, NSLnumStr);
    // snprintf(fieldBuf, sizeof(fieldBuf), "NSL%sk", NSLnumStr);

    dtostrf(adcToVoltage(potBaselineFiltered), 4, 2, potNumStr);
    snprintf(fieldBuf, sizeof(fieldBuf), "POT %sV  NSL%sK", potNumStr, NSLnumStr);
    drawFieldIfChanged(0, 1, prevNumPot, sizeof(prevNumPot), fieldBuf);

    snprintf(fieldBuf, sizeof(fieldBuf), "PWM %3u  TGT %3u", currentPWM, receivedPwmTarget);
    drawFieldIfChanged(0, 2, prevNumPwm, sizeof(prevNumPwm), fieldBuf);

    {
        dtostrf(seamLengthMm, 6, 1, numStr);
        snprintf(fieldBuf, sizeof(fieldBuf), "SEAM %smm", numStr);
    }
    drawFieldIfChanged(0, 3, prevDrawSeamMm, sizeof(prevDrawSeamMm), fieldBuf);
}

char prevDrawPotV[16] = "";
char prevDrawPwm[10] = "";

void drawDrawingScreen_S2() // Screen 2
{
    drawHeader();

    char fieldBuf[21];
    char numStr[8];
    char seamRateStr[8];

    // Tube/Min, blended from two independent estimates for stability:
    //  - velocity / seamLength: continuous, but inherits any noise in
    //    Actual_Velocity / Exp_Window_Out.
    //  - debounced pulse period average (v8.2.2): mean of the last
    //    TUBE_PULSE_GAP_HISTORY accepted-pulse gaps, each a precise ISR
    //    timestamp delta - replaces the old fixed-window pulse count,
    //    which was too coarse at production speed (only ~6-7 pulses per
    //    2000ms window) and jittery on top of that (window length itself
    //    varied with loop() timing). FIXED unit bug here too -
    //    actualVelocity is m/min, seamLengthMm is mm, so the m->mm
    //    conversion (x1000) was missing; without it this read ~1000x low.
    const float tubePerMinuteFromVelocity = (seamLengthMm > 0.0F)
                                                ? (static_cast<float>(actualVelocity) * 1000.0F / seamLengthMm)
                                                : 0.0F;
    const float tubePerMinuteFromPulses = readTubePerMinuteFromPulses();
    // Pulse-count side reads 0.0F until its first rate window has
    // elapsed since boot - fall back to the velocity-only estimate
    // rather than averaging against a not-yet-valid zero.
    const float tubePerMinute = (tubePerMinuteFromPulses > 0.0F)
                                    ? (tubePerMinuteFromVelocity + tubePerMinuteFromPulses) / 2.0F
                                    : tubePerMinuteFromVelocity;
    dtostrf(tubePerMinute, 5, 1, seamRateStr);
    snprintf(fieldBuf, sizeof(fieldBuf), "VELOCITY   %s t/m", seamRateStr);
    drawFieldIfChanged(0, 1, prevDrawPotV, sizeof(prevDrawPotV), fieldBuf);

    snprintf(fieldBuf, sizeof(fieldBuf), "PWM %3u  TGT %3u", currentPWM, receivedPwmTarget);
    drawFieldIfChanged(0, 2, prevNumPwm, sizeof(prevNumPwm), fieldBuf);

    {
        dtostrf(seamLengthMm, 6, 1, numStr);
        snprintf(fieldBuf, sizeof(fieldBuf), "SEAM %smm", numStr);
    }
    drawFieldIfChanged(0, 3, prevDrawSeamMm, sizeof(prevDrawSeamMm), fieldBuf);
}

char prevDiagASeqErr[20] = "";
char prevDiagAA5[12] = "";
char prevDiagAPirCmd[16] = "";

// Diagnostics split A/B (NEW Build 18) - the original single Diagnostics
// screen's 8 fields (SEQ, ERR, TGT, A5, PIR, CMD, Rex, POS, SET) didn't
// fit in 3 content rows on a 20x4 char LCD without becoming unreadable.
// A = comms/classify half. B = AGC/pot half. Michou's call: split into
// two screens in the button cycle rather than drop fields or compress
// further.
void drawDiagnosticsAScreen_S3() // Screen 3
{
    drawHeader();
    char fieldBuf[20];

    // SEQ/ERR compact, TGT appended on the same row - all three fit
    // comfortably in 20 columns once ERR's width is trimmed back down
    // (the old OLED version widened ERR to 5 digits defensively; on this
    // row layout there isn't space to spare for that unlikely case).
    snprintf(fieldBuf, sizeof(fieldBuf), "SEQ%3u  ERR %1u  T%3u", replySequence, communicationErrorCount, receivedPwmTarget);
    drawFieldIfChanged(0, 1, prevDiagASeqErr, sizeof(prevDiagASeqErr), fieldBuf);

    // A5 + Seam Distance combined on one row, same reasoning as the
    // Numerical screen's combined resistance/seam row.
    {
        char seamNumStr[8];
        dtostrf(seamLengthMm, 6, 1, seamNumStr);
        snprintf(fieldBuf, sizeof(fieldBuf), "A5%4u/%3s", pointAFiltered, seamNumStr);
    }
    drawFieldIfChanged(0, 2, prevDiagAA5, sizeof(prevDiagAA5), fieldBuf);

    // PIR bits + CMD on the last content row.
    char pirBits[6];

    if (plcPirEnable)
    {
        for (uint8_t i = 0; i < 5; ++i)
            pirBits[i] = (pirMask & (1U << i)) ? '1' : '0';
        pirBits[5] = '\0';
    }
    else
        strcpy(pirBits, "OFF  ");
    // commandText() returns a PROGMEM (__FlashStringHelper*) pointer -
    // AVR is Harvard architecture, so this must go through strncpy_P()
    // before snprintf's %s can use it; handing a flash pointer straight
    // to a RAM-string function reads garbage on AVR.
    char cmdRam[8];
    strncpy_P(cmdRam, reinterpret_cast<const char *>(commandText(lastExecutedCommand)), sizeof(cmdRam) - 1);
    cmdRam[sizeof(cmdRam) - 1] = '\0';
    snprintf(fieldBuf, sizeof(fieldBuf), "PIR %-5s %-6s", pirBits, cmdRam);
    drawFieldIfChanged(0, 3, prevDiagAPirCmd, sizeof(prevDiagAPirCmd), fieldBuf);
}

char prevDiagBRex[16] = "";
char prevDiagBPosSet[12] = "";

void drawDiagnosticsBScreen_S4() // Screen 4
{
    drawHeader();
    char fieldBuf[20];
    char numStr[8];

    if (estimatedSeriesResistanceOhms >= 1000.0F)
    {
        dtostrf(estimatedSeriesResistanceOhms / 1000.0F, 5, 2, numStr);
        snprintf(fieldBuf, sizeof(fieldBuf), "Rex %s k%s", numStr, OHM_UNIT);
    }
    else
    {
        snprintf(fieldBuf, sizeof(fieldBuf), "Rex %4u%s", static_cast<uint16_t>(estimatedSeriesResistanceOhms), OHM_UNIT);
    }
    drawFieldIfChanged(0, 1, prevDiagBRex, sizeof(prevDiagBRex), fieldBuf);

    snprintf(fieldBuf, sizeof(fieldBuf), "POS %3u  SET%s", pointBPwmValue, potSettled ? "*" : " ");
    drawFieldIfChanged(0, 2, prevDiagBPosSet, sizeof(prevDiagBPosSet), fieldBuf);
}

char prevPulseWidth[16] = "";
char prevPulseMinMax[24] = "";  // "MIN 999999 MAX 999999" = 21 chars + NUL, rounded up
char prevPulseGapSeam[20] = ""; // combined GAP + SEAM, see below
char prevSelectVelocity[16] = "";
char prevSelectAName[21] = "";
char prevSelectAStatus[21] = "";
char prevSelectBName[21] = "";
char prevSelectBStatus[21] = "";

void drawSensorSelectionScreen(char *previousName, char *previousStatus, char sensor, bool selected)
{
    drawHeader();

    char fieldBuf[21];
    char seamRateStr[8];

    const float tubePerMinuteFromVelocity = (seamLengthMm > 0.0F)
                                                ? (static_cast<float>(actualVelocity) * 1000.0F / seamLengthMm)
                                                : 0.0F;
    const float tubePerMinuteFromPulses = readTubePerMinuteFromPulses();
    const float tubePerMinute = (tubePerMinuteFromPulses > 0.0F)
                                    ? (tubePerMinuteFromVelocity + tubePerMinuteFromPulses) / 2.0F
                                    : tubePerMinuteFromVelocity;
    dtostrf(tubePerMinute, 5, 1, seamRateStr);
    snprintf(fieldBuf, sizeof(fieldBuf), "VELOCITY   %s t/m", seamRateStr);
    drawFieldIfChanged(0, 1, prevSelectVelocity, sizeof(prevSelectVelocity), fieldBuf);

    snprintf(fieldBuf, sizeof(fieldBuf), "    Seam Sensor %c  ", sensor);
    drawFieldIfChanged(0, 2, previousName, 21, fieldBuf);

    snprintf(fieldBuf, sizeof(fieldBuf), "    is Selected !   ");
    drawFieldIfChanged(0, 3, previousStatus, 21, fieldBuf);
}

void drawSelectAScreen_S6() // Screen 6
{
    drawSensorSelectionScreen(prevSelectAName, prevSelectAStatus, 'A', plcSelectA);
}

void drawSelectBScreen() // Screen 7
{
    drawSensorSelectionScreen(prevSelectBName, prevSelectBStatus, 'B', plcSelectB);
}

// Reads the ISR's volatile pulse-timing state under a brief critical
// section (32-bit reads are not atomic on an 8-bit AVR), then formats
// it - width/gap/min/max in microseconds, "----" until at least one full
// pulse has been captured since boot (pulseWidthSampleCount tracks total
// captures since boot, independent of the display-only min/max reset in
// resetPulseWidthStats()).
void drawPulseWidthScreen_S5() // Screen 5
{
    drawHeader();

    uint32_t widthUs, gapUs, minUs, maxUs;
    uint16_t sampleCount;
    noInterrupts();
    widthUs = lastPulseWidthUs;
    gapUs = lastPulseGapUs;
    minUs = minPulseWidthUs;
    maxUs = maxPulseWidthUs;
    sampleCount = pulseWidthSampleCount;
    interrupts();

    char fieldBuf[24];
    // char numStr[8]; // dtostrf scratch for the SEAM mm conversion below

    if (sampleCount == 0)
        snprintf(fieldBuf, sizeof(fieldBuf), "WID %5s us", "----");
    else
        snprintf(fieldBuf, sizeof(fieldBuf), "WID %5lu us", static_cast<unsigned long>(widthUs));
    drawFieldIfChanged(0, 1, prevPulseWidth, sizeof(prevPulseWidth), fieldBuf);

    if (sampleCount == 0)
        snprintf(fieldBuf, sizeof(fieldBuf), "MIN %5s MAX %5s", "----", "----");
    else
        snprintf(fieldBuf, sizeof(fieldBuf), "MIN %5lu MAX %5lu", static_cast<unsigned long>(minUs), static_cast<unsigned long>(maxUs));
    drawFieldIfChanged(0, 2, prevPulseMinMax, sizeof(prevPulseMinMax), fieldBuf);

    // GAP + SEAM combined on the last content row - was raw encoder
    // counts ("EXP <counts>") on much older firmware, converted to mm
    // using ENCODER_COUNTS_PER_METER so it reads as the actual measured
    // seam-to-seam web length. Still live "RUN_DATA_DB".Exp_Window_Out
    // underneath (see acceptValidPlcTelegram()) - 0 both before the
    // first telegram AND legitimately while FB600 hasn't acquired a
    // reference yet, so this reads "0.0mm" in either case rather than
    // being specially gated - a flat 0 is itself useful information (no
    // reference acquired yet).

    if (sampleCount == 0)
        snprintf(fieldBuf, sizeof(fieldBuf), "GAP %5s us", "----");
    else
        snprintf(fieldBuf, sizeof(fieldBuf), "GAP %5lu us", static_cast<unsigned long>(gapUs));

    drawFieldIfChanged(0, 3, prevPulseGapSeam, sizeof(prevPulseGapSeam), fieldBuf);
}

void updateLCD()
{
    if (!lcdAvailable)
        return;
    const uint32_t now = millis();

    if (sensorSelectionMessageActive && sensorSelectionMessageScreen == DisplayScreen::SelectA &&
        static_cast<int32_t>(now - sensorSelectionMessageDueMs) >= 0)
    {
        sensorSelectionMessageActive = false;
        currentScreen = DisplayScreen::Numerical;
        forceFullRedraw = true;
    }

    if (!lcdSleeping && now - lastMeaningfulActivityMs >= LCD_SLEEP_DELAY_MS)
    {
        lcdSleeping = true;
        lcd.noBacklight(); // character LCD sleep analog - see wakeLCD()
        return;
    }
    // Never start a redraw mid-telegram (receivingTelegram): AGC comes
    // first.

    if (lcdSleeping || receivingTelegram || now - previousDisplayMs < LCD_UPDATE_MS)
        return;
    previousDisplayMs = now;

    if (forceFullRedraw)
        lcd.clear();

    switch (currentScreen)
    {
    case DisplayScreen::Drawing:
        drawNumericalScreen_S1();
        break;
    case DisplayScreen::DiagnosticsA:
        drawDiagnosticsAScreen_S3();
        break;
    case DisplayScreen::DiagnosticsB:
        drawDiagnosticsBScreen_S4();
        break;
    case DisplayScreen::PulseWidth:
        drawPulseWidthScreen_S5();
        break;
    case DisplayScreen::SelectA:
        drawSelectAScreen_S6();
        break;
    // case DisplayScreen::SelectB:
    //     drawSelectBScreen();
    //     break;
    default:
        drawDrawingScreen_S2();
        break;
    }
    forceFullRedraw = false;
}

//======================================================================
// Local serial terminal override (USB Serial only - Serial1 stays
// dedicated to the PLC telegram link). Lets the bench operator drive
// the NSL-32 PWM directly, in unity (+-1) steps, without a PLC connected.
//   S/s = override ON   (PLC AGC commands ignored until turned off)
//   X/x = override OFF  (control returns to PLC/AGC)
//   A/a = PWM up by 1    (only while override is ON)
//   Z/z = PWM down by 1  (only while override is ON)
// setPwmOutput() itself prints the [PWM] old->new line for every A/Z
// step (and every PLC-driven move) - no separate resistance readout
// here anymore.
//======================================================================
void processSerialCommands()
{
    while (Serial.available() > 0)
    {
        const char command = static_cast<char>(Serial.read());

        switch (command)
        {
        case 'S':
            //    Serial.print(receiveTelegram[3], HEX);
            //        break;
        case 's':
            manualOverrideActive = true;
            noteMeaningfulActivity();
            Serial.println(F("[MANUAL] Control ON  - local override active, PLC AGC ignored"));
            break;

        case 'X':
        case 'x':
            manualOverrideActive = false;
            noteMeaningfulActivity();
            Serial.println(F("[MANUAL] Control OFF - returned to PLC/AGC control"));
            break;

        case 'A':
        case 'a':
            if (!manualOverrideActive)
            {
                Serial.println(F("[MANUAL] 'A' ignored - press 'S' to enable override first"));
                break;
            }
            setPwmOutput(static_cast<uint8_t>(min(255, static_cast<int16_t>(currentPWM) + 1)));
            lastExecutedCommand = AgcCommand::PwmUp;
            updateCorrectionLEDs();
            noteMeaningfulActivity();
            Serial.print(F("[MANUAL] PWM UP   pwm="));
            Serial.println(currentPWM);
            break;

        case 'Z':
        case 'z':
            if (!manualOverrideActive)
            {
                Serial.println(F("[MANUAL] 'Z' ignored - press 'S' to enable override first"));
                break;
            }
            setPwmOutput(static_cast<uint8_t>(max(0, static_cast<int16_t>(currentPWM) - 1)));
            lastExecutedCommand = AgcCommand::PwmDown;
            updateCorrectionLEDs();
            noteMeaningfulActivity();
            Serial.print(F("[MANUAL] PWM DOWN pwm="));
            Serial.println(currentPWM);
            break;

        default:
            break; // ignore newlines, carriage returns, unmapped keys
        }
    }
}

// NEW Build 11 - screen-advance now fires on RELEASE, not press (per
// Michou: wanted the ability to press-and-hold on the PulseWidth screen
// right after its min/max reset without an advance firing out from
// under them the instant they touch the button - it only commits once
// they let go). Waking a sleeping LCD still happens immediately on
// press (no reason to make the operator wait for release just to see
// the screen light up); if that's what the press did, the matching
// release is "spent" (suppressNextAdvance) and does not also advance.
void processScreenButton()
{
    const uint32_t now = millis();
    const bool raw = digitalRead(SCREEN_BUTTON_PIN);

    if (raw != previousRawButton)
    {
        previousRawButton = raw;
        buttonChangeMs = now;
    }

    if (now - buttonChangeMs < BUTTON_DEBOUNCE_MS || raw == stableButton)
        return;

    const bool previousStable = stableButton;
    stableButton = raw;

    if (stableButton == LOW)
    {
        // Debounced PRESS.
        suppressNextAdvance = lcdSleeping;
        wakeLCD();
        return;
    }

    // Debounced RELEASE. previousStable should always be LOW here (press
    // and release strictly alternate) - the check is defensive, not load-
    // bearing.
    if (previousStable != LOW)
        return;

    if (suppressNextAdvance)
    {
        suppressNextAdvance = false;
        return;
    }

    switch (currentScreen)
    {
    case DisplayScreen::Numerical:
        currentScreen = DisplayScreen::Drawing;
        break;
    case DisplayScreen::Drawing:
        currentScreen = DisplayScreen::DiagnosticsA;
        break;
    case DisplayScreen::DiagnosticsA:
        currentScreen = DisplayScreen::DiagnosticsB;
        break;
    case DisplayScreen::DiagnosticsB:
        currentScreen = DisplayScreen::PulseWidth;
        break;
    case DisplayScreen::PulseWidth:
        currentScreen = DisplayScreen::SelectA;
        break;
    case DisplayScreen::SelectA:
        currentScreen = DisplayScreen::SelectB;
        break;
    default:
        currentScreen = DisplayScreen::Numerical;
        break;
    }
    // Entering PulseWidth starts its rolling min/max fresh, so a few
    // seconds on the screen shows the current settling behaviour rather
    // than a min/max carried over from whenever the screen was last
    // visited (possibly a very different bench state).
    if (currentScreen == DisplayScreen::PulseWidth)
        resetPulseWidthStats();
    forceFullRedraw = true;
}

uint8_t calculateChecksum(const uint8_t *telegram)
{
    uint16_t sum = 0;

    for (uint8_t i = 0; i < 15; ++i)
        sum += telegram[i];
    return static_cast<uint8_t>(sum & 0xFFU);
}

void writeUInt16BigEndian(uint8_t *buffer, uint8_t offset, uint16_t value)
{
    buffer[offset] = static_cast<uint8_t>((value >> 8) & 0xFFU);
    buffer[offset + 1U] = static_cast<uint8_t>(value & 0xFFU);
}

uint8_t buildStatusByte()
{
    uint8_t status = STATUS_READY;
    if (plcAutoMode)
        status |= STATUS_AUTO_RECEIVED;

    if (plcMachineRun)
        status |= STATUS_RUN_RECEIVED;

    if (currentPWM > 0)
        status |= STATUS_NSL_OUTPUT_ACTIVE;

    if (lcdAvailable && !lcdSleeping)
        status |= STATUS_LCD_AWAKE;

    if (plcPirEnable)
        status |= STATUS_PIR_PROCESSING_ACTIVE;

    if (protocolErrorActive)
        status |= STATUS_PROTOCOL_ERROR;

    if (hardwareFaultActive)
        status |= STATUS_HARDWARE_FAULT;

    return status;
}

void clearCommunicationDiagnostics()
{
    communicationErrorCount = 0;
    protocolErrorActive = false;
}

void sendPendingReply()
{
    if (!replyPending || static_cast<int32_t>(millis() - replyDueMs) < 0)
        return;
    replyPending = false;
    transmitTelegram[0] = PROTOCOL_VERSION;
    transmitTelegram[1] = MEGA_REPLY_TYPE;
    transmitTelegram[2] = replySequence;
    transmitTelegram[3] = buildStatusByte();
    transmitTelegram[4] = MEGA_CAPABILITY_BYTE;
    transmitTelegram[5] = currentPWM; // was Actual_Index - no index left, mirrors DBB6
    transmitTelegram[6] = currentPWM; // Actual_PWM - authoritative confirmation of what's driving the NSL-32
    // DBW7-8: was NSL_Resistance - PLC side doesn't need it (per
    // Michou), reserved/zero until DB30's rework lands. Left as a
    // writeUInt16BigEndian(...,0) rather than deleting the call so the
    // byte offset stays put and this is easy to find/repurpose later.
    writeUInt16BigEndian(transmitTelegram, 7, 0U);
    writeUInt16BigEndian(transmitTelegram, 9, potBaselineFiltered);
    transmitTelegram[11] = scaleAdcToByte(pointAFiltered);
    transmitTelegram[12] = plcPirEnable ? pirMask : 0;
    transmitTelegram[13] = static_cast<uint8_t>(lastExecutedCommand);
    // DBB14: Extended_Status_Byte - Bit0 = Pot_Moved, Bit1 = Pot_Settled
    // (inverse of Pot_Moved, valid once at least one quiet-window sample
    // has been taken - was Local_Target_Indicated, retired: that bit
    // wrongly treated A6/Point B as a resistance/target proxy, when it
    // was only ever meant to read pot position/movement), Bit2 =
    // Manual_Override_Active (bench serial-terminal override engaged -
    // PLC AGC commands are being ignored while this is set), bits 3-7
    // reserved for future use. Update DB30's struct comment on the PLC
    // side to match (rename Local_Target_Indicated -> Pot_Settled).
    transmitTelegram[14] = (potMoved ? 0x01 : 0x00) |
                           (potSettled ? 0x02 : 0x00) |
                           (manualOverrideActive ? 0x04 : 0x00);
    transmitTelegram[15] = calculateChecksum(transmitTelegram);
    Serial1.write(transmitTelegram, TELEGRAM_LENGTH);
    // Serial1.flush() intentionally NOT called - see V7.1 changelog at
    // the top of this file. The UART TX buffer (64 bytes) transmits
    // these 16 bytes via interrupt in the background; nothing here needs
    // the blocking guarantee flush() provides.
}

void acceptValidPlcTelegram()
{
    const uint8_t sequence = receiveTelegram[2];
    const uint8_t control = receiveTelegram[3];
    const bool newMachineRun = (control & CONTROL_MACHINE_RUN) != 0;
    const bool newAutoMode = (control & CONTROL_AUTO_MODE) != 0;
    const bool resetDiagnostics = (control & CONTROL_RESET_DIAGNOSTICS) != 0;
    const bool newPirEnable = (control & CONTROL_PIR_ENABLE) != 0;
    const bool lcdWakeBit = (control & CONTROL_LCD_WAKE) != 0;
    const bool remoteResetBit = (control & CONTROL_REMOTE_RESET) != 0;
    const bool newSelectA = (control & CONTROL_SELECT_A) != 0;
    const bool newSelectB = (control & CONTROL_SELECT_B) != 0;
    const bool selectARose = newSelectA && !plcSelectA;
    const bool selectBRose = newSelectB && !plcSelectB;

    if (newMachineRun != plcMachineRun || newAutoMode != plcAutoMode || newPirEnable != plcPirEnable ||
        newSelectA != plcSelectA || newSelectB != plcSelectB)
        noteMeaningfulActivity();

    plcMachineRun = newMachineRun;
    plcAutoMode = newAutoMode;
    plcPirEnable = newPirEnable;
    plcSelectA = newSelectA;
    plcSelectB = newSelectB;

    if (selectARose || selectBRose)
    {
        sensorSelectionMessageScreen = selectARose ? DisplayScreen::SelectA : DisplayScreen::SelectB;
        sensorSelectionMessageActive = true;
        sensorSelectionMessageDueMs = selectARose ? millis() + SENSOR_SELECTION_MESSAGE_MS : 0;
        currentScreen = sensorSelectionMessageScreen;
        forceFullRedraw = true;
    }

    if (resetDiagnostics && !previousResetDiagnosticsBit)
        clearCommunicationDiagnostics();
    previousResetDiagnosticsBit = resetDiagnostics;

    if (lcdWakeBit && !previousLcdWakeBit)
        wakeLCD();
    previousLcdWakeBit = lcdWakeBit;

    if (remoteResetBit && !previousRemoteResetBit)
    {
        remoteResetRequested = true;
        Serial.println(F("[RESET] Remote reset requested via RS232 - WDT will fire"));
    }
    previousRemoteResetBit = remoteResetBit;

    const uint8_t commandValue = receiveTelegram[4];

    if (commandValue <= static_cast<uint8_t>(AgcCommand::AbsolutePwm))
        receivedCommand = static_cast<AgcCommand>(commandValue);
    else
    {
        receivedCommand = AgcCommand::Hold;
        protocolErrorActive = true;

        if (communicationErrorCount < 65535U)
            ++communicationErrorCount;

        Serial.print(F("[REJECTED] AGC_Command="));
        Serial.print(commandValue);
        Serial.print(F(" out of range (max "));
        Serial.print(static_cast<uint8_t>(AgcCommand::AbsolutePwm));
        Serial.print(F(") | errCount="));
        Serial.println(communicationErrorCount);
    }

    // Raw 0-255 PWM byte from FB610's AGC_Out (DB20 DBB5, sent when
    // AGC_Command=3). No clamping needed - it's already a full-range
    // byte. FB610 does the AGC math on the PLC side; the Mega just
    // applies it directly (see executeNewPlcCommand()).
    receivedPwmTarget = receiveTelegram[5];

    if (receivedPwmTarget != lastLoggedPwmTarget)
    {
        Serial.print(F("[TELEGRAM] AGC_Out changed: "));
        Serial.print(lastLoggedPwmTarget);
        Serial.print(F(" -> "));
        Serial.println(receivedPwmTarget);
        lastLoggedPwmTarget = receivedPwmTarget;
    }
    // DB20 DBB6 (Step_Size) is intentionally not read - it only ever
    // meant something for the now-removed table-index stepping.

    // DB20 v5.2 - ExpWindow redesigned back to a 2-byte INT at DBB7-8
    // (Hi/Lo, ±32767 clamp - see FC120 v1.5), with DBB9-10 restored as
    // Reserved/spare bytes. This used to be a 4-byte DINT spanning
    // DBB7-10 (Build 10 / DB20 v5.1) - that reassembly is stale now.
    // Reading DBB9-10 as low bytes today folds in whatever garbage sits
    // in those spare bytes (uncleared on PLC reset/reprogram) as the low
    // 16 bits of a bogus 32-bit value - e.g. a real 66.0mm reading
    // (raw count 260 = 0x0104) turning into 4327138.0mm the moment
    // DBB9-10 isn't zero. Reassemble via uint16_t first, then
    // reinterpret as int16_t before sign-extending to int32_t - shifting
    // a negative value into the sign bit position directly is undefined
    // behaviour in C++, this sidesteps that even though Exp_Window_Out
    // is not expected to go negative in normal operation.
    const uint16_t expWindowRaw16 =
        (static_cast<uint16_t>(receiveTelegram[7]) << 8) |
        static_cast<uint16_t>(receiveTelegram[8]);
    expWindowOut = static_cast<int32_t>(static_cast<int16_t>(expWindowRaw16));
    seamLengthMm = calculateSeamLengthMm();

    // NEW Build 17 - DB20 v5.3 DBB11 (Actual_Velocity). Already filtered
    // PLC-side, so this is a direct byte read, same as receivedPwmTarget.
    actualVelocity = receiveTelegram[11];
    updateSeamLengthMmWithVelocity();

    plcTelegramValid = true;
    lastValidTelegramMs = millis();

    if (lcdAvailable)
        hardwareFaultActive = false; // clear comm-loss fault; leave set if it was a real LCD init failure
    replySequence = sequence;
    executeNewPlcCommand(sequence);
    replyDueMs = millis() + REPLY_DELAY_MS;
    replyPending = true;
}

void processReceivedTelegram()
{
    const bool validHeader = receiveTelegram[0] == PROTOCOL_VERSION;
    const bool validType = receiveTelegram[1] == PLC_COMMAND_TYPE;
    const bool validChecksum = calculateChecksum(receiveTelegram) == receiveTelegram[15];

    if (!validHeader || !validType || !validChecksum)
    {
        protocolErrorActive = true;

        if (communicationErrorCount < 65535U)
            ++communicationErrorCount;
        Serial.print(F("[REJECTED] "));

        if (!validHeader)
        {
            Serial.print(F("bad header=0x"));
            Serial.print(receiveTelegram[0], HEX);
            Serial.print(F(" want=0x"));
            Serial.print(PROTOCOL_VERSION, HEX);
            Serial.print(F(" "));
        }
        if (!validType)
        {
            Serial.print(F("bad type=0x"));
            Serial.print(receiveTelegram[1], HEX);
            Serial.print(F(" want=0x"));
            Serial.print(PLC_COMMAND_TYPE, HEX);
            Serial.print(F(" "));
        }
        if (!validChecksum)
        {
            Serial.print(F("bad checksum=0x"));
            Serial.print(receiveTelegram[15], HEX);
            Serial.print(F(" calc=0x"));
            Serial.print(calculateChecksum(receiveTelegram), HEX);
            Serial.print(F(" "));
        }
        Serial.print(F("| errCount="));
        Serial.println(communicationErrorCount);
        return;
    }
    protocolErrorActive = false;
    acceptValidPlcTelegram();
}

void serviceSerialCommunication()
{
    const uint32_t nowRx = millis();

    // Abort a partially-received telegram if the next byte doesn't arrive
    // in time. Without this, a single dropped/corrupted byte permanently
    // desyncs the framing: receiveIndex never reaches TELEGRAM_LENGTH, so
    // every subsequent PLC telegram lands at the wrong offset forever.
    if (receivingTelegram && (nowRx - lastReceiveByteMs > RECEIVE_BYTE_TIMEOUT_MS))
    {
        const uint8_t bytesReceivedSoFar = receiveIndex;
        receivingTelegram = false;
        receiveIndex = 0;
        protocolErrorActive = true;

        if (communicationErrorCount < 65535U)
            ++communicationErrorCount;

        Serial.print(F("[REJECTED] byte timeout mid-telegram (got "));
        Serial.print(bytesReceivedSoFar);
        Serial.print(F(" bytes) | errCount="));
        Serial.println(communicationErrorCount);
    }

    while (Serial1.available() > 0)
    {
        const uint8_t value = static_cast<uint8_t>(Serial1.read());
        lastReceiveByteMs = millis();

        if (!receivingTelegram)
        {
            if (value == PROTOCOL_VERSION)
            {
                receiveTelegram[0] = value;
                receiveIndex = 1;
                receivingTelegram = true;
            }
            continue;
        }

        receiveTelegram[receiveIndex++] = value;

        if (receiveIndex >= TELEGRAM_LENGTH)
        {
            receivingTelegram = false;
            receiveIndex = 0;
            processReceivedTelegram();
        }
    }

    sendPendingReply();

    if (plcTelegramValid && millis() - lastValidTelegramMs > COMM_TIMEOUT_MS)
    {
        plcTelegramValid = false;
        plcMachineRun = false;
        plcAutoMode = false;
        lastExecutedCommand = AgcCommand::Hold;
        // NSL-32 PWM intentionally NOT reset here - AGC bias holds its
        // last commanded value on comm loss rather than snapping to a
        // default. hardwareFaultActive now flags this state so it's
        // visible in the status byte instead of failing silently.
        hardwareFaultActive = true;
        updateCorrectionLEDs();
        noteMeaningfulActivity();
    }
}

void showStartupScreen()
{
    // One-time boot screen, before normal PLC telegram traffic is a
    // concern - left as a simple blocking call, same as before.
    if (!lcdAvailable)
        return;
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print(F("410 ROTALINER"));
    lcd.setCursor(2, 1);
    lcd.print(F("MEGA REMOTE I/O"));
    lcd.setCursor(0, 2);
    lcd.print(FW_VERSION);
    lcd.setCursor(6, 3);
    lcd.print(F("WAIT PLC"));
    delay(1200);
}

void setup()
{
    wdt_disable(); // clear any WDT state inherited from a watchdog-triggered reset before re-arming below

    Serial.begin(115200);
    Serial1.begin(9600, SERIAL_8E1);

    pinMode(NSL32_PWM_PIN, OUTPUT);
    pinMode(BANNER_DIGITAL_PIN, INPUT);
    pinMode(STATUS_LED_PIN, OUTPUT);
    pinMode(LED_CORRECTION_UP_PIN, OUTPUT);
    pinMode(LED_CORRECTION_DOWN_PIN, OUTPUT);
    pinMode(LED_STANDBY_PIN, OUTPUT);
    pinMode(WHITE_AUX_SW_PIN, INPUT_PULLUP);
    pinMode(SCREEN_BUTTON_PIN, INPUT_PULLUP);

    for (uint8_t i = 0; i < 5; ++i)
        pinMode(PIR_PINS[i], INPUT);

    setPwmOutput(STARTUP_PWM);
    updateCorrectionLEDs();
    Wire.begin();
    Wire.setClock(100000UL); // 100kHz Standard Mode - bench-confirmed more reliable than 400kHz Fast Mode on this wiring. The V7.1 changelog's claim that 400kHz was set here was the actual error - this 100000UL is intentional, not a leftover typo.
    // LiquidCrystal_I2C's init() has no return value, so availability is
    // checked manually: a plain I2C transaction to the PCF8574AT's
    // address either ACKs (backpack present) or doesn't.
    Wire.beginTransmission(LCD_ADDRESS);
    lcdAvailable = (Wire.endTransmission() == 0);

    if (!lcdAvailable)
        hardwareFaultActive = true;
    else
    {
        lcd.init();
        lcd.backlight();
        showStartupScreen();
    }
    lastMeaningfulActivityMs = millis();

    // NEW Build 10 - interrupt-driven Banner pulse width/gap capture.
    // BANNER_DIGITAL_PIN (3) is INT5-capable on the Mega 2560; CHANGE
    // fires bannerEdgeISR() on both edges. Attached after the pin's own
    // pinMode() call above; independent of updatePotMovedDetection()'s
    // polled digitalRead() on the same pin, which only needs the
    // (much coarser, millisecond-scale) falling edge for its settle
    // timer and is unaffected by this.
    attachInterrupt(digitalPinToInterrupt(BANNER_DIGITAL_PIN), bannerEdgeISR, CHANGE);

    Serial.println();
    Serial.println(F("410 Rotaliner Mega Remote I/O"));
    Serial.print(F("Firmware: "));
    Serial.print(FW_VERSION);
    Serial.print(F("  Date: "));
    Serial.println(FW_DATE);
    Serial.println(F("PLC/FM350 (FB610) owns AGC decisions - Mega applies PWM + confirms."));
    Serial.println(F("Manual override (USB serial): S=ON X=OFF A=up Z=down"));

    wdt_enable(WDTO_1S); // recover from I2C lockups instead of hanging indefinitely
}

void loop()
{
    if (!remoteResetRequested)
        wdt_reset();
    processSerialCommands(); // bench operator input serviced first, ahead of PLC comms/LCD/analog
    serviceSerialCommunication();
    updateAnalogMeasurements();
    updatePotMovedDetection();
    updateSeamLengthMmWithVelocity();
    readPIRInputs();
    processScreenButton();
    updateLCD(); // draws next frame - no flush step needed, drawFieldIfChanged() writes each changed cell immediately
    digitalWrite(STATUS_LED_PIN, digitalRead(BANNER_DIGITAL_PIN) == HIGH ? HIGH : LOW);
}