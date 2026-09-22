#include <Wire.h>
#include <LiquidCrystal_I2C.h>

/*
 * ============================================================
 * Cruise Control System Using PID Controller
 * ============================================================
 *
 * Course      : Design Lab II
 * Institution : IIT Ropar
 * Author      : Jahnavi Sharma
 *
 * Description:
 * Hardware-based closed-loop cruise control for a 4-wheel
 * vehicle using Arduino UNO, DC motors, IBT-2 motor drivers,
 * rotary encoder feedback, and a PID controller.
 *
 * Control principle:
 *
 * Set Speed → Error Calculation → PID Controller
 *           → PWM Motor Command → Vehicle Motion
 *           → Encoder Feedback → Repeat
 *
 * Main features:
 * - Rotary encoder based speed measurement
 * - 100 ms speed-control interval
 * - Moving-average RPM filtering
 * - PID-based speed regulation
 * - Integral anti-windup
 * - PWM motor control
 * - LCD-based monitoring
 *
 * ============================================================
 */


// ============================================================
// Pin Mapping
// ============================================================

// IBT-2 Motor Driver - Left Motor
static constexpr uint8_t PIN_R_PWM_L = 5;
static constexpr uint8_t PIN_L_PWM_L = 6;

// IBT-2 Motor Driver - Right Motor
static constexpr uint8_t PIN_R_PWM_R = 9;
static constexpr uint8_t PIN_L_PWM_R = 10;

// Rotary Encoder
// Encoder CLK must be connected to Arduino interrupt pin 2.
static constexpr uint8_t PIN_ENC_CLK = 2;


// ============================================================
// Encoder and Control Parameters
// ============================================================

// Encoder pulses per revolution
static constexpr uint8_t ENCODER_PPR = 20;

// PID / RPM control interval
// 100 ms = 10 control updates per second
static constexpr uint32_t RPM_INTERVAL = 100UL;

// Number of RPM samples used for moving-average filtering
static constexpr uint8_t RPM_AVG_SIZE = 5;


// ============================================================
// PWM Limits
// ============================================================

// Minimum PWM required to overcome motor friction
static constexpr uint8_t PWM_MIN = 80;

// Maximum PWM output used by the controller
static constexpr uint8_t PWM_MAX = 200;


// ============================================================
// PID Parameters
// ============================================================

static constexpr float KP = 1.2f;
static constexpr float KI = 0.1f;
static constexpr float KD = 0.01f;

// Control interval in seconds
static constexpr float DT = RPM_INTERVAL / 1000.0f;

// Integral anti-windup limit
static constexpr float INTEGRAL_CLAMP = 255.0f / KI;


// ============================================================
// LCD Configuration
// ============================================================

LiquidCrystal_I2C lcd(0x27, 16, 2);


// ============================================================
// Global Variables
// ============================================================

// Encoder pulse counter
// volatile because it is modified inside the interrupt routine
volatile uint32_t encPulseCount = 0;

// RPM moving-average buffer
uint16_t rpmBuffer[RPM_AVG_SIZE] = {0};

uint8_t rpmBufIdx = 0;

// Filtered RPM value
uint16_t filteredRPM = 0;


// ============================================================
// PID State Variables
// ============================================================

float pidIntegral = 0.0f;
float pidPrevError = 0.0f;

// Current PWM command
uint8_t currentPWM = 0;

// Fixed target speed
// Kept at 100 RPM for stable operation
float targetRPM = 100.0f;


// ============================================================
// Timing
// ============================================================

uint32_t lastTickMs = 0;


// ============================================================
// Encoder Interrupt Service Routine
// ============================================================

void encoderISR()
{
    encPulseCount++;
}


// ============================================================
// RPM Calculation
// ============================================================

inline void computeRPM()
{
    /*
     * Temporarily disable interrupts while copying and
     * resetting the encoder pulse count.
     */
    noInterrupts();

    uint32_t pulses = encPulseCount;

    encPulseCount = 0;

    interrupts();


    /*
     * Calculate instantaneous RPM.
     *
     * RPM =
     *     pulses × 60000
     *     -----------------------------
     *     PPR × measurement interval(ms)
     */
    rpmBuffer[rpmBufIdx] =
        (pulses * 60000UL) /
        (ENCODER_PPR * RPM_INTERVAL);


    // Move to next position in circular buffer
    rpmBufIdx =
        (rpmBufIdx + 1) % RPM_AVG_SIZE;


    /*
     * Moving-average filtering.
     *
     * Averaging multiple RPM measurements reduces
     * fluctuations in the encoder feedback signal.
     */
    uint32_t sum = 0;

    for (uint8_t i = 0; i < RPM_AVG_SIZE; i++)
    {
        sum += rpmBuffer[i];
    }

    filteredRPM =
        sum / RPM_AVG_SIZE;
}


// ============================================================
// PID Controller
// ============================================================

inline uint8_t computePID(float setpoint, float measured)
{
    // Calculate speed error
    float error = setpoint - measured;


    // --------------------------------------------------------
    // Integral Term with Anti-Windup
    // --------------------------------------------------------

    pidIntegral += error * DT;

    pidIntegral =
        constrain(
            pidIntegral,
            -INTEGRAL_CLAMP,
            INTEGRAL_CLAMP
        );


    // --------------------------------------------------------
    // Derivative Term
    // --------------------------------------------------------

    float derivative =
        (error - pidPrevError) / DT;


    // --------------------------------------------------------
    // PID Output
    // --------------------------------------------------------

    float output =
          (KP * error)
        + (KI * pidIntegral)
        + (KD * derivative);


    // Store current error for next cycle
    pidPrevError = error;


    // --------------------------------------------------------
    // Limit PWM Output
    // --------------------------------------------------------

    return (uint8_t)constrain(
        output,
        PWM_MIN,
        PWM_MAX
    );
}


// ============================================================
// LCD Display
// ============================================================

inline void updateDisplay()
{
    // First row: Set speed and actual speed
    lcd.setCursor(0, 0);

    lcd.print(F("SET:"));
    lcd.print((int)targetRPM);

    lcd.print(F("  ACT:"));
    lcd.print(filteredRPM);


    // Second row: PWM command
    lcd.setCursor(0, 1);

    lcd.print(F("PWM:"));
    lcd.print(currentPWM);

    lcd.print(F("    STABLE  "));
}


// ============================================================
// Setup
// ============================================================

void setup()
{
    // --------------------------------------------------------
    // Configure Motor Output Pins
    // --------------------------------------------------------

    pinMode(PIN_R_PWM_L, OUTPUT);
    pinMode(PIN_L_PWM_L, OUTPUT);

    pinMode(PIN_R_PWM_R, OUTPUT);
    pinMode(PIN_L_PWM_R, OUTPUT);


    // --------------------------------------------------------
    // Configure Encoder Input
    // --------------------------------------------------------

    pinMode(
        PIN_ENC_CLK,
        INPUT_PULLUP
    );


    // --------------------------------------------------------
    // Initialize Motor Outputs
    //
    // L-PWM pins are kept LOW to define the initial
    // forward-drive configuration.
    // --------------------------------------------------------

    analogWrite(PIN_L_PWM_L, 0);
    analogWrite(PIN_L_PWM_R, 0);


    // --------------------------------------------------------
    // Attach Encoder Interrupt
    // --------------------------------------------------------

    attachInterrupt(
        digitalPinToInterrupt(PIN_ENC_CLK),
        encoderISR,
        RISING
    );


    // --------------------------------------------------------
    // Initialize LCD
    // --------------------------------------------------------

    lcd.init();
    lcd.backlight();
    lcd.clear();


    // --------------------------------------------------------
    // Initialize Timing
    // --------------------------------------------------------

    lastTickMs = millis();
}


// ============================================================
// Main Control Loop
// ============================================================

void loop()
{
    uint32_t now = millis();


    /*
     * Execute the cruise-control algorithm every 100 ms.
     */
    if ((now - lastTickMs) >= RPM_INTERVAL)
    {
        lastTickMs = now;


        // ----------------------------------------------------
        // 1. Measure Actual Speed
        // ----------------------------------------------------

        computeRPM();


        // ----------------------------------------------------
        // 2. Calculate PID Correction
        // ----------------------------------------------------

        currentPWM =
            computePID(
                targetRPM,
                (float)filteredRPM
            );


        // ----------------------------------------------------
        // 3. Apply Same PWM to Both Motors
        // ----------------------------------------------------

        analogWrite(
            PIN_R_PWM_L,
            currentPWM
        );

        analogWrite(
            PIN_R_PWM_R,
            currentPWM
        );


        // ----------------------------------------------------
        // 4. Update LCD
        // ----------------------------------------------------

        updateDisplay();
    }
}
