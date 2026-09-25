// ESP32 ADAPTIVE PULSE VOLTAGE STABILIZER

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "BluetoothSerial.h"

// PIN DEFINITIONS

const int ADC_PIN = 34;
const int MOTOR_EN  = 18;
const int MOTOR_IN1 = 19;
const int MOTOR_IN2 = 23;
const int LCD_SDA = 21;
const int LCD_SCL = 22;

LiquidCrystal_I2C lcd(0x27, 16, 2);
BluetoothSerial SerialBT;

// ELECTRICAL CONSTANTS

const float ADC_REFERENCE = 3.3;
const int ADC_MAX_VALUE = 4095;
const float DIVIDER_RATIO = 13.0;
const float TOTAL_DIODE_DROP = 1.05;
const float TRANSFORMER_RATIO = 16.25;
float CALIBRATION_GAIN = 1.0;
float CALIBRATION_OFFSET = 0.0;

// TARGET LIMITS

const float MIN_TARGET_VOLTAGE = 70.0;
const float MAX_TARGET_VOLTAGE = 450.0;

// CONTROLLER SETTINGS

const int MOTOR_PWM = 255;
const float PRECISION_BAND = 0.5;

// Pulse limits

const unsigned long MAX_PULSE_MS = 180;
const unsigned long MIN_PULSE_MS = 5;

// Starting pulse.

const unsigned long START_PULSE_MS = 80;

// Settling time
// After hard braking, allow voltage measurement to settle.

const unsigned long SETTLE_TIME_MS = 250;

// Re-check interval while stable

const unsigned long STABLE_CHECK_MS = 150;

// SYSTEM VARIABLES

float targetVoltage = 0.0;
float currentVoltage = 0.0;
float previousVoltage = 0.0;
float currentError = 0.0;
float previousError = 0.0;
bool stabilizing = false;
bool waitingForTarget = true;
String inputBuffer = "";

// CONTROLLER STATE

enum ControlState
{
    CONTROL_MEASURE,
    CONTROL_MOVING,
    CONTROL_SETTLING,
    CONTROL_STABLE
};

ControlState controlState = CONTROL_MEASURE;

unsigned long pulseStartTime = 0;
unsigned long settleStartTime = 0;
unsigned long stableStartTime = 0;

unsigned long currentPulseMs = START_PULSE_MS;

bool movingForward = true;

// ADAPTIVE CONTROL VARIABLES

// Last movement direction
int lastDirection = 0;

// Last measured correction
float lastVoltageChange = 0.0;

// Used to reduce pulse size near target
unsigned long finePulseMs = START_PULSE_MS;

// STATISTICS

float maximumError = 0.0;
float errorSum = 0.0;
unsigned long errorCount = 0;

// DISPLAY

int displayPage = 1;
unsigned long lastDisplayTime = 0;
const unsigned long DISPLAY_INTERVAL = 300;

// FUNCTION DECLARATIONS

void initializeHardware();

void handleBluetooth();
void handleKey(char key);
void handleNumber(char key);
void handleButtonA();
void handleButtonC();
void handleButtonD();
float readOutputVoltage();
float readGPIOVoltage();
float calculateOutputVoltage(float gpioVoltage);
void runController();
void startCorrection();
void finishMovement();
void choosePulse();
void motorForward(int pwm);
void motorReverse(int pwm);
void motorHardBrake();
void resetController();
void updateStatistics(float error);
void resetStatistics();
float getAverageError();
void updateDisplay();
void displayPage1();
void displayPage2();
void displayTargetInput();

// SETUP

void setup()
{
    Serial.begin(115200);
    SerialBT.begin("ESP32_Stabilizer");
    initializeHardware();
    motorHardBrake();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Voltage");
    lcd.setCursor(0, 1);
    lcd.print("Stabilizer");
    delay(2000);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Enter Target V");
    lcd.setCursor(0, 1);
    lcd.print("Input: ---V");

}

// MAIN LOOP

void loop()
{
    handleBluetooth();
    if (stabilizing)
    {
        runController();
    }
    updateDisplay();
}

// HARDWARE INITIALIZATION

void initializeHardware()
{
    pinMode(ADC_PIN, INPUT);
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    ledcAttach(MOTOR_EN, 5000, 8);
    Wire.begin(LCD_SDA, LCD_SCL);
    lcd.init();
    lcd.backlight();
}

// BLUETOOTH

void handleBluetooth()
{
    while (SerialBT.available())
    {
        char key = SerialBT.read();
        handleKey(key);
    }

    while (Serial.available())
    {
        char key = Serial.read();
        handleKey(key);
    }
}

// KEY HANDLER

void handleKey(char key)
{
    if (key == '\n' || key == '\r')
    {
        return;
    }

    if (key >= '0' && key <= '9')
    {
        handleNumber(key);
        return;
    }

    switch (key)
    {
        case 'A':
        case 'a':
            handleButtonA();
            break;

        case 'C':
        case 'c':
            handleButtonC();
            break;

        case 'D':
        case 'd':
            handleButtonD();
            break;
    }
}

// NUMBER INPUT

void handleNumber(char key)
{
    if (inputBuffer.length() < 3)
    {
        inputBuffer += key;
    }

    if (waitingForTarget)
    {
        displayTargetInput();
    }
}

// BUTTON A

void handleButtonA()
{
    if (waitingForTarget)
    {
        return;
    }

    if (displayPage == 1)
    {
        displayPage = 2;
    }
    else
    {
        displayPage = 1;
    }

    lcd.clear();
}

// BUTTON C

void handleButtonC()
{
    motorHardBrake();

    stabilizing = false;
    waitingForTarget = true;
    resetController();
    resetStatistics();
    inputBuffer = "";
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Motor Stopped");
    lcd.setCursor(0, 1);
    lcd.print("New Target");
    delay(700);
    lcd.clear();
    displayTargetInput();
}

// BUTTON D

void handleButtonD()
{
    if (inputBuffer.length() == 0)
    {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Enter Target");
        lcd.setCursor(0, 1);
        lcd.print("First!");
        delay(1000);
        lcd.clear();
        displayTargetInput();
        return;
    }
    float newTarget = inputBuffer.toFloat();
    if (newTarget >= MIN_TARGET_VOLTAGE && newTarget <= MAX_TARGET_VOLTAGE)
    {
        targetVoltage = newTarget;
        inputBuffer = "";
        waitingForTarget = false;

        resetController();
        resetStatistics();

        currentVoltage = readOutputVoltage();
        previousVoltage = currentVoltage;
        currentError = targetVoltage - currentVoltage;

        stabilizing = true;
        controlState = CONTROL_MEASURE;

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Target Set");
        lcd.setCursor(0, 1);
        lcd.print(targetVoltage, 1);
        lcd.print("V");

        delay(700);
        lcd.clear();
    }
    else
    {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Invalid Target");
        lcd.setCursor(0, 1);
        lcd.print("70-450V");

        delay(1000);

        inputBuffer = "";
        lcd.clear();
        displayTargetInput();
    }
}
// ============================================================
// MAIN CONTROLLER
// ============================================================

void runController()
{
    unsigned long now = millis();

    // MOTOR IS CURRENTLY MOVING

    if (controlState == CONTROL_MOVING)
    {
        if (now - pulseStartTime >= currentPulseMs)
        {
            motorHardBrake();
            settleStartTime = now;
            controlState = CONTROL_SETTLING;
        }

        return;
    }

    // WAIT AFTER BRAKING

    if (controlState == CONTROL_SETTLING)
    {
        if (now - settleStartTime >= SETTLE_TIME_MS)
        {
            finishMovement();
        }

        return;
    }

    // STABLE STATE

    if (controlState == CONTROL_STABLE)
    {
        if (now - stableStartTime >= STABLE_CHECK_MS)
        {
            currentVoltage = readOutputVoltage();
            currentError =
                targetVoltage - currentVoltage;
            updateStatistics(currentError);
            if (fabs(currentError) > PRECISION_BAND)
            {
                controlState = CONTROL_MEASURE;
            }

            stableStartTime = now;
        }

        return;
    }

    // MEASURE AND DECIDE

    if (controlState == CONTROL_MEASURE)
    {
        currentVoltage = readOutputVoltage();

        currentError =
            targetVoltage - currentVoltage;

        updateStatistics(currentError);

        // Target reached
        if (fabs(currentError) <= PRECISION_BAND)
        {
            motorHardBrake();
            controlState = CONTROL_STABLE;
            stableStartTime = now;
            return;
        }

        choosePulse();
        startCorrection();
    }
}

// CHOOSE PULSE

void choosePulse()
{
    float error = fabs(currentError);

    // Large error

    if (error > 50.0)
    {
        currentPulseMs = 180;
    }

    // Medium-large error

    else if (error > 20.0)
    {
        currentPulseMs = 120;
    }

    // Medium error

    else if (error > 10.0)
    {
        currentPulseMs = 80;
    }

    // Small error

    else if (error > 5.0)
    {
        currentPulseMs = 40;
    }

    // Fine correction

    else if (error > 2.0)
    {
        currentPulseMs = 20;
    }

    // Very fine correction

    else
    {
        currentPulseMs = finePulseMs;

        if (currentPulseMs > 10)
        {
            currentPulseMs = 10;
        }
    }

    currentPulseMs =
        constrain(
            currentPulseMs,
            MIN_PULSE_MS,
            MAX_PULSE_MS
        );
}

// START CORRECTION

void startCorrection()
{
    if (currentError > 0)
    {
        // Move in direction that increases voltage
        movingForward = true;
        motorForward(MOTOR_PWM);
        lastDirection = 1;
    }
    else
    {
        // Output too HIGH
        movingForward = false;
        motorReverse(MOTOR_PWM);
        lastDirection = -1;
    }

    pulseStartTime = millis();
    controlState = CONTROL_MOVING;
}

// FINISH MOVEMENT

void finishMovement()
{
    previousVoltage = currentVoltage;

    currentVoltage = readOutputVoltage();

    currentError =
        targetVoltage - currentVoltage;

    lastVoltageChange =
        currentVoltage - previousVoltage;

    if (
        (previousVoltage < targetVoltage &&
         currentVoltage > targetVoltage)
        ||
        (previousVoltage > targetVoltage &&
         currentVoltage < targetVoltage)
    )
    {
        finePulseMs /= 2;

        if (finePulseMs < MIN_PULSE_MS)
        {
            finePulseMs = MIN_PULSE_MS;
        }
    }

    if (fabs(currentError) <= 5.0)
    {
        finePulseMs = min(
            finePulseMs,
            (unsigned long)10
        );
    }

    updateStatistics(currentError);

    // Target reached

    if (fabs(currentError) <= PRECISION_BAND)
    {
        motorHardBrake();
        controlState = CONTROL_STABLE;
        stableStartTime = millis();
        return;
    }

    controlState = CONTROL_MEASURE;
}

// VOLTAGE READING

float readOutputVoltage()
{
    float gpioVoltage = readGPIOVoltage();

    return calculateOutputVoltage(gpioVoltage);
}

// ADC READING

float readGPIOVoltage()
{
    const int SAMPLE_COUNT = 30;
    long sum = 0;
    for (int i = 0; i < SAMPLE_COUNT; i++)
    {
        sum += analogRead(ADC_PIN);
    }
    int adcValue =
        sum / SAMPLE_COUNT;
    float voltage =
        ((float)adcValue / ADC_MAX_VALUE)
        * ADC_REFERENCE;
    return voltage;
}

// VOLTAGE CALCULATION

float calculateOutputVoltage(float gpioVoltage)
{
    const float GPIO_NO_INPUT_THRESHOLD = 0.10;

    if (gpioVoltage < GPIO_NO_INPUT_THRESHOLD)
    {
        return 0.0;
    }

    float rectifiedPeakVoltage =
        gpioVoltage * DIVIDER_RATIO;
    float secondaryPeakVoltage =
        rectifiedPeakVoltage +
        TOTAL_DIODE_DROP;
    float secondaryRMSVoltage =
        secondaryPeakVoltage /
        sqrt(2.0);
    float outputVoltage =
        secondaryRMSVoltage *
        TRANSFORMER_RATIO;
    outputVoltage =
        (outputVoltage * CALIBRATION_GAIN)
        + CALIBRATION_OFFSET;
    if (outputVoltage < 0.0)
    {
        outputVoltage = 0.0;
    }
    return outputVoltage;
}

// MOTOR FORWARD

void motorForward(int pwm)
{
    pwm = constrain(pwm, 0, 255);
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
    ledcWrite(MOTOR_EN, pwm);
}

// MOTOR REVERSE

void motorReverse(int pwm)
{
    pwm = constrain(pwm, 0, 255);
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
    ledcWrite(MOTOR_EN, pwm);
}

// HARD BRAKE

void motorHardBrake()
{
    ledcWrite(MOTOR_EN, 0);
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, HIGH);
}

// RESET CONTROLLER

void resetController()
{
    motorHardBrake();
    controlState = CONTROL_MEASURE;
    currentPulseMs = START_PULSE_MS;
    finePulseMs = START_PULSE_MS;
    lastDirection = 0;
    lastVoltageChange = 0.0;
    previousError = 0.0;
}

// STATISTICS

void updateStatistics(float error)
{
    float absError = fabs(error);
    if (absError > maximumError)
    {
        maximumError = absError;
    }
    errorSum += absError;
    errorCount++;
}

void resetStatistics()
{
    maximumError = 0.0;
    errorSum = 0.0;
    errorCount = 0;
}

float getAverageError()
{
    if (errorCount == 0)
    {
        return 0.0;
    }

    return errorSum / errorCount;
}

// DISPLAY

void updateDisplay()
{
    unsigned long now = millis();
    if (now - lastDisplayTime < DISPLAY_INTERVAL)
    {
        return;
    }

    lastDisplayTime = now;
    if (waitingForTarget)
    {
        displayTargetInput();
        return;
    }
    if (displayPage == 1)
    {
        displayPage1();
    }
    else
    {
        displayPage2();
    }
}

// TARGET INPUT DISPLAY

void displayTargetInput()
{
    lcd.setCursor(0, 0);
    lcd.print("Enter Target V ");
    lcd.setCursor(0, 1);
    lcd.print("Input:          ");
    lcd.setCursor(7, 1);
    if (inputBuffer.length() > 0)
    {
        lcd.print(inputBuffer);
        lcd.print("V");
    }
    else
    {
        lcd.print("---V");
    }
}

// PAGE 1
void displayPage1()
{
    lcd.setCursor(0, 0);
    lcd.print("Target:         ");
    lcd.setCursor(8, 0);
    lcd.print(targetVoltage, 1);
    lcd.print("V");
    lcd.setCursor(0, 1);
    lcd.print("Output:         ");
    lcd.setCursor(8, 1);
    lcd.print(currentVoltage, 1);
    lcd.print("V");
}

// PAGE 2

void displayPage2()
{
    lcd.setCursor(0, 0);
    lcd.print("MaxErr:         ");
    lcd.setCursor(8, 0);
    lcd.print(maximumError, 1);
    lcd.print("V");
    lcd.setCursor(0, 1);
    lcd.print("AvgErr:         ");
    lcd.setCursor(8, 1);
    lcd.print(getAverageError(), 1);
    lcd.print("V");
}
