#include <Arduino.h>
#include <Wire.h>
#include <HardwareTimer.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include <crsf.h>

namespace Config {
constexpr uint32_t kLedToggleIntervalMs = 500;
constexpr uint32_t kGyroPrintIntervalMs = 100;
constexpr uint32_t kPidPrintIntervalMs = 100;
constexpr uint32_t kElrsStatusIntervalMs = 100;
constexpr uint16_t kImuCalibrationSamples = 250;
constexpr uint32_t kImuHealthCheckDurationMs = 3000;
constexpr uint32_t kImuBootSettleDelayMs = 2500;
constexpr uint32_t kImuCalibrationRetryIntervalMs = 1000;
constexpr uint8_t kMpuAddress = 0x68;
constexpr uint8_t kRegWhoAmI = 0x75;
constexpr uint8_t kRegPwrMgmt1 = 0x6B;
constexpr uint8_t kRegAccelConfig = 0x1C;
constexpr uint8_t kRegGyroConfig = 0x1B;
constexpr uint8_t kRegAccelData = 0x3B;
constexpr size_t kCrsfReadBufferSize = 64;
constexpr uint32_t kElrsSerialBaud = 420000;
constexpr uint16_t kServoMinUs = 1000;
constexpr uint16_t kServoMidUs = 1500;
constexpr uint16_t kServoMaxUs = 2000;
constexpr uint32_t kEscPwmFrequencyHz = 400;
constexpr uint32_t kAuxPwmFrequencyHz = 50;
constexpr float kMaxAngleTargetDeg = 30.0f;
constexpr float kMaxYawRateDegPerSec = 120.0f;
constexpr float kMaxRollPitchRateDegPerSec = 220.0f;
constexpr float kSurfaceOutputLimit = 35.0f;
constexpr float kYawOutputLimit = 30.0f;
constexpr float kIntegralLimitAngle = 25.0f;
constexpr float kIntegralLimitRate = 40.0f;
constexpr float kStickDeadband = 0.05f;
constexpr uint32_t kRcSignalTimeoutMs = 300;
}

#ifndef LED_PIN
#define LED_PIN PC13
#endif

#ifndef SERIAL_BAUD
#define SERIAL_BAUD 115200
#endif

#ifndef DEBUG_UART_RX
#define DEBUG_UART_RX PA10
#endif

#ifndef DEBUG_UART_TX
#define DEBUG_UART_TX PA9
#endif

#ifndef ELRS_UART_RX
#define ELRS_UART_RX PB11
#endif

#ifndef ELRS_UART_TX
#define ELRS_UART_TX PB10
#endif

#ifndef PWM_AILERON_PIN
#define PWM_AILERON_PIN PA0
#endif

#ifndef PWM_ELEVATOR_PIN
#define PWM_ELEVATOR_PIN PA1
#endif

#ifndef PWM_THROTTLE_PIN
#define PWM_THROTTLE_PIN PA2
#endif

#ifndef PWM_RUDDER_PIN
#define PWM_RUDDER_PIN PA3
#endif

#ifndef PWM_AUX_PIN
#define PWM_AUX_PIN PA6
#endif

#ifdef BOARD_STM32
HardwareSerial DebugSerial(DEBUG_UART_RX, DEBUG_UART_TX);
HardwareSerial ElrsSerial(ELRS_UART_RX, ELRS_UART_TX);
TwoWire MpuWire(MPU6050_I2C_SDA, MPU6050_I2C_SCL);
#endif

namespace Debug {
    Print *port = nullptr;

    void logf(const char *format, ...) {
        if (port == nullptr) {
            return;
        }

        char buffer[256];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        port->print(buffer);
    }

    void csvFloat(float value, uint8_t digits = 3) {
        if (port != nullptr) {
            port->print(value, digits);
        }
    }

    void comma() {
        if (port != nullptr) {
            port->print(',');
        }
    }

    void endLine() {
        if (port != nullptr) {
            port->print("\r\n");
        }
    }

    void keyValue(const char *label, float value, uint8_t digits = 3) {
        if (port == nullptr) {
            return;
        }

        port->print(label);
        port->print(':');
        port->print(value, digits);
    }
}

namespace LED {
    uint32_t nextToggleTimeMs = 0;
    bool ledState = false;

    void setup() {
        pinMode(LED_PIN, OUTPUT);
        digitalWrite(LED_PIN, HIGH);
    }

    void loop() {
        const uint32_t now = millis();
        if (now < nextToggleTimeMs) {
            return;
        }

        ledState = !ledState;
        digitalWrite(LED_PIN, ledState ? LOW : HIGH);
        nextToggleTimeMs = now + Config::kLedToggleIntervalMs;
    }
}

namespace PWM {
#ifdef BOARD_STM32
    HardwareTimer auxTimer(TIM3);
#endif
    bool ready = false;
    uint16_t lastMotor1Us = Config::kServoMinUs;
    uint16_t lastMotor2Us = Config::kServoMinUs;
    uint16_t lastMotor3Us = Config::kServoMinUs;
    uint16_t lastMotor4Us = Config::kServoMinUs;
    uint16_t lastAuxUs = Config::kServoMidUs;
    constexpr uint32_t kEscTimerClockHz = 9000000UL;
    constexpr uint16_t kEscPrescaler = 8 - 1;
    constexpr uint16_t kEscArr = 22500 - 1;

    uint16_t clampMicros(uint16_t pulseUs) {
        return constrain(pulseUs, Config::kServoMinUs, Config::kServoMaxUs);
    }

    uint16_t percentToMicros(int percent) {
        return clampMicros(map(constrain(percent, 0, 100), 0, 100, Config::kServoMinUs, Config::kServoMaxUs));
    }

#ifdef BOARD_STM32
    uint32_t microsToEscTicks(uint16_t pulseUs) {
        return static_cast<uint32_t>(pulseUs) * (kEscTimerClockHz / 1000000UL);
    }

    uint32_t dutyToEscTicks(uint8_t dutyPercent) {
        return (static_cast<uint32_t>(kEscArr + 1) * constrain(dutyPercent, 0, 100)) / 100U;
    }

    void initEscTimerRegisters() {
        __HAL_RCC_AFIO_CLK_ENABLE();
        __HAL_RCC_TIM2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        AFIO->MAPR &= ~AFIO_MAPR_TIM2_REMAP;

        // PA0~PA3 = AF Push-Pull, 50MHz
        GPIOA->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0 |
                        GPIO_CRL_MODE1 | GPIO_CRL_CNF1 |
                        GPIO_CRL_MODE2 | GPIO_CRL_CNF2 |
                        GPIO_CRL_MODE3 | GPIO_CRL_CNF3);
        GPIOA->CRL |= (GPIO_CRL_MODE0_1 | GPIO_CRL_MODE0_0 | GPIO_CRL_CNF0_1 |
                       GPIO_CRL_MODE1_1 | GPIO_CRL_MODE1_0 | GPIO_CRL_CNF1_1 |
                       GPIO_CRL_MODE2_1 | GPIO_CRL_MODE2_0 | GPIO_CRL_CNF2_1 |
                       GPIO_CRL_MODE3_1 | GPIO_CRL_MODE3_0 | GPIO_CRL_CNF3_1);

        TIM2->CR1 = 0;
        TIM2->CR2 = 0;
        TIM2->SMCR = 0;
        TIM2->DIER = 0;
        TIM2->CCER = 0;
        TIM2->CCMR1 = 0;
        TIM2->CCMR2 = 0;
        TIM2->PSC = kEscPrescaler;
        TIM2->ARR = kEscArr;
        TIM2->CCR1 = microsToEscTicks(lastMotor1Us);
        TIM2->CCR2 = microsToEscTicks(lastMotor2Us);
        TIM2->CCR3 = microsToEscTicks(lastMotor3Us);
        TIM2->CCR4 = microsToEscTicks(lastMotor4Us);
        TIM2->EGR = TIM_EGR_UG;

        TIM2->CCMR1 |= (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
        TIM2->CCMR1 |= (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
        TIM2->CCMR2 |= (6U << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
        TIM2->CCMR2 |= (6U << TIM_CCMR2_OC4M_Pos) | TIM_CCMR2_OC4PE;

        TIM2->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
        TIM2->CR1 |= TIM_CR1_ARPE;
        TIM2->CR1 |= TIM_CR1_CEN;
    }

    void writeEscTicks(uint32_t motor1Ticks, uint32_t motor2Ticks, uint32_t motor3Ticks, uint32_t motor4Ticks) {
        TIM2->CCR1 = motor1Ticks;
        TIM2->CCR2 = motor2Ticks;
        TIM2->CCR3 = motor3Ticks;
        TIM2->CCR4 = motor4Ticks;
    }
#endif

    void setup() {
        lastMotor1Us = Config::kServoMinUs;
        lastMotor2Us = Config::kServoMinUs;
        lastMotor3Us = Config::kServoMinUs;
        lastMotor4Us = Config::kServoMinUs;
        lastAuxUs = Config::kServoMidUs;

#ifdef BOARD_STM32
        initEscTimerRegisters();

        auxTimer.pause();
        auxTimer.setOverflow(Config::kAuxPwmFrequencyHz, HERTZ_FORMAT);
        auxTimer.setMode(1, TIMER_OUTPUT_COMPARE_PWM1, PWM_AUX_PIN);
        auxTimer.setCaptureCompare(1, lastAuxUs, MICROSEC_COMPARE_FORMAT);
        auxTimer.refresh();
        auxTimer.resume();
#endif
        ready = true;
    }

    void applyEscPulseAll(uint16_t pulseUs) {
        if (!ready) {
            return;
        }

        const uint16_t clampedPulseUs = clampMicros(pulseUs);
        lastMotor1Us = clampedPulseUs;
        lastMotor2Us = clampedPulseUs;
        lastMotor3Us = clampedPulseUs;
        lastMotor4Us = clampedPulseUs;

#ifdef BOARD_STM32
        writeEscTicks(
            microsToEscTicks(clampedPulseUs),
            microsToEscTicks(clampedPulseUs),
            microsToEscTicks(clampedPulseUs),
            microsToEscTicks(clampedPulseUs)
        );
#endif
    }

    void setMotorPercents(int motor1Percent, int motor2Percent, int motor3Percent, int motor4Percent, int auxValue) {
        if (!ready) {
            return;
        }

        lastMotor1Us = percentToMicros(motor1Percent);
        lastMotor2Us = percentToMicros(motor2Percent);
        lastMotor3Us = percentToMicros(motor3Percent);
        lastMotor4Us = percentToMicros(motor4Percent);
        lastAuxUs = percentToMicros(auxValue);

#ifdef BOARD_STM32
        writeEscTicks(
            microsToEscTicks(lastMotor1Us),
            microsToEscTicks(lastMotor2Us),
            microsToEscTicks(lastMotor3Us),
            microsToEscTicks(lastMotor4Us)
        );

        auxTimer.setCaptureCompare(1, lastAuxUs, MICROSEC_COMPARE_FORMAT);
        auxTimer.refresh();
#endif
    }

    void setEscSquareDutyAll(uint8_t dutyPercent) {
#ifdef BOARD_STM32
        lastMotor1Us = 0;
        lastMotor2Us = 0;
        lastMotor3Us = 0;
        lastMotor4Us = 0;
        const uint32_t dutyTicks = dutyToEscTicks(dutyPercent);
        writeEscTicks(dutyTicks, dutyTicks, dutyTicks, dutyTicks);
#else
        (void)dutyPercent;
#endif
    }

    void logEscTimerState(const char *label) {
#ifdef BOARD_STM32
        Debug::logf(
            "%s freq=%luHz psc=%lu arr=%lu ccr1=%lu ccr2=%lu ccr3=%lu ccr4=%lu\r\n",
            label,
            static_cast<unsigned long>(Config::kEscPwmFrequencyHz),
            static_cast<unsigned long>(TIM2->PSC),
            static_cast<unsigned long>(TIM2->ARR + 1),
            static_cast<unsigned long>(TIM2->CCR1),
            static_cast<unsigned long>(TIM2->CCR2),
            static_cast<unsigned long>(TIM2->CCR3),
            static_cast<unsigned long>(TIM2->CCR4)
        );
#else
        (void)label;
#endif
    }
}

namespace ManualControl {
    bool enabled = false;
    int masterThrottlePercent = 0;
    int motorPercents[4] = {0, 0, 0, 0};

    void setEnabled(bool value) {
        enabled = value;
        if (!enabled) {
            masterThrottlePercent = 0;
            for (int &motorPercent : motorPercents) {
                motorPercent = 0;
            }
        }
    }

    void setAllMotorsPercent(int percent) {
        masterThrottlePercent = constrain(percent, 0, 100);
        for (int &motorPercent : motorPercents) {
            motorPercent = masterThrottlePercent;
        }
    }

    void setMotorPercent(size_t motorIndex, int percent) {
        if (motorIndex >= 4) {
            return;
        }

        motorPercents[motorIndex] = constrain(percent, 0, 100);
    }
}

namespace PwmTest {
    enum class Mode : uint8_t {
        Off = 0,
        SquareWave400Hz,
        EscPulse400Hz,
        RawTim2Ch1Square400Hz,
    };

    Mode mode = Mode::Off;
    uint16_t pulseUs = 1500;
    uint8_t dutyPercent = 50;

    void setMode(Mode value) {
        mode = value;
    }

    void setPulseUs(int pulse) {
        pulseUs = constrain(pulse, static_cast<int>(Config::kServoMinUs), static_cast<int>(Config::kServoMaxUs));
    }

    void setDutyPercent(int duty) {
        dutyPercent = constrain(duty, 0, 100);
    }
}

namespace PinTest {
    enum class Mode : uint8_t {
        Off = 0,
        Pa0High,
        Pa0Low,
        Pa0Blink1Hz,
    };

    Mode mode = Mode::Off;
    bool pa0StateHigh = false;
    uint32_t nextToggleTimeMs = 0;

    void applyPa0Level(bool high) {
        pinMode(PWM_AILERON_PIN, OUTPUT);
        digitalWrite(PWM_AILERON_PIN, high ? HIGH : LOW);
        pa0StateHigh = high;
    }

    void restorePwmOutputs() {
        PWM::setup();
    }

    void setMode(Mode value) {
        mode = value;
        if (mode == Mode::Off) {
            restorePwmOutputs();
            Debug::logf("PINTEST PA0 OFF, PWM restored\r\n");
            return;
        }

        if (mode == Mode::Pa0High) {
            applyPa0Level(true);
            Debug::logf("PINTEST PA0 HIGH OK\r\n");
            return;
        }

        if (mode == Mode::Pa0Low) {
            applyPa0Level(false);
            Debug::logf("PINTEST PA0 LOW OK\r\n");
            return;
        }

        applyPa0Level(false);
        nextToggleTimeMs = millis() + 500;
        Debug::logf("PINTEST PA0 BLINK1HZ OK\r\n");
    }

    void loop() {
        if (mode != Mode::Pa0Blink1Hz) {
            return;
        }

        const uint32_t now = millis();
        if (now < nextToggleTimeMs) {
            return;
        }

        applyPa0Level(!pa0StateHigh);
        nextToggleTimeMs = now + 500;
    }
}

namespace RawTimerTest {
    bool enabled = false;

    void enablePa0Tim2Square400Hz50() {
        if (enabled) {
            return;
        }

#ifdef BOARD_STM32
        pinMode(PWM_AILERON_PIN, OUTPUT);

        __HAL_RCC_AFIO_CLK_ENABLE();
        __HAL_RCC_TIM2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        AFIO->MAPR &= ~AFIO_MAPR_TIM2_REMAP;

        GPIOA->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0);
        GPIOA->CRL |= GPIO_CRL_MODE0_1 | GPIO_CRL_MODE0_0;
        GPIOA->CRL |= GPIO_CRL_CNF0_1;

        TIM2->CR1 = 0;
        TIM2->CR2 = 0;
        TIM2->SMCR = 0;
        TIM2->DIER = 0;
        TIM2->CCER = 0;
        TIM2->CCMR1 = 0;
        TIM2->CCMR2 = 0;
        TIM2->PSC = 8 - 1;       // 72MHz / 8 = 9MHz timer clock
        TIM2->ARR = 22500 - 1;   // 9MHz / 22500 = 400Hz
        TIM2->CCR1 = 11250;      // 50% duty
        TIM2->EGR = TIM_EGR_UG;
        TIM2->CCMR1 |= (6U << TIM_CCMR1_OC1M_Pos);
        TIM2->CCMR1 |= TIM_CCMR1_OC1PE;
        TIM2->CCER |= TIM_CCER_CC1E;
        TIM2->CR1 |= TIM_CR1_ARPE;
        TIM2->BDTR = 0;
        TIM2->CR1 |= TIM_CR1_CEN;
#endif
        enabled = true;
    }

    void disable() {
        enabled = false;
        PWM::setup();
    }

    void logState(const char *label) {
#ifdef BOARD_STM32
        Debug::logf(
            "%s MAPR=0x%08lX GPIOA_CRL=0x%08lX TIM2_CR1=0x%08lX TIM2_CCMR1=0x%08lX TIM2_CCER=0x%08lX PSC=%lu ARR=%lu CCR1=%lu\r\n",
            label,
            static_cast<unsigned long>(AFIO->MAPR),
            static_cast<unsigned long>(GPIOA->CRL),
            static_cast<unsigned long>(TIM2->CR1),
            static_cast<unsigned long>(TIM2->CCMR1),
            static_cast<unsigned long>(TIM2->CCER),
            static_cast<unsigned long>(TIM2->PSC),
            static_cast<unsigned long>(TIM2->ARR),
            static_cast<unsigned long>(TIM2->CCR1)
        );
#else
        (void)label;
#endif
    }
}

namespace GYRO {
    struct ImuSample {
        int16_t accelX;
        int16_t accelY;
        int16_t accelZ;
        int16_t temperature;
        int16_t gyroX;
        int16_t gyroY;
        int16_t gyroZ;
    };

    struct ImuScaledSample {
        float accelXG;
        float accelYG;
        float accelZG;
        float gyroXDps;
        float gyroYDps;
        float gyroZDps;
        float temperatureC;
    };

    uint32_t lastUpdateTimeUs = 0;
    bool ready = false;
    bool calibrated = false;
    bool healthCheckComplete = false;
    uint8_t whoAmI = 0;
    float gyroOffsetX = 0.0f;
    float gyroOffsetY = 0.0f;
    float gyroOffsetZ = 0.0f;
    float accelOffsetX = 0.0f;
    float accelOffsetY = 0.0f;
    float accelOffsetZ = 0.0f;
    ImuScaledSample latestScaledSample {};
    float accelRollDeg = 0.0f;
    float accelPitchDeg = 0.0f;
    float rollDeg = 0.0f;
    float pitchDeg = 0.0f;
    float yawDeg = 0.0f;
    float lastDtSeconds = 0.0f;
    uint32_t healthCheckStartTimeMs = 0;
    uint32_t nextAutoCalibrationTimeMs = 0;
    uint16_t autoCalibrationAttemptCount = 0;
    uint32_t healthSampleCount = 0;
    float healthGyroAbsSumX = 0.0f;
    float healthGyroAbsSumY = 0.0f;
    float healthGyroAbsSumZ = 0.0f;
    float healthGyroMaxAbsX = 0.0f;
    float healthGyroMaxAbsY = 0.0f;
    float healthGyroMaxAbsZ = 0.0f;
    float healthAccelMagnitudeSum = 0.0f;
    float healthAccelMagnitudeMin = 1000.0f;
    float healthAccelMagnitudeMax = 0.0f;
    int16_t calibrationGyroMinX = INT16_MAX;
    int16_t calibrationGyroMinY = INT16_MAX;
    int16_t calibrationGyroMinZ = INT16_MAX;
    int16_t calibrationGyroMaxX = INT16_MIN;
    int16_t calibrationGyroMaxY = INT16_MIN;
    int16_t calibrationGyroMaxZ = INT16_MIN;
    float calibrationAccelMagnitudeMin = 1000.0f;
    float calibrationAccelMagnitudeMax = 0.0f;
    constexpr float kAccelLsbPerG = 16384.0f;
    constexpr float kGyroLsbPerDps = 131.0f;
    constexpr float kComplementaryAlpha = 0.98f;
    constexpr int16_t kCalibrationGyroSpanLimit = 350;
    constexpr float kCalibrationAccelMagnitudeSpanLimit = 0.18f;

    void printPlotterHeader();
    bool isReadyForTelemetry();

    float absfLocal(float value) {
        return (value >= 0.0f) ? value : -value;
    }

    void resetHealthCheck() {
        healthCheckComplete = false;
        healthCheckStartTimeMs = millis();
        healthSampleCount = 0;
        healthGyroAbsSumX = 0.0f;
        healthGyroAbsSumY = 0.0f;
        healthGyroAbsSumZ = 0.0f;
        healthGyroMaxAbsX = 0.0f;
        healthGyroMaxAbsY = 0.0f;
        healthGyroMaxAbsZ = 0.0f;
        healthAccelMagnitudeSum = 0.0f;
        healthAccelMagnitudeMin = 1000.0f;
        healthAccelMagnitudeMax = 0.0f;
    }

    bool writeRegister(uint8_t reg, uint8_t value) {
        MpuWire.beginTransmission(Config::kMpuAddress);
        MpuWire.write(reg);
        MpuWire.write(value);
        return MpuWire.endTransmission() == 0;
    }

    bool readBytes(uint8_t reg, uint8_t *buffer, size_t length) {
        MpuWire.beginTransmission(Config::kMpuAddress);
        MpuWire.write(reg);

        if (MpuWire.endTransmission(false) != 0) {
            return false;
        }

        const size_t received = MpuWire.requestFrom(Config::kMpuAddress, static_cast<uint8_t>(length));
        if (received != length) {
            return false;
        }

        for (size_t i = 0; i < length; ++i) {
            buffer[i] = static_cast<uint8_t>(MpuWire.read());
        }

        return true;
    }

    bool readSample(ImuSample &sample) {
        uint8_t raw[14] = {};
        if (!readBytes(Config::kRegAccelData, raw, sizeof(raw))) {
            return false;
        }

        sample.accelX = static_cast<int16_t>((raw[0] << 8) | raw[1]);
        sample.accelY = static_cast<int16_t>((raw[2] << 8) | raw[3]);
        sample.accelZ = static_cast<int16_t>((raw[4] << 8) | raw[5]);
        sample.temperature = static_cast<int16_t>((raw[6] << 8) | raw[7]);
        sample.gyroX = static_cast<int16_t>((raw[8] << 8) | raw[9]);
        sample.gyroY = static_cast<int16_t>((raw[10] << 8) | raw[11]);
        sample.gyroZ = static_cast<int16_t>((raw[12] << 8) | raw[13]);
        return true;
    }

    ImuScaledSample scaleSample(const ImuSample &sample) {
        ImuScaledSample scaled {};
        scaled.accelXG = (static_cast<float>(sample.accelX) - accelOffsetX) / kAccelLsbPerG;
        scaled.accelYG = (static_cast<float>(sample.accelY) - accelOffsetY) / kAccelLsbPerG;
        scaled.accelZG = (static_cast<float>(sample.accelZ) - accelOffsetZ) / kAccelLsbPerG;
        scaled.gyroXDps = (static_cast<float>(sample.gyroX) - gyroOffsetX) / kGyroLsbPerDps;
        scaled.gyroYDps = (static_cast<float>(sample.gyroY) - gyroOffsetY) / kGyroLsbPerDps;
        scaled.gyroZDps = (static_cast<float>(sample.gyroZ) - gyroOffsetZ) / kGyroLsbPerDps;
        scaled.temperatureC = static_cast<float>(sample.temperature) / 340.0f + 36.53f;
        return scaled;
    }

    bool calibrate() {
        Debug::logf("IMU calibration start, keep the board still on the desk...\r\n");

        int64_t accelSumX = 0;
        int64_t accelSumY = 0;
        int64_t accelSumZ = 0;
        int64_t gyroSumX = 0;
        int64_t gyroSumY = 0;
        int64_t gyroSumZ = 0;
        uint16_t validSamples = 0;

        calibrationGyroMinX = INT16_MAX;
        calibrationGyroMinY = INT16_MAX;
        calibrationGyroMinZ = INT16_MAX;
        calibrationGyroMaxX = INT16_MIN;
        calibrationGyroMaxY = INT16_MIN;
        calibrationGyroMaxZ = INT16_MIN;
        calibrationAccelMagnitudeMin = 1000.0f;
        calibrationAccelMagnitudeMax = 0.0f;

        for (uint16_t i = 0; i < Config::kImuCalibrationSamples; ++i) {
            ImuSample sample {};
            if (!readSample(sample)) {
                delay(2);
                continue;
            }

            accelSumX += sample.accelX;
            accelSumY += sample.accelY;
            accelSumZ += sample.accelZ;
            gyroSumX += sample.gyroX;
            gyroSumY += sample.gyroY;
            gyroSumZ += sample.gyroZ;
            calibrationGyroMinX = min(calibrationGyroMinX, sample.gyroX);
            calibrationGyroMinY = min(calibrationGyroMinY, sample.gyroY);
            calibrationGyroMinZ = min(calibrationGyroMinZ, sample.gyroZ);
            calibrationGyroMaxX = max(calibrationGyroMaxX, sample.gyroX);
            calibrationGyroMaxY = max(calibrationGyroMaxY, sample.gyroY);
            calibrationGyroMaxZ = max(calibrationGyroMaxZ, sample.gyroZ);

            const float accelMagnitude = sqrtf(
                (static_cast<float>(sample.accelX) * static_cast<float>(sample.accelX)) +
                (static_cast<float>(sample.accelY) * static_cast<float>(sample.accelY)) +
                (static_cast<float>(sample.accelZ) * static_cast<float>(sample.accelZ))
            ) / kAccelLsbPerG;
            calibrationAccelMagnitudeMin = min(calibrationAccelMagnitudeMin, accelMagnitude);
            calibrationAccelMagnitudeMax = max(calibrationAccelMagnitudeMax, accelMagnitude);
            ++validSamples;
            delay(2);
        }

        if (validSamples < (Config::kImuCalibrationSamples / 2)) {
            Debug::logf("IMU calibration failed, validSamples=%u\r\n", validSamples);
            return false;
        }

        accelOffsetX = static_cast<float>(accelSumX) / validSamples;
        accelOffsetY = static_cast<float>(accelSumY) / validSamples;
        accelOffsetZ = static_cast<float>(accelSumZ) / validSamples;
        gyroOffsetX = static_cast<float>(gyroSumX) / validSamples;
        gyroOffsetY = static_cast<float>(gyroSumY) / validSamples;
        gyroOffsetZ = static_cast<float>(gyroSumZ) / validSamples;

        // Preserve 1g on the vertical axis after calibration.
        accelOffsetZ += (accelOffsetZ >= 0.0f) ? -kAccelLsbPerG : kAccelLsbPerG;

        const int16_t gyroSpanX = calibrationGyroMaxX - calibrationGyroMinX;
        const int16_t gyroSpanY = calibrationGyroMaxY - calibrationGyroMinY;
        const int16_t gyroSpanZ = calibrationGyroMaxZ - calibrationGyroMinZ;
        const float accelMagnitudeSpan = calibrationAccelMagnitudeMax - calibrationAccelMagnitudeMin;
        const bool boardStillEnough = (gyroSpanX < kCalibrationGyroSpanLimit) &&
                                      (gyroSpanY < kCalibrationGyroSpanLimit) &&
                                      (gyroSpanZ < kCalibrationGyroSpanLimit) &&
                                      (accelMagnitudeSpan < kCalibrationAccelMagnitudeSpanLimit);

        if (!boardStillEnough) {
            Debug::logf("IMU calibration failed: board moved ");
            Debug::logf("gyroSpan=(%d,%d,%d)", gyroSpanX, gyroSpanY, gyroSpanZ);
            Debug::comma();
            Debug::keyValue("accelMagSpan_g", accelMagnitudeSpan, 3);
            Debug::endLine();
            calibrated = false;
            return false;
        }

        ImuScaledSample calibratedGravity {};
        calibratedGravity.accelXG = 0.0f;
        calibratedGravity.accelYG = 0.0f;
        calibratedGravity.accelZG = (accelOffsetZ >= 0.0f) ? 1.0f : -1.0f;
        accelRollDeg = atan2f(calibratedGravity.accelYG, calibratedGravity.accelZG) * RAD_TO_DEG;
        accelPitchDeg = atan2f(calibratedGravity.accelXG, sqrtf(
            calibratedGravity.accelYG * calibratedGravity.accelYG +
            calibratedGravity.accelZG * calibratedGravity.accelZG
        )) * RAD_TO_DEG;
        rollDeg = accelRollDeg;
        pitchDeg = accelPitchDeg;
        yawDeg = 0.0f;
        calibrated = true;

        Debug::logf("IMU calibration done\r\n");
        Debug::keyValue("gyroOffsetX", gyroOffsetX, 1);
        Debug::comma();
        Debug::keyValue("gyroOffsetY", gyroOffsetY, 1);
        Debug::comma();
        Debug::keyValue("gyroOffsetZ", gyroOffsetZ, 1);
        Debug::comma();
        Debug::keyValue("accelOffsetX", accelOffsetX, 1);
        Debug::comma();
        Debug::keyValue("accelOffsetY", accelOffsetY, 1);
        Debug::comma();
        Debug::keyValue("accelOffsetZ", accelOffsetZ, 1);
        Debug::endLine();
        return true;
    }

    bool recalibrate() {
        if (!ready) {
            Debug::logf("IMU recalibration failed: sensor not ready\r\n");
            calibrated = false;
            return false;
        }

        calibrated = calibrate();
        lastUpdateTimeUs = micros();
        lastDtSeconds = 0.0f;
        latestScaledSample = {};
        resetHealthCheck();
        if (calibrated) {
            printPlotterHeader();
        }
        return calibrated;
    }

    void scheduleAutoCalibration(uint32_t delayMs) {
        nextAutoCalibrationTimeMs = millis() + delayMs;
    }

    void resetYaw() {
        yawDeg = 0.0f;
        Debug::logf("Yaw reset to 0 deg\r\n");
    }

    void updateAttitude(const ImuScaledSample &sample, float dtSeconds) {
        accelRollDeg = atan2f(sample.accelYG, sample.accelZG) * RAD_TO_DEG;
        accelPitchDeg = atan2f(sample.accelXG, sqrtf(sample.accelYG * sample.accelYG + sample.accelZG * sample.accelZG)) * RAD_TO_DEG;

        if (dtSeconds <= 0.0f || dtSeconds > 0.5f) {
            rollDeg = accelRollDeg;
            pitchDeg = accelPitchDeg;
            return;
        }

        rollDeg = (kComplementaryAlpha * (rollDeg + sample.gyroXDps * dtSeconds)) + ((1.0f - kComplementaryAlpha) * accelRollDeg);
        pitchDeg = (kComplementaryAlpha * (pitchDeg + sample.gyroYDps * dtSeconds)) + ((1.0f - kComplementaryAlpha) * accelPitchDeg);
        yawDeg += sample.gyroZDps * dtSeconds;
    }

    void updateHealthCheck(const ImuScaledSample &sample) {
        if (healthCheckComplete) {
            return;
        }

        const float absGyroX = absfLocal(sample.gyroXDps);
        const float absGyroY = absfLocal(sample.gyroYDps);
        const float absGyroZ = absfLocal(sample.gyroZDps);
        const float accelMagnitude = sqrtf(
            (sample.accelXG * sample.accelXG) +
            (sample.accelYG * sample.accelYG) +
            (sample.accelZG * sample.accelZG)
        );

        healthGyroAbsSumX += absGyroX;
        healthGyroAbsSumY += absGyroY;
        healthGyroAbsSumZ += absGyroZ;
        healthGyroMaxAbsX = max(healthGyroMaxAbsX, absGyroX);
        healthGyroMaxAbsY = max(healthGyroMaxAbsY, absGyroY);
        healthGyroMaxAbsZ = max(healthGyroMaxAbsZ, absGyroZ);
        healthAccelMagnitudeSum += accelMagnitude;
        healthAccelMagnitudeMin = min(healthAccelMagnitudeMin, accelMagnitude);
        healthAccelMagnitudeMax = max(healthAccelMagnitudeMax, accelMagnitude);
        ++healthSampleCount;

        const uint32_t now = millis();
        if ((now - healthCheckStartTimeMs) < Config::kImuHealthCheckDurationMs) {
            return;
        }

        if (healthSampleCount == 0) {
            Debug::logf("IMU health check failed: no samples collected\r\n");
            healthCheckComplete = true;
            return;
        }

        const float avgGyroX = healthGyroAbsSumX / healthSampleCount;
        const float avgGyroY = healthGyroAbsSumY / healthSampleCount;
        const float avgGyroZ = healthGyroAbsSumZ / healthSampleCount;
        const float avgAccelMagnitude = healthAccelMagnitudeSum / healthSampleCount;
        const float accelSpan = healthAccelMagnitudeMax - healthAccelMagnitudeMin;
        const bool gyroHealthy = (avgGyroX < 1.5f) && (avgGyroY < 1.5f) && (avgGyroZ < 1.5f) &&
                                (healthGyroMaxAbsX < 4.0f) && (healthGyroMaxAbsY < 4.0f) && (healthGyroMaxAbsZ < 4.0f);
        const bool accelHealthy = (avgAccelMagnitude > 0.90f) && (avgAccelMagnitude < 1.10f) && (accelSpan < 0.15f);

        Debug::logf("IMU health %s\r\n", (gyroHealthy && accelHealthy) ? "OK" : "WARN");
        Debug::keyValue("avgGyroX_dps", avgGyroX, 2);
        Debug::comma();
        Debug::keyValue("avgGyroY_dps", avgGyroY, 2);
        Debug::comma();
        Debug::keyValue("avgGyroZ_dps", avgGyroZ, 2);
        Debug::comma();
        Debug::keyValue("maxGyroX_dps", healthGyroMaxAbsX, 2);
        Debug::comma();
        Debug::keyValue("maxGyroY_dps", healthGyroMaxAbsY, 2);
        Debug::comma();
        Debug::keyValue("maxGyroZ_dps", healthGyroMaxAbsZ, 2);
        Debug::comma();
        Debug::keyValue("accelMagAvg_g", avgAccelMagnitude, 3);
        Debug::comma();
        Debug::keyValue("accelMagSpan_g", accelSpan, 3);
        Debug::endLine();
        healthCheckComplete = true;
    }

    void printPlotterHeader() {
        Debug::logf("PLOTTER fields: ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,roll_deg,pitch_deg,yaw_deg,rc1,rc2,rc3,rc4,rc5,pid_roll_out,pid_pitch_out,pid_yaw_out,motor1_us,motor2_us,motor3_us,motor4_us\r\n");
    }

    bool isReadyForTelemetry() {
        return ready && calibrated;
    }

    void setup() {
        MpuWire.begin();
        MpuWire.setClock(400000);
        delay(100);

        uint8_t id = 0;
        if (!readBytes(Config::kRegWhoAmI, &id, 1)) {
            Debug::logf("MPU6050 read WHO_AM_I failed. Check PB8/PB9 wiring and 3.3V power.\r\n");
            return;
        }

        whoAmI = id;
        ready = (whoAmI == 0x68 || whoAmI == 0x70);

        if (!ready) {
            Debug::logf("MPU6050 unexpected WHO_AM_I=0x%02X\r\n", whoAmI);
            return;
        }

        const bool wakeOk = writeRegister(Config::kRegPwrMgmt1, 0x00);
        const bool accelOk = writeRegister(Config::kRegAccelConfig, 0x00);
        const bool gyroOk = writeRegister(Config::kRegGyroConfig, 0x00);

        ready = wakeOk && accelOk && gyroOk;
        Debug::logf("MPU6050 ready=%d WHO_AM_I=0x%02X on SDA=%d SCL=%d\r\n", ready ? 1 : 0, whoAmI, MPU6050_I2C_SDA, MPU6050_I2C_SCL);

        if (ready) {
            lastUpdateTimeUs = micros();
            resetHealthCheck();
            autoCalibrationAttemptCount = 0;
            scheduleAutoCalibration(Config::kImuBootSettleDelayMs);
            Debug::logf(
                "IMU auto calibration scheduled in %lu ms, keep the board still after power-on.\r\n",
                Config::kImuBootSettleDelayMs
            );
        }
    }

    void loop() {
        if (!ready) {
            return;
        }

        const uint32_t now = millis();
        if (!calibrated) {
            if (now < nextAutoCalibrationTimeMs) {
                return;
            }

            ++autoCalibrationAttemptCount;
            Debug::logf("IMU auto calibration attempt=%u\r\n", autoCalibrationAttemptCount);
            calibrated = calibrate();
            lastUpdateTimeUs = micros();
            lastDtSeconds = 0.0f;
            latestScaledSample = {};
            resetHealthCheck();

            if (calibrated) {
                Debug::logf("IMU auto calibration OK\r\n");
                printPlotterHeader();
            } else {
                Debug::logf(
                    "IMU auto calibration retry in %lu ms\r\n",
                    Config::kImuCalibrationRetryIntervalMs
                );
                scheduleAutoCalibration(Config::kImuCalibrationRetryIntervalMs);
            }
            return;
        }

        ImuSample sample {};
        if (readSample(sample)) {
            const ImuScaledSample scaled = scaleSample(sample);
            const uint32_t currentTimeUs = micros();
            const float dtSeconds = (lastUpdateTimeUs == 0)
                ? 0.0f
                : static_cast<float>(currentTimeUs - lastUpdateTimeUs) / 1000000.0f;
            lastUpdateTimeUs = currentTimeUs;
            lastDtSeconds = dtSeconds;
            latestScaledSample = scaled;

            updateAttitude(scaled, dtSeconds);
            updateHealthCheck(scaled);
        } else {
            Debug::logf("MPU6050 data read failed\r\n");
        }
    }
}

namespace ELRS {
    extern uint16_t rcValues[RC_INPUT_MAX_CHANNELS];
    extern uint16_t rcCount;
    extern bool signalValid;
    extern int rollPercent;
    extern int pitchPercent;
    extern int throttlePercent;
    extern int yawPercent;
    extern int auxPercent;
}

namespace PID {
    struct AxisState {
        float angleTargetDeg = 0.0f;
        float angleMeasurementDeg = 0.0f;
        float angleErrorDeg = 0.0f;
        float angleIntegral = 0.0f;
        float angleOutputRateDegPerSec = 0.0f;
        float angleKp = 0.0f;
        float angleKi = 0.0f;

        float rateTargetDegPerSec = 0.0f;
        float rateMeasurementDegPerSec = 0.0f;
        float rateErrorDegPerSec = 0.0f;
        float rateIntegral = 0.0f;
        float rateDerivative = 0.0f;
        float output = 0.0f;
        float rateKp = 0.0f;
        float rateKi = 0.0f;
        float kp = 0.0f;
        float kd = 0.0f;
        float lastRateMeasurementDegPerSec = 0.0f;
        float outputLimit = 0.0f;
        float integralLimit = 0.0f;
    };

    AxisState roll;
    AxisState pitch;
    AxisState yaw;
    uint32_t nextPrintTimeMs = 0;

    float rcToNormalized(uint16_t rcValue) {
        const float normalized = (static_cast<float>(rcValue) - 1500.0f) / 500.0f;
        return constrain(normalized, -1.0f, 1.0f);
    }

    float clampAbs(float value, float limit) {
        return constrain(value, -limit, limit);
    }

    float applyDeadband(float value, float deadband) {
        if (fabsf(value) <= deadband) {
            return 0.0f;
        }

        if (value > 0.0f) {
            return (value - deadband) / (1.0f - deadband);
        }

        return (value + deadband) / (1.0f - deadband);
    }

    void resetAxis(AxisState &axis) {
        axis.angleIntegral = 0.0f;
        axis.angleOutputRateDegPerSec = 0.0f;
        axis.rateTargetDegPerSec = 0.0f;
        axis.rateIntegral = 0.0f;
        axis.rateDerivative = 0.0f;
        axis.output = 0.0f;
        axis.lastRateMeasurementDegPerSec = axis.rateMeasurementDegPerSec;
    }

    void resetAll() {
        resetAxis(roll);
        resetAxis(pitch);
        resetAxis(yaw);
    }

    void setup() {
        roll.angleKp = 4.5f;
        roll.angleKi = 0.1f;
        roll.rateKp = 0.20f;
        roll.rateKi = 0.08f;
        roll.kd = 0.002f;
        roll.outputLimit = Config::kSurfaceOutputLimit;
        roll.integralLimit = Config::kIntegralLimitRate;

        pitch.angleKp = 4.5f;
        pitch.angleKi = 0.1f;
        pitch.rateKp = 0.20f;
        pitch.rateKi = 0.08f;
        pitch.kd = 0.002f;
        pitch.outputLimit = Config::kSurfaceOutputLimit;
        pitch.integralLimit = Config::kIntegralLimitRate;

        yaw.angleKp = 0.0f;
        yaw.angleKi = 0.0f;
        yaw.rateKp = 0.18f;
        yaw.rateKi = 0.05f;
        yaw.kd = 0.001f;
        yaw.outputLimit = Config::kYawOutputLimit;
        yaw.integralLimit = Config::kIntegralLimitRate;
        Debug::logf("PID debug framework ready\r\n");
    }

    void updateAngleLoop(AxisState &axis, float measurementDeg, float maxRateDegPerSec, float dtSeconds) {
        axis.angleMeasurementDeg = measurementDeg;
        axis.angleErrorDeg = axis.angleTargetDeg - axis.angleMeasurementDeg;

        if (dtSeconds > 0.0f && dtSeconds < 0.5f) {
            axis.angleIntegral += axis.angleErrorDeg * dtSeconds;
            axis.angleIntegral = clampAbs(axis.angleIntegral, Config::kIntegralLimitAngle);
        }

        axis.angleOutputRateDegPerSec =
            (axis.angleKp * axis.angleErrorDeg) +
            (axis.angleKi * axis.angleIntegral);
        axis.angleOutputRateDegPerSec = clampAbs(axis.angleOutputRateDegPerSec, maxRateDegPerSec);
    }

    void updateRateLoop(AxisState &axis, float measurementRateDegPerSec, float dtSeconds) {
        axis.rateMeasurementDegPerSec = measurementRateDegPerSec;
        axis.rateErrorDegPerSec = axis.rateTargetDegPerSec - axis.rateMeasurementDegPerSec;

        if (dtSeconds > 0.0f && dtSeconds < 0.5f) {
            const float proposedIntegral = axis.rateIntegral + (axis.rateErrorDegPerSec * dtSeconds);
            const float derivative = (axis.rateMeasurementDegPerSec - axis.lastRateMeasurementDegPerSec) / dtSeconds;
            float unclampedOutput =
                (axis.rateKp * axis.rateErrorDegPerSec) +
                (axis.rateKi * proposedIntegral) -
                (axis.kd * derivative);

            if ((unclampedOutput < axis.outputLimit && unclampedOutput > -axis.outputLimit) ||
                ((unclampedOutput >= axis.outputLimit) && (axis.rateErrorDegPerSec < 0.0f)) ||
                ((unclampedOutput <= -axis.outputLimit) && (axis.rateErrorDegPerSec > 0.0f))) {
                axis.rateIntegral = clampAbs(proposedIntegral, axis.integralLimit);
            }

            axis.rateDerivative = derivative;
        } else {
            axis.rateDerivative = 0.0f;
        }

        axis.output =
            (axis.rateKp * axis.rateErrorDegPerSec) +
            (axis.rateKi * axis.rateIntegral) -
            (axis.kd * axis.rateDerivative);
        axis.output = clampAbs(axis.output, axis.outputLimit);
        axis.lastRateMeasurementDegPerSec = axis.rateMeasurementDegPerSec;
    }

    void applyControlOutputs() {
        if (PwmTest::mode == PwmTest::Mode::SquareWave400Hz) {
            PWM::setEscSquareDutyAll(PwmTest::dutyPercent);
            return;
        }

        if (PwmTest::mode == PwmTest::Mode::EscPulse400Hz) {
            const int testPercent = map(PwmTest::pulseUs, Config::kServoMinUs, Config::kServoMaxUs, 0, 100);
            PWM::setMotorPercents(
                testPercent,
                testPercent,
                testPercent,
                testPercent,
                50);
            return;
        }

        if (PwmTest::mode == PwmTest::Mode::RawTim2Ch1Square400Hz) {
            if (!RawTimerTest::enabled) {
                RawTimerTest::enablePa0Tim2Square400Hz50();
            }
            return;
        }

        if (ManualControl::enabled) {
            PWM::setMotorPercents(
                ManualControl::motorPercents[0],
                ManualControl::motorPercents[1],
                ManualControl::motorPercents[2],
                ManualControl::motorPercents[3],
                50);
            return;
        }

        const int throttlePercent = constrain(ELRS::throttlePercent, 0, 100);
        const int auxPercent = constrain(ELRS::auxPercent, 0, 100);
        const float rollMix = roll.output;
        const float pitchMix = pitch.output;
        const float yawMix = yaw.output;

        // Quad X mixer:
        // M1 = Front Left  (CCW)
        // M2 = Front Right (CW)
        // M3 = Rear Right  (CCW)
        // M4 = Rear Left   (CW)
        const int motor1Percent = constrain(static_cast<int>(throttlePercent + pitchMix + rollMix - yawMix), 0, 100);
        const int motor2Percent = constrain(static_cast<int>(throttlePercent + pitchMix - rollMix + yawMix), 0, 100);
        const int motor3Percent = constrain(static_cast<int>(throttlePercent - pitchMix - rollMix - yawMix), 0, 100);
        const int motor4Percent = constrain(static_cast<int>(throttlePercent - pitchMix + rollMix + yawMix), 0, 100);
        PWM::setMotorPercents(motor1Percent, motor2Percent, motor3Percent, motor4Percent, auxPercent);
    }

    void printTelemetry(float yawDeg) {
        const uint32_t now = millis();
        if (now < nextPrintTimeMs) {
            return;
        }

        Debug::csvFloat(GYRO::latestScaledSample.accelXG, 3);
        Debug::comma();
        Debug::csvFloat(GYRO::latestScaledSample.accelYG, 3);
        Debug::comma();
        Debug::csvFloat(GYRO::latestScaledSample.accelZG, 3);
        Debug::comma();
        Debug::csvFloat(GYRO::latestScaledSample.gyroXDps, 2);
        Debug::comma();
        Debug::csvFloat(GYRO::latestScaledSample.gyroYDps, 2);
        Debug::comma();
        Debug::csvFloat(GYRO::latestScaledSample.gyroZDps, 2);
        Debug::comma();
        Debug::csvFloat(roll.angleMeasurementDeg, 2);
        Debug::comma();
        Debug::csvFloat(pitch.angleMeasurementDeg, 2);
        Debug::comma();
        Debug::csvFloat(yawDeg, 2);
        Debug::comma();
        Debug::csvFloat(static_cast<float>(ELRS::rcValues[0]), 0);
        Debug::comma();
        Debug::csvFloat(static_cast<float>(ELRS::rcValues[1]), 0);
        Debug::comma();
        Debug::csvFloat(static_cast<float>(ELRS::rcValues[2]), 0);
        Debug::comma();
        Debug::csvFloat(static_cast<float>(ELRS::rcValues[3]), 0);
        Debug::comma();
        Debug::csvFloat(static_cast<float>(ELRS::rcValues[4]), 0);
        Debug::comma();
        Debug::csvFloat(roll.output, 2);
        Debug::comma();
        Debug::csvFloat(pitch.output, 2);
        Debug::comma();
        Debug::csvFloat(yaw.output, 2);
        Debug::comma();
        Debug::csvFloat(static_cast<float>(PWM::lastMotor1Us), 0);
        Debug::comma();
        Debug::csvFloat(static_cast<float>(PWM::lastMotor2Us), 0);
        Debug::comma();
        Debug::csvFloat(static_cast<float>(PWM::lastMotor3Us), 0);
        Debug::comma();
        Debug::csvFloat(static_cast<float>(PWM::lastMotor4Us), 0);
        Debug::endLine();
        nextPrintTimeMs = now + Config::kPidPrintIntervalMs;
    }

    void loop(float rollDeg, float pitchDeg, float yawDeg, float dtSeconds) {
        if (PinTest::mode != PinTest::Mode::Off) {
            return;
        }

        if (PwmTest::mode != PwmTest::Mode::Off) {
            applyControlOutputs();
            if (GYRO::isReadyForTelemetry()) {
                roll.angleMeasurementDeg = rollDeg;
                pitch.angleMeasurementDeg = pitchDeg;
                printTelemetry(yawDeg);
            }
            return;
        }

        if (ManualControl::enabled) {
            applyControlOutputs();
            if (GYRO::isReadyForTelemetry()) {
                roll.angleMeasurementDeg = rollDeg;
                pitch.angleMeasurementDeg = pitchDeg;
                printTelemetry(yawDeg);
            }
            return;
        }

        if (!GYRO::isReadyForTelemetry()) {
            resetAll();
            return;
        }

        if (ELRS::signalValid) {
            const float rollStick = applyDeadband(rcToNormalized(ELRS::rcValues[0]), Config::kStickDeadband);
            const float pitchStick = applyDeadband(rcToNormalized(ELRS::rcValues[1]), Config::kStickDeadband);
            const float yawStick = applyDeadband(rcToNormalized(ELRS::rcValues[3]), Config::kStickDeadband);

            roll.angleTargetDeg = rollStick * Config::kMaxAngleTargetDeg;
            pitch.angleTargetDeg = pitchStick * Config::kMaxAngleTargetDeg;
            yaw.rateTargetDegPerSec = yawStick * Config::kMaxYawRateDegPerSec;
        } else {
            roll.angleTargetDeg = 0.0f;
            pitch.angleTargetDeg = 0.0f;
            yaw.rateTargetDegPerSec = 0.0f;
            resetAll();
        }

        updateAngleLoop(roll, rollDeg, Config::kMaxRollPitchRateDegPerSec, dtSeconds);
        updateAngleLoop(pitch, pitchDeg, Config::kMaxRollPitchRateDegPerSec, dtSeconds);

        roll.rateTargetDegPerSec = roll.angleOutputRateDegPerSec;
        pitch.rateTargetDegPerSec = pitch.angleOutputRateDegPerSec;

        updateRateLoop(roll, GYRO::latestScaledSample.gyroXDps, dtSeconds);
        updateRateLoop(pitch, GYRO::latestScaledSample.gyroYDps, dtSeconds);
        updateRateLoop(yaw, GYRO::latestScaledSample.gyroZDps, dtSeconds);
        applyControlOutputs();
        printTelemetry(yawDeg);
    }
}

namespace Command {
    char lineBuffer[48] = {};
    size_t lineLength = 0;

    void handleLine(const char *line) {
        if (strcmp(line, "CALIBRATE") == 0) {
            Debug::logf("CMD CALIBRATE\r\n");
            const bool ok = GYRO::recalibrate();
            PID::resetAll();
            Debug::logf("CMD CALIBRATE %s\r\n", ok ? "OK" : "FAIL");
            return;
        }

        if (strcmp(line, "RESET_YAW") == 0) {
            Debug::logf("CMD RESET_YAW\r\n");
            GYRO::resetYaw();
            PID::resetAll();
            Debug::logf("CMD RESET_YAW OK\r\n");
            return;
        }

        if (strcmp(line, "PING") == 0) {
            Debug::logf("CMD PONG\r\n");
            return;
        }

        if (strcmp(line, "MODE AUTO") == 0) {
            RawTimerTest::disable();
            PinTest::setMode(PinTest::Mode::Off);
            ManualControl::setEnabled(false);
            PwmTest::setMode(PwmTest::Mode::Off);
            Debug::logf("CMD MODE AUTO OK\r\n");
            return;
        }

        if (strcmp(line, "MODE MANUAL") == 0) {
            RawTimerTest::disable();
            PinTest::setMode(PinTest::Mode::Off);
            PwmTest::setMode(PwmTest::Mode::Off);
            ManualControl::setEnabled(true);
            ManualControl::setAllMotorsPercent(0);
            Debug::logf("CMD MODE MANUAL OK\r\n");
            return;
        }

        if (strcmp(line, "PINTEST PA0 HIGH") == 0) {
            RawTimerTest::disable();
            ManualControl::setEnabled(false);
            PwmTest::setMode(PwmTest::Mode::Off);
            PinTest::setMode(PinTest::Mode::Pa0High);
            Debug::logf("CMD PINTEST PA0 HIGH OK\r\n");
            return;
        }

        if (strcmp(line, "PINTEST PA0 LOW") == 0) {
            RawTimerTest::disable();
            ManualControl::setEnabled(false);
            PwmTest::setMode(PwmTest::Mode::Off);
            PinTest::setMode(PinTest::Mode::Pa0Low);
            Debug::logf("CMD PINTEST PA0 LOW OK\r\n");
            return;
        }

        if (strcmp(line, "PINTEST PA0 BLINK") == 0) {
            RawTimerTest::disable();
            ManualControl::setEnabled(false);
            PwmTest::setMode(PwmTest::Mode::Off);
            PinTest::setMode(PinTest::Mode::Pa0Blink1Hz);
            Debug::logf("CMD PINTEST PA0 BLINK OK\r\n");
            return;
        }

        if (strcmp(line, "PINTEST OFF") == 0) {
            RawTimerTest::disable();
            PinTest::setMode(PinTest::Mode::Off);
            Debug::logf("CMD PINTEST OFF OK\r\n");
            return;
        }

        if (strcmp(line, "PWMTEST RAWTIM2 ON") == 0) {
            ManualControl::setEnabled(false);
            PinTest::setMode(PinTest::Mode::Off);
            PwmTest::setMode(PwmTest::Mode::RawTim2Ch1Square400Hz);
            RawTimerTest::enablePa0Tim2Square400Hz50();
            Debug::logf("CMD PWMTEST RAWTIM2 ON OK\r\n");
            RawTimerTest::logState("PWMTEST RAWTIM2");
            return;
        }

        if (strcmp(line, "PWMTEST SQUARE ON") == 0) {
            RawTimerTest::disable();
            PinTest::setMode(PinTest::Mode::Off);
            ManualControl::setEnabled(false);
            PwmTest::setMode(PwmTest::Mode::SquareWave400Hz);
            PWM::setEscSquareDutyAll(PwmTest::dutyPercent);
            Debug::logf("CMD PWMTEST SQUARE ON OK duty=%u\r\n", PwmTest::dutyPercent);
            PWM::logEscTimerState("PWMTEST SQUARE");
            return;
        }

        if (strcmp(line, "PWMTEST ESC ON") == 0) {
            RawTimerTest::disable();
            PinTest::setMode(PinTest::Mode::Off);
            ManualControl::setEnabled(false);
            PwmTest::setMode(PwmTest::Mode::EscPulse400Hz);
            PWM::applyEscPulseAll(PwmTest::pulseUs);
            Debug::logf("CMD PWMTEST ESC ON OK pulse=%u\r\n", PwmTest::pulseUs);
            PWM::logEscTimerState("PWMTEST ESC");
            return;
        }

        if (strcmp(line, "PWMTEST OFF") == 0) {
            RawTimerTest::disable();
            PwmTest::setMode(PwmTest::Mode::Off);
            Debug::logf("CMD PWMTEST OFF OK\r\n");
            return;
        }

        int throttlePercent = 0;
        if (sscanf(line, "THROTTLE %d", &throttlePercent) == 1) {
            ManualControl::setAllMotorsPercent(throttlePercent);
            Debug::logf("CMD THROTTLE %d OK\r\n", ManualControl::masterThrottlePercent);
            return;
        }

        int pulseUs = 0;
        if (sscanf(line, "PWMTESTUS %d", &pulseUs) == 1) {
            PwmTest::setPulseUs(pulseUs);
            Debug::logf("CMD PWMTESTUS %u OK\r\n", PwmTest::pulseUs);
            if (PwmTest::mode == PwmTest::Mode::EscPulse400Hz) {
                PWM::applyEscPulseAll(PwmTest::pulseUs);
                PWM::logEscTimerState("PWMTEST ESC UPDATE");
            }
            return;
        }

        int dutyPercent = 0;
        if (sscanf(line, "PWMTESTDUTY %d", &dutyPercent) == 1) {
            PwmTest::setDutyPercent(dutyPercent);
            Debug::logf("CMD PWMTESTDUTY %u OK\r\n", PwmTest::dutyPercent);
            if (PwmTest::mode == PwmTest::Mode::SquareWave400Hz) {
                PWM::setEscSquareDutyAll(PwmTest::dutyPercent);
                PWM::logEscTimerState("PWMTEST SQUARE UPDATE");
            }
            return;
        }

        int motorIndex = 0;
        if (sscanf(line, "MOTOR%d %d", &motorIndex, &throttlePercent) == 2) {
            if (motorIndex >= 1 && motorIndex <= 4) {
                ManualControl::setMotorPercent(static_cast<size_t>(motorIndex - 1), throttlePercent);
                Debug::logf("CMD MOTOR%d %d OK\r\n", motorIndex, ManualControl::motorPercents[motorIndex - 1]);
            } else {
                Debug::logf("CMD MOTOR INDEX INVALID:%d\r\n", motorIndex);
            }
            return;
        }

        if (line[0] != '\0') {
            Debug::logf("CMD UNKNOWN:%s\r\n", line);
        }
    }

    void loop() {
        while (DebugSerial.available() > 0) {
            const char incoming = static_cast<char>(DebugSerial.read());
            if (incoming == '\r') {
                continue;
            }

            if (incoming == '\n') {
                lineBuffer[lineLength] = '\0';
                handleLine(lineBuffer);
                lineLength = 0;
                continue;
            }

            if (lineLength < (sizeof(lineBuffer) - 1)) {
                lineBuffer[lineLength++] = incoming;
            } else {
                lineLength = 0;
            }
        }
    }
}

namespace ELRS {
    uint8_t rxBuffer[Config::kCrsfReadBufferSize] = {};
    uint16_t rcValues[RC_INPUT_MAX_CHANNELS] = {};
    uint16_t rcCount = 0;
    uint32_t nextStatusTimeMs = 0;
    uint32_t lastFrameTimeMs = 0;
    bool signalValid = false;
    int rollPercent = 50;
    int pitchPercent = 50;
    int throttlePercent = 0;
    int yawPercent = 50;
    int auxPercent = 50;

    int pwmPercentFromRc(uint16_t rcValue, int defaultPercent) {
        if (rcValue < 800 || rcValue > 2200) {
            return defaultPercent;
        }

        return constrain(map(rcValue, 1000, 2000, 0, 100), 0, 100);
    }

    void setup() {
        ElrsSerial.begin(Config::kElrsSerialBaud, SERIAL_8N1);
        Debug::logf("ELRS UART ready on RX=%d TX=%d baud=%lu\r\n", ELRS_UART_RX, ELRS_UART_TX, Config::kElrsSerialBaud);
    }

    void loop() {
        size_t bytesRead = 0;
        while (ElrsSerial.available() > 0 && bytesRead < sizeof(rxBuffer)) {
            rxBuffer[bytesRead++] = static_cast<uint8_t>(ElrsSerial.read());
        }

        if (bytesRead > 0 && crsf_parse(rxBuffer, static_cast<unsigned>(bytesRead), rcValues, &rcCount, RC_INPUT_MAX_CHANNELS)) {
            const int aileron = pwmPercentFromRc(rcValues[0], 50);
            const int elevator = pwmPercentFromRc(rcValues[1], 50);
            const int throttle = pwmPercentFromRc(rcValues[2], 0);
            const int rudder = pwmPercentFromRc(rcValues[3], 50);
            const int aux = pwmPercentFromRc(rcValues[4], 50);
            const uint32_t now = millis();
            signalValid = true;
            lastFrameTimeMs = now;
            rollPercent = aileron;
            pitchPercent = elevator;
            throttlePercent = throttle;
            yawPercent = rudder;
            auxPercent = aux;

            if (now >= nextStatusTimeMs) {
                Debug::logf(
                    "CRSF ch=%u c1=%u c2=%u c3=%u c4=%u c5=%u\r\n",
                    rcCount, rcValues[0], rcValues[1], rcValues[2], rcValues[3], rcValues[4]
                );
                nextStatusTimeMs = now + Config::kElrsStatusIntervalMs;
            }
        }

        const uint32_t now = millis();
        if (signalValid && (now - lastFrameTimeMs) > Config::kRcSignalTimeoutMs) {
            signalValid = false;
            rollPercent = 50;
            pitchPercent = 50;
            throttlePercent = 0;
            yawPercent = 50;
            auxPercent = 50;
        }
    }
}

void setup() {
    DebugSerial.begin(SERIAL_BAUD);
    Debug::port = &DebugSerial;
    delay(300);

    Debug::logf("\r\nBooting Blue Pill flightcontroller bring-up\r\n");
    Debug::logf("Debug UART TX=%d RX=%d baud=%lu\r\n", DEBUG_UART_TX, DEBUG_UART_RX, SERIAL_BAUD);

    LED::setup();
    PWM::setup();
    GYRO::setup();
    PID::setup();
    ELRS::setup();
}

void loop() {
    Command::loop();
    PinTest::loop();
    LED::loop();
    GYRO::loop();
    ELRS::loop();
    PID::loop(GYRO::rollDeg, GYRO::pitchDeg, GYRO::yawDeg, GYRO::lastDtSeconds);
}
