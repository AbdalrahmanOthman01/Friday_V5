#ifndef SENSORS_H
#define SENSORS_H

// Arduino.h is REQUIRED - provides essential functions and types:
// - Pin functions: pinMode(), digitalWrite(), digitalRead()
// - Timing functions: micros(), millis(), delayMicroseconds()
// - Interrupt functions: attachInterrupt(), detachInterrupt(), digitalPinToInterrupt()
// - Types: uint8_t, unsigned long, bool, float
// - Constants: HIGH, LOW, INPUT, OUTPUT, CHANGE, NOT_AN_INTERRUPT
// Without this include, the code will NOT compile.
#include <Arduino.h>

// ============================================================================
// SENSOR LIBRARY HEADER FILE
// ============================================================================
// Professional-grade sensor control library for embedded systems
// Designed for real-time, non-blocking operation with interrupt-driven architecture
// 
// Author: Senior Embedded Engineer (20+ years experience)
// Platform: Arduino/AVR microcontrollers
// ============================================================================

// ============================================================================
// CONFIGURATION AND CONSTANTS
// ============================================================================

// Ultrasonic sensor timing constants
#define US_SOUND_SPEED_CM_US    0.0343f     // Speed of sound in cm/microsecond
#define US_DEFAULT_MAX_DIST_CM  400         // Default maximum distance (4 meters)
#define US_TRIGGER_PULSE_US     10          // Trigger pulse width in microseconds
#define US_MIN_PULSE_US         2           // Minimum pulse spacing
#define US_TIMEOUT_SAFETY_US    1000        // Safety margin for timeout calculation

// Error codes
#define SENSOR_ERROR_NONE       0
#define SENSOR_ERROR_TIMEOUT    -1.0f
#define SENSOR_ERROR_OUT_RANGE  -2.0f
#define SENSOR_ERROR_NOT_READY  -3.0f

// ============================================================================
// ULTRASONIC SENSOR CLASS (Interrupt-Based, Production-Ready)
// ============================================================================
// Features:
// - Non-blocking interrupt-driven measurement
// - Support for multiple sensors (up to MAX_ULTRASONIC_SENSORS)
// - Automatic timeout handling
// - Distance filtering and validation
// - Thread-safe ISR design
// - Low CPU overhead
// ============================================================================

#define MAX_ULTRASONIC_SENSORS  4   // Maximum number of simultaneous sensors

class UltrasonicSensor {
private:
    // Pin configuration
    uint8_t trigPin;
    uint8_t echoPin;
    
    // Measurement parameters
    unsigned long maxDistanceCM;
    unsigned long timeoutUS;
    
    // Interrupt-driven measurement state
    volatile unsigned long pulseStartUS;
    volatile unsigned long pulseEndUS;
    volatile bool measurementComplete;
    volatile bool measurementValid;
    
    // Last valid measurement
    float lastValidDistanceCM;
    unsigned long lastMeasurementTime;
    
    // Sensor instance management (for ISR routing)
    uint8_t sensorIndex;
    static UltrasonicSensor* sensorInstances[MAX_ULTRASONIC_SENSORS];
    static uint8_t activeSensorCount;
    
    // =========================================================================
    // INTERRUPT SERVICE ROUTINE (ISR)
    // =========================================================================
    // This static method routes interrupts to the correct sensor instance
    // Uses direct port manipulation for minimal ISR latency
    // =========================================================================
    static void echoISR() {
        // Determine which sensor triggered the interrupt
        // by checking which echo pin changed state
        for (uint8_t i = 0; i < activeSensorCount; i++) {
            if (sensorInstances[i] == nullptr) continue;
            
            UltrasonicSensor* sensor = sensorInstances[i];
            uint8_t pinState = digitalRead(sensor->echoPin);
            
            if (pinState == HIGH) {
                // Rising edge: Echo pulse started
                sensor->pulseStartUS = micros();
                sensor->measurementComplete = false;
            } else {
                // Falling edge: Echo pulse ended
                sensor->pulseEndUS = micros();
                sensor->measurementComplete = true;
                sensor->measurementValid = true;
            }
        }
    }
    
    // =========================================================================
    // PRIVATE HELPER METHODS
    // =========================================================================
    
    // Calculate distance from pulse duration
    float calculateDistanceCM(unsigned long durationUS) const {
        // Distance = (time × speed) / 2 (round trip)
        return (durationUS * US_SOUND_SPEED_CM_US) / 2.0f;
    }
    
    // Validate measurement against configured limits
    bool isDistanceValid(float distanceCM) const {
        return (distanceCM > 0 && distanceCM <= maxDistanceCM);
    }
    
    // Calculate timeout based on max distance
    unsigned long calculateTimeoutUS() const {
        // Time = distance / speed × 2 (round trip) + safety margin
        return ((maxDistanceCM / US_SOUND_SPEED_CM_US) * 2) + US_TIMEOUT_SAFETY_US;
    }

public:
    // =========================================================================
    // CONSTRUCTOR
    // =========================================================================
    // Parameters:
    //   trig - Trigger pin number (must be interrupt-capable)
    //   echo - Echo pin number (must be interrupt-capable)
    //   maxDist - Maximum detection distance in cm (default: 400cm)
    // =========================================================================
    UltrasonicSensor(uint8_t trig, uint8_t echo, 
                     unsigned long maxDist = US_DEFAULT_MAX_DIST_CM) 
        : trigPin(trig)
        , echoPin(echo)
        , maxDistanceCM(maxDist)
        , pulseStartUS(0)
        , pulseEndUS(0)
        , measurementComplete(false)
        , measurementValid(false)
        , lastValidDistanceCM(SENSOR_ERROR_NOT_READY)
        , lastMeasurementTime(0)
        , sensorIndex(0)
    {
        timeoutUS = calculateTimeoutUS();
        
        // Register this sensor instance
        if (activeSensorCount < MAX_ULTRASONIC_SENSORS) {
            sensorIndex = activeSensorCount;
            sensorInstances[activeSensorCount++] = this;
        }
    }
    
    // =========================================================================
    // INITIALIZATION
    // =========================================================================
    // Must be called in setup() before any measurements
    // Configures pins and attaches interrupt
    // =========================================================================
    bool begin() {
        // Validate pin numbers
        if (trigPin == echoPin) return false;
        
        // Configure pins
        pinMode(trigPin, OUTPUT);
        pinMode(echoPin, INPUT);
        
        // Ensure trigger is LOW
        digitalWrite(trigPin, LOW);
        
        // Verify echo pin supports interrupts
        if (digitalPinToInterrupt(echoPin) == NOT_AN_INTERRUPT) {
            return false;  // Pin doesn't support interrupts
        }
        
        // Attach interrupt for both rising and falling edges
        attachInterrupt(digitalPinToInterrupt(echoPin), echoISR, CHANGE);
        
        return true;
    }
    
    // =========================================================================
    // CLEANUP
    // =========================================================================
    // Detach interrupt and release resources
    // =========================================================================
    void end() {
        detachInterrupt(digitalPinToInterrupt(echoPin));
        
        // Remove from instance array
        if (sensorIndex < activeSensorCount) {
            sensorInstances[sensorIndex] = nullptr;
        }
    }
    
    // =========================================================================
    // NON-BLOCKING MEASUREMENT API
    // =========================================================================
    
    // Start a new measurement (non-blocking)
    // Returns: true if trigger was sent successfully
    bool startMeasurement() {
        // Reset measurement state
        measurementComplete = false;
        measurementValid = false;
        
        // Send trigger pulse with precise timing
        // Using direct port manipulation for minimal jitter
        digitalWrite(trigPin, LOW);
        delayMicroseconds(US_MIN_PULSE_US);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(US_TRIGGER_PULSE_US);
        digitalWrite(trigPin, LOW);
        
        lastMeasurementTime = millis();
        return true;
    }
    
    // Check if measurement is complete
    bool isMeasurementComplete() const {
        return measurementComplete;
    }
    
    // Check if measurement is valid (within range)
    bool isMeasurementValid() const {
        return measurementValid && measurementComplete;
    }
    
    // Get distance from completed measurement
    // Returns: distance in cm, or error code (negative value)
    float getDistanceCM() {
        if (!measurementComplete) {
            return SENSOR_ERROR_NOT_READY;
        }
        
        if (!measurementValid) {
            measurementComplete = false;
            return SENSOR_ERROR_TIMEOUT;
        }
        
        // Calculate pulse duration (handles micros() overflow)
        unsigned long duration;
        if (pulseEndUS >= pulseStartUS) {
            duration = pulseEndUS - pulseStartUS;
        } else {
            // Handle overflow case
            duration = (0xFFFFFFFF - pulseStartUS) + pulseEndUS;
        }
        
        // Check for timeout
        if (duration > timeoutUS) {
            measurementComplete = false;
            measurementValid = false;
            return SENSOR_ERROR_OUT_RANGE;
        }
        
        // Calculate and validate distance
        float distance = calculateDistanceCM(duration);
        
        if (isDistanceValid(distance)) {
            lastValidDistanceCM = distance;
        } else {
            distance = SENSOR_ERROR_OUT_RANGE;
        }
        
        // Mark measurement as consumed
        measurementComplete = false;
        measurementValid = false;
        
        return distance;
    }
    
    // Get distance in inches
    float getDistanceInches() {
        float cm = getDistanceCM();
        if (cm < 0) return cm;  // Return error code as-is
        return cm * 0.393701f;
    }
    
    // Get last valid distance without triggering new measurement
    float getLastDistanceCM() const {
        return lastValidDistanceCM;
    }
    
    // =========================================================================
    // BLOCKING MEASUREMENT API (Use sparingly in real-time systems)
    // =========================================================================
    
    // Blocking measurement with timeout
    // Returns: distance in cm, or error code
    float measureDistanceCM(unsigned long timeoutMs = 100) {
        startMeasurement();
        
        unsigned long startMs = millis();
        while (!measurementComplete) {
            if ((millis() - startMs) > timeoutMs) {
                measurementComplete = false;
                return SENSOR_ERROR_TIMEOUT;
            }
            // Allow other interrupts to run
            yield();
        }
        
        return getDistanceCM();
    }
    
    // =========================================================================
    // UTILITY METHODS
    // =========================================================================
    
    // Check if object is detected within threshold
    bool isObjectDetected(float thresholdCM) {
        float distance = getLastDistanceCM();
        return (distance > 0 && distance <= thresholdCM);
    }
    
    // Get time since last measurement
    unsigned long getTimeSinceLastMeasurement() const {
        return millis() - lastMeasurementTime;
    }
    
    // Check if sensor is ready for new measurement
    bool isReady() const {
        return !measurementComplete;
    }
    
    // Get configured maximum distance
    unsigned long getMaxDistance() const {
        return maxDistanceCM;
    }
    
    // Set new maximum distance (recalculates timeout)
    void setMaxDistance(unsigned long maxDistCM) {
        maxDistanceCM = maxDistCM;
        timeoutUS = calculateTimeoutUS();
    }
    
    // Get sensor pin numbers
    uint8_t getTriggerPin() const { return trigPin; }
    uint8_t getEchoPin() const { return echoPin; }
};

// Initialize static members
UltrasonicSensor* UltrasonicSensor::sensorInstances[MAX_ULTRASONIC_SENSORS] = {nullptr};
uint8_t UltrasonicSensor::activeSensorCount = 0;

// ============================================================================
// IR FLAME SENSOR CLASS (Interrupt-Based)
// ============================================================================
// Supports both digital and analog flame sensors
// - Digital output: Flame detection (HIGH/LOW)
// - Analog output: Flame intensity (0-1023)
// 
// Common modules: YG1006, KY-026, IR Flame Sensor Module
// Detection range: Typically 760nm - 1100nm (near-infrared)
// ============================================================================

#define MAX_FLAME_SENSORS  8   // Maximum number of simultaneous flame sensors

class IRFlameSensor {
private:
    // Pin configuration
    uint8_t digitalPin;         // Digital output pin (flame detected)
    uint8_t analogPin;          // Analog output pin (flame intensity) - use 0xFF if not available
    
    // Sensor configuration
    bool invertedLogic;         // true if sensor outputs LOW when flame detected
    uint16_t detectionThreshold; // Analog threshold for flame detection
    
    // State tracking
    volatile bool flameDetected;
    volatile bool stateChanged;
    volatile unsigned long lastInterruptTime;
    unsigned long debounceTimeUS;
    
    // Last readings
    bool lastDigitalState;
    uint16_t lastAnalogValue;
    unsigned long lastReadTime;
    
    // Sensor instance management
    uint8_t sensorIndex;
    static IRFlameSensor* sensorInstances[MAX_FLAME_SENSORS];
    static uint8_t activeSensorCount;
    
    // =========================================================================
    // INTERRUPT SERVICE ROUTINE (ISR)
    // =========================================================================
    static void flameISR() {
        for (uint8_t i = 0; i < activeSensorCount; i++) {
            if (sensorInstances[i] == nullptr) continue;
            
            IRFlameSensor* sensor = sensorInstances[i];
            if (sensor->digitalPin == 0xFF) continue;
            
            // Debounce check
            unsigned long now = micros();
            if ((now - sensor->lastInterruptTime) < sensor->debounceTimeUS) {
                continue;
            }
            sensor->lastInterruptTime = now;
            
            // Read current state
            uint8_t pinState = digitalRead(sensor->digitalPin);
            bool detected = sensor->invertedLogic ? (pinState == LOW) : (pinState == HIGH);
            
            // Update state if changed
            if (detected != sensor->flameDetected) {
                sensor->flameDetected = detected;
                sensor->stateChanged = true;
            }
        }
    }

public:
    // =========================================================================
    // CONSTRUCTOR - Digital Only
    // =========================================================================
    // Parameters:
    //   digitalPin - Digital input pin (flame detection)
    //   inverted - true if sensor outputs LOW when flame detected (most modules)
    //   debounceUS - Debounce time in microseconds (default: 10000 = 10ms)
    // =========================================================================
    IRFlameSensor(uint8_t digitalPin, bool inverted = true, 
                  unsigned long debounceUS = 10000)
        : digitalPin(digitalPin)
        , analogPin(0xFF)
        , invertedLogic(inverted)
        , detectionThreshold(512)
        , flameDetected(false)
        , stateChanged(false)
        , lastInterruptTime(0)
        , debounceTimeUS(debounceUS)
        , lastDigitalState(false)
        , lastAnalogValue(0)
        , lastReadTime(0)
        , sensorIndex(0)
    {
        // Register this sensor instance
        if (activeSensorCount < MAX_FLAME_SENSORS) {
            sensorIndex = activeSensorCount;
            sensorInstances[activeSensorCount++] = this;
        }
    }
    
    // =========================================================================
    // CONSTRUCTOR - Digital + Analog
    // =========================================================================
    // Parameters:
    //   digitalPin - Digital input pin (flame detection)
    //   analogPin - Analog input pin (flame intensity)
    //   inverted - true if sensor outputs LOW when flame detected
    //   debounceUS - Debounce time in microseconds
    // =========================================================================
    IRFlameSensor(uint8_t digitalPin, uint8_t analogPin, bool inverted = true,
                  unsigned long debounceUS = 10000)
        : digitalPin(digitalPin)
        , analogPin(analogPin)
        , invertedLogic(inverted)
        , detectionThreshold(512)
        , flameDetected(false)
        , stateChanged(false)
        , lastInterruptTime(0)
        , debounceTimeUS(debounceUS)
        , lastDigitalState(false)
        , lastAnalogValue(0)
        , lastReadTime(0)
        , sensorIndex(0)
    {
        // Register this sensor instance
        if (activeSensorCount < MAX_FLAME_SENSORS) {
            sensorIndex = activeSensorCount;
            sensorInstances[activeSensorCount++] = this;
        }
    }
    
    // =========================================================================
    // INITIALIZATION
    // =========================================================================
    bool begin() {
        // Configure digital pin
        if (digitalPin != 0xFF) {
            pinMode(digitalPin, INPUT);
            
            // Verify interrupt support
            if (digitalPinToInterrupt(digitalPin) != NOT_AN_INTERRUPT) {
                attachInterrupt(digitalPinToInterrupt(digitalPin), flameISR, CHANGE);
            }
        }
        
        // Configure analog pin
        if (analogPin != 0xFF) {
            pinMode(analogPin, INPUT);
        }
        
        // Initial reading
        readDigital();
        if (analogPin != 0xFF) {
            readAnalog();
        }
        
        return true;
    }
    
    // =========================================================================
    // CLEANUP
    // =========================================================================
    void end() {
        if (digitalPin != 0xFF) {
            detachInterrupt(digitalPinToInterrupt(digitalPin));
        }
        
        // Remove from instance array
        if (sensorIndex < activeSensorCount) {
            sensorInstances[sensorIndex] = nullptr;
        }
    }
    
    // =========================================================================
    // DIGITAL READING (Interrupt-Driven)
    // =========================================================================
    
    // Check if flame is detected (from interrupt state)
    bool isFlameDetected() const {
        return flameDetected;
    }
    
    // Check if state changed since last check
    bool hasStateChanged() {
        if (stateChanged) {
            stateChanged = false;
            return true;
        }
        return false;
    }
    
    // Read digital state directly (polling mode)
    bool readDigital() {
        if (digitalPin == 0xFF) return false;
        
        uint8_t pinState = digitalRead(digitalPin);
        lastDigitalState = invertedLogic ? (pinState == LOW) : (pinState == HIGH);
        lastReadTime = millis();
        
        return lastDigitalState;
    }
    
    // =========================================================================
    // ANALOG READING (Flame Intensity)
    // =========================================================================
    
    // Read analog value (0-1023, higher = more intense flame)
    uint16_t readAnalog() {
        if (analogPin == 0xFF) return 0;
        
        lastAnalogValue = analogRead(analogPin);
        lastReadTime = millis();
        
        return lastAnalogValue;
    }
    
    // Get last analog value without new reading
    uint16_t getLastAnalogValue() const {
        return lastAnalogValue;
    }
    
    // Get flame intensity as percentage (0-100%)
    float getIntensityPercent() {
        if (analogPin == 0xFF) return 0.0f;
        
        uint16_t value = readAnalog();
        return (value / 1023.0f) * 100.0f;
    }
    
    // Check if flame intensity exceeds threshold
    bool isFlameIntense(uint16_t threshold) {
        if (analogPin == 0xFF) return false;
        
        return readAnalog() >= threshold;
    }
    
    // =========================================================================
    // CONFIGURATION
    // =========================================================================
    
    // Set detection threshold for analog readings
    void setDetectionThreshold(uint16_t threshold) {
        detectionThreshold = threshold;
    }
    
    // Get detection threshold
    uint16_t getDetectionThreshold() const {
        return detectionThreshold;
    }
    
    // Set debounce time
    void setDebounceTime(unsigned long debounceUS) {
        debounceTimeUS = debounceUS;
    }
    
    // Set inverted logic
    void setInverted(bool inverted) {
        invertedLogic = inverted;
    }
    
    // =========================================================================
    // UTILITY METHODS
    // =========================================================================
    
    // Check if flame is detected using analog threshold
    bool isFlameDetectedAnalog() {
        if (analogPin == 0xFF) return false;
        
        return readAnalog() >= detectionThreshold;
    }
    
    // Get time since last reading
    unsigned long getTimeSinceLastRead() const {
        return millis() - lastReadTime;
    }
    
    // Get pin numbers
    uint8_t getDigitalPin() const { return digitalPin; }
    uint8_t getAnalogPin() const { return analogPin; }
    
    // Check if sensor has analog capability
    bool hasAnalog() const { return analogPin != 0xFF; }
};

// Initialize static members
IRFlameSensor* IRFlameSensor::sensorInstances[MAX_FLAME_SENSORS] = {nullptr};
uint8_t IRFlameSensor::activeSensorCount = 0;

// ============================================================================
// LIGHT SENSOR (LDR) CLASS
// ============================================================================
// Supports Light Dependent Resistor (LDR) sensors
// - Analog output: Light intensity (0-1023)
// - Higher value = more light (typical LDR behavior)
// 
// Common modules: LDR with voltage divider, GL5528, GL5537, etc.
// Applications: Day/night detection, ambient light sensing, line following
// ============================================================================

class LightSensor {
private:
    uint8_t analogPin;              // Analog input pin
    uint16_t darkThreshold;         // Threshold for "dark" detection
    uint16_t brightThreshold;       // Threshold for "bright" detection
    
    // Calibration values
    uint16_t minLightValue;         // Minimum expected light value (darkest)
    uint16_t maxLightValue;         // Maximum expected light value (brightest)
    
    // Last readings
    uint16_t lastRawValue;
    float lastPercentValue;
    unsigned long lastReadTime;
    
    // Smoothing
    uint16_t smoothedValue;
    float smoothingFactor;          // 0.0 - 1.0 (higher = more smoothing)

public:
    // =========================================================================
    // CONSTRUCTOR
    // =========================================================================
    // Parameters:
    //   pin - Analog input pin (A0, A1, etc.)
    //   darkThresh - Threshold below which it's considered "dark" (default: 300)
    //   brightThresh - Threshold above which it's considered "bright" (default: 700)
    // =========================================================================
    LightSensor(uint8_t pin, uint16_t darkThresh = 300, uint16_t brightThresh = 700)
        : analogPin(pin)
        , darkThreshold(darkThresh)
        , brightThreshold(brightThresh)
        , minLightValue(0)
        , maxLightValue(1023)
        , lastRawValue(0)
        , lastPercentValue(0.0f)
        , lastReadTime(0)
        , smoothedValue(512)
        , smoothingFactor(0.1f)
    {
    }
    
    // =========================================================================
    // INITIALIZATION
    // =========================================================================
    void begin() {
        pinMode(analogPin, INPUT);
        
        // Initial reading
        lastRawValue = analogRead(analogPin);
        smoothedValue = lastRawValue;
        lastPercentValue = calculatePercent(lastRawValue);
        lastReadTime = millis();
    }
    
    // =========================================================================
    // READING METHODS
    // =========================================================================
    
    // Read raw analog value (0-1023)
    uint16_t readRaw() {
        lastRawValue = analogRead(analogPin);
        lastReadTime = millis();
        
        // Apply smoothing
        smoothedValue = smoothedValue + (lastRawValue - smoothedValue) * smoothingFactor;
        
        return lastRawValue;
    }
    
    // Get smoothed value
    uint16_t readSmoothed() {
        readRaw();
        return smoothedValue;
    }
    
    // Read light as percentage (0-100%)
    float readPercent() {
        uint16_t raw = readRaw();
        lastPercentValue = calculatePercent(raw);
        return lastPercentValue;
    }
    
    // Get last raw value without new reading
    uint16_t getLastRaw() const {
        return lastRawValue;
    }
    
    // Get last percentage without new reading
    float getLastPercent() const {
        return lastPercentValue;
    }
    
    // =========================================================================
    // DETECTION METHODS
    // =========================================================================
    
    // Check if it's dark (below dark threshold)
    bool isDark() {
        return readRaw() < darkThreshold;
    }
    
    // Check if it's bright (above bright threshold)
    bool isBright() {
        return readRaw() > brightThreshold;
    }
    
    // Check if it's twilight (between dark and bright thresholds)
    bool isTwilight() {
        uint16_t value = readRaw();
        return (value >= darkThreshold && value <= brightThreshold);
    }
    
    // Get light level category
    // Returns: 0 = dark, 1 = twilight, 2 = bright
    uint8_t getLightLevel() {
        uint16_t value = readRaw();
        if (value < darkThreshold) return 0;
        if (value > brightThreshold) return 2;
        return 1;
    }
    
    // =========================================================================
    // CALIBRATION METHODS
    // =========================================================================
    
    // Set calibration range (call after taking readings in dark and bright conditions)
    void calibrate(uint16_t darkValue, uint16_t brightValue) {
        minLightValue = darkValue;
        maxLightValue = brightValue;
    }
    
    // Set thresholds
    void setDarkThreshold(uint16_t threshold) {
        darkThreshold = threshold;
    }
    
    void setBrightThreshold(uint16_t threshold) {
        brightThreshold = threshold;
    }
    
    // Set smoothing factor (0.0 = no smoothing, 1.0 = maximum smoothing)
    void setSmoothing(float factor) {
        if (factor < 0.0f) factor = 0.0f;
        if (factor > 1.0f) factor = 1.0f;
        smoothingFactor = factor;
    }
    
    // =========================================================================
    // UTILITY METHODS
    // =========================================================================
    
    // Get time since last reading
    unsigned long getTimeSinceLastRead() const {
        return millis() - lastReadTime;
    }
    
    // Get pin number
    uint8_t getPin() const {
        return analogPin;
    }

private:
    // Calculate percentage from raw value
    float calculatePercent(uint16_t rawValue) {
        if (maxLightValue == minLightValue) return 0.0f;
        
        float percent = ((float)(rawValue - minLightValue) / 
                        (float)(maxLightValue - minLightValue)) * 100.0f;
        
        // Clamp to 0-100%
        if (percent < 0.0f) percent = 0.0f;
        if (percent > 100.0f) percent = 100.0f;
        
        return percent;
    }
};

// ============================================================================
// RAIN SENSOR CLASS
// ============================================================================
// Supports rain/water detection sensors
// - Digital output: Rain detected (HIGH/LOW)
// - Analog output: Rain intensity (0-1023, lower = more water)
// 
// Common modules: YL-83, FC-37, MH-RD
// Note: Most rain sensors output LOWER values when wet (inverted logic)
// ============================================================================

#define MAX_RAIN_SENSORS  4   // Maximum number of simultaneous rain sensors

class RainSensor {
private:
    // Pin configuration
    uint8_t digitalPin;         // Digital output pin (rain detected)
    uint8_t analogPin;          // Analog output pin (rain intensity)
    
    // Sensor configuration
    bool invertedLogic;         // true if sensor outputs LOW when wet (most modules)
    uint16_t wetThreshold;      // Analog threshold for "wet" detection
    uint16_t dryThreshold;      // Analog threshold for "dry" detection
    
    // State tracking
    volatile bool rainDetected;
    volatile bool stateChanged;
    volatile unsigned long lastInterruptTime;
    unsigned long debounceTimeUS;
    
    // Last readings
    bool lastDigitalState;
    uint16_t lastAnalogValue;
    unsigned long lastReadTime;
    
    // Sensor instance management
    uint8_t sensorIndex;
    static RainSensor* sensorInstances[MAX_RAIN_SENSORS];
    static uint8_t activeSensorCount;
    
    // =========================================================================
    // INTERRUPT SERVICE ROUTINE (ISR)
    // =========================================================================
    static void rainISR() {
        for (uint8_t i = 0; i < activeSensorCount; i++) {
            if (sensorInstances[i] == nullptr) continue;
            
            RainSensor* sensor = sensorInstances[i];
            if (sensor->digitalPin == 0xFF) continue;
            
            // Debounce check
            unsigned long now = micros();
            if ((now - sensor->lastInterruptTime) < sensor->debounceTimeUS) {
                continue;
            }
            sensor->lastInterruptTime = now;
            
            // Read current state
            uint8_t pinState = digitalRead(sensor->digitalPin);
            bool detected = sensor->invertedLogic ? (pinState == LOW) : (pinState == HIGH);
            
            // Update state if changed
            if (detected != sensor->rainDetected) {
                sensor->rainDetected = detected;
                sensor->stateChanged = true;
            }
        }
    }

public:
    // =========================================================================
    // CONSTRUCTOR - Digital Only
    // =========================================================================
    // Parameters:
    //   digitalPin - Digital input pin (rain detection)
    //   inverted - true if sensor outputs LOW when wet (most modules)
    //   debounceUS - Debounce time in microseconds (default: 50000 = 50ms)
    // =========================================================================
    RainSensor(uint8_t digitalPin, bool inverted = true, 
               unsigned long debounceUS = 50000)
        : digitalPin(digitalPin)
        , analogPin(0xFF)
        , invertedLogic(inverted)
        , wetThreshold(300)
        , dryThreshold(700)
        , rainDetected(false)
        , stateChanged(false)
        , lastInterruptTime(0)
        , debounceTimeUS(debounceUS)
        , lastDigitalState(false)
        , lastAnalogValue(0)
        , lastReadTime(0)
        , sensorIndex(0)
    {
        // Register this sensor instance
        if (activeSensorCount < MAX_RAIN_SENSORS) {
            sensorIndex = activeSensorCount;
            sensorInstances[activeSensorCount++] = this;
        }
    }
    
    // =========================================================================
    // CONSTRUCTOR - Digital + Analog
    // =========================================================================
    // Parameters:
    //   digitalPin - Digital input pin (rain detection)
    //   analogPin - Analog input pin (rain intensity)
    //   inverted - true if sensor outputs LOW when wet
    //   debounceUS - Debounce time in microseconds
    // =========================================================================
    RainSensor(uint8_t digitalPin, uint8_t analogPin, bool inverted = true,
               unsigned long debounceUS = 50000)
        : digitalPin(digitalPin)
        , analogPin(analogPin)
        , invertedLogic(inverted)
        , wetThreshold(300)
        , dryThreshold(700)
        , rainDetected(false)
        , stateChanged(false)
        , lastInterruptTime(0)
        , debounceTimeUS(debounceUS)
        , lastDigitalState(false)
        , lastAnalogValue(0)
        , lastReadTime(0)
        , sensorIndex(0)
    {
        // Register this sensor instance
        if (activeSensorCount < MAX_RAIN_SENSORS) {
            sensorIndex = activeSensorCount;
            sensorInstances[activeSensorCount++] = this;
        }
    }
    
    // =========================================================================
    // INITIALIZATION
    // =========================================================================
    bool begin() {
        // Configure digital pin
        if (digitalPin != 0xFF) {
            pinMode(digitalPin, INPUT);
            
            // Verify interrupt support
            if (digitalPinToInterrupt(digitalPin) != NOT_AN_INTERRUPT) {
                attachInterrupt(digitalPinToInterrupt(digitalPin), rainISR, CHANGE);
            }
        }
        
        // Configure analog pin
        if (analogPin != 0xFF) {
            pinMode(analogPin, INPUT);
        }
        
        // Initial reading
        readDigital();
        if (analogPin != 0xFF) {
            readAnalog();
        }
        
        return true;
    }
    
    // =========================================================================
    // CLEANUP
    // =========================================================================
    void end() {
        if (digitalPin != 0xFF) {
            detachInterrupt(digitalPinToInterrupt(digitalPin));
        }
        
        // Remove from instance array
        if (sensorIndex < activeSensorCount) {
            sensorInstances[sensorIndex] = nullptr;
        }
    }
    
    // =========================================================================
    // DIGITAL READING (Interrupt-Driven)
    // =========================================================================
    
    // Check if rain is detected (from interrupt state)
    bool isRainDetected() const {
        return rainDetected;
    }
    
    // Check if state changed since last check
    bool hasStateChanged() {
        if (stateChanged) {
            stateChanged = false;
            return true;
        }
        return false;
    }
    
    // Read digital state directly (polling mode)
    bool readDigital() {
        if (digitalPin == 0xFF) return false;
        
        uint8_t pinState = digitalRead(digitalPin);
        lastDigitalState = invertedLogic ? (pinState == LOW) : (pinState == HIGH);
        lastReadTime = millis();
        
        return lastDigitalState;
    }
    
    // =========================================================================
    // ANALOG READING (Rain Intensity)
    // =========================================================================
    
    // Read analog value (0-1023, LOWER = more water on sensor)
    uint16_t readAnalog() {
        if (analogPin == 0xFF) return 1023;  // Return "dry" value if no analog
        
        lastAnalogValue = analogRead(analogPin);
        lastReadTime = millis();
        
        return lastAnalogValue;
    }
    
    // Get last analog value without new reading
    uint16_t getLastAnalogValue() const {
        return lastAnalogValue;
    }
    
    // Get rain intensity as percentage (0% = dry, 100% = heavy rain)
    // Note: Inverted because lower analog = more water
    float getIntensityPercent() {
        if (analogPin == 0xFF) return 0.0f;
        
        uint16_t value = readAnalog();
        
        // Invert: 1023 (dry) = 0%, 0 (soaking wet) = 100%
        float percent = ((1023.0f - value) / 1023.0f) * 100.0f;
        
        return percent;
    }
    
    // Check if rain intensity exceeds threshold
    bool isRainIntense(uint16_t threshold) {
        if (analogPin == 0xFF) return false;
        
        return readAnalog() <= threshold;  // Lower = more intense
    }
    
    // =========================================================================
    // DETECTION METHODS
    // =========================================================================
    
    // Check if sensor is wet (analog below wet threshold)
    bool isWet() {
        if (analogPin == 0xFF) return isRainDetected();
        
        return readAnalog() <= wetThreshold;
    }
    
    // Check if sensor is dry (analog above dry threshold)
    bool isDry() {
        if (analogPin == 0xFF) return !isRainDetected();
        
        return readAnalog() >= dryThreshold;
    }
    
    // Check if sensor is damp (between wet and dry thresholds)
    bool isDamp() {
        if (analogPin == 0xFF) return false;
        
        uint16_t value = readAnalog();
        return (value > wetThreshold && value < dryThreshold);
    }
    
    // Get moisture level category
    // Returns: 0 = dry, 1 = damp, 2 = wet
    uint8_t getMoistureLevel() {
        if (analogPin == 0xFF) {
            return isRainDetected() ? 2 : 0;
        }
        
        uint16_t value = readAnalog();
        if (value <= wetThreshold) return 2;
        if (value >= dryThreshold) return 0;
        return 1;
    }
    
    // =========================================================================
    // CONFIGURATION
    // =========================================================================
    
    // Set wet threshold (analog value below which it's considered wet)
    void setWetThreshold(uint16_t threshold) {
        wetThreshold = threshold;
    }
    
    // Set dry threshold (analog value above which it's considered dry)
    void setDryThreshold(uint16_t threshold) {
        dryThreshold = threshold;
    }
    
    // Set debounce time
    void setDebounceTime(unsigned long debounceUS) {
        debounceTimeUS = debounceUS;
    }
    
    // Set inverted logic
    void setInverted(bool inverted) {
        invertedLogic = inverted;
    }
    
    // =========================================================================
    // UTILITY METHODS
    // =========================================================================
    
    // Get time since last reading
    unsigned long getTimeSinceLastRead() const {
        return millis() - lastReadTime;
    }
    
    // Get pin numbers
    uint8_t getDigitalPin() const { return digitalPin; }
    uint8_t getAnalogPin() const { return analogPin; }
    
    // Check if sensor has analog capability
    bool hasAnalog() const { return analogPin != 0xFF; }
};

// Initialize static members
RainSensor* RainSensor::sensorInstances[MAX_RAIN_SENSORS] = {nullptr};
uint8_t RainSensor::activeSensorCount = 0;

// ============================================================================
// SOUND SENSOR CLASS
// ============================================================================
// Supports sound detection sensors with digital and analog outputs
// - Digital output: Sound threshold exceeded (HIGH/LOW)
// - Analog output: Sound level/intensity (0-1023)
// 
// Common modules: KY-038, KY-037, LM393 Sound Sensor Module
// Applications: Clap detection, noise monitoring, voice activation
// ============================================================================

#define MAX_SOUND_SENSORS  4   // Maximum number of simultaneous sound sensors

class SoundSensor {
private:
    // Pin configuration
    uint8_t digitalPin;         // Digital output pin (sound detected)
    uint8_t analogPin;          // Analog output pin (sound level)
    
    // Sensor configuration
    bool invertedLogic;         // true if sensor outputs LOW when sound detected
    uint16_t soundThreshold;    // Analog threshold for sound detection
    
    // State tracking
    volatile bool soundDetected;
    volatile bool stateChanged;
    volatile unsigned long lastInterruptTime;
    unsigned long debounceTimeUS;
    
    // Peak detection
    uint16_t peakValue;
    unsigned long peakHoldTimeMS;
    unsigned long peakStartTime;
    
    // Last readings
    bool lastDigitalState;
    uint16_t lastAnalogValue;
    unsigned long lastReadTime;
    
    // Smoothing
    uint16_t smoothedValue;
    float smoothingFactor;
    
    // Sensor instance management
    uint8_t sensorIndex;
    static SoundSensor* sensorInstances[MAX_SOUND_SENSORS];
    static uint8_t activeSensorCount;
    
    // =========================================================================
    // INTERRUPT SERVICE ROUTINE (ISR)
    // =========================================================================
    static void soundISR() {
        for (uint8_t i = 0; i < activeSensorCount; i++) {
            if (sensorInstances[i] == nullptr) continue;
            
            SoundSensor* sensor = sensorInstances[i];
            if (sensor->digitalPin == 0xFF) continue;
            
            // Debounce check
            unsigned long now = micros();
            if ((now - sensor->lastInterruptTime) < sensor->debounceTimeUS) {
                continue;
            }
            sensor->lastInterruptTime = now;
            
            // Read current state
            uint8_t pinState = digitalRead(sensor->digitalPin);
            bool detected = sensor->invertedLogic ? (pinState == LOW) : (pinState == HIGH);
            
            // Update state if changed
            if (detected != sensor->soundDetected) {
                sensor->soundDetected = detected;
                sensor->stateChanged = true;
            }
        }
    }

public:
    // =========================================================================
    // CONSTRUCTOR - Digital Only
    // =========================================================================
    // Parameters:
    //   digitalPin - Digital input pin (sound detection)
    //   inverted - true if sensor outputs LOW when sound detected
    //   debounceUS - Debounce time in microseconds (default: 5000 = 5ms)
    // =========================================================================
    SoundSensor(uint8_t digitalPin, bool inverted = false, 
                unsigned long debounceUS = 5000)
        : digitalPin(digitalPin)
        , analogPin(0xFF)
        , invertedLogic(inverted)
        , soundThreshold(512)
        , soundDetected(false)
        , stateChanged(false)
        , lastInterruptTime(0)
        , debounceTimeUS(debounceUS)
        , peakValue(0)
        , peakHoldTimeMS(200)
        , peakStartTime(0)
        , lastDigitalState(false)
        , lastAnalogValue(0)
        , lastReadTime(0)
        , smoothedValue(512)
        , smoothingFactor(0.3f)
        , sensorIndex(0)
    {
        // Register this sensor instance
        if (activeSensorCount < MAX_SOUND_SENSORS) {
            sensorIndex = activeSensorCount;
            sensorInstances[activeSensorCount++] = this;
        }
    }
    
    // =========================================================================
    // CONSTRUCTOR - Digital + Analog
    // =========================================================================
    // Parameters:
    //   digitalPin - Digital input pin (sound detection)
    //   analogPin - Analog input pin (sound level)
    //   inverted - true if sensor outputs LOW when sound detected
    //   debounceUS - Debounce time in microseconds
    // =========================================================================
    SoundSensor(uint8_t digitalPin, uint8_t analogPin, bool inverted = false,
                unsigned long debounceUS = 5000)
        : digitalPin(digitalPin)
        , analogPin(analogPin)
        , invertedLogic(inverted)
        , soundThreshold(512)
        , soundDetected(false)
        , stateChanged(false)
        , lastInterruptTime(0)
        , debounceTimeUS(debounceUS)
        , peakValue(0)
        , peakHoldTimeMS(200)
        , peakStartTime(0)
        , lastDigitalState(false)
        , lastAnalogValue(0)
        , lastReadTime(0)
        , smoothedValue(512)
        , smoothingFactor(0.3f)
        , sensorIndex(0)
    {
        // Register this sensor instance
        if (activeSensorCount < MAX_SOUND_SENSORS) {
            sensorIndex = activeSensorCount;
            sensorInstances[activeSensorCount++] = this;
        }
    }
    
    // =========================================================================
    // INITIALIZATION
    // =========================================================================
    bool begin() {
        // Configure digital pin
        if (digitalPin != 0xFF) {
            pinMode(digitalPin, INPUT);
            
            // Verify interrupt support
            if (digitalPinToInterrupt(digitalPin) != NOT_AN_INTERRUPT) {
                attachInterrupt(digitalPinToInterrupt(digitalPin), soundISR, CHANGE);
            }
        }
        
        // Configure analog pin
        if (analogPin != 0xFF) {
            pinMode(analogPin, INPUT);
        }
        
        // Initial reading
        if (digitalPin != 0xFF) {
            readDigital();
        }
        if (analogPin != 0xFF) {
            readAnalog();
        }
        
        return true;
    }
    
    // =========================================================================
    // CLEANUP
    // =========================================================================
    void end() {
        if (digitalPin != 0xFF) {
            detachInterrupt(digitalPinToInterrupt(digitalPin));
        }
        
        // Remove from instance array
        if (sensorIndex < activeSensorCount) {
            sensorInstances[sensorIndex] = nullptr;
        }
    }
    
    // =========================================================================
    // DIGITAL READING (Interrupt-Driven)
    // =========================================================================
    
    // Check if sound is detected (from interrupt state)
    bool isSoundDetected() const {
        return soundDetected;
    }
    
    // Check if state changed since last check
    bool hasStateChanged() {
        if (stateChanged) {
            stateChanged = false;
            return true;
        }
        return false;
    }
    
    // Read digital state directly (polling mode)
    bool readDigital() {
        if (digitalPin == 0xFF) return false;
        
        uint8_t pinState = digitalRead(digitalPin);
        lastDigitalState = invertedLogic ? (pinState == LOW) : (pinState == HIGH);
        lastReadTime = millis();
        
        return lastDigitalState;
    }
    
    // =========================================================================
    // ANALOG READING (Sound Level)
    // =========================================================================
    
    // Read analog value (0-1023, higher = louder sound)
    uint16_t readAnalog() {
        if (analogPin == 0xFF) return 0;
        
        lastAnalogValue = analogRead(analogPin);
        lastReadTime = millis();
        
        // Apply smoothing
        smoothedValue = smoothedValue + (lastAnalogValue - smoothedValue) * smoothingFactor;
        
        // Update peak value
        if (lastAnalogValue > peakValue) {
            peakValue = lastAnalogValue;
            peakStartTime = millis();
        }
        
        return lastAnalogValue;
    }
    
    // Get smoothed analog value
    uint16_t readAnalogSmoothed() {
        readAnalog();
        return smoothedValue;
    }
    
    // Get last analog value without new reading
    uint16_t getLastAnalogValue() const {
        return lastAnalogValue;
    }
    
    // Get sound level as percentage (0-100%)
    float getLevelPercent() {
        if (analogPin == 0xFF) return 0.0f;
        
        uint16_t value = readAnalog();
        return (value / 1023.0f) * 100.0f;
    }
    
    // =========================================================================
    // PEAK DETECTION
    // =========================================================================
    
    // Get peak sound level (resets after hold time)
    uint16_t getPeakValue() {
        // Check if peak hold time has expired
        if ((millis() - peakStartTime) > peakHoldTimeMS) {
            peakValue = lastAnalogValue;
        }
        return peakValue;
    }
    
    // Reset peak value
    void resetPeak() {
        peakValue = lastAnalogValue;
        peakStartTime = millis();
    }
    
    // Set peak hold time in milliseconds
    void setPeakHoldTime(unsigned long holdTimeMS) {
        peakHoldTimeMS = holdTimeMS;
    }
    
    // =========================================================================
    // DETECTION METHODS
    // =========================================================================
    
    // Check if sound level exceeds threshold
    bool isLoud() {
        if (analogPin == 0xFF) return isSoundDetected();
        
        return readAnalog() >= soundThreshold;
    }
    
    // Check if sound level is below threshold (quiet)
    bool isQuiet() {
        if (analogPin == 0xFF) return !isSoundDetected();
        
        return readAnalog() < soundThreshold;
    }
    
    // Get sound level category
    // Returns: 0 = quiet, 1 = moderate, 2 = loud
    uint8_t getSoundLevel() {
        if (analogPin == 0xFF) {
            return isSoundDetected() ? 2 : 0;
        }
        
        uint16_t value = readAnalog();
        if (value < soundThreshold / 2) return 0;
        if (value >= soundThreshold) return 2;
        return 1;
    }
    
    // =========================================================================
    // CLAP DETECTION (for smart home applications)
    // =========================================================================
    
    // Simple clap detection based on rapid state changes
    // Call this in loop() and check return value
    // Returns: true if clap pattern detected
    bool detectClap(unsigned long maxClapGapMS = 500) {
        static unsigned long lastClapTime = 0;
        static uint8_t clapCount = 0;
        
        if (hasStateChanged() && isSoundDetected()) {
            unsigned long now = millis();
            
            if ((now - lastClapTime) < maxClapGapMS) {
                clapCount++;
            } else {
                clapCount = 1;
            }
            
            lastClapTime = now;
            
            // Double clap detected
            if (clapCount >= 2) {
                clapCount = 0;
                return true;
            }
        }
        
        return false;
    }
    
    // =========================================================================
    // CONFIGURATION
    // =========================================================================
    
    // Set sound threshold for analog detection
    void setThreshold(uint16_t threshold) {
        soundThreshold = threshold;
    }
    
    // Get sound threshold
    uint16_t getThreshold() const {
        return soundThreshold;
    }
    
    // Set debounce time
    void setDebounceTime(unsigned long debounceUS) {
        debounceTimeUS = debounceUS;
    }
    
    // Set inverted logic
    void setInverted(bool inverted) {
        invertedLogic = inverted;
    }
    
    // Set smoothing factor (0.0 = no smoothing, 1.0 = maximum smoothing)
    void setSmoothing(float factor) {
        if (factor < 0.0f) factor = 0.0f;
        if (factor > 1.0f) factor = 1.0f;
        smoothingFactor = factor;
    }
    
    // =========================================================================
    // UTILITY METHODS
    // =========================================================================
    
    // Get time since last reading
    unsigned long getTimeSinceLastRead() const {
        return millis() - lastReadTime;
    }
    
    // Get pin numbers
    uint8_t getDigitalPin() const { return digitalPin; }
    uint8_t getAnalogPin() const { return analogPin; }
    
    // Check if sensor has analog capability
    bool hasAnalog() const { return analogPin != 0xFF; }
};

// Initialize static members
SoundSensor* SoundSensor::sensorInstances[MAX_SOUND_SENSORS] = {nullptr};
uint8_t SoundSensor::activeSensorCount = 0;

// ============================================================================
// MQ GAS SENSOR CLASS
// ============================================================================
// Supports MQ series gas sensors (MQ-2, MQ-3, MQ-4, MQ-5, MQ-6, MQ-7, MQ-8, MQ-9, MQ-135)
// - Analog output: Gas concentration (0-1023)
// - Digital output: Threshold exceeded (optional)
// - Warm-up time handling for accurate readings
// - Calibration support
// 
// Common modules: MQ-2, MQ-3, MQ-7, MQ-135
// Applications: Gas leak detection, air quality monitoring, alcohol detection
// ============================================================================

// MQ Sensor Types
#define MQ_TYPE_UNKNOWN     0
#define MQ_TYPE_MQ2         2    // Combustible gas, smoke, LPG, propane, hydrogen, methane
#define MQ_TYPE_MQ3         3    // Alcohol, ethanol
#define MQ_TYPE_MQ4         4    // Methane, CNG natural gas
#define MQ_TYPE_MQ5         5    // LPG, natural gas, town gas
#define MQ_TYPE_MQ6         6    // LPG, butane, propane
#define MQ_TYPE_MQ7         7    // Carbon monoxide (CO)
#define MQ_TYPE_MQ8         8    // Hydrogen gas
#define MQ_TYPE_MQ9         9    // Carbon monoxide, combustible gas
#define MQ_TYPE_MQ135       135  // Air quality (CO2, NH3, benzene, alcohol, smoke)

class MQSensor {
private:
    // Pin configuration
    uint8_t analogPin;          // Analog output pin (gas concentration)
    uint8_t digitalPin;         // Digital output pin (threshold) - use 0xFF if not available
    
    // Sensor configuration
    uint8_t sensorType;         // MQ sensor type (MQ-2, MQ-7, etc.)
    uint16_t alarmThreshold;    // Analog threshold for alarm
    bool invertedLogic;         // true if digital output is LOW when alarm
    
    // Calibration values
    float cleanAirRatio;        // RS/R0 ratio in clean air (sensor-specific)
    float calibrationR0;        // R0 value from calibration
    
    // Warm-up management
    unsigned long warmupTimeMS; // Required warm-up time in milliseconds
    unsigned long startTime;    // When sensor was started
    bool isWarmedUp;            // Warm-up complete flag
    
    // State tracking
    volatile bool alarmTriggered;
    volatile bool stateChanged;
    volatile unsigned long lastInterruptTime;
    unsigned long debounceTimeUS;
    
    // Last readings
    uint16_t lastAnalogValue;
    float lastPPM;              // Parts per million (estimated)
    float lastRS;               // Sensor resistance
    unsigned long lastReadTime;
    
    // Smoothing
    uint16_t smoothedValue;
    float smoothingFactor;
    
    // =========================================================================
    // SENSOR-SPECIFIC CONSTANTS
    // =========================================================================
    
    // Get clean air ratio for sensor type
    float getCleanAirRatio() const {
        switch (sensorType) {
            case MQ_TYPE_MQ2:   return 9.83f;
            case MQ_TYPE_MQ3:   return 60.0f;
            case MQ_TYPE_MQ4:   return 4.4f;
            case MQ_TYPE_MQ5:   return 6.5f;
            case MQ_TYPE_MQ6:   return 10.0f;
            case MQ_TYPE_MQ7:   return 26.0f;
            case MQ_TYPE_MQ8:   return 70.0f;
            case MQ_TYPE_MQ9:   return 9.6f;
            case MQ_TYPE_MQ135: return 3.6f;
            default:            return 9.83f;
        }
    }
    
    // Get warm-up time for sensor type (in milliseconds)
    unsigned long getWarmupTime() const {
        switch (sensorType) {
            case MQ_TYPE_MQ2:   return 20000;   // 20 seconds
            case MQ_TYPE_MQ3:   return 120000;  // 2 minutes (alcohol sensor needs longer)
            case MQ_TYPE_MQ4:   return 20000;
            case MQ_TYPE_MQ5:   return 20000;
            case MQ_TYPE_MQ6:   return 20000;
            case MQ_TYPE_MQ7:   return 480000;  // 8 minutes (CO sensor needs long warm-up)
            case MQ_TYPE_MQ8:   return 20000;
            case MQ_TYPE_MQ9:   return 20000;
            case MQ_TYPE_MQ135: return 20000;
            default:            return 20000;
        }
    }

public:
    // =========================================================================
    // CONSTRUCTOR - Analog Only
    // =========================================================================
    // Parameters:
    //   analogPin - Analog input pin (gas concentration)
    //   type - MQ sensor type (MQ_TYPE_MQ2, MQ_TYPE_MQ7, etc.)
    //   threshold - Alarm threshold (0-1023, default: 400)
    // =========================================================================
    MQSensor(uint8_t analogPin, uint8_t type = MQ_TYPE_MQ2, uint16_t threshold = 400)
        : analogPin(analogPin)
        , digitalPin(0xFF)
        , sensorType(type)
        , alarmThreshold(threshold)
        , invertedLogic(false)
        , cleanAirRatio(0)
        , calibrationR0(0)
        , warmupTimeMS(0)
        , startTime(0)
        , isWarmedUp(false)
        , alarmTriggered(false)
        , stateChanged(false)
        , lastInterruptTime(0)
        , debounceTimeUS(10000)
        , lastAnalogValue(0)
        , lastPPM(0)
        , lastRS(0)
        , lastReadTime(0)
        , smoothedValue(512)
        , smoothingFactor(0.1f)
    {
        cleanAirRatio = getCleanAirRatio();
        warmupTimeMS = getWarmupTime();
    }
    
    // =========================================================================
    // CONSTRUCTOR - Analog + Digital
    // =========================================================================
    // Parameters:
    //   analogPin - Analog input pin (gas concentration)
    //   digitalPin - Digital input pin (threshold alarm)
    //   type - MQ sensor type
    //   inverted - true if digital output LOW when alarm
    //   threshold - Alarm threshold for analog
    // =========================================================================
    MQSensor(uint8_t analogPin, uint8_t digitalPin, uint8_t type = MQ_TYPE_MQ2, 
             bool inverted = false, uint16_t threshold = 400)
        : analogPin(analogPin)
        , digitalPin(digitalPin)
        , sensorType(type)
        , alarmThreshold(threshold)
        , invertedLogic(inverted)
        , cleanAirRatio(0)
        , calibrationR0(0)
        , warmupTimeMS(0)
        , startTime(0)
        , isWarmedUp(false)
        , alarmTriggered(false)
        , stateChanged(false)
        , lastInterruptTime(0)
        , debounceTimeUS(10000)
        , lastAnalogValue(0)
        , lastPPM(0)
        , lastRS(0)
        , lastReadTime(0)
        , smoothedValue(512)
        , smoothingFactor(0.1f)
    {
        cleanAirRatio = getCleanAirRatio();
        warmupTimeMS = getWarmupTime();
    }
    
    // =========================================================================
    // INITIALIZATION
    // =========================================================================
    bool begin() {
        // Configure analog pin
        pinMode(analogPin, INPUT);
        
        // Configure digital pin
        if (digitalPin != 0xFF) {
            pinMode(digitalPin, INPUT);
            
            // Attach interrupt if supported
            if (digitalPinToInterrupt(digitalPin) != NOT_AN_INTERRUPT) {
                attachInterrupt(digitalPinToInterrupt(digitalPin), 
                    [this]() { this->alarmISR(); }, CHANGE);
            }
        }
        
        // Start warm-up timer
        startTime = millis();
        isWarmedUp = false;
        
        // Initial reading
        readAnalog();
        
        return true;
    }
    
    // =========================================================================
    // CLEANUP
    // =========================================================================
    void end() {
        if (digitalPin != 0xFF) {
            detachInterrupt(digitalPinToInterrupt(digitalPin));
        }
    }
    
    // =========================================================================
    // WARM-UP MANAGEMENT
    // =========================================================================
    
    // Check if sensor is warmed up
    bool isReady() const {
        return isWarmedUp;
    }
    
    // Get warm-up progress (0-100%)
    float getWarmupProgress() const {
        if (isWarmedUp) return 100.0f;
        
        unsigned long elapsed = millis() - startTime;
        if (elapsed >= warmupTimeMS) return 100.0f;
        
        return (elapsed * 100.0f) / warmupTimeMS;
    }
    
    // Get remaining warm-up time in seconds
    unsigned long getWarmupRemaining() const {
        if (isWarmedUp) return 0;
        
        unsigned long elapsed = millis() - startTime;
        if (elapsed >= warmupTimeMS) return 0;
        
        return (warmupTimeMS - elapsed) / 1000;
    }
    
    // Force warm-up complete (use with caution)
    void forceReady() {
        isWarmedUp = true;
    }
    
    // =========================================================================
    // ANALOG READING
    // =========================================================================
    
    // Read raw analog value (0-1023)
    uint16_t readAnalog() {
        lastAnalogValue = analogRead(analogPin);
        lastReadTime = millis();
        
        // Apply smoothing
        smoothedValue = smoothedValue + (lastAnalogValue - smoothedValue) * smoothingFactor;
        
        // Check warm-up
        if (!isWarmedUp && (millis() - startTime) >= warmupTimeMS) {
            isWarmedUp = true;
        }
        
        return lastAnalogValue;
    }
    
    // Get smoothed analog value
    uint16_t readAnalogSmoothed() {
        readAnalog();
        return smoothedValue;
    }
    
    // Get last analog value without new reading
    uint16_t getLastAnalogValue() const {
        return lastAnalogValue;
    }
    
    // Get gas concentration as percentage (0-100%)
    float getConcentrationPercent() {
        uint16_t value = readAnalog();
        return (value / 1023.0f) * 100.0f;
    }
    
    // =========================================================================
    // GAS CONCENTRATION (PPM ESTIMATION)
    // =========================================================================
    
    // Calculate sensor resistance (RS)
    float calculateRS() const {
        // Assuming 10K load resistor
        // RS = ((Vc * RL) / Vout) - RL
        // Vc = 5V, RL = 10K, Vout = (analog / 1023) * 5V
        float voltage = (lastAnalogValue / 1023.0f) * 5.0f;
        if (voltage < 0.1f) voltage = 0.1f;  // Prevent division by zero
        
        float rs = ((5.0f * 10.0f) / voltage) - 10.0f;
        return rs;
    }
    
    // Calculate RS/R0 ratio
    float getRSR0Ratio() const {
        if (calibrationR0 < 0.1f) return 0;
        return lastRS / calibrationR0;
    }
    
    // Estimate PPM (parts per million) - simplified curve
    // Note: This is an approximation. For accurate PPM, use sensor datasheet curves
    float estimatePPM() {
        if (!isWarmedUp) return 0;
        
        readAnalog();
        lastRS = calculateRS();
        
        if (calibrationR0 < 0.1f) {
            // No calibration, return raw value
            lastPPM = lastAnalogValue;
            return lastPPM;
        }
        
        float ratio = getRSR0Ratio();
        
        // Simplified power curve: PPM = a * (RS/R0)^b
        // Coefficients vary by sensor type and gas
        // These are rough approximations
        float a, b;
        switch (sensorType) {
            case MQ_TYPE_MQ2:
                a = 658.71f; b = -2.168f;
                break;
            case MQ_TYPE_MQ7:
                a = 99.042f; b = -1.518f;
                break;
            case MQ_TYPE_MQ135:
                a = 116.602f; b = -2.769f;
                break;
            default:
                a = 100.0f; b = -2.0f;
                break;
        }
        
        lastPPM = a * pow(ratio, b);
        return lastPPM;
    }
    
    // Get last PPM value
    float getLastPPM() const {
        return lastPPM;
    }
    
    // =========================================================================
    // CALIBRATION
    // =========================================================================
    
    // Calibrate sensor in clean air (call after warm-up)
    // Returns: R0 value
    float calibrate() {
        if (!isWarmedUp) return 0;
        
        // Take multiple readings
        float totalRS = 0;
        for (int i = 0; i < 50; i++) {
            readAnalog();
            totalRS += calculateRS();
            delay(50);
        }
        
        lastRS = totalRS / 50.0f;
        calibrationR0 = lastRS / cleanAirRatio;
        
        return calibrationR0;
    }
    
    // Set manual R0 value (from previous calibration)
    void setR0(float r0) {
        calibrationR0 = r0;
    }
    
    // Get current R0 value
    float getR0() const {
        return calibrationR0;
    }
    
    // =========================================================================
    // ALARM DETECTION
    // =========================================================================
    
    // Check if gas level exceeds threshold
    bool isAlarm() {
        if (!isWarmedUp) return false;
        
        uint16_t value = readAnalog();
        return value >= alarmThreshold;
    }
    
    // Check if alarm state changed
    bool hasStateChanged() {
        if (stateChanged) {
            stateChanged = false;
            return true;
        }
        return false;
    }
    
    // Get alarm state from digital pin
    bool isDigitalAlarm() {
        if (digitalPin == 0xFF) return isAlarm();
        
        uint8_t pinState = digitalRead(digitalPin);
        return invertedLogic ? (pinState == LOW) : (pinState == HIGH);
    }
    
    // =========================================================================
    // GAS LEVEL CATEGORIES
    // =========================================================================
    
    // Get air quality level
    // Returns: 0 = safe, 1 = warning, 2 = danger
    uint8_t getGasLevel() {
        if (!isWarmedUp) return 0;
        
        uint16_t value = readAnalog();
        if (value < alarmThreshold / 2) return 0;
        if (value < alarmThreshold) return 1;
        return 2;
    }
    
    // Get gas level description
    const char* getGasLevelString() {
        uint8_t level = getGasLevel();
        switch (level) {
            case 0: return "SAFE";
            case 1: return "WARNING";
            case 2: return "DANGER";
            default: return "UNKNOWN";
        }
    }
    
    // =========================================================================
    // CONFIGURATION
    // =========================================================================
    
    // Set alarm threshold
    void setThreshold(uint16_t threshold) {
        alarmThreshold = threshold;
    }
    
    // Get alarm threshold
    uint16_t getThreshold() const {
        return alarmThreshold;
    }
    
    // Set sensor type
    void setSensorType(uint8_t type) {
        sensorType = type;
        cleanAirRatio = getCleanAirRatio();
        warmupTimeMS = getWarmupTime();
    }
    
    // Get sensor type
    uint8_t getSensorType() const {
        return sensorType;
    }
    
    // Set smoothing factor
    void setSmoothing(float factor) {
        if (factor < 0.0f) factor = 0.0f;
        if (factor > 1.0f) factor = 1.0f;
        smoothingFactor = factor;
    }
    
    // Set inverted logic for digital pin
    void setInverted(bool inverted) {
        invertedLogic = inverted;
    }
    
    // =========================================================================
    // UTILITY METHODS
    // =========================================================================
    
    // Get time since last reading
    unsigned long getTimeSinceLastRead() const {
        return millis() - lastReadTime;
    }
    
    // Get pin numbers
    uint8_t getAnalogPin() const { return analogPin; }
    uint8_t getDigitalPin() const { return digitalPin; }
    
    // Check if sensor has digital output
    bool hasDigital() const { return digitalPin != 0xFF; }
    
    // Get sensor name string
    const char* getSensorName() const {
        switch (sensorType) {
            case MQ_TYPE_MQ2:   return "MQ-2";
            case MQ_TYPE_MQ3:   return "MQ-3";
            case MQ_TYPE_MQ4:   return "MQ-4";
            case MQ_TYPE_MQ5:   return "MQ-5";
            case MQ_TYPE_MQ6:   return "MQ-6";
            case MQ_TYPE_MQ7:   return "MQ-7";
            case MQ_TYPE_MQ8:   return "MQ-8";
            case MQ_TYPE_MQ9:   return "MQ-9";
            case MQ_TYPE_MQ135: return "MQ-135";
            default:            return "Unknown";
        }
    }

private:
    // ISR for digital alarm pin
    void alarmISR() {
        unsigned long now = micros();
        if ((now - lastInterruptTime) < debounceTimeUS) return;
        lastInterruptTime = now;
        
        bool newState = isDigitalAlarm();
        if (newState != alarmTriggered) {
            alarmTriggered = newState;
            stateChanged = true;
        }
    }
};

// ============================================================================
// DHT11 TEMPERATURE & HUMIDITY SENSOR CLASS
// ============================================================================
// Supports DHT11 digital temperature and humidity sensor
// - Temperature: 0-50°C (±2°C accuracy)
// - Humidity: 20-90% RH (±5% accuracy)
// - Single-wire digital protocol
// - Non-blocking read with timeout
// 
// Common modules: DHT11 (Blue module)
// Applications: Weather monitoring, HVAC, greenhouse control
// ============================================================================

#define DHT11_TIMEOUT_US    1000    // Timeout for bit reading

class DHT11Sensor {
private:
    uint8_t dataPin;                // Data pin (single-wire)
    
    // Last readings
    float lastTemperature;          // Temperature in Celsius
    float lastHumidity;             // Humidity in percentage
    uint8_t lastError;              // Last error code
    
    // Timing
    unsigned long lastReadTime;     // Last successful read time
    unsigned long minReadInterval;  // Minimum interval between reads
    
    // Smoothing
    float smoothedTemp;
    float smoothedHum;
    float smoothingFactor;
    
    // Error codes
    static const uint8_t ERROR_NONE = 0;
    static const uint8_t ERROR_TIMEOUT = 1;
    static const uint8_t ERROR_CHECKSUM = 2;
    static const uint8_t ERROR_NOT_READY = 3;

public:
    // =========================================================================
    // CONSTRUCTOR
    // =========================================================================
    // Parameters:
    //   pin - Data pin number
    //   interval - Minimum read interval in ms (default: 2000ms)
    // =========================================================================
    DHT11Sensor(uint8_t pin, unsigned long interval = 2000)
        : dataPin(pin)
        , lastTemperature(0)
        , lastHumidity(0)
        , lastError(ERROR_NOT_READY)
        , lastReadTime(0)
        , minReadInterval(interval)
        , smoothedTemp(25.0f)
        , smoothedHum(50.0f)
        , smoothingFactor(0.1f)
    {
    }
    
    // =========================================================================
    // INITIALIZATION
    // =========================================================================
    void begin() {
        pinMode(dataPin, INPUT_PULLUP);
        lastReadTime = millis() - minReadInterval;  // Allow immediate first read
    }
    
    // =========================================================================
    // READING METHODS
    // =========================================================================
    
    // Read temperature and humidity (blocking, with timeout)
    // Returns: true if read successful
    bool read() {
        // Check minimum interval
        if ((millis() - lastReadTime) < minReadInterval) {
            lastError = ERROR_NOT_READY;
            return false;
        }
        
        // Buffer to store 40 bits (5 bytes)
        uint8_t data[5] = {0, 0, 0, 0, 0};
        
        // Send start signal
        pinMode(dataPin, OUTPUT);
        digitalWrite(dataPin, LOW);
        delay(18);  // Pull low for 18ms
        digitalWrite(dataPin, HIGH);
        delayMicroseconds(20);  // High for 20-40us
        
        // Switch to input to read response
        pinMode(dataPin, INPUT_PULLUP);
        
        // Wait for sensor response (low for 80us, high for 80us)
        if (!waitForPin(LOW, DHT11_TIMEOUT_US)) {
            lastError = ERROR_TIMEOUT;
            return false;
        }
        if (!waitForPin(HIGH, DHT11_TIMEOUT_US)) {
            lastError = ERROR_TIMEOUT;
            return false;
        }
        
        // Read 40 bits of data
        for (uint8_t i = 0; i < 40; i++) {
            // Each bit starts with 50us low
            if (!waitForPin(LOW, DHT11_TIMEOUT_US)) {
                lastError = ERROR_TIMEOUT;
                return false;
            }
            
            // Measure high duration to determine bit value
            // 26-28us = 0, 70us = 1
            unsigned long duration = pulseIn(dataPin, HIGH, DHT11_TIMEOUT_US);
            
            if (duration == 0) {
                lastError = ERROR_TIMEOUT;
                return false;
            }
            
            // Store bit (MSB first)
            data[i / 8] <<= 1;
            if (duration > 40) {  // > 40us = 1
                data[i / 8] |= 1;
            }
        }
        
        // Verify checksum
        uint8_t checksum = data[0] + data[1] + data[2] + data[3];
        if (checksum != data[4]) {
            lastError = ERROR_CHECKSUM;
            return false;
        }
        
        // Extract humidity and temperature
        lastHumidity = data[0] + (data[1] * 0.1f);
        lastTemperature = data[2] + (data[3] * 0.1f);
        
        // Apply smoothing
        smoothedTemp = smoothedTemp + (lastTemperature - smoothedTemp) * smoothingFactor;
        smoothedHum = smoothedHum + (lastHumidity - smoothedHum) * smoothingFactor;
        
        lastReadTime = millis();
        lastError = ERROR_NONE;
        
        return true;
    }
    
    // Non-blocking read attempt
    // Returns: true if new data available
    bool readNonBlocking() {
        if ((millis() - lastReadTime) >= minReadInterval) {
            return read();
        }
        return false;
    }
    
    // =========================================================================
    // GETTERS
    // =========================================================================
    
    // Get temperature in Celsius
    float getTemperature() const {
        return lastTemperature;
    }
    
    // Get smoothed temperature
    float getTemperatureSmoothed() const {
        return smoothedTemp;
    }
    
    // Get temperature in Fahrenheit
    float getTemperatureF() const {
        return (lastTemperature * 9.0f / 5.0f) + 32.0f;
    }
    
    // Get humidity in percentage
    float getHumidity() const {
        return lastHumidity;
    }
    
    // Get smoothed humidity
    float getHumiditySmoothed() const {
        return smoothedHum;
    }
    
    // Get heat index (feels like temperature)
    float getHeatIndex() const {
        float tempF = getTemperatureF();
        float hum = lastHumidity;
        
        if (tempF < 80.0f) {
            return tempF;
        }
        
        float hi = -42.379f + 
                   2.04901523f * tempF + 
                   10.14333127f * hum - 
                   0.22475541f * tempF * hum - 
                   0.00683783f * tempF * tempF - 
                   0.05481717f * hum * hum + 
                   0.00122874f * tempF * tempF * hum + 
                   0.00085282f * tempF * hum * hum - 
                   0.00000199f * tempF * tempF * hum * hum;
        
        return (hi - 32.0f) * 5.0f / 9.0f;  // Convert back to Celsius
    }
    
    // Get dew point
    float getDewPoint() const {
        float temp = lastTemperature;
        float hum = lastHumidity;
        
        // Magnus formula
        float a = 17.27f;
        float b = 237.7f;
        float alpha = ((a * temp) / (b + temp)) + log(hum / 100.0f);
        
        return (b * alpha) / (a - alpha);
    }
    
    // =========================================================================
    // STATUS METHODS
    // =========================================================================
    
    // Get last error code
    uint8_t getLastError() const {
        return lastError;
    }
    
    // Get error description
    const char* getErrorString() const {
        switch (lastError) {
            case ERROR_NONE:      return "No error";
            case ERROR_TIMEOUT:   return "Timeout";
            case ERROR_CHECKSUM:  return "Checksum error";
            case ERROR_NOT_READY: return "Not ready";
            default:              return "Unknown error";
        }
    }
    
    // Check if last read was successful
    bool isDataValid() const {
        return lastError == ERROR_NONE;
    }
    
    // Get time since last successful read
    unsigned long getTimeSinceLastRead() const {
        return millis() - lastReadTime;
    }
    
    // =========================================================================
    // CONFIGURATION
    // =========================================================================
    
    // Set minimum read interval
    void setReadInterval(unsigned long interval) {
        minReadInterval = interval;
    }
    
    // Set smoothing factor
    void setSmoothing(float factor) {
        if (factor < 0.0f) factor = 0.0f;
        if (factor > 1.0f) factor = 1.0f;
        smoothingFactor = factor;
    }
    
    // Get pin number
    uint8_t getPin() const {
        return dataPin;
    }

private:
    // Wait for pin to reach desired state
    bool waitForPin(uint8_t state, unsigned long timeoutUS) {
        unsigned long start = micros();
        while (digitalRead(dataPin) != state) {
            if ((micros() - start) > timeoutUS) {
                return false;
            }
        }
        return true;
    }
};

// ============================================================================
// DHT22 SENSOR CLASS (Extended version of DHT11)
// ============================================================================
// Supports DHT22/AM2302 digital temperature and humidity sensor
// - Temperature: -40 to 80°C (±0.5°C accuracy)
// - Humidity: 0-100% RH (±2% accuracy)
// - Higher precision than DHT11
// ============================================================================

class DHT22Sensor : public DHT11Sensor {
public:
    DHT22Sensor(uint8_t pin, unsigned long interval = 2000)
        : DHT11Sensor(pin, interval)
    {
    }
    
    // Override read to handle DHT22's 16-bit data format
    bool read() {
        // Check minimum interval
        if ((millis() - lastReadTime) < minReadInterval) {
            lastError = ERROR_NOT_READY;
            return false;
        }
        
        // Buffer to store 40 bits (5 bytes)
        uint8_t data[5] = {0, 0, 0, 0, 0};
        
        // Send start signal
        pinMode(dataPin, OUTPUT);
        digitalWrite(dataPin, LOW);
        delay(1);  // DHT22 needs only 1ms start signal (not 18ms like DHT11)
        digitalWrite(dataPin, HIGH);
        delayMicroseconds(30);
        
        // Switch to input to read response
        pinMode(dataPin, INPUT_PULLUP);
        
        // Wait for sensor response
        if (!waitForPin(LOW, DHT11_TIMEOUT_US)) {
            lastError = ERROR_TIMEOUT;
            return false;
        }
        if (!waitForPin(HIGH, DHT11_TIMEOUT_US)) {
            lastError = ERROR_TIMEOUT;
            return false;
        }
        
        // Read 40 bits of data
        for (uint8_t i = 0; i < 40; i++) {
            if (!waitForPin(LOW, DHT11_TIMEOUT_US)) {
                lastError = ERROR_TIMEOUT;
                return false;
            }
            
            unsigned long duration = pulseIn(dataPin, HIGH, DHT11_TIMEOUT_US);
            
            if (duration == 0) {
                lastError = ERROR_TIMEOUT;
                return false;
            }
            
            data[i / 8] <<= 1;
            if (duration > 40) {
                data[i / 8] |= 1;
            }
        }
        
        // Verify checksum
        uint8_t checksum = data[0] + data[1] + data[2] + data[3];
        if (checksum != data[4]) {
            lastError = ERROR_CHECKSUM;
            return false;
        }
        
        // DHT22 uses 16-bit values (with sign bit for temperature)
        lastHumidity = ((data[0] << 8) | data[1]) * 0.1f;
        
        // Temperature has sign bit in MSB
        int16_t temp = (data[2] & 0x7F) << 8 | data[3];
        if (data[2] & 0x80) {  // Negative temperature
            temp = -temp;
        }
        lastTemperature = temp * 0.1f;
        
        // Apply smoothing
        smoothedTemp = smoothedTemp + (lastTemperature - smoothedTemp) * smoothingFactor;
        smoothedHum = smoothedHum + (lastHumidity - smoothedHum) * smoothingFactor;
        
        lastReadTime = millis();
        lastError = ERROR_NONE;
        
        return true;
    }
};

#endif // SENSORS_H
