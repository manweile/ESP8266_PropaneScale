#include "HX711.h"
#include <limits.h>

/**
 * @brief Constructs an HX711 driver instance with safe defaults.
 * @details Pins are initialized to 0, gain pulses default to Channel A / gain 128,
 * offset defaults to 0, and scale defaults to 1.0.
 */
HX711::HX711()
    : _dout(0), _sck(0), _gainPulses(1), _offset(0L), _scale(1.0f) {}

/**
 * @brief Initializes HX711 GPIO pins and gain configuration.
 * @details Configures DOUT as input and SCK as output, drives SCK low,
 * then performs a dummy read via setGain() so the HX711 applies the
 * requested channel/gain setting for the next conversion.
 * @param[in] {uint8_t} dout HX711 DOUT pin.
 * @param[in] {uint8_t} sck HX711 SCK pin.
 * @param[in] {HX711Gain} gain Channel/gain selection for subsequent conversions.
 */
void HX711::begin(uint8_t dout, uint8_t sck, HX711Gain gain) {
    _dout = dout;
    _sck  = sck;

    pinMode(_dout, INPUT);
    pinMode(_sck,  OUTPUT);
    digitalWrite(_sck, LOW);

    // Programme gain register with one dummy conversion
    setGain(gain);
}

/**
 * @brief Updates HX711 channel/gain selection.
 * @details Stores the number of post-read gain pulses and performs one dummy
 * conversion read so the new setting is latched by the HX711.
 * @param[in] {HX711Gain} gain Channel/gain configuration.
 */
void HX711::setGain(HX711Gain gain) {
    _gainPulses = static_cast<uint8_t>(gain);
    readRaw();   // dummy read to clock the gain setting into the chip
}

/**
 * @brief Indicates whether a conversion result is ready.
 * @return {bool} True when DOUT is LOW (data ready), otherwise false.
 */
bool HX711::isReady() const {
    return digitalRead(_dout) == LOW;
}

/**
 * @brief Shifts in one 24-bit HX711 sample and clocks gain pulses.
 * @details Reads 24 bits MSB-first using 1 us SCK high/low timing, then emits
 * _gainPulses pulses to set the next channel/gain selection. The returned value
 * is sign-extended from 24-bit two's complement to 32-bit signed long.
 * This method must be called with interrupts disabled.
 * @return {long} Signed raw ADC reading after 24-bit sign extension.
 */
long HX711::_shiftIn() {
    long data = 0;

    // Read 24 data bits, MSB first
    for (uint8_t i = 0; i < 24; ++i) {
        digitalWrite(_sck, HIGH);
        delayMicroseconds(1);
        data = (data << 1) | static_cast<long>(digitalRead(_dout));
        digitalWrite(_sck, LOW);
        delayMicroseconds(1);
    }

    // Gain-selection pulses for the next conversion
    for (uint8_t i = 0; i < _gainPulses; ++i) {
        digitalWrite(_sck, HIGH);
        delayMicroseconds(1);
        digitalWrite(_sck, LOW);
        delayMicroseconds(1);
    }

    // Sign-extend 24-bit two's complement to 32-bit signed integer
    if (data & 0x800000L) {
        data |= 0xFF000000L;
    }

    return data;
}

/**
 * @brief Reads one raw conversion from the HX711.
 * @details Waits for DOUT to go LOW with a 1 second timeout. While waiting,
 * yield() is called to service background ESP32 tasks. The bit-bang read is
 * protected by noInterrupts()/interrupts() to preserve timing integrity.
 * @return {long} Raw ADC reading, or LONG_MIN if timeout occurs before ready.
 */
long HX711::readRaw() {
    // Wait for conversion to complete (DOUT goes LOW), with 1-second timeout
    const unsigned long deadline = millis() + 1000UL;
    while (!isReady()) {
        if (millis() > deadline) {
            return LONG_MIN;   // chip not responding — explicit timeout sentinel
        }
        yield();   // service ESP32 WiFi/background tasks while waiting
    }

    noInterrupts();           // protect the ~60 µs timing-critical section
    long value = _shiftIn();
    interrupts();

    return value;
}

/**
 * @brief Computes an average of multiple raw HX711 readings.
 * @details Collects up to times samples, ignoring timeout sentinel values
 * (LONG_MIN). The average is computed from valid samples only. If no valid
 * samples are available, LONG_MIN is returned.
 * @param[in] {uint8_t} times Number of readings to attempt; 0 is treated as 1.
 * @return {long} Average raw value from valid readings, or LONG_MIN if none.
 */
long HX711::readAverage(uint8_t times) {
    if (times == 0U) times = 1U;

    long long sum = 0;
    uint8_t validCount = 0;
    for (uint8_t i = 0; i < times; ++i) {
        const long raw = readRaw();
        if (raw != LONG_MIN) {
            sum += static_cast<long long>(raw);
            ++validCount;
        }
        yield();   // allow ESP32 background processing between samples
    }

    if (validCount == 0U) {
        return LONG_MIN;
    }

    return static_cast<long>(sum / static_cast<long long>(validCount));
}

/**
 * @brief Returns a scaled weight/value in user units.
 * @details Computes (average_raw - offset) / scale using readAverage(times).
 * If no valid raw sample is available, returns 0.0.
 * @param[in] {uint8_t} times Number of samples to average.
 * @return {double} Scaled value in user units.
 */
double HX711::getUnits(uint8_t times) {
    const long avg = readAverage(times);
    if (avg == LONG_MIN) {
        return 0.0;
    }
    return static_cast<double>(avg - _offset) / _scale;
}

/**
 * @brief Sets tare offset from the current averaged raw reading.
 * @details Updates internal offset only when readAverage(times) returns a
 * valid value (not LONG_MIN).
 * @param[in] {uint8_t} times Number of samples to average for tare.
 */
void HX711::tare(uint8_t times) {
    const long avg = readAverage(times);
    if (avg != LONG_MIN) {
        _offset = avg;
    }
}

/**
 * @brief Sets the scale factor used for unit conversion.
 * @param[in] {float} scale Raw counts per user unit.
 */
void   HX711::setScale(float scale)  { _scale  = scale;  }

/**
 * @brief Returns the currently configured scale factor.
 * @return {float} Raw counts per user unit.
 */
float  HX711::getScale()  const      { return _scale;    }

/**
 * @brief Sets the tare offset in raw ADC counts.
 * @param[in] {long} offset Raw offset value.
 */
void   HX711::setOffset(long offset) { _offset = offset; }

/**
 * @brief Returns the tare offset in raw ADC counts.
 * @return {long} Current raw offset value.
 */
long   HX711::getOffset() const      { return _offset;   }

/**
 * @brief Places the HX711 into power-down mode.
 * @details Ensures SCK transitions LOW to HIGH and remains HIGH for at least
 * 60 us (65 us used) per HX711 datasheet.
 */
void HX711::powerDown() {
    digitalWrite(_sck, LOW);
    digitalWrite(_sck, HIGH);
    delayMicroseconds(65);   // datasheet minimum: 60 µs
}

/**
 * @brief Wakes the HX711 from power-down mode.
 * @details Drives SCK LOW and waits approximately 400 ms for the first
 * conversion to become available.
 */
void HX711::powerUp() {
    digitalWrite(_sck, LOW);
    delay(400);
}
