#ifndef HX711_H
#define HX711_H

#include <Arduino.h>

/**
 * @brief HX711 channel and gain selection.
 * @details The number of extra SCK pulses sent after the 24 data bits
 * determines the channel and gain used for the next conversion:
 * 25 total pulses for Channel A gain 128, 26 for Channel B gain 32,
 * and 27 for Channel A gain 64.
 */
enum HX711Gain : uint8_t {
    // For a standard single load cell scale, use GAIN_A_128 (default) for best resolution.
    GAIN_A_128 = 1,   // 25 total pulses — Channel A, Gain 128
    GAIN_B_32  = 2,   // 26 total pulses — Channel B, Gain 32
    GAIN_A_64  = 3    // 27 total pulses — Channel A, Gain 64
};

/**
 * @brief HX711 ADC driver for load-cell measurements.
 * @details Provides initialization, raw and averaged sampling, tare/scale
 * calibration helpers, gain selection, and power control.
 */
class HX711 {
public:
    /**
     * @brief Constructs an HX711 driver instance.
     */
    HX711();

    /**
     * @brief Initializes GPIO pins and applies gain selection.
     * @details Configures DOUT as input and SCK as output, then performs a
     * dummy conversion to latch the selected gain/channel for subsequent reads.
     * @param[in] {uint8_t} dout HX711 DOUT pin.
     * @param[in] {uint8_t} sck HX711 SCK pin.
     * @param[in] {HX711Gain} gain Channel/gain setting to apply.
     */
    void   begin(uint8_t dout, uint8_t sck, HX711Gain gain = GAIN_A_128);

    /**
     * @brief Checks whether conversion data is ready.
     * @return {bool} True when DOUT is LOW, otherwise false.
     */
    bool   isReady() const;

    /**
     * @brief Reads a single raw 24-bit sample.
     * @details Blocks until the HX711 is ready or timeout is reached.
     * @return {long} Signed raw ADC value, or LONG_MIN on timeout.
     */
    long   readRaw();

    /**
     * @brief Averages multiple raw readings.
     * @param[in] {uint8_t} times Number of readings to average.
     * @return {long} Averaged raw value, or LONG_MIN if no valid samples.
     */
    long   readAverage(uint8_t times = 10);

    /**
     * @brief Converts averaged raw readings into user units.
     * @details Uses the relation (average_raw - offset) / scale.
     * @param[in] {uint8_t} times Number of readings to average.
     * @return {double} Scaled value in user-defined units.
     */
    double getUnits(uint8_t times = 1);

    /**
     * @brief Sets tare offset from the current average reading.
     * @param[in] {uint8_t} times Number of readings to average for tare.
     */
    void   tare(uint8_t times = 10);

    /**
     * @brief Sets calibration scale factor.
     * @param[in] {float} scale Raw ADC counts per user unit.
     */
    void   setScale(float scale);

    /**
     * @brief Gets current calibration scale factor.
     * @return {float} Raw ADC counts per user unit.
     */
    float  getScale()  const;

    /**
     * @brief Sets tare offset in raw ADC counts.
     * @param[in] {long} offset Tare offset value.
     */
    void   setOffset(long offset);

    /**
     * @brief Gets tare offset in raw ADC counts.
     * @return {long} Current tare offset.
     */
    long   getOffset() const;

    /**
     * @brief Changes HX711 channel/gain selection.
     * @details Applies selection by issuing a dummy read cycle.
     * @param[in] {HX711Gain} gain Channel/gain to select.
     */
    void   setGain(HX711Gain gain);

    /**
     * @brief Puts HX711 into low-power mode.
     */
    void   powerDown();

    /**
     * @brief Wakes HX711 from low-power mode.
     */
    void   powerUp();

private:
    /** @brief HX711 DOUT pin number. */
    uint8_t _dout;
    /** @brief HX711 SCK pin number. */
    uint8_t _sck;
    /** @brief Extra SCK pulses after 24 data bits (1, 2, or 3). */
    uint8_t _gainPulses;
    /** @brief Tare offset in raw ADC counts. */
    long    _offset;
    /** @brief Calibration scale factor in counts per user unit. */
    float   _scale;

    /**
     * @brief Reads one 24-bit sample and clocks gain pulses.
     * @details Must be called with interrupts disabled to preserve strict
     * HX711 timing requirements.
     * @return {long} Sign-extended 24-bit raw sample.
     */
    long    _shiftIn();
};

#endif // HX711_H
