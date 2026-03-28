#ifndef HX711_H
#define HX711_H

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Channel and gain selection
//
// The number of *extra* SCK pulses sent after the 24 data bits determines
// the channel and gain used for the NEXT conversion:
//
//   25 total pulses → Channel A, Gain 128  (±20 mV input range)
//   26 total pulses → Channel B, Gain 32   (±80 mV input range)
//   27 total pulses → Channel A, Gain 64   (±40 mV input range)
//
// For a standard single load-cell scale, use GAIN_A_128 (default).
// ---------------------------------------------------------------------------
enum HX711Gain : uint8_t {
    GAIN_A_128 = 1,   // 25 total pulses — Channel A, Gain 128
    GAIN_B_32  = 2,   // 26 total pulses — Channel B, Gain 32
    GAIN_A_64  = 3    // 27 total pulses — Channel A, Gain 64
};

// ---------------------------------------------------------------------------
// HX711 driver class
//
// Usage:
//   HX711 scale;
//   scale.begin(DOUT_PIN, SCK_PIN);
//   scale.tare();                       // zero with nothing on scale
//   scale.setScale(cal_factor);         // loaded from EEPROM or calibration
//   float weight = scale.getUnits(4);   // average of 4 readings
// ---------------------------------------------------------------------------
class HX711 {
public:
    HX711();

    // Configure GPIO and send one dummy read to programme the gain register.
    void   begin(uint8_t dout, uint8_t sck, HX711Gain gain = GAIN_A_128);

    // Returns true when DOUT is LOW (conversion complete and data ready).
    bool   isReady() const;

    // Read one raw 24-bit two's-complement sample.
    // Blocks until the chip is ready or 1-second timeout elapses.
    long   readRaw();

    // Blocking average of 'times' successive raw readings.
    long   readAverage(uint8_t times = 10);

    // Weighted result in user units: (average_raw − offset) / scale
    double getUnits(uint8_t times = 1);

    // Set the tare offset to the current average reading.
    void   tare(uint8_t times = 10);

    // Calibration scale factor: raw ADC counts per user unit (e.g. per lb).
    void   setScale(float scale);
    float  getScale()  const;

    // Tare offset in raw ADC counts.
    void   setOffset(long offset);
    long   getOffset() const;

    // Change channel / gain; issues one dummy read to apply the new setting.
    void   setGain(HX711Gain gain);

    // Power management (datasheet section 3.4)
    void   powerDown();   // Hold SCK HIGH for >60 µs to enter power-down
    void   powerUp();     // Pull SCK LOW; waits ~400 ms for first conversion

private:
    uint8_t _dout;
    uint8_t _sck;
    uint8_t _gainPulses;  // extra SCK pulses after 24 data bits (1, 2, or 3)
    long    _offset;
    float   _scale;

    // Shifts in 24 data bits then sends gain-selection pulses.
    // MUST be called with interrupts disabled.
    long    _shiftIn();
};

#endif // HX711_H
