#include "HX711.h"

// ---------------------------------------------------------------------------
HX711::HX711()
    : _dout(0), _sck(0), _gainPulses(1), _offset(0L), _scale(1.0f) {}

// ---------------------------------------------------------------------------
void HX711::begin(uint8_t dout, uint8_t sck, HX711Gain gain) {
    _dout = dout;
    _sck  = sck;

    pinMode(_dout, INPUT);
    pinMode(_sck,  OUTPUT);
    digitalWrite(_sck, LOW);

    // Programme gain register with one dummy conversion
    setGain(gain);
}

// ---------------------------------------------------------------------------
void HX711::setGain(HX711Gain gain) {
    _gainPulses = static_cast<uint8_t>(gain);
    readRaw();   // dummy read to clock the gain setting into the chip
}

// ---------------------------------------------------------------------------
bool HX711::isReady() const {
    return digitalRead(_dout) == LOW;
}

// ---------------------------------------------------------------------------
// _shiftIn — reads 24 data bits MSB-first, then sends gain-select pulses.
//
// Timing requirements (HX711 datasheet):
//   t1  SCK high time  ≥ 200 ns  (1 µs here, well within spec)
//   t2  SCK low  time  ≥ 200 ns  (1 µs here)
//   t3  DOUT valid after SCK rising edge: ≤ 100 ns — read on falling edge
//
// This function must be invoked inside a noInterrupts() block so that
// the ~60 µs bit-bang sequence cannot be interrupted and corrupted.
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
long HX711::readRaw() {
    // Wait for conversion to complete (DOUT goes LOW), with 1-second timeout
    const unsigned long deadline = millis() + 1000UL;
    while (!isReady()) {
        if (millis() > deadline) {
            return 0L;   // chip not responding — return neutral value
        }
        yield();   // service ESP8266 WiFi/background tasks while waiting
    }

    noInterrupts();           // protect the ~60 µs timing-critical section
    long value = _shiftIn();
    interrupts();

    return value;
}

// ---------------------------------------------------------------------------
long HX711::readAverage(uint8_t times) {
    long sum = 0;
    for (uint8_t i = 0; i < times; ++i) {
        sum += readRaw();
        yield();   // allow ESP8266 background processing between samples
    }
    return sum / static_cast<long>(times);
}

// ---------------------------------------------------------------------------
double HX711::getUnits(uint8_t times) {
    return static_cast<double>(readAverage(times) - _offset) / _scale;
}

// ---------------------------------------------------------------------------
void HX711::tare(uint8_t times) {
    _offset = readAverage(times);
}

// ---------------------------------------------------------------------------
void   HX711::setScale(float scale)  { _scale  = scale;  }
float  HX711::getScale()  const      { return _scale;    }
void   HX711::setOffset(long offset) { _offset = offset; }
long   HX711::getOffset() const      { return _offset;   }

// ---------------------------------------------------------------------------
// powerDown — pull SCK HIGH for >60 µs to enter low-power mode (<1 µA).
// ---------------------------------------------------------------------------
void HX711::powerDown() {
    digitalWrite(_sck, LOW);
    digitalWrite(_sck, HIGH);
    delayMicroseconds(65);   // datasheet minimum: 60 µs
}

// ---------------------------------------------------------------------------
// powerUp — pull SCK LOW; chip requires ~400 ms for first conversion.
// ---------------------------------------------------------------------------
void HX711::powerUp() {
    digitalWrite(_sck, LOW);
    delay(400);
}
