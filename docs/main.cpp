
#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <stdexcept>
#include <climits>
#include <cstdint>

#ifndef SAMPLE_RATE
#define SAMPLE_RATE 8000
#endif

#ifndef ADC_MIDPOINT
#define ADC_MIDPOINT 128
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

float adc_midpoint = ADC_MIDPOINT;

/* ========================= RingBuffer ========================= */

template<typename T>
class RingBuffer {
public:
    explicit RingBuffer(size_t capacity)
        : _capacity(capacity), _buffer(capacity), _head(0), _count(0) {}

    void push(const T& value) {
        _buffer[_head] = value;
        _head = (_head + 1) % _capacity;

        if (_count < _capacity) {
            ++_count;
        }
    }

    size_t size() const { return _count; }
    size_t capacity() const { return _capacity; }

    T& operator[](size_t index) {
        if (index >= _count) {
            throw std::out_of_range("RingBuffer index out of range");
        }
        size_t realIndex = (_head + _capacity - _count + index) % _capacity;
        return _buffer[realIndex];
    }

    const T& operator[](size_t index) const {
        if (index >= _count) {
            throw std::out_of_range("RingBuffer index out of range");
        }
        size_t realIndex = (_head + _capacity - _count + index) % _capacity;
        return _buffer[realIndex];
    }

    void clear() {
        _head = 0;
        _count = 0;
    }

private:
    size_t _capacity;
    std::vector<T> _buffer;
    size_t _head;
    size_t _count;
};

/* ========================= Goertzel ========================= */

class Goertzel {
public:
    Goertzel(float freq, int rate = SAMPLE_RATE);
    float Mag(int samples[], int depth);

private:
    float COEF;
};

Goertzel::Goertzel(float freq, int rate) {
    COEF = 2.0f * std::cos((2.0f * M_PI * freq) / rate);
}

float Goertzel::Mag(int samples[], int depth) {
    float Q1 = 0, Q2 = 0;

    for (int n = 0; n < depth; ++n) {
        float Q0 = COEF * Q1 - Q2 + (samples[n] - adc_midpoint);
        Q2 = Q1;
        Q1 = Q0;
    }

    return std::sqrt(Q1 * Q1 + Q2 * Q2 - COEF * Q1 * Q2);
}

/* ========================= Utils ========================= */

bool bit_to_byte(uint8_t bit, uint8_t* out_byte) {
    static uint8_t shift_reg = 0;
    static uint8_t bit_count = 0;

    shift_reg = (shift_reg >> 1) | (bit ? 0x80 : 0x00);
    ++bit_count;

    if (bit_count == 8) {
        *out_byte = shift_reg;
        bit_count = 0;
        shift_reg = 0;
        return true;
    }
    return false;
}

uint32_t getBits(const std::vector<uint8_t>& data, uint32_t bitPos, uint32_t bitLen) {
    uint32_t value = 0;

    for (uint32_t i = 0; i < bitLen; ++i) {
        uint32_t byteIndex = (bitPos + i) / 8;
        uint32_t bitIndex  = (bitPos + i) % 8;

        uint8_t bit = (data[byteIndex] >> bitIndex) & 1;
        value |= (bit << i);
    }
    return value;
}

/* ========================= Parser ========================= */

void parse_data(std::vector<uint8_t> frame) {

    if (frame.size() != 16) return;

    if (frame[0] != 0x68 || frame[1] != 0x0A ||
        frame[2] != 0x0A || frame[3] != 0x68) return;

    uint8_t sum = 0;
    for (int i = 4; i < 14; ++i) sum += frame[i];

    if (frame[14] != sum) return;
    if (frame[15] != 0x16) return;

    constexpr size_t DATA_START = 7;
    constexpr size_t DATA_LEN   = 16;

    std::vector<uint8_t> data(
        frame.begin() + DATA_START,
        frame.begin() + DATA_START + DATA_LEN
    );

    uint32_t millis  = getBits(data, 0, 10);
    uint32_t sec     = getBits(data, 10, 6);
    uint32_t min     = getBits(data, 16, 6);
    uint32_t hour    = getBits(data, 24, 5);
    bool     dst     = getBits(data, 31, 1);
    uint32_t day     = getBits(data, 32, 5);
    uint32_t weekday = getBits(data, 37, 3);
    uint32_t month   = getBits(data, 40, 4);
    uint32_t year    = getBits(data, 48, 7);

    std::cout << "TIME: 20" << year
              << "-" << std::setw(2) << std::setfill('0') << month
              << "-" << std::setw(2) << std::setfill('0') << day << " "
              << std::setw(2) << std::setfill('0') << hour << ":"
              << std::setw(2) << std::setfill('0') << min << ":"
              << std::setw(2) << std::setfill('0') << sec
              << "." << std::setw(3) << std::setfill('0') << millis
              << " , DST: " << (dst ? "[igen]" : "[nem]")
              << " , Wday: " << weekday << "\n";
}

/* ========================= MAIN ========================= */

int main(int argc, char* argv[]) {

    std::string fileName = (argc > 1) ? argv[1] : "./hga22.raw";
    printf("fileName: %s\r\n", fileName.c_str());

    std::ifstream f1(fileName, std::ios::binary);
    if (!f1.is_open()) {
        std::cerr << "Nem sikerült megnyitni a bemeneti fájlt!\n";
        return 1;
    }

    f1.seekg(0, std::ios::end);
    std::streamsize length = f1.tellg();
    f1.seekg(0, std::ios::beg);

    std::vector<unsigned char> buffer(length);
    f1.read(reinterpret_cast<char*>(buffer.data()), length);
    f1.close();

    int sample_time_ms    = 4;
    int samples_offset_ms = 1;

    int samples        = sample_time_ms * (SAMPLE_RATE / 1000);
    int samples_offset = samples_offset_ms * (SAMPLE_RATE / 1000);

#define SAMPLES samples
#define OFFSET  samples_offset

    int Samples[SAMPLES];

    Goertzel ONE(1730, SAMPLE_RATE);
    Goertzel TWO(2070, SAMPLE_RATE);

    int DEPTH = sizeof(Samples) / sizeof(Samples[0]);

    RingBuffer<int> fsk_bit(5);
    RingBuffer<int> byte_buffer(16);

    int start = 0, send = 0, offset = OFFSET;
    int bit_cnt = 0;

    for (size_t i = 0; i < buffer.size() - SAMPLES; i += offset) {

        int min = INT_MAX, max = INT_MIN;

        for (int n = 0; n < SAMPLES; ++n) {
            Samples[n] = buffer[i + n];
            if (Samples[n] < min) min = Samples[n];
            if (Samples[n] > max) max = Samples[n];
        }

        adc_midpoint = (max + min) / 2;

        float mag1 = ONE.Mag(Samples, DEPTH);
        float mag2 = TWO.Mag(Samples, DEPTH);

        fsk_bit.push(mag2 > mag1 ? 0 : 1);

        if (fsk_bit.size() == 5 && start == 0) {
            int sum = 0;
            for (size_t i = 0; i < fsk_bit.size(); ++i) sum += fsk_bit[i];

            if (sum < 2) {
                start = 1;
                send = 5;
                bit_cnt = 0;
            }
        }

        if (start && ++send >= 5) {

            int sum = 0;
            for (size_t i = 0; i < fsk_bit.size(); ++i) sum += fsk_bit[i];

            send = 0;

            static int state = 0;
            static uint8_t byte = 0;
            static uint8_t parity = 0;

            uint8_t bit = (sum >= 3) ? 1 : 0;

            switch (state) {
                case 0:
                    if (bit == 0) {
                        state++;
                        byte = 0;
                    }
                    break;

                case 1:
                    if (bit_to_byte(bit, &byte)) state++;
                    break;

                case 2: {
#define BYTE_PARITY(b) (__builtin_parity((unsigned char)(b)))
                    parity = BYTE_PARITY(byte);

                    if (parity != bit) {
                        printf("parity error %X\r\n", parity);
                        offset = OFFSET;
                    }
                    state++;
                    break;
                }

                case 3:
                    if (bit != 1) {
                        printf(" stop error \r\n");
                        offset = OFFSET;
                        state = 255;
                    } else {
                        byte_buffer.push(byte);

                        std::vector<uint8_t> frame(16);
                        for (size_t i = 0; i < byte_buffer.size(); ++i)
                            frame[i] = byte_buffer[i];

                        parse_data(frame);
                        state = 255;
                    }
                    break;

                default:
                    break;
            }

            if (++bit_cnt >= 11) {
                start = 0;
                send = 0;
                bit_cnt = 0;

                state = 0;
                byte = 0;
                parity = 0;
            }
        }
    }

    return 0;
}

