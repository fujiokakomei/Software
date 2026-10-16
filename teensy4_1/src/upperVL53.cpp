#include "upperVL53.hpp"

namespace uppervl53s
{
void UpperVL53s::getVL53s(uint16_t _distance_data[NUM_SENSORS])
{
    if (_rp2040.available() >= EXPECTED_BYTES) // 新しいのがあったらとる
    {
        // バイト列をそのままuint16_t配列に読み込む
        _rp2040.readBytes((uint8_t*)distance_data, EXPECTED_BYTES);
    }
    //_distance_data = distance_data;
    memcpy(_distance_data, distance_data, EXPECTED_BYTES);
}
}