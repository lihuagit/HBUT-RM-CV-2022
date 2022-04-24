/**
 * @file ExtImu.cpp
 * @brief 
 * @author Lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2022-04-17
 * 
 */
#include "ExtImu.hpp"

void ExtImu::stop_trigger() {
    int32_t ch = 0;
    Serial::write((uint8_t *) &ch, 4);
}

void ExtImu::once_trigger(int32_t delay_ms) {
    if (delay_ms < 0) throw std::logic_error("ExtImu::once_trigger() assert failed: 'delay_ms >= 0'");
    delay_ms = -(delay_ms + 1);
    Serial::write((uint8_t *) &delay_ms, 4);
}

void ExtImu::periodic_trigger(int32_t period_ms) {
    if (period_ms <= 0) throw std::logic_error("ExtImu::periodic_trigger() assert failed: 'period_ms > 0'");
    Serial::write((uint8_t *) &period_ms, 4);
}

void ExtImu::read_sensor(sensor_data *p_data) {
    Serial::read((uint8_t *) p_data, sizeof(sensor_data));
}
