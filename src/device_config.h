#pragma once

#include <stdint.h>
#include "usb_config.h"

#define DAC_OUTPUT_ENABLE       1

constexpr uint8_t  device_output_channels = 2;
constexpr uint8_t  device_input_channels = 2;
constexpr uint8_t  max_resolution_bits = 24;
constexpr uint32_t max_sampling_frequency = 96000;
constexpr size_t   max_output_samples_1ms = max_sampling_frequency*device_output_channels/1000;
constexpr size_t   max_input_samples_1ms = max_sampling_frequency*device_input_channels/1000;


namespace gpio_assign
{
//    constexpr uint dac_data = 18;
//    constexpr uint dac_bck_lrck = 16;

    constexpr uint dac_data = 3;
    constexpr uint dac_bck_lrck = 8;

    constexpr uint dac_sck = 10;
    constexpr uint dac_mute = 0;
}
