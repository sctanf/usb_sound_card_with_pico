
#include <array>
#include <memory>
#include <algorithm>
#include <hardware/timer.h>
#include "tusb.h"
#include "device_config.h"

#if USB_IF_AUDIO_ENABLE

#include "support.h"
#include "circular_buffer.h"
#include "streaming.h"
#include "streaming_dac_out.h"

namespace streaming
{
    using namespace data_structure;

    enum PIO0_SM_TYPE
    {
        PIO0_SM_SPDIF_OUT,
        PIO0_SM_DAC_OUT,
        PIO0_SM_ADC_CLK,
    };
    enum PIO1_SM_TYPE
    {
        PIO1_SM_ADC_IN,
        PIO1_SM_SPDIF_IN,
        PIO1_SM_SPDIF_IN_RAW,
    };

    template <typename T>
    PIO get_sm_pio(T);
    template <>
    inline PIO get_sm_pio(PIO0_SM_TYPE) { return pio0; }
    template <>
    inline PIO get_sm_pio(PIO1_SM_TYPE) { return pio1; }
    template <typename T>
    uint8_t get_sm_pio_index(T);
    template <>
    inline uint8_t get_sm_pio_index(PIO0_SM_TYPE) { return 0; }
    template <>
    inline uint8_t get_sm_pio_index(PIO1_SM_TYPE) { return 1; }

    static constexpr uint16_t device_buffer_duration = 8;
    static constexpr uint16_t output_mixing_processing_buffer_duration_per_cycle = device_buffer_duration / 4;

    static uint32_t g_output_sampling_frequency = 0;
    static uint8_t g_output_resolution_bits = 0;
    static uint8_t g_device_output_channels = 0;

    static dac_out g_dac_out;
    static dac_out::buffer<device_buffer_duration> g_dac_out_buffer;

    void set_rx_format(uint32_t sampling_frequency, uint32_t bits, uint8_t channels)
    {
        if (g_output_sampling_frequency == sampling_frequency && g_output_resolution_bits == bits && g_device_output_channels == channels)
        {
            return;
        }

        g_dac_out.stop();

        g_output_sampling_frequency = sampling_frequency;
        g_output_resolution_bits = bits;
        g_device_output_channels = channels;

        g_dac_out.set_format(sampling_frequency, bits, channels);
    }

    void close_rx()
    {
        g_dac_out.stop();
    }

    void push_rx_data(size_t (*fn)(uint8_t *, size_t), size_t data_size)
    {
        static std::array<uint8_t, max_output_samples_1ms * output_mixing_processing_buffer_duration_per_cycle * sizeof(uint32_t)> data_tmp_buf;
        fn(data_tmp_buf.begin(), data_size);
        const auto fetch_samples = data_size / bits_to_bytes(g_output_resolution_bits);

        if (g_dac_out.is_running())
        {
            if (fetch_samples > g_dac_out.get_buffer_left_count())
            {
                return;
            }
        }

        g_dac_out.write(data_tmp_buf.begin(), data_tmp_buf.begin() + data_size);

        if (!g_dac_out.is_running())
            g_dac_out.start();
    }

    static void init_system()
    {
        auto i2s_out_program_offset = (uint8_t)pio_add_program(get_sm_pio(PIO0_SM_DAC_OUT), &audio_i2s_32_out_program);
        auto adc_clk_program_offset = (uint8_t)pio_add_program(get_sm_pio(PIO0_SM_DAC_OUT), &pulse_out_program);
        decltype(g_dac_out)::init_config dac_out_config = {
            .buffer_begin = g_dac_out_buffer.begin(),
            .buffer_end = g_dac_out_buffer.end(),

            .i2s_out_pio_program_offset = i2s_out_program_offset,
            .i2s_out_pio = get_sm_pio_index(PIO0_SM_DAC_OUT),
            .i2s_out_sm = 0,
            .i2s_out_data_pin = gpio_assign::dac_data,
            .i2s_out_bck_lrck_pin = gpio_assign::dac_bck_lrck,

            .clk_pio_program_offset = adc_clk_program_offset,
            .clk_pio = get_sm_pio_index(PIO0_SM_DAC_OUT),
            .clk_sm = 1,
            .i2s_in_sck_pin = gpio_assign::dac_sck,

            .dac_mute_pin = gpio_assign::dac_mute,
            .dma_irq_n = 0};
        g_dac_out.init(dac_out_config);

        set_rx_format(48000, 16, 2);

    }

    void init()
    {
        init_system();
    }

    uint32_t get_samples()
    {
        return g_dac_out.get_buffer_available_samples();
    }

    uint32_t get_samples_left()
    {
        return g_dac_out.get_buffer_left_count();
    }
}

#endif
