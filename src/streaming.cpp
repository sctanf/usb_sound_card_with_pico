
#include <array>
#include <memory>
#include <algorithm>
#include <hardware/timer.h>
#include "tusb.h"
#include "device_config.h"

#if USB_IF_AUDIO_ENABLE

#include "debug.h"
#include "profiler.h"
#include "profile_measurement_list.h"
#include "support.h"
#include "circular_buffer.h"
#include "converter.h"
#include "mixer.h"
#include "streaming.h"
#include "streaming_internal.h"
#include "streaming_adc_in.h"
#include "streaming_spdif_in.h"
#include "streaming_spdif_out.h"
#include "streaming_dac_out.h"

#define STREAM_LOG(...) TU_LOG1("[STREAM] " __VA_ARGS__)

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

    struct job_mix_out_info : public job_queue::work_fn
    {
        uint8_t sample_bytes;
        size_t buffer_size;
    };
    struct job_mix_out_io_info : public job_queue::work_fn
    {
        uint8_t *data_begin;
        uint8_t *data_end;
        uint32_t require_samples;
        uint32_t result_size;
    };
    struct job_mix_in_info : public job_queue::work_fn
    {
        uint64_t timeout;
        uint32_t require_samples;
        uint32_t buffer_size;
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

    static constexpr uint32_t task_priority_default = 16;

    static constexpr uint16_t device_buffer_duration = 8;
    static constexpr uint16_t output_mixing_processing_buffer_duration_per_cycle = device_buffer_duration / 4;

    static circular_buffer<container_array<uint8_t, max_output_samples_1ms * device_buffer_duration * sizeof(uint32_t)>> g_rx_stream_buffer;
    static uint8_t *g_rx_stream_buffer_write_addr;
    static const uint8_t *g_rx_stream_buffer_read_addr;

    static uint32_t g_output_sampling_frequency = 0;
    static uint8_t g_output_resolution_bits = 0;
    static processing::mixer g_output_mixer;
    static uint8_t g_output_mixer_rx_volume = 0xff;
    static uint8_t g_output_mixer_mixed_input_volume = 0xff;
    static bool g_output_process_task_active;
    static uint8_t g_output_device_charge_count = 0;

    static job_mix_out_info g_job_mix_out = {};
#if DAC_OUTPUT_ENABLE
    static job_mix_out_io_info g_job_mix_out_dac = {};
#endif

#if DAC_OUTPUT_ENABLE
    static dac_out g_dac_out;
    static dac_out::buffer<device_buffer_duration> g_dac_out_buffer;
#endif

#if USB_IF_CONTROL_ENABLE
    struct debug_stats
    {
        uint32_t received_bytes;
        uint32_t transfar_bytes;
        struct
        {
            uint32_t src_left;
            uint32_t input_left;
            uint32_t processed_bytes;
        } outmix;
        struct
        {
            uint32_t processed_bytes;
            uint32_t adc_in_samples;
            uint32_t spdif_in_samples;
        } inmix;
    };
    debug_stats g_debug_stats;
#endif

    void update_output_mixer()
    {
        if (g_output_resolution_bits == 0)
        {
            return;
        }

        processing::mixer::config mixer_config = {
            .bits = g_output_resolution_bits,
            .stride = bits_to_bytes(g_output_resolution_bits),
            .channels = device_output_channels,
            .use_interp = true};
        g_output_mixer.setup(mixer_config);
    }



    static void start_output_process_job();
    static void stop_output_process_job();

    void set_rx_format(uint32_t sampling_frequency, uint32_t bits)
    {
        g_output_device_charge_count = (device_buffer_duration / output_mixing_processing_buffer_duration_per_cycle) >> 1;

        if (g_output_sampling_frequency == sampling_frequency && g_output_resolution_bits == bits)
        {
            return;
        }
        STREAM_LOG("set rx format %u %u\n", sampling_frequency, bits);

        stop_output_process_job();

#if DAC_OUTPUT_ENABLE
        g_dac_out.stop();
#endif

        g_output_sampling_frequency = sampling_frequency;
        g_output_resolution_bits = bits;

        g_rx_stream_buffer.resize(get_samples_duration_ms(device_buffer_duration, g_output_sampling_frequency, device_output_channels) * bits_to_bytes(g_output_resolution_bits));
        g_rx_stream_buffer_write_addr = g_rx_stream_buffer.begin();
        g_rx_stream_buffer_read_addr = g_rx_stream_buffer.begin();

#if DAC_OUTPUT_ENABLE
        g_dac_out.set_format(sampling_frequency, bits);
#endif
        update_output_mixer();

        start_output_process_job();
    }

    void close_rx()
    {
#if DAC_OUTPUT_ENABLE
        g_dac_out.stop();
#endif
    }

    void get_rx_buffer_size(uint32_t &left, uint32_t &max_size)
    {
        left = g_rx_stream_buffer.distance(g_rx_stream_buffer_write_addr, g_rx_stream_buffer_read_addr);
        max_size = g_rx_stream_buffer.size();
    }

    void push_rx_data(size_t (*fn)(uint8_t *, size_t), size_t data_size)
    {
        auto write_addr = g_rx_stream_buffer_write_addr;
        auto data_size_saved = data_size;

        while (data_size)
        {
            size_t sz = std::min(data_size, (size_t)(g_rx_stream_buffer.end() - g_rx_stream_buffer_write_addr));
            fn(g_rx_stream_buffer_write_addr, sz);

            data_size -= sz;
            g_rx_stream_buffer_write_addr += sz;
            if (g_rx_stream_buffer_write_addr >= g_rx_stream_buffer.end())
                g_rx_stream_buffer_write_addr = g_rx_stream_buffer.begin();
        }

#if USB_IF_CONTROL_ENABLE
        g_debug_stats.received_bytes += g_rx_stream_buffer.distance(g_rx_stream_buffer_write_addr, write_addr);
#endif

        g_job_mix_out.set_pending();
    }

    static void job_mix_output_init(job_queue::work *);
    static void job_mix_output_process(job_queue::work *);
    static void job_mix_output_dac_write(job_queue::work *);
    static void job_mix_output_spdif_write(job_queue::work *);

    static void start_output_process_job()
    {
        STREAM_LOG("start output process job\n");

#if DAC_OUTPUT_ENABLE
        g_job_mix_out_dac.set_callback(job_mix_output_dac_write);
        g_job_mix_out_dac.activate();
#endif
        g_job_mix_out.set_callback(job_mix_output_init);
        g_job_mix_out.activate();
        g_job_mix_out.set_pending();
    }
    static void stop_output_process_job()
    {
        STREAM_LOG("stop output process job\n");

        g_job_mix_out.deactivate();
        g_job_mix_out.wait_done();
#if DAC_OUTPUT_ENABLE
        g_job_mix_out_dac.deactivate();
        g_job_mix_out_dac.wait_done();
#endif
    }

    static void job_mix_output_init(job_queue::work *)
    {
        JOB_TRACE_LOG("job_mix_output_init\n");

        g_job_mix_out.sample_bytes = bits_to_bytes(g_output_resolution_bits);
        g_job_mix_out.buffer_size = get_samples_duration_ms(output_mixing_processing_buffer_duration_per_cycle, g_output_sampling_frequency, device_output_channels) * g_job_mix_out.sample_bytes;
        g_job_mix_out.set_callback(job_mix_output_process);
        g_job_mix_out.set_pending();
    }

    static void job_mix_output_process(job_queue::work *)
    {
        JOB_TRACE_LOG("job_mix_output_process\n");

        static std::array<uint8_t, max_output_samples_1ms * output_mixing_processing_buffer_duration_per_cycle * sizeof(uint32_t)> data_tmp_buf;
        static std::array<uint8_t, max_output_samples_1ms * output_mixing_processing_buffer_duration_per_cycle * sizeof(uint32_t)> mix_tmp_buf;

        const uint8_t output_sample_bytes = g_job_mix_out.sample_bytes;
        const size_t buffer_size = g_job_mix_out.buffer_size;

        bool is_idle_write_job = true;
#if DAC_OUTPUT_ENABLE
        is_idle_write_job &= g_job_mix_out_dac.is_idle();
#endif

        if (!is_idle_write_job)
        {
            g_job_mix_out.set_pending_delay_us(100);
            return;
        }

        if (g_rx_stream_buffer.distance(g_rx_stream_buffer_write_addr, g_rx_stream_buffer_read_addr) < buffer_size)
        {
            g_job_mix_out.set_pending_delay_us(200);
            return;
        }

#if USB_IF_CONTROL_ENABLE
        g_debug_stats.outmix.src_left = g_rx_stream_buffer.distance(g_rx_stream_buffer_write_addr, g_rx_stream_buffer_read_addr);
#endif

        PROFILE_MEASURE_BEGIN(PROF_MIXOUT_USBDATA);
        
        const auto rx_stream_buffer_write_addr = g_rx_stream_buffer_write_addr;
        auto read_addr = g_rx_stream_buffer_read_addr;

        size_t fetch_bytes = buffer_size;
        g_rx_stream_buffer_read_addr =
            g_rx_stream_buffer.copy_to(rx_stream_buffer_write_addr, g_rx_stream_buffer_read_addr, data_tmp_buf.begin(), fetch_bytes);

        g_output_mixer.apply(
            g_output_mixer_rx_volume,
            data_tmp_buf.begin(), data_tmp_buf.begin() + fetch_bytes,
            mix_tmp_buf.begin(), mix_tmp_buf.begin() + fetch_bytes, true);

        PROFILE_MEASURE_END();


        const auto fetch_samples = fetch_bytes / output_sample_bytes;
#if DAC_OUTPUT_ENABLE
        g_job_mix_out_dac.require_samples = fetch_samples;
        g_job_mix_out_dac.data_begin = mix_tmp_buf.begin();
        g_job_mix_out_dac.data_end = mix_tmp_buf.begin() + fetch_bytes;
        g_job_mix_out_dac.set_pending();
#endif
        if (g_output_device_charge_count)
            --g_output_device_charge_count;

        g_job_mix_out.set_pending_delay_us(100);
    }

#if DAC_OUTPUT_ENABLE
    static void job_mix_output_dac_write(job_queue::work *)
    {
        JOB_TRACE_LOG("job_mix_output_dac_write\n");

        auto &job = g_job_mix_out_dac;
        if (g_dac_out.is_running())
        {
            if (job.require_samples > g_dac_out.get_buffer_left_count())
            {
                job.set_pending_delay_us(200);
                return;
            }
        }

        PROFILE_MEASURE_BEGIN(PROF_MIXOUT_DAC_WRITE);
        job.result_size = g_dac_out.write(job.data_begin, job.data_end);
        PROFILE_MEASURE_END();

        if (g_output_device_charge_count == 0 && !g_dac_out.is_running())
            g_dac_out.start();
    }
#endif

    static void dma0_irq_handler()
    {
        dbg_assert(get_core_num() == 0);

#if DAC_OUTPUT_ENABLE
        g_dac_out.on_dma_isr();
#endif
    }

    static void init_system()
    {
        STREAM_LOG("system initialize\n");

#if DAC_OUTPUT_ENABLE
        auto i2s_out_program_offset = (uint8_t)pio_add_program(get_sm_pio(PIO0_SM_DAC_OUT), &audio_i2s_32_out_program);
        decltype(g_dac_out)::init_config dac_out_config = {
            .buffer_begin = g_dac_out_buffer.begin(),
            .buffer_end = g_dac_out_buffer.end(),

            .i2s_out_pio_program_offset = i2s_out_program_offset,
            .i2s_out_pio = get_sm_pio_index(PIO0_SM_DAC_OUT),
            .i2s_out_sm = PIO0_SM_DAC_OUT,
            .i2s_out_data_pin = gpio_assign::dac_data,
            .i2s_out_bck_lrck_pin = gpio_assign::dac_bck_lrck,

            .dma_irq_n = 0};
        g_dac_out.init(dac_out_config);
#endif

        set_rx_format(48000, 16);

    }

    void init()
    {
        STREAM_LOG("initialize\n");

        const uint8_t core0mask = (1 << 0);
        const uint8_t core1mask = (1 << 1);
        const uint8_t core_both_mask = core0mask | core1mask;

        g_job_mix_out.set_affinity_mask(core_both_mask);

#if DAC_OUTPUT_ENABLE
        g_job_mix_out_dac.set_affinity_mask(core_both_mask);
#endif

        init_system();
    }

#if PRINT_STATS
    void print_debug_stats()
    {
        dbg_printf(
            "  rx:\n"
            "    bytes: %u\n",
            g_debug_stats.received_bytes);
        dbg_printf(
            "  tx:\n"
            "    bytes: %u\n",
            g_debug_stats.transfar_bytes);
        dbg_printf(
            "  outmix:\n"
            "    src left: %u/%u\n"
            "    input left: %u/%u\n"
            "    processed bytes: %u\n",
            g_debug_stats.outmix.src_left, g_rx_stream_buffer.size(), g_debug_stats.outmix.input_left, g_input_mixing_buffer.size(), g_debug_stats.outmix.processed_bytes);

        dbg_printf(
            "  inmix:\n"
            "    adc in left: %u\n"
            "    spdif in left: %u\n"
            "    processed bytes: %u\n",
            g_debug_stats.inmix.adc_in_samples, g_debug_stats.inmix.spdif_in_samples, g_debug_stats.inmix.processed_bytes);

        g_debug_stats = {};
    }
#endif

}

#endif
