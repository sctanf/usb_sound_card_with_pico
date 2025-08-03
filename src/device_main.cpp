
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <array>
#include <hardware/pll.h>
#include <hardware/clocks.h>
#include <hardware/structs/systick.h>
#include <pico/multicore.h>
#include <tusb.h>
#include "usb_descriptors.h"
#include "device_config.h"
#include "streaming.h"

//--------------------------------------------------------------------+

#define DEVICE_LOG(...)   TU_LOG1("[DEVICE] " __VA_ARGS__);

// List of supported sample rates
constexpr uint32_t sample_rates[] = {48000, 96000};

// Resolution per format
constexpr uint8_t resolutions_per_format[2] = {16, 24};

// Channel per format
constexpr uint8_t channels_per_format[3] = {2, 4, 6};

// feedback descriptor bInterval value is supported only 1 on windows uac2 driver.
// the interval is measured manually in the sof callback.
constexpr uint16_t feedback_interval = 4000;

#define N_SAMPLE_RATES TU_ARRAY_SIZE(sample_rates)

std::array<uint32_t, UAC2_ENTITY_CLOCK_END - UAC2_ENTITY_CLOCK_START> g_current_sample_rates;
std::array<uint8_t, ITF_NUM_AUDIO_TOTAL> g_current_resolutions;
std::array<uint8_t, 3> g_current_channels;

void core1_loop();
void tud_update_job();
void audio_task();
void bootsel_task();
extern "C" __attribute__ ((weak)) void tusb_pico_reserve_buffer(uint8_t ep_adr, uint16_t size);

/*------------- MAIN -------------*/
int main(void)
{
    std::fill(g_current_sample_rates.begin(), g_current_sample_rates.end(), 48000);
    std::fill(g_current_resolutions.begin(), g_current_resolutions.end(), 16);
    std::fill(g_current_channels.begin(), g_current_channels.end(), 2);

    set_sys_clock_khz(240000, true);

    clock_configure(clk_sys,
                    CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                    CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                    48 * MHZ,
                    48 * MHZ);

    // Turn off PLL sys for good measure

    const uint32_t vco_freq = 1248 * MHZ;
    const uint32_t div1 = 5;
    const uint32_t div2 = 2;

    pll_deinit(pll_sys);
    pll_init(pll_sys, 1, vco_freq, div1, div2);

    const uint32_t frequency  = vco_freq/div1/div2;
    clock_configure(clk_sys,
                    CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                    CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
                    frequency,
                    frequency);

    set_sys_clock_khz(240000, true);

    tusb_init();

    if(tusb_pico_reserve_buffer)
    {
#if USB_IF_AUDIO_ENABLE
        tusb_pico_reserve_buffer(EP_AUDIO_CONTROL, 256);
        tusb_pico_reserve_buffer(EP_AUDIO_CONTROL | 0x80, 256);
        tusb_pico_reserve_buffer(EP_AUDIO_STREAM, CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ);
        tusb_pico_reserve_buffer(EP_AUDIO_STREAM_OUT_FB, 16);
        tusb_pico_reserve_buffer(EP_AUDIO_STREAM_OUT_FB | 0x80, 16);
#endif
#if USB_IF_CONTROL_ENABLE
        tusb_pico_reserve_buffer(EP_AUDIO_USER_CONTROL, 256);
        tusb_pico_reserve_buffer(EP_AUDIO_USER_CONTROL | 0x80, 256);
#endif
    }

#if USB_IF_AUDIO_ENABLE
    streaming::init();
#endif

    while (true)
    {
        tud_update_job();
        audio_task();
        bootsel_task();
    }

    multicore_launch_core1(core1_loop);

    return 0;
}

void core1_loop()
{
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
}

#if USB_IF_AUDIO_ENABLE

// Helper for clock get requests
static bool tud_audio_clock_get_request(uint8_t rhport, audio_control_request_t const *request)
{
    TU_ASSERT(IS_UNIT_TYPE(request->bEntityID, CLOCK));

    const int clock_index = request->bEntityID - UAC2_ENTITY_CLOCK_START;

    if (request->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ)
    {
        const auto sample_rate = g_current_sample_rates[clock_index];

        if (request->bRequest == AUDIO_CS_REQ_CUR)
        {
            DEVICE_LOG("Clock %d get current freq %u\n", clock_index, g_current_sample_rates[clock_index]);

            audio_control_cur_4_t curf = {(int32_t)tu_htole32(sample_rate)};
            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &curf, sizeof(curf));
        }
        else if (request->bRequest == AUDIO_CS_REQ_RANGE)
        {
            audio_control_range_4_n_t(N_SAMPLE_RATES) rangef;
            rangef.wNumSubRanges = tu_htole16(N_SAMPLE_RATES);
            DEVICE_LOG("Clock %d get %d freq ranges\n", clock_index, N_SAMPLE_RATES);
            for (uint8_t i = 0; i < N_SAMPLE_RATES; i++)
            {
                rangef.subrange[i].bMin = sample_rates[i];
                rangef.subrange[i].bMax = sample_rates[i];
                rangef.subrange[i].bRes = 0;
                DEVICE_LOG("Range %d (%d, %d, %d)\n", i, (int)rangef.subrange[i].bMin, (int)rangef.subrange[i].bMax, (int)rangef.subrange[i].bRes);
            }

            return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &rangef, sizeof(rangef));
        }
    }
    else if (request->bControlSelector == AUDIO_CS_CTRL_CLK_VALID &&
             request->bRequest == AUDIO_CS_REQ_CUR)
    {
        audio_control_cur_1_t cur_valid = {.bCur = 1};
        DEVICE_LOG("Clock %d get is valid %u\n", clock_index, cur_valid.bCur);
        return tud_audio_buffer_and_schedule_control_xfer(rhport, (tusb_control_request_t const *)request, &cur_valid, sizeof(cur_valid));
    }
    DEVICE_LOG("Clock %d get request not supported, entity = %u, selector = %u, request = %u\n",
            clock_index, request->bEntityID, request->bControlSelector, request->bRequest);
    return false;
}

// Helper for clock set requests
static bool tud_audio_clock_set_request(uint8_t rhport, audio_control_request_t const *request, uint8_t const *buf)
{
    (void)rhport;

    TU_ASSERT(IS_UNIT_TYPE(request->bEntityID, CLOCK));
    TU_VERIFY(request->bRequest == AUDIO_CS_REQ_CUR);

    const int clock_index = request->bEntityID - UAC2_ENTITY_CLOCK_START;

    if (request->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ)
    {
        TU_VERIFY(request->wLength == sizeof(audio_control_cur_4_t));

        g_current_sample_rates[clock_index] = (uint32_t)((audio_control_cur_4_t const *)buf)->bCur;

        DEVICE_LOG("Clock %d set current freq: %d\n", clock_index, g_current_sample_rates[clock_index]);

        return true;
    }
    else
    {
        DEVICE_LOG("Clock %d set request not supported, entity = %u, selector = %u, request = %u\n",
                clock_index, request->bEntityID, request->bControlSelector, request->bRequest);
        return false;
    }
}

//--------------------------------------------------------------------+
// Application Callback API Implementations
//--------------------------------------------------------------------+

// Invoked when audio class specific get request received for an entity
bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    audio_control_request_t const *request = (audio_control_request_t const *)p_request;

    if (IS_UNIT_TYPE(request->bEntityID, CLOCK))
        return tud_audio_clock_get_request(rhport, request);

    DEVICE_LOG("Get request not handled, entity = %d, selector = %d, request = %d\n",
            request->bEntityID, request->bControlSelector, request->bRequest);

    return false;
}

// Invoked when audio class specific set request received for an entity
bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *buf)
{
    audio_control_request_t const *request = (audio_control_request_t const *)p_request;

    if (IS_UNIT_TYPE(request->bEntityID, CLOCK))
        return tud_audio_clock_set_request(rhport, request, buf);

    DEVICE_LOG("Set request not handled, entity = %d, selector = %d, request = %d\n",
            request->bEntityID, request->bControlSelector, request->bRequest);

    return false;
}

bool tud_audio_set_itf_close_EP_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    (void)rhport;
    (void)p_request;

    uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
    uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

    DEVICE_LOG("Close interface itf %d alt %d\n", itf, alt);

    return true;
}

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    (void)rhport;

    uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
    uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

    DEVICE_LOG("Set interface %d alt %d\n", itf, alt);

    // Clear buffer when streaming format is changed
    // spk_data_size = 0;
    if(alt > 0)
    {
        g_current_resolutions[itf] = resolutions_per_format[(alt - 1)&1];
        g_current_channels[itf] = channels_per_format[((alt - 1) / 2) % 3];
    }

    switch(itf)
    {
        case ITF_NUM_AUDIO_STREAMING_HOST_TX:
            if(alt)
            {
                streaming::set_rx_format(
                    g_current_sample_rates[UAC2_ENTITY_USB_INPUT_CLOCK - UAC2_ENTITY_CLOCK_START], 
                    g_current_resolutions[ITF_NUM_AUDIO_STREAMING_HOST_TX],
                    g_current_channels[ITF_NUM_AUDIO_STREAMING_HOST_TX]);
            }
            else
            {
                streaming::close_rx();
            }
            break;
    }

    return true;
}

bool tud_audio_get_req_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    return false;
}

bool tud_audio_set_req_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff)
{
    return false;
}

bool tud_audio_get_req_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    return false;
}

bool tud_audio_set_req_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff)
{
    return false;
}

bool tud_audio_rx_done_pre_read_cb(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id, uint8_t ep_out, uint8_t cur_alt_setting)
{
    streaming::push_rx_data(
        +[](uint8_t *buffer, size_t size) -> size_t
        {
            return tud_audio_read(buffer, (uint16_t)size);
        },
        n_bytes_received);

    return true;
}

bool tud_audio_tx_done_pre_load_cb(uint8_t rhport, uint8_t itf, uint8_t ep_in, uint8_t cur_alt_setting)
{
    return false;
}

#endif

#if USB_IF_CONTROL_ENABLE

bool device_control_request(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request);

// Invoked when a control transfer occurred on an interface of this class
// Driver response accordingly to the request and the transfer stage (setup/data/ack)
// return false to stall control endpoint (e.g unsupported request)
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{
    // DEVICE_LOG("ctrl type:%x req:%x idx:%x val:%x len:%d\n", request->bmRequestType_bit.type, request->bRequest, request->wIndex, request->wValue, request->wLength);

    switch (request->bmRequestType_bit.type)
    {
        case TUSB_REQ_TYPE_VENDOR:
            switch (request->bRequest)
            {
                case VENDOR_REQUEST_MICROSOFT:
                    if (request->wIndex == 7)
                    {
                        if (stage == CONTROL_STAGE_SETUP)
                        {
                            // Get Microsoft OS 2.0 compatible descriptor
                            uint16_t total_len;
                            memcpy(&total_len, get_msos2_descriptor() + 8, 2);

                            return tud_control_xfer(rhport, request, (void *)(uintptr_t)get_msos2_descriptor(), total_len);
                        }
                    }
                    else
                    {
                        return false;
                    }
                    break;
            
                case VENDOR_REQUEST_CONTROLLER:
                    return device_control_request(rhport, stage, request);

                default:
                    return false;
            }
            break;        

        default: 
            return false;
    }

    return true;
}

bool device_control_request(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{
    return false;
}

#endif

//--------------------------------------------------------------------+
// AUDIO Task
//--------------------------------------------------------------------+

void tud_update_job(void)
{
    tud_task(); // tinyusb device task
}

#define I2S_TARGET_LEVEL_MIN_US    1500
#define I2S_TARGET_LEVEL_MAX_US    2000

void audio_task(void)
{
    //1ms間隔でフィードバック
    static uint32_t start_ms = 0;
    uint32_t curr_ms = to_ms_since_boot(get_absolute_time());
    if (curr_ms > start_ms)
    {
        // i dont know what this value is actually
        int32_t avail = streaming::get_samples();
        int32_t left = streaming::get_samples_left();

        // TODO: hardcoded
        uint32_t feedback = 48 << 16;
        uint32_t min_feedback = 47 << 16;
        uint32_t max_feedback = 49 << 16;

        if (avail < left)
            feedback = max_feedback;
        else if (avail > left)
            feedback = min_feedback;

        tud_audio_fb_set(feedback);
        start_ms = curr_ms;
    }
}

//--------------------------------------------------------------------+
// BOOTSEL TASK
//--------------------------------------------------------------------+

// https://github.com/jasongaunt/rp2040-bootsel-reboot-example/
#include "hardware/watchdog.h"
#include "bsp/board_api.h"

bool timer_interrupt(__unused struct repeating_timer *t)
{
    return true;
}

bool watchdog_enabled = false;
struct repeating_timer timer;

void bootsel_task(void)
{
    if (!watchdog_enabled)
    {
        watchdog_enable(500, 1); // enable watchdog now
        watchdog_enabled = true;
    }
    add_repeating_timer_ms(400, timer_interrupt, NULL, &timer);
    __wfi(); // if there are no irq, watchdog will also time out (ex. usb stopped receiving data or something?)
    watchdog_update();
    cancel_repeating_timer(&timer); // reset timer if something already interrupted in time
    // TODO: this is causing issues if the device is connected but no audio streaming to it, which is nice in some instances but very bad in others
    // maybe find another way
    if (board_button_read())
        while(1); // time out the watchdog
}
