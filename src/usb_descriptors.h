#pragma once

#include "usb_config.h"

#ifdef __cplusplus
extern "C"{
#endif

// Control unit identifer
enum
{
    UAC2_ENTITY_UNDEFINED,

    UAC2_ENTITY_CLOCK_START,
    UAC2_ENTITY_USB_INPUT_CLOCK = UAC2_ENTITY_CLOCK_START,
    UAC2_ENTITY_LINEIN_CLOCK,
    UAC2_ENTITY_CLOCK_END,

    UAC2_ENTITY_TERMINAL_START,
    UAC2_ENTITY_USB_INPUT_TERMINAL = UAC2_ENTITY_TERMINAL_START,
    UAC2_ENTITY_SPEAKER_OUTPUT_TERMINAL,
    UAC2_ENTITY_LINEIN_INPUT_TERMINAL,
    UAC2_ENTITY_LINEIN_OUTPUT_TERMINAL,
    UAC2_ENTITY_TERMINAL_END
};
#define AUDIO_TERM_TYPE_EXTERNAL_LINE 0x0603
#define AUDIO_TERM_TYPE_EXTERNAL_SPDIF 0x0605

#define IS_UNIT_TYPE(val, type) ((val) >= (UAC2_ENTITY_##type##_START) && (val) < (UAC2_ENTITY_##type##_END))

enum
{
    VENDOR_REQUEST_CONTROLLER = 1,
    VENDOR_REQUEST_MICROSOFT
};

#if USB_IF_CONTROL_ENABLE
extern const uint8_t* get_msos2_descriptor();
#endif

enum
{
#if USB_IF_AUDIO_ENABLE
    ITF_NUM_AUDIO_CONTROL,
    ITF_NUM_AUDIO_STREAMING_HOST_TX,
    ITF_NUM_AUDIO_STREAMING_HOST_RX,
#endif
#if USB_IF_CONTROL_ENABLE
    ITF_NUM_FN_CONTROL,
#endif
#if USB_IF_DEBUG_CDC_ENABLE
    ITF_NUM_CDC_DEBUG,
    ITF_NUM_CDC_DEBUG_DATA,
#endif
    ITF_NUM_TOTAL
};

#define ITF_NUM_AUDIO_TOTAL (USB_IF_AUDIO_ENABLE ? 3 : 0)
#define ITF_NUM_FN_TOTAL (USB_IF_CONTROL_ENABLE ? 1 : 0)
#define ITF_NUM_DEBUG_TOTAL (USB_IF_DEBUG_CDC_ENABLE ? 2 : 0)

static_assert(ITF_NUM_FN_TOTAL + ITF_NUM_AUDIO_TOTAL + ITF_NUM_DEBUG_TOTAL == ITF_NUM_TOTAL);

enum 
{
    EP_AUDIO_CONTROL,
    EP_AUDIO_STREAM,
    EP_AUDIO_STREAM_OUT_FB,
    EP_AUDIO_USER_CONTROL,
    EP_DEBUG_CDC_NOTIFY,
    EP_DEBUG_CDC_DATA,
};

#ifdef __cplusplus
}
#endif