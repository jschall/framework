#include <modules/can/can.h>
#include <modules/worker_thread/worker_thread.h>
#include <modules/timing/timing.h>
#include <common/helpers.h>
#include <common/ctor.h>
#include "usbcfg.h"
#include <hal.h>
#include <ch.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#ifdef MODULE_UAVCAN_DEBUG_ENABLED
#include <modules/uavcan_debug/uavcan_debug.h>
#endif

#ifndef USB_SLCAN_WORKER_THREAD
#error Please define USB_SLCAN_WORKER_THREAD in framework_conf.h.
#endif

// #define SLCAN_DIRECT_LOOPBACK_DEBUG

#define WT USB_SLCAN_WORKER_THREAD
WORKER_THREAD_DECLARE_EXTERN(WT)

static struct worker_thread_listener_task_s can_rx_listener_task;
static void can_rx_listener_task_func(size_t msg_size, const void* buf, void* ctx);

static THD_WORKING_AREA(slcan_rx_wa, 512);
static THD_FUNCTION(slcan_rx_thread, arg);

static struct worker_thread_timer_task_s usb_connect_timer_task;
static void usb_connect_timer_task_func(struct worker_thread_timer_task_s* task);


struct slcan_instance_s {
    struct can_instance_s* can_instance;
    bool started;
    bool timestamp_enable;
    bool flags_enable;
    bool ignore_next_frame;
    bool loopback_enable;

    char cmd_buf[28];
    size_t cmd_buf_len;
};

RUN_AFTER(CAN_INIT) {
    setup_usb_strings();
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);
    usbDisconnectBus(serusbcfg.usbp);

    static struct slcan_instance_s instance;

    instance.can_instance = can_get_instance(0);
    instance.timestamp_enable = true;
    instance.flags_enable = false;
    instance.loopback_enable = false;

    worker_thread_add_timer_task(&WT, &usb_connect_timer_task, usb_connect_timer_task_func, &instance, chTimeS2I(1), false);
    chThdCreateStatic(slcan_rx_wa, sizeof(slcan_rx_wa), WT.priority, slcan_rx_thread, &instance);
    worker_thread_add_listener_task(&WT, &can_rx_listener_task, can_get_rx_topic(instance.can_instance), can_rx_listener_task_func, &instance);
}

static void usb_connect_timer_task_func(struct worker_thread_timer_task_s* task) {
    (void)task;
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);
}


static uint8_t hex_to_nibble(char c) {
    const char* hex_chars = "0123456789ABCDEF";
    const char* chrptr = strchr(hex_chars, ascii_toupper(c));
    if (!chrptr) {
        return 255;
    }
    return chrptr-hex_chars;
}

static size_t can_frame_to_slcan(struct slcan_instance_s* instance, const struct can_frame_s* frame, char* slcan_frame, size_t max_len, bool loopback) {
    const char *hex = "0123456789ABCDEF";

    if (max_len < 64) {
        return 0; // Insufficient buffer
    }

    size_t slcan_frame_len = 1;

    if (frame->RTR) {
        slcan_frame[0] = 'r';
    } else {
        slcan_frame[0] = 't';
    }

    if (frame->IDE) {
        slcan_frame[0] = ascii_toupper(slcan_frame[0]);
        for (uint8_t i=0; i<8; i++) {
            slcan_frame[slcan_frame_len++] = hex[(frame->EID >> ((7-i)*4))&0xf];
        }
    } else {
        for (uint8_t i=0; i<3; i++) {
            slcan_frame[slcan_frame_len++] = hex[(frame->SID >> ((2-i)*4))&0xf];
        }
    }

    slcan_frame[slcan_frame_len++] = hex[frame->DLC];

    if (!frame->RTR) {
        for (uint8_t i=0; i<frame->DLC; i++) {
            slcan_frame[slcan_frame_len++] = hex[(frame->data[i]>>4)&0xf];
            slcan_frame[slcan_frame_len++] = hex[frame->data[i]&0xf];
        }
    }

    if (instance->timestamp_enable) {
        // TODO use rx timestamp
        uint32_t millis_mod_60k = millis() % 60000;
        for (uint8_t i=0; i<4; i++) {
            slcan_frame[slcan_frame_len++] = hex[(millis_mod_60k>>((3-i)*4))&0xf];
        }
    }

    if (instance->flags_enable && loopback) {
        slcan_frame[slcan_frame_len++] = 'L';
    }

    slcan_frame[slcan_frame_len++] = '\r';

    return slcan_frame_len;
}

static void process_slcan_cmd(struct slcan_instance_s* instance, size_t cmd_len) {
// #ifdef MODULE_UAVCAN_DEBUG_ENABLED
//     uavcan_send_debug_msg(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG, "SLCAN", "PROC: %c cmd, len=%u", instance->cmd_buf[0], (unsigned)cmd_len);
// #endif

    // Unsupported commands that are just ACKed
    switch(instance->cmd_buf[0]) {
        case 'C': // Close CAN channel
            can_set_filtering_enabled(instance->can_instance, true);
            // fall through
        case 'S': // Set bitrate
        case 'M':
        case 'm': {
            chnWriteTimeout(&SDU1, (uint8_t*)"\r", 1, TIME_IMMEDIATE);
            return;
        }
    }
    // Other relevant commands that are supported by Zubax Babel and we ignore (but might be supported later):
    // 'Z' - enable or disable timestamping
    // 'F' - get and clear status flags

    switch(instance->cmd_buf[0]) {
        case 'L': { // Open CAN channel in silent mode
            instance->loopback_enable = false;
            can_set_filtering_enabled(instance->can_instance, false);
            chnWriteTimeout(&SDU1, (uint8_t*)"\r", 1, TIME_IMMEDIATE);
            return;
        }
        case 'O': { // Open CAN channel in normal mode
            instance->loopback_enable = false;
            can_set_filtering_enabled(instance->can_instance, false);
            chnWriteTimeout(&SDU1, (uint8_t*)"\r", 1, TIME_IMMEDIATE);
            return;
        }
        case 'l': {
            instance->loopback_enable = true;
            can_set_filtering_enabled(instance->can_instance, false);
            chnWriteTimeout(&SDU1, (uint8_t*)"\r", 1, TIME_IMMEDIATE);
            return;
        }
    }

    if (instance->cmd_buf[0] == 'T' || instance->cmd_buf[0] == 't' || instance->cmd_buf[0] == 'R' || instance->cmd_buf[0] == 'r') {
        // transmit command
        struct can_frame_s frame;

        frame.RTR = instance->cmd_buf[0] == 'R' || instance->cmd_buf[0] == 'r';
        frame.IDE = instance->cmd_buf[0] == 'T' || instance->cmd_buf[0] == 'R';

        size_t data_begin_idx = frame.IDE ? 10 : 5;

        // check if cmd is long enough to contain DLC
        // note: cmd_len does not include \r
        if (cmd_len < data_begin_idx || (frame.IDE && cmd_len < data_begin_idx)) {
            chnWriteTimeout(&SDU1, (uint8_t*)"\a", 1, TIME_IMMEDIATE);
            return;
        }

        // fill in DLC field
        if (frame.IDE) {
            uint8_t DLC = hex_to_nibble(instance->cmd_buf[9]);
            if (DLC > 8) {
                chnWriteTimeout(&SDU1, (uint8_t*)"\a", 1, TIME_IMMEDIATE);
                return;
            }
            frame.DLC = DLC;
        } else {
            uint8_t DLC = hex_to_nibble(instance->cmd_buf[4]);
            if (DLC > 8) {
                chnWriteTimeout(&SDU1, (uint8_t*)"\a", 1, TIME_IMMEDIATE);
                return;
            }
            frame.DLC = DLC;
        }

        // check that cmd is the correct length
        if (cmd_len != data_begin_idx + (frame.RTR ? 0 : frame.DLC*2)) {
            chnWriteTimeout(&SDU1, (uint8_t*)"\a", 1, TIME_IMMEDIATE);
            return;
        }

        // Fill in ID field
        if (frame.IDE) {
            uint32_t EID = 0;
            for (size_t i=0; i<8; i++) {
                uint8_t nib = hex_to_nibble(instance->cmd_buf[i+1]);
                if (nib == 255) {
                    chnWriteTimeout(&SDU1, (uint8_t*)"\a", 1, TIME_IMMEDIATE);
                    return;
                }
                EID |= nib << ((7-i)*4);
            }

            if (EID >= 1<<29) {
                chnWriteTimeout(&SDU1, (uint8_t*)"\a", 1, TIME_IMMEDIATE);
                return;
            }

            frame.EID = EID;
        } else {
            uint32_t SID = 0;
            for (size_t i=0; i<3; i++) {
                uint8_t nib = hex_to_nibble(instance->cmd_buf[i+1]);
                if (nib == 255) {
                    chnWriteTimeout(&SDU1, (uint8_t*)"\a", 1, TIME_IMMEDIATE);
                    return;
                }
                SID |= nib << ((2-i)*4);
            }

            if (SID >= 1<<11) {
                chnWriteTimeout(&SDU1, (uint8_t*)"\a", 1, TIME_IMMEDIATE);
                return;
            }

            frame.SID = SID;
        }

        // Fill in data field
        if (!frame.RTR) {
            for (uint8_t i=0; i<frame.DLC; i++) {
                uint8_t msn = hex_to_nibble(instance->cmd_buf[data_begin_idx+i*2]);
                uint8_t lsn = hex_to_nibble(instance->cmd_buf[data_begin_idx+i*2+1]);
                if (msn == 255 || lsn == 255) {
                    chnWriteTimeout(&SDU1, (uint8_t*)"\a", 1, TIME_IMMEDIATE);
                    return;
                }

                frame.data[i] = msn<<4|lsn;
            }
        }

        can_bridge_transmit(instance->can_instance, &frame);

        if (frame.IDE) {
            chnWriteTimeout(&SDU1, (uint8_t*)"Z\r", 2, TIME_IMMEDIATE);
        } else {
            chnWriteTimeout(&SDU1, (uint8_t*)"z\r", 2, TIME_IMMEDIATE);
        }

#ifdef SLCAN_DIRECT_LOOPBACK_DEBUG
        // Echo the processed frame back to the host
        char slcan_frame[64];
        size_t slcan_frame_len = can_frame_to_slcan(instance, &frame, slcan_frame, sizeof(slcan_frame), true);

        chnWriteTimeout(&SDU1, (uint8_t*)slcan_frame, slcan_frame_len, TIME_IMMEDIATE);
#endif
    }


}

static THD_FUNCTION(slcan_rx_thread, arg) {
    struct slcan_instance_s* instance = arg;
    chRegSetThreadName("slcan_rx");

    while (true) {
        if (USBD1.state != USB_ACTIVE) {
            instance->cmd_buf_len = 0;
            chThdSleepMilliseconds(50);
            continue;
        }

        size_t space = sizeof(instance->cmd_buf) - instance->cmd_buf_len;
        if (space == 0) {
            instance->cmd_buf_len = 0;
            space = sizeof(instance->cmd_buf);
        }

        size_t n = chnReadTimeout(&SDU1, (uint8_t*)&instance->cmd_buf[instance->cmd_buf_len], space, TIME_MS2I(1));
        instance->cmd_buf_len += n;

        // Process all complete commands in the buffer
        while (instance->cmd_buf_len > 0) {
            size_t cmd_len = 0;
            for (size_t i=0; i<instance->cmd_buf_len; i++) {
                if (instance->cmd_buf[i] == '\r') {
                    cmd_len = i+1;
                    break;
                }
            }

            if (cmd_len != 0) {
                process_slcan_cmd(instance, cmd_len-1);
                instance->cmd_buf_len -= cmd_len;
                memmove(instance->cmd_buf, instance->cmd_buf+cmd_len, instance->cmd_buf_len);
            } else {
                // No complete command found
                if (instance->cmd_buf_len == sizeof(instance->cmd_buf) || USBD1.state != USB_ACTIVE) {
                    instance->cmd_buf_len = 0;
                }
                break;
            }
        }
    }
}

static void can_rx_listener_task_func(size_t buf_size, const void* buf, void* ctx) {
    (void)buf_size;

    if (USBD1.state != USB_ACTIVE) {
        return;
    }

    struct slcan_instance_s* instance = ctx;

    const struct can_rx_frame_s* rx_frame = buf;

    if (!instance->loopback_enable && rx_frame->origin == CAN_FRAME_ORIGIN_BRIDGE) {
        return;
    }

    char slcan_frame[64];
    size_t slcan_frame_len = can_frame_to_slcan(instance, &rx_frame->content, slcan_frame, sizeof(slcan_frame), rx_frame->origin == CAN_FRAME_ORIGIN_BRIDGE);

    chnWriteTimeout(&SDU1, (uint8_t*)slcan_frame, slcan_frame_len, TIME_IMMEDIATE);
}
