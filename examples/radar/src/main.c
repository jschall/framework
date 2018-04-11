#include <modules/driver_dw1000/dw1000.h>
#include <modules/param/param.h>
#include <modules/worker_thread/worker_thread.h>
#include <common/ctor.h>
#include <modules/uavcan_debug/uavcan_debug.h>
#include <string.h>

#include <modules/uavcan/uavcan.h>
#include <com.matternet.equipment.uwb.Radar.h>

#ifndef UWB_WORKER_THREAD
#error Please define UWB_WORKER_THREAD in worker_threads_conf.h.
#endif

#define WT UWB_WORKER_THREAD
WORKER_THREAD_DECLARE_EXTERN(WT)

static struct dw1000_instance_s instance;

PARAM_DEFINE_BOOL_PARAM_STATIC(param_receive, "RX", true);

static struct worker_thread_timer_task_s uwb_task;

static struct __attribute__((packed)) {
    uint8_t dummy;
    struct __attribute__((packed)) {
        int16_t real;
        int16_t imag;
    } data[1024];
} cir;

static void uwb_task_func(struct worker_thread_timer_task_s* task) {
    (void)task;
    
    if (!param_receive) {
        dw1000_disable_transceiver(&instance);
        dw1000_transmit(&instance, strlen("hello world")+1, "hello world", false);
    } else {
        uint8_t buf[256] = {};
        struct dw1000_rx_frame_info_s rx_info = dw1000_receive(&instance, sizeof(buf), buf);
        
        if (rx_info.err_code == DW1000_RX_ERROR_NONE) {
            uavcan_send_debug_msg(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_INFO, "uwb", buf);
            uavcan_send_debug_msg(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_INFO, "", "fp_ampl1 = %u fp_ampl2 = %u fp_ampl3 = %u", rx_info.fp_ampl1, rx_info.fp_ampl2, rx_info.fp_ampl3);

            uint16_t fp_index = (dw1000_get_fp_index(&instance) >> 6)&0x03ff;
            memset(&cir,0,sizeof(cir));
            dw1000_read_cir(&instance, &cir);

            struct com_matternet_equipment_uwb_Radar_s msg;

            msg.fp_idx = fp_index;
            for (uint8_t i=0; i<60; i++) {
                msg.cir[i].real = cir.data[fp_index+i].real;
                msg.cir[i].imag = cir.data[fp_index+i].imag;
            }

            uavcan_broadcast(0, &com_matternet_equipment_uwb_Radar_descriptor, CANARD_TRANSFER_PRIORITY_LOW, &msg);

            dw1000_disable_transceiver(&instance);
            dw1000_rx_enable(&instance);
        }
    }
}

RUN_AFTER(UAVCAN_INIT) {
    dw1000_init(&instance, 4, BOARD_PAL_LINE_SPI_UWB_CS, BOARD_PAL_LINE_UWB_NRST);
    dw1000_rx_enable(&instance);
    dw1000_set_tx_power(&instance, 0x85);
    worker_thread_add_timer_task(&WT, &uwb_task, uwb_task_func, NULL, MS2ST(param_receive ? 10 : 250), true);
}
