#include "usbcfg.h"
#include <common/ctor.h>
#include <modules/worker_thread/worker_thread.h>
#include <hal.h>

#ifndef USB_SERIAL_WORKER_THREAD
#error Please define USB_SERIAL_WORKER_THREAD in framework_conf.h.
#endif

#define WT USB_SERIAL_WORKER_THREAD
WORKER_THREAD_DECLARE_EXTERN(WT)

// static struct worker_thread_timer_task_s usb_serial_connect_task;
// static void usb_serial_connect_task_func(struct worker_thread_timer_task_s* task);

RUN_AFTER(WORKER_THREADS_INIT) {
//     worker_thread_add_timer_task(&WT, &usb_serial_connect_task, usb_serial_connect_task_func, NULL, LL_MS2ST(1500), false);
    setup_usb_strings();

    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);
    usbDisconnectBus(serusbcfg.usbp);
    chThdSleep(LL_MS2ST(1500));
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);
}

// static void usb_serial_connect_task_func(struct worker_thread_timer_task_s* task) {
//     (void)task;
//     usbStart(serusbcfg.usbp, &usbcfg);
//     usbConnectBus(serusbcfg.usbp);
// }
