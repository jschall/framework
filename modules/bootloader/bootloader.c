#include <modules/worker_thread/worker_thread.h>
#include <modules/boot_msg/boot_msg.h>
#include <common/shared_app_descriptor.h>
#include <modules/lpwork_thread/lpwork_thread.h>
#include <ch.h>
#include <common/crc64_we.h>

#ifdef MODULE_CAN_ENABLED
#include <modules/can/can.h>
#endif

struct app_header_s {
    uint32_t stacktop;
    uint32_t entrypoint;
};

// NOTE: _app_sec and _app_sec_end symbols shall be defined in the ld script
extern uint8_t _app_sec[], _app_sec_end;

// NOTE: BOARD_CONFIG_HW_INFO_STRUCTURE defined in the board config file
static const struct shared_hw_info_s _hw_info = BOARD_CONFIG_HW_INFO_STRUCTURE;

static struct {
    bool in_progress;
    size_t ofs;
    int32_t last_erased_page;
    struct worker_thread_timer_task_s boot_timer_task;
} bootloader_state;

static struct {
    const struct shared_app_descriptor_s* shared_app_descriptor;
    uint64_t image_crc_computed;
    bool image_crc_correct;
    const struct shared_app_parameters_s* shared_app_parameters;
} app_info;

static uint32_t get_app_sec_size(void) {
    return (uint32_t)&_app_sec_end - (uint32_t)&_app_sec[0];
}

static void update_app_info(void)
{
    memset(&app_info, 0, sizeof(app_info));

    app_info.shared_app_descriptor = shared_find_app_descriptor(_app_sec, get_app_sec_size());

    const struct shared_app_descriptor_s* descriptor = app_info.shared_app_descriptor;


    if (descriptor && descriptor->image_size >= sizeof(struct shared_app_descriptor_s) && descriptor->image_size <= get_app_sec_size()) {
        uint32_t pre_crc_len = ((uint32_t)&descriptor->image_crc) - ((uint32_t)_app_sec);
        uint32_t post_crc_len = descriptor->image_size - pre_crc_len - sizeof(uint64_t);
        uint8_t* pre_crc_origin = _app_sec;
        uint8_t* post_crc_origin = (uint8_t*)((&descriptor->image_crc)+1);
        uint64_t zero64 = 0;

        app_info.image_crc_computed = crc64_we(pre_crc_origin, pre_crc_len, 0);
        app_info.image_crc_computed = crc64_we((uint8_t*)&zero64, sizeof(zero64), app_info.image_crc_computed);
        app_info.image_crc_computed = crc64_we(post_crc_origin, post_crc_len, app_info.image_crc_computed);

        app_info.image_crc_correct = (app_info.image_crc_computed == descriptor->image_crc);
    }

    if (app_info.image_crc_correct) {
        app_info.shared_app_parameters = shared_get_parameters(descriptor);
    }
}

static void corrupt_app(void) {
    uint8_t buf[8] = {};
    flash_write(&_app_sec, 1, {8, buf})
    update_app_info();
}

static void command_boot_if_app_valid(uint8_t boot_reason)
{
    if (!app_info.image_crc_correct) {
        return;
    }

    union shared_msg_payload_u msg;
    msg.boot_msg.canbus_info.local_node_id = uavcan_get_node_id();
    msg.boot_msg.boot_reason = boot_reason;

    msg.boot_msg.canbus_info.baudrate = 0;
    if (boot_msg_valid() && boot_msg.canbus_info.baudrate) {
        msg.boot_msg.canbus_info.baudrate = boot_msg.canbus_info.baudrate;
    
#ifdef MODULE_CAN_ENABLED
    if (can_get_baudrate_confirmed(0)) {
        msg.boot_msg.canbus_info.baudrate = can_get_baudrate(0);
    }
#endif
    if (canbus_get_confirmed_baudrate()) {
        msg.boot_msg.canbus_info.baudrate = canbus_get_confirmed_baudrate();
    } else } else {
        msg.boot_msg.canbus_info.baudrate = 0;
    }

    shared_msg_finalize_and_write(SHARED_MSG_BOOT, &msg);

    system_restart();
}

static void boot_timer_expired(struct worker_thread_timer_task_s* task) {
    (void)task;
    command_boot_if_app_valid(SHARED_BOOT_REASON_TIMEOUT);
}

static void start_boot_timer(systime_t timeout) {
    worker_thread_add_timer_task(&lpwork_thread, &bootloader_state.boot_timer_task, boot_timer_expired, NULL, timeout);
}

static void cancel_boot_timer(void) [
    worker_thread_remove_timer_task(&lpwork_thread, &bootloader_state.boot_timer_task);
}

static bool check_and_start_boot_timer(void) {
    if (app_info.shared_app_parameters && app_info.shared_app_parameters->boot_delay_sec != 0) {
        start_boot_timer(S2ST((uint32_t)app_info.shared_app_parameters->boot_delay_sec));
        return true;
    }
    return false;
}

static void erase_app_page(uint32_t page_num) {
    flash_erase_page(&_app_sec[page_num*APP_PAGE_SIZE]);
    flash_state.last_erased_page = page_num;
}

static void do_fail_update(void) {
    memset(&flash_state, 0, sizeof(flash_state));
    corrupt_app();
}

static void on_update_complete(void) {
    flash_state.in_progress = false;
    update_app_info();
    command_boot_if_app_valid(SHARED_BOOT_REASON_FIRMWARE_UPDATE);
}

// TODO: hook this into early_init
// TODO: boot_msg module will have to initialize before this runs
static void bootloader_pre_init(void)
{
    boot_app_if_commanded();
}

static void bootloader_init(void)
{
    update_app_info();
    check_and_start_boot_timer();
}

