#include <modules/worker_thread/worker_thread.h>
#include <modules/boot_msg/boot_msg.h>
#include <common/shared_app_descriptor.h>
#include <ch.h>
#include <hal.h>
#include <common/crc64_we.h>
#include <modules/flash/flash.h>
#include <modules/uavcan/uavcan.h>
#include <modules/can/can.h>
#include <modules/timing/timing.h>
#include <modules/system/system.h>
#include <modules/uavcan_nodestatus_publisher/uavcan_nodestatus_publisher.h>
#include <uavcan.protocol.file.BeginFirmwareUpdate.h>
#include <uavcan.protocol.file.Read.h>
#include <uavcan.protocol.RestartNode.h>
#include <uavcan.protocol.GetNodeInfo.h>

#ifdef MODULE_UAVCAN_DEBUG_ENABLED
#include <modules/uavcan_debug/uavcan_debug.h>
#define BL_DEBUG(...) uavcan_send_debug_msg(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG, "BL", __VA_ARGS__)
#else
#define BL_DEBUG(...) {}
#endif

#ifdef BOOTLOADER_SUPPORT_BROADCAST_UPDATE
#include <com.hex.file.FileStreamStart.h>
#include <com.hex.file.FileStreamChunk.h>
#endif // BOOTLOADER_SUPPORT_BROADCAST_UPDATE

#ifndef BOOTLOADER_APP_THREAD
#error Please define BOOTLOADER_APP_THREAD in worker_threads_conf.h.
#endif

#define WT BOOTLOADER_APP_THREAD
WORKER_THREAD_DECLARE_EXTERN(WT)

struct app_header_s {
    uint32_t stacktop;
    uint32_t entrypoint;
};

// NOTE: _app_app_flash_sec_sec and _app_flash_sec_end symbols shall be defined in the ld script
extern uint8_t _app_flash_sec[], _app_flash_sec_end;

// NOTE: BOARD_CONFIG_HW_INFO_STRUCTURE defined in the board config file
static const struct shared_hw_info_s _hw_info = BOARD_CONFIG_HW_INFO_STRUCTURE;

static struct {
    bool in_progress;
    uint32_t ofs;
    uint32_t read_req_ofs;
    uint32_t app_start_ofs;
    uint8_t uavcan_idx;
    uint8_t read_transfer_id;
    uint8_t retries;
    uint8_t source_node_id;
    int32_t last_erased_page;
    char path[201];
#ifdef BOOTLOADER_SUPPORT_BROADCAST_UPDATE
    bool using_stream_mode;
#endif // BOOTLOADER_SUPPORT_BROADCAST_UPDATE
} flash_state;

static struct {
    const struct shared_app_descriptor_s* shared_app_descriptor;
    uint64_t image_crc_computed;
    bool image_crc_correct;
    const struct shared_app_parameters_s* shared_app_parameters;
} app_info;

static struct worker_thread_timer_task_s boot_timer_task;
static struct worker_thread_timer_task_s read_timeout_task;
static struct worker_thread_listener_task_s beginfirmwareupdate_req_listener_task;
static struct worker_thread_listener_task_s file_read_res_task;
static struct worker_thread_listener_task_s restart_req_listener_task;
static struct worker_thread_timer_task_s delayed_restart_task;
static struct worker_thread_listener_task_s getnodeinfo_req_listener_task;

#ifdef BOOTLOADER_SUPPORT_BROADCAST_UPDATE
static struct worker_thread_listener_task_s filestreamchunk_listener_task;
static void filestreamchunk_handler(size_t msg_size, const void* buf, void* ctx);
#endif // BOOTLOADER_SUPPORT_BROADCAST_UPDATE

static void file_beginfirmwareupdate_request_handler(size_t msg_size, const void* buf, void* ctx);
static void begin_flash_from_path(uint8_t uavcan_idx, uint8_t source_node_id, const char* path);
static void file_read_response_handler(size_t msg_size, const void* buf, void* ctx);
static void do_send_read_request(bool retry);
static uint32_t get_app_sec_size(void);
static void start_boot(struct worker_thread_timer_task_s* task);
static void update_app_info(void);
static void corrupt_app(void);
static void boot_app_if_commanded(void);
static void command_boot_if_app_valid(uint8_t boot_reason);
static void start_boot_timer(systime_t timeout);
static void cancel_boot_timer(void);
static void check_and_start_boot_timer(void);
static void erase_app_page(uint32_t page_num);
static void do_fail_update(void);
static void on_update_complete(void);
static void bootloader_pre_init(void);
static void bootloader_init(void);
static void restart_req_handler(size_t msg_size, const void* buf, void* ctx);
static void delayed_restart_func(struct worker_thread_timer_task_s* task);
static void read_request_response_timeout(struct worker_thread_timer_task_s* task);
static uint32_t get_app_page_from_ofs(uint32_t ofs);
static uint32_t get_app_address_from_ofs(uint32_t ofs);
static void getnodeinfo_req_handler(size_t msg_size, const void* buf, void* ctx);

RUN_AFTER(BOOT_MSG_RETRIEVAL) {
    bootloader_pre_init();
}

RUN_BEFORE(INIT_END) {
    bootloader_init();
}

RUN_AFTER(UAVCAN_INIT) {
    struct pubsub_topic_s* beginfirmwareupdate_req_topic = uavcan_get_message_topic(0, &uavcan_protocol_file_BeginFirmwareUpdate_req_descriptor);
    worker_thread_add_listener_task(&WT, &beginfirmwareupdate_req_listener_task, beginfirmwareupdate_req_topic, file_beginfirmwareupdate_request_handler, NULL);

    struct pubsub_topic_s* file_read_topic = uavcan_get_message_topic(0, &uavcan_protocol_file_Read_res_descriptor);
    worker_thread_add_listener_task(&WT, &file_read_res_task, file_read_topic, file_read_response_handler, NULL);

    struct pubsub_topic_s* restart_topic = uavcan_get_message_topic(0, &uavcan_protocol_RestartNode_req_descriptor);
    worker_thread_add_listener_task(&WT, &restart_req_listener_task, restart_topic, restart_req_handler, NULL);

    struct pubsub_topic_s* getnodeinfo_req_topic = uavcan_get_message_topic(0, &uavcan_protocol_GetNodeInfo_req_descriptor);
    worker_thread_add_listener_task(&WT, &getnodeinfo_req_listener_task, getnodeinfo_req_topic, getnodeinfo_req_handler, NULL);

#ifdef BOOTLOADER_SUPPORT_BROADCAST_UPDATE
    struct pubsub_topic_s* filestreamchunk_topic = uavcan_get_message_topic(0, &com_hex_file_FileStreamChunk_descriptor);
    worker_thread_add_listener_task(&WT, &filestreamchunk_listener_task, filestreamchunk_topic, filestreamchunk_handler, NULL);
#endif // BOOTLOADER_SUPPORT_BROADCAST_UPDATE
}

static void getnodeinfo_req_handler(size_t msg_size, const void* buf, void* ctx) {
    (void)msg_size;
    (void)ctx;

    const struct uavcan_deserialized_message_s* msg_wrapper = buf;

    struct uavcan_protocol_GetNodeInfo_res_s res;
    memset(&res, 0, sizeof(struct uavcan_protocol_GetNodeInfo_res_s));

    res.status = *uavcan_nodestatus_publisher_get_nodestatus_message();

    board_get_unique_id(res.hardware_version.unique_id, sizeof(res.hardware_version.unique_id));

    strncpy((char*)res.name, _hw_info.hw_name, sizeof(res.name));
    res.name_len = strnlen((char*)res.name, sizeof(res.name));
    res.hardware_version.major = _hw_info.hw_major_version;
    res.hardware_version.minor = _hw_info.hw_minor_version;

    if (app_info.shared_app_descriptor && app_info.image_crc_correct) {
        res.software_version.optional_field_flags = UAVCAN_PROTOCOL_SOFTWAREVERSION_OPTIONAL_FIELD_FLAG_VCS_COMMIT |
        UAVCAN_PROTOCOL_SOFTWAREVERSION_OPTIONAL_FIELD_FLAG_IMAGE_CRC;

        res.software_version.vcs_commit = app_info.shared_app_descriptor->vcs_commit;
        res.software_version.image_crc = app_info.shared_app_descriptor->image_crc;

        res.software_version.major = app_info.shared_app_descriptor->major_version;
        res.software_version.minor = app_info.shared_app_descriptor->minor_version;
    }

    uavcan_respond(msg_wrapper->uavcan_idx, msg_wrapper, &res);
}

static void file_beginfirmwareupdate_request_handler(size_t msg_size, const void* buf, void* ctx) {
    (void)msg_size;
    (void)ctx;
    const struct uavcan_deserialized_message_s* msg_wrapper = buf;
    const struct uavcan_protocol_file_BeginFirmwareUpdate_req_s* req = (const struct uavcan_protocol_file_BeginFirmwareUpdate_req_s*)msg_wrapper->msg;

    struct uavcan_protocol_file_BeginFirmwareUpdate_res_s res;
    res.optional_error_message_len = 0;
    if (!flash_state.in_progress) {
        res.error = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RES_ERROR_OK;
        uavcan_respond(msg_wrapper->uavcan_idx, msg_wrapper, &res);
        char path[sizeof(req->image_file_remote_path)+1] = {};
        memcpy(path, req->image_file_remote_path.path, req->image_file_remote_path.path_len);

        begin_flash_from_path(msg_wrapper->uavcan_idx, req->source_node_id != 0 ? req->source_node_id : msg_wrapper->source_node_id, path);
    } else {
        res.error = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RES_ERROR_IN_PROGRESS;
        uavcan_respond(msg_wrapper->uavcan_idx, msg_wrapper, &res);
    }
}

static void begin_flash_from_path(uint8_t uavcan_idx, uint8_t source_node_id, const char* path) {
    cancel_boot_timer();
    memset(&flash_state, 0, sizeof(flash_state));
    flash_state.in_progress = true;
    flash_state.ofs = 0;
    flash_state.read_transfer_id = 255;
    flash_state.source_node_id = source_node_id;
    flash_state.uavcan_idx = uavcan_idx;
    strncpy(flash_state.path, path, 200);
    worker_thread_add_timer_task(&WT, &read_timeout_task, read_request_response_timeout, NULL, chTimeMS2I(2000), false);
    do_send_read_request(false);

    corrupt_app();
    flash_state.last_erased_page = -1;
}

static void process_chunk(size_t chunk_size, const void* chunk) {
    if (flash_state.ofs + chunk_size > get_app_sec_size()) {
        do_fail_update();
        BL_DEBUG("fail: file too large");
        return;
    }

    if (chunk_size == 0) {
        on_update_complete();
        return;
    }

    int32_t curr_page = get_app_page_from_ofs(flash_state.ofs + chunk_size);
    if (curr_page > flash_state.last_erased_page) {
        for (int32_t i=flash_state.last_erased_page+1; i<=curr_page; i++) {
            erase_app_page(i);
        }
    }

    if (chunk_size < 256) {
        struct flash_write_buf_s buf = {((chunk_size/FLASH_WORD_SIZE) + 1) * FLASH_WORD_SIZE, chunk};
        flash_write((void*)get_app_address_from_ofs(flash_state.ofs), 1, &buf);
        on_update_complete();
    } else {
        struct flash_write_buf_s buf = {chunk_size, chunk};
        flash_write((void*)get_app_address_from_ofs(flash_state.ofs), 1, &buf);
        flash_state.ofs += chunk_size;
        do_send_read_request(false);
    }
}

// TODO factor common code out of read_response_handler and filestreamchunk_handler
#ifdef BOOTLOADER_SUPPORT_BROADCAST_UPDATE
static void filestreamchunk_handler(size_t msg_size, const void* buf, void* ctx) {
    (void)msg_size;
    (void)ctx;

    if (flash_state.in_progress) {
        const struct uavcan_deserialized_message_s* msg_wrapper = buf;
        const struct com_hex_file_FileStreamChunk_s *msg = (const struct com_hex_file_FileStreamChunk_s*)msg_wrapper->msg;

        if (msg->path.path_len != strnlen(flash_state.path, 200) || memcmp(flash_state.path, msg->path.path, msg->path.path_len) != 0) {
            // Not our stream
            return;
        }

        flash_state.using_stream_mode = true;
        worker_thread_timer_task_reschedule(&WT, &read_timeout_task, chTimeMS2I(2000));

        if (msg->offset > flash_state.ofs) {
            // We need to ask for the stream to go back to our offset
            struct com_hex_file_FileStreamStart_req_s req;
            req.path.path_len = strnlen(flash_state.path, 200);
            req.offset = flash_state.ofs;
            memcpy(req.path.path, flash_state.path, req.path.path_len);
            uavcan_request(flash_state.uavcan_idx, &com_hex_file_FileStreamStart_req_descriptor, CANARD_TRANSFER_PRIORITY_MEDIUM+1, flash_state.source_node_id, &req);
            return;
        }

        if (msg->offset != flash_state.ofs) {
            return;
        }

        process_chunk(msg->data_len, msg->data);
    }
}
#endif // BOOTLOADER_SUPPORT_BROADCAST_UPDATE

static void file_read_response_handler(size_t msg_size, const void* buf, void* ctx) {
    (void)msg_size;
    (void)ctx;
    if (flash_state.in_progress) {
        const struct uavcan_deserialized_message_s* msg_wrapper = buf;
        const struct uavcan_protocol_file_Read_res_s *res = (const struct uavcan_protocol_file_Read_res_s*)msg_wrapper->msg;

        if (msg_wrapper->transfer_id != flash_state.read_transfer_id) {
            BL_DEBUG("received read response with wrong transfer id");
            return;
        }

        if (flash_state.ofs != flash_state.read_req_ofs) {
            BL_DEBUG("received read response with wrong ofs %u expect %u", (unsigned)flash_state.read_req_ofs, (unsigned)flash_state.ofs);
            return;
        }

        if (res->error.value != 0) {
            do_fail_update();
            BL_DEBUG("fail: file read error");
            return;
        }

        BL_DEBUG("received read response");

        process_chunk(res->data_len, res->data);
    }
}

static void do_send_read_request(bool retry) {

#ifdef BOOTLOADER_SUPPORT_BROADCAST_UPDATE
    if (!flash_state.using_stream_mode) {
        struct uavcan_protocol_file_Read_req_s read_req;
        flash_state.read_req_ofs = read_req.offset = flash_state.ofs;
        size_t copy_len = strnlen(flash_state.path, sizeof(read_req.path.path));
        memcpy(read_req.path.path, flash_state.path, copy_len);
        read_req.path.path_len = copy_len;
        flash_state.read_transfer_id = uavcan_request(flash_state.uavcan_idx, &uavcan_protocol_file_Read_req_descriptor, CANARD_TRANSFER_PRIORITY_MEDIUM+1, flash_state.source_node_id, &read_req);
    }

    if ((retry && flash_state.using_stream_mode) || (flash_state.ofs < 2048 && !flash_state.using_stream_mode)) {
        // attempt to start stream mode with the first few requests
        struct com_hex_file_FileStreamStart_req_s req;
        req.offset = flash_state.ofs;
        req.path.path_len = strnlen(flash_state.path, 200);
        memcpy(req.path.path, flash_state.path, req.path.path_len);
        uint8_t stream_transfer_id = uavcan_request(flash_state.uavcan_idx, &com_hex_file_FileStreamStart_req_descriptor, CANARD_TRANSFER_PRIORITY_MEDIUM+1, flash_state.source_node_id, &req);
    }
#else
    struct uavcan_protocol_file_Read_req_s read_req;
    flash_state.read_req_ofs = read_req.offset = flash_state.ofs;
    size_t copy_len = strnlen(flash_state.path, sizeof(read_req.path.path));
    memcpy(read_req.path.path, flash_state.path, copy_len);
    read_req.path.path_len = copy_len;
    flash_state.read_transfer_id = uavcan_request(flash_state.uavcan_idx, &uavcan_protocol_file_Read_req_descriptor, CANARD_TRANSFER_PRIORITY_MEDIUM+1, flash_state.source_node_id, &read_req);
#endif

    worker_thread_timer_task_reschedule(&WT, &read_timeout_task, chTimeMS2I(1000));

    if (retry) {
        flash_state.retries++;
    } else {
        flash_state.retries = 0;
    }
}

static uint32_t get_app_sec_size(void) {
    return (uint32_t)&_app_flash_sec_end - (uint32_t)&_app_flash_sec[0];
}

static uint32_t get_app_page_from_ofs(uint32_t ofs)
{
    return flash_get_page_num((void*)((uint32_t)&_app_flash_sec[0] + ofs)) - flash_get_page_num((void*)((uint32_t)&_app_flash_sec[0]));
}

static uint32_t get_app_address_from_ofs(uint32_t ofs)
{
    return (uint32_t)&_app_flash_sec[0] + ofs;
}

static void update_app_info(void) {
    memset(&app_info, 0, sizeof(app_info));

    app_info.shared_app_descriptor = shared_find_app_descriptor(_app_flash_sec, get_app_sec_size());

    const struct shared_app_descriptor_s* descriptor = app_info.shared_app_descriptor;


    if (descriptor && descriptor->image_size >= sizeof(struct shared_app_descriptor_s) && descriptor->image_size <= get_app_sec_size()) {
        uint32_t pre_crc_len = ((uint32_t)&descriptor->image_crc) - ((uint32_t)_app_flash_sec);
        uint32_t post_crc_len = descriptor->image_size - pre_crc_len - sizeof(uint64_t);
        uint8_t* pre_crc_origin = _app_flash_sec;
        uint8_t* post_crc_origin = (uint8_t*)((&descriptor->image_crc)+1);
        uint64_t zero64 = 0;

        app_info.image_crc_computed = crc64_we(pre_crc_origin, pre_crc_len, 0);
        app_info.image_crc_computed = crc64_we((uint8_t*)&zero64, sizeof(zero64), app_info.image_crc_computed);
        app_info.image_crc_computed = crc64_we(post_crc_origin, post_crc_len, app_info.image_crc_computed);

        app_info.image_crc_correct = (app_info.image_crc_computed == descriptor->image_crc);
    }
    if (flash_state.in_progress) {
        set_node_health(UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK);
        set_node_mode(UAVCAN_PROTOCOL_NODESTATUS_MODE_SOFTWARE_UPDATE);
    } else {
        set_node_health(app_info.image_crc_correct ? UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK : UAVCAN_PROTOCOL_NODESTATUS_HEALTH_CRITICAL);
        set_node_mode(UAVCAN_PROTOCOL_NODESTATUS_MODE_MAINTENANCE);
    }
    if (app_info.image_crc_correct) {
        app_info.shared_app_parameters = shared_get_parameters(descriptor);
    }
}

static void corrupt_app(void) {
    erase_app_page(0);
    update_app_info();
}

static void boot_app_if_commanded(void) {
    if (!get_boot_msg_valid() || boot_msg_id != SHARED_MSG_BOOT) {
        return;
    }

    union shared_msg_payload_u msg;
    memcpy(&msg.canbus_info, &boot_msg.canbus_info, sizeof(boot_msg.canbus_info));
    msg.boot_info_msg.boot_reason = boot_msg.boot_msg.boot_reason;
    msg.boot_info_msg.hw_info = &_hw_info;

    shared_msg_finalize_and_write(SHARED_MSG_BOOT_INFO, &msg);

    struct app_header_s* app_header = (struct app_header_s*)_app_flash_sec;

    // offset the vector table
    SCB->VTOR = (uint32_t)&(app_header->stacktop);

    asm volatile(
        "msr msp, %0	\n"
        "bx	%1	\n"
        : : "r"(app_header->stacktop), "r"(app_header->entrypoint) :);
}

static void command_boot_if_app_valid(uint8_t boot_reason) {
    if (!app_info.image_crc_correct) {
        return;
    }

    union shared_msg_payload_u msg;
    boot_msg_fill_shared_canbus_info(&msg.canbus_info);
    msg.boot_msg.boot_reason = boot_reason;

    shared_msg_finalize_and_write(SHARED_MSG_BOOT, &msg);

    system_restart();
}

static void start_boot(struct worker_thread_timer_task_s* task)
{
    (void)task;
    command_boot_if_app_valid(SHARED_BOOT_REASON_TIMEOUT);
}

static void start_boot_timer(systime_t timeout) {
    worker_thread_add_timer_task(&WT, &boot_timer_task, start_boot, NULL, timeout, false);
}

static void cancel_boot_timer(void) {
    worker_thread_remove_timer_task(&WT, &boot_timer_task);
}

static void check_and_start_boot_timer(void) {
    if (get_boot_msg_valid() && boot_msg_id == SHARED_MSG_BOOTLOADER_HOLD) {
        return;
    }

    if (!app_info.shared_app_parameters || app_info.shared_app_parameters->boot_delay_sec == 0) {
        return;
    }

    start_boot_timer(chTimeS2I((uint32_t)app_info.shared_app_parameters->boot_delay_sec));
}

static void erase_app_page(uint32_t page_num) {
    flash_erase_page(flash_get_page_addr(flash_get_page_num(_app_flash_sec) + page_num));
    flash_state.last_erased_page = page_num;
}

static void do_fail_update(void) {
    memset(&flash_state, 0, sizeof(flash_state));
    corrupt_app();
}

static void on_update_complete(void) {
    flash_state.in_progress = false;
    worker_thread_remove_timer_task(&WT, &read_timeout_task);
    update_app_info();
    command_boot_if_app_valid(SHARED_BOOT_REASON_FIRMWARE_UPDATE);
}

// TODO: hook this into early_init
// TODO: boot_msg module will have to initialize before this runs
static void bootloader_pre_init(void) {
    boot_app_if_commanded();
}

static void bootloader_init(void) {
    update_app_info();

    if (get_boot_msg_valid() && boot_msg_id == SHARED_MSG_FIRMWAREUPDATE) {
        begin_flash_from_path(0, boot_msg.firmwareupdate_msg.source_node_id, boot_msg.firmwareupdate_msg.path);
    } else {
        check_and_start_boot_timer();
    }
}

static void restart_req_handler(size_t msg_size, const void* buf, void* ctx) {
    (void)msg_size;
    (void)ctx;

    const struct uavcan_deserialized_message_s* msg_wrapper = buf;
    const struct uavcan_protocol_RestartNode_req_s* msg = (const struct uavcan_protocol_RestartNode_req_s*)msg_wrapper->msg;

    struct uavcan_protocol_RestartNode_res_s res;

    res.ok = false;

    if ((msg->magic_number == UAVCAN_PROTOCOL_RESTARTNODE_REQ_MAGIC_NUMBER) && system_get_restart_allowed()) {
        res.ok = true;
        worker_thread_add_timer_task(&WT, &delayed_restart_task, delayed_restart_func, NULL, chTimeMS2I(1000), false);
    }

    uavcan_respond(msg_wrapper->uavcan_idx, msg_wrapper, &res);
}

static void delayed_restart_func(struct worker_thread_timer_task_s* task) {
    (void)task;
    // try to boot if image is valid
    command_boot_if_app_valid(SHARED_BOOT_REASON_REBOOT_COMMAND);
    // otherwise, just reset
    NVIC_SystemReset();
}

static void read_request_response_timeout(struct worker_thread_timer_task_s* task) {
    (void)task;
    if (flash_state.in_progress) {
        do_send_read_request(true);
        if (flash_state.retries > 100) { // retry for 5 seconds
            do_fail_update();
            BL_DEBUG("fail: out of retries");
        }
    }
}
