#include <common/shared_boot_msg.h>
#include <common/crc64_we.h>
#include <string.h>
#include <ch.h>

#define SHARED_MSG_MAGIC 0xDEADBEEF

struct shared_msg_header_s {
    uint64_t crc64;
    uint32_t magic;
    uint8_t msgid;
} SHARED_MSG_PACKED;

struct shared_msg_s {
    struct shared_msg_header_s header;
    union shared_msg_payload_u payload;
    uint32_t dummy;
} SHARED_MSG_PACKED;

// NOTE: _app_bl_shared_sec symbol shall be defined by the ld script
extern volatile struct shared_msg_s _app_bl_shared_sec;

static int16_t get_payload_length(enum shared_msg_t msgid) {
    switch(msgid) {
        case SHARED_MSG_BOOT:
            return sizeof(struct shared_boot_msg_s);
        case SHARED_MSG_FIRMWAREUPDATE:
            return sizeof(struct shared_firmwareupdate_msg_s);
        case SHARED_MSG_BOOT_INFO:
            return sizeof(struct shared_boot_info_msg_s);
        case SHARED_MSG_CANBUS_INFO:
            return sizeof(struct shared_canbus_info_s);
        case SHARED_MSG_BOOTLOADER_HOLD:
            return sizeof(struct shared_bootloader_hold_msg_s);
    };

    return -1;
}

static uint32_t compute_mailbox_crc64(int16_t payload_len) {
    return crc64_we((uint8_t*)(&_app_bl_shared_sec.header.crc64+1), sizeof(_app_bl_shared_sec.header)+payload_len-sizeof(uint64_t), 0);
}

static bool mailbox_valid(void) {
    if (_app_bl_shared_sec.header.magic != SHARED_MSG_MAGIC) {
        return false;
    }

    int16_t payload_len = get_payload_length((enum shared_msg_t)_app_bl_shared_sec.header.msgid);
    if (payload_len == -1) {
        return false;
    }

    if (compute_mailbox_crc64(payload_len) != _app_bl_shared_sec.header.crc64) {
        return false;
    }

    return true;
}

bool shared_msg_check_and_retreive(enum shared_msg_t* msgid, union shared_msg_payload_u* msg_payload) {
    if (!mailbox_valid()) {
        return false;
    }

    *msgid = (enum shared_msg_t)_app_bl_shared_sec.header.msgid;
    *msg_payload = _app_bl_shared_sec.payload;
    return true;
}

void shared_msg_finalize_and_write(enum shared_msg_t msgid, const union shared_msg_payload_u* msg_payload) {
    _app_bl_shared_sec.header.msgid = (uint8_t)msgid;
    memcpy((void*)&_app_bl_shared_sec.payload, msg_payload, sizeof(union shared_msg_payload_u));
    _app_bl_shared_sec.header.magic = SHARED_MSG_MAGIC;
    _app_bl_shared_sec.header.crc64 = compute_mailbox_crc64(get_payload_length(msgid));

    // STM32H7 reference manual: when an incomplete word is written to an internal SRAM and a reset occurs, the last incomplete word is not really written. This is due to the ECC behavior. To ensure that an incomplete word is written to SRAM, write an additional dummy incomplete word to the same RAM at a different address before issuing a reset.

    _app_bl_shared_sec.dummy = SHARED_MSG_MAGIC;
#if defined(STM32H7) || defined(STM32F7)
    SCB_CleanInvalidateDCache();
#endif
}

void shared_msg_clear(void) {
    memset((void*)&_app_bl_shared_sec, 0, sizeof(_app_bl_shared_sec));
#if defined(STM32H7) || defined(STM32F7)
    SCB_CleanInvalidateDCache();
#endif
}
