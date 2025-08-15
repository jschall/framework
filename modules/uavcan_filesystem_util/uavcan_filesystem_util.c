#include "uavcan_filesystem_util.h"

#include <modules/uavcan/uavcan.h>
#include <modules/pubsub/pubsub.h>
#include <uavcan.protocol.file.Read.h>
#include <string.h>

struct file_read_receive_handler_ctx_s {
    uint8_t transfer_id;
    bool success;
    uint8_t* data;
    uint32_t len;
};

static void file_read_receive_handler(size_t msg_size, const void* buf, void* ctx_p) {
    (void)msg_size;
    const struct uavcan_deserialized_message_s* msg_wrapper = buf;
    const struct uavcan_protocol_file_Read_res_s *res = (const struct uavcan_protocol_file_Read_res_s*)msg_wrapper->msg;

    struct file_read_receive_handler_ctx_s* ctx = ctx_p;

    // Check if the transfer id matches
    if (msg_wrapper->transfer_id != ctx->transfer_id || res->error.value != 0) {
        return;
    }

    ctx->success = true;

    if (res->data_len < ctx->len) {
        ctx->len = res->data_len;
    }

    memcpy(ctx->data, res->data, ctx->len);
}

static size_t uavcan_filesystem_read_single_chunk_timeout(uint8_t node_id, const char* path, uint64_t ofs, uint8_t* buf, uint32_t max_len_recv, systime_t timeout) {
    systime_t t0 = chVTGetSystemTimeX();

    struct file_read_receive_handler_ctx_s ctx = {0, false, buf, max_len_recv};

    // Create a listener for read responses
    struct pubsub_listener_s listener;
    struct pubsub_topic_s* file_read_topic = uavcan_get_message_topic(0, &uavcan_protocol_file_Read_res_descriptor);
    pubsub_listener_init_and_register(&listener, file_read_topic, file_read_receive_handler, &ctx);

    // Send read request
    struct uavcan_protocol_file_Read_req_s read_req;
    read_req.offset = ofs;
    strncpy((char*)read_req.path.path, path, sizeof(read_req.path));
    read_req.path.path_len = strnlen(path, sizeof(read_req.path.path));
    ctx.transfer_id = uavcan_request(0, &uavcan_protocol_file_Read_req_descriptor, CANARD_TRANSFER_PRIORITY_LOW-1, node_id, &read_req);

    while (true) {
        systime_t tnow = chVTGetSystemTimeX();
        systime_t elapsed = tnow-t0;

        if (elapsed > timeout) {
            break;
        }

        systime_t remaining = timeout-elapsed;

        // Await response
        pubsub_listener_handle_one_timeout(&listener, remaining);
        if (ctx.success) {
            pubsub_listener_unregister(&listener);
            return ctx.len;
        }
    }

    pubsub_listener_unregister(&listener);
    return (size_t)-1;
}

bool uavcan_filesystem_read_timeout(uint8_t node_id, const char* path, uint64_t ofs, size_t* len, uint8_t* buf, systime_t timeout) {
    systime_t t0 = chVTGetSystemTimeX();

    size_t total_len_recv = 0;

    while (true) {
        size_t bytes_remaining = *len-total_len_recv;

        if (bytes_remaining == 0) { // Done
            *len = total_len_recv;
            return true;
        }


        systime_t tnow = chVTGetSystemTimeX();
        systime_t elapsed = tnow-t0;

        if (elapsed > timeout) {
            break;
        }

        systime_t t_remaining = timeout-elapsed;

        size_t len_recv = uavcan_filesystem_read_single_chunk_timeout(node_id, path, ofs+total_len_recv, buf, bytes_remaining, t_remaining);

        if (len_recv == 0) { // EOF
            *len = total_len_recv;
            return true;
        }

        if (len_recv != (size_t)-1) {
            total_len_recv += len_recv;
        }
    }

    return false;
}
