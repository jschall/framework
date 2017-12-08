#include <modules/pubsub/pubsub.h>
#include <common/helpers.h>
#include <common/ctor.h>
#include <string.h>

#ifndef PUBSUB_DEFAULT_ALLOCATOR_POOL_SIZE
#define PUBSUB_DEFAULT_ALLOCATOR_POOL_SIZE 512
#endif

PUBSUB_DECLARE_TOPIC_GROUP_STATIC(default_topic_group, PUBSUB_DEFAULT_ALLOCATOR_POOL_SIZE)

static void pubsub_dbg_set_topic_group_initialized(struct pubsub_topic_group_s* topic_group);
static void pubsub_dbg_set_topic_initialized(struct pubsub_topic_s* topic);
static void pubsub_dbg_set_listener_initialized(struct pubsub_listener_s* listener);
static void pubsub_dbg_check_topic_group_initialized(struct pubsub_topic_group_s* topic_group);
static void pubsub_dbg_check_topic_initialized(struct pubsub_topic_s* topic);
static void pubsub_dbg_check_listener_initialized(struct pubsub_listener_s* listener);

static void pubsub_delete_handler(void* delete_block);

void pubsub_create_topic_group(struct pubsub_topic_group_s* topic_group, size_t memory_pool_size, void* memory_pool) {
    if (!topic_group || !memory_pool) {
        return;
    }

    fifoallocator_init(&topic_group->allocator, memory_pool_size, memory_pool, pubsub_delete_handler);
    chMtxObjectInit(&topic_group->mtx);

    pubsub_dbg_set_topic_group_initialized(topic_group);
}

void pubsub_init_topic(struct pubsub_topic_s* topic, struct pubsub_topic_group_s* topic_group) {
    chDbgCheck(topic != NULL);
    if (!topic) {
        return;
    }

    if (!topic_group) {
        topic_group = &default_topic_group;
    }

    pubsub_dbg_check_topic_group_initialized(topic_group);

    topic->message_list_tail = NULL;
    topic->group = topic_group;
    topic->listener_list_head = NULL;

    pubsub_dbg_set_topic_initialized(topic);
}

void pubsub_listener_init_and_register(struct pubsub_listener_s* listener, struct pubsub_topic_s* topic, pubsub_message_handler_func_ptr handler_cb, void* handler_cb_ctx) {
    chDbgCheck(listener != NULL);
    pubsub_dbg_check_topic_initialized(topic);

    if (!topic || !topic->group || !listener) {
        return;
    }

    // initialize listener
    listener->topic = topic;
    listener->next_message = NULL;
    listener->waiting_thread_reference_ptr = NULL;
    listener->handler_cb = handler_cb;
    listener->handler_cb_ctx = handler_cb_ctx;
    chMtxObjectInit(&listener->mtx);
    listener->next = NULL;

    // lock topic group
    chMtxLock(&topic->group->mtx);

    // append listener to topic's listener list
    LINKED_LIST_APPEND(struct pubsub_listener_s, next, topic->listener_list_head, listener);

    // unlock topic group
    chMtxUnlock(&topic->group->mtx);

    pubsub_dbg_set_listener_initialized(listener);
}

void pubsub_listener_unregister(struct pubsub_listener_s* listener) {
    pubsub_dbg_check_listener_initialized(listener);
    if (!listener) {
        return;
    }

    // lock topic group
    chMtxLock(&listener->topic->group->mtx);

    // remove listener from topic's listener list
    LINKED_LIST_REMOVE(struct pubsub_listener_s, next, listener->topic->listener_list_head, listener);

    chMtxUnlock(&listener->topic->group->mtx);
}

void pubsub_listener_reset(struct pubsub_listener_s* listener) {
    pubsub_dbg_check_listener_initialized(listener);

    if (!listener) {
        return;
    }

    chMtxLock(&listener->mtx);
    listener->next_message = NULL;
    chMtxUnlock(&listener->mtx);
}

bool pubsub_listener_has_message(struct pubsub_listener_s* listener) {
    pubsub_dbg_check_listener_initialized(listener);
    return listener->next_message != NULL;
}

void pubsub_copy_writer_func(size_t msg_size, void* msg, void* ctx) {
    memcpy(msg, ctx, msg_size);
}

void pubsub_publish_message(struct pubsub_topic_s* topic, size_t size, pubsub_message_writer_func_ptr writer_cb, void* ctx) {
    pubsub_dbg_check_topic_initialized(topic);
    if (!topic || !topic->group || !topic->listener_list_head) {
        return;
    }

    // lock topic group
    chMtxLock(&topic->group->mtx);

    struct pubsub_message_s* message = fifoallocator_allocate(&topic->group->allocator, size+sizeof(struct pubsub_message_s));

    if (!message) {
        chMtxUnlock(&topic->group->mtx);
        return;
    }

    message->topic = topic;
    message->next_in_topic = NULL;

    if (topic->message_list_tail) {
        chDbgCheck(topic->message_list_tail != message); // Circular reference
        topic->message_list_tail->next_in_topic = message;
    }
    topic->message_list_tail = message;

    if (writer_cb) {
        writer_cb(size, message->data, ctx);
    }

    struct pubsub_listener_s* listener = topic->listener_list_head;
    while (listener) {
        if (!listener->next_message) {
            // Lock to ensure atomic write of next_message
            chMtxLock(&listener->mtx);
            listener->next_message = message;
            chMtxUnlock(&listener->mtx);
        }

        chSysLock();
        if (listener->waiting_thread_reference_ptr) {
            chThdResumeS(listener->waiting_thread_reference_ptr, (msg_t)listener);
        }
        chSysUnlock();

        listener = listener->next;
    }

    chMtxUnlock(&topic->group->mtx);
}

void pubsub_listener_set_handler_cb(struct pubsub_listener_s* listener, pubsub_message_handler_func_ptr handler_cb, void* handler_cb_ctx) {
    pubsub_dbg_check_listener_initialized(listener);
    if (!listener) {
        return;
    }

    chMtxLock(&listener->mtx);
    listener->handler_cb = handler_cb;
    listener->handler_cb_ctx = handler_cb_ctx;
    chMtxUnlock(&listener->mtx);
}

void pubsub_listener_handle_until_timeout(struct pubsub_listener_s* listener, systime_t timeout) {
    pubsub_dbg_check_listener_initialized(listener);
    pubsub_multiple_listener_handle_until_timeout(1, &listener, timeout);
}

bool pubsub_listener_handle_one_timeout(struct pubsub_listener_s* listener, systime_t timeout) {
    pubsub_dbg_check_listener_initialized(listener);
    return pubsub_multiple_listener_handle_one_timeout(1, &listener, timeout);
}

static struct pubsub_listener_s* pubsub_multiple_listener_wait_timeout_S(size_t num_listeners, struct pubsub_listener_s** listeners, systime_t timeout);

bool pubsub_multiple_listener_handle_one_timeout(size_t num_listeners, struct pubsub_listener_s** listeners, systime_t timeout) {
    for (size_t i=0; i<num_listeners; i++) {
        pubsub_dbg_check_listener_initialized(listeners[i]);
    }
    chSysLock();
    struct pubsub_listener_s* listener_with_message = pubsub_multiple_listener_wait_timeout_S(num_listeners, listeners, timeout);

    if (listener_with_message) {
        chMtxLockS(&listener_with_message->mtx);
        chSysUnlock();

        struct pubsub_message_s* message = listener_with_message->next_message;

        if (listener_with_message->handler_cb) {
            size_t message_size = fifoallocator_get_block_size(message)-sizeof(struct pubsub_message_s);

            listener_with_message->handler_cb(message_size, message->data, listener_with_message->handler_cb_ctx);
        }

        listener_with_message->next_message = message->next_in_topic;

        chMtxUnlock(&listener_with_message->mtx);
        return true;
    } else {
        chSysUnlock();
        return false;
    }
}

void pubsub_multiple_listener_handle_until_timeout(size_t num_listeners, struct pubsub_listener_s** listeners, systime_t timeout) {
    systime_t start = chVTGetSystemTimeX();
    systime_t elapsed = 0;

    do {
        pubsub_multiple_listener_handle_one_timeout(num_listeners, listeners, timeout-elapsed);

        if (timeout != TIME_INFINITE) {
            elapsed = chVTTimeElapsedSinceX(start);
        }
    } while(elapsed < timeout);
}

void pubsub_listener_set_waiting_thread_reference(struct pubsub_listener_s* listener, thread_reference_t* trpp) {
    pubsub_dbg_check_listener_initialized(listener);
    if (!listener) {
        return;
    }

    listener->waiting_thread_reference_ptr = trpp;
}

static struct pubsub_listener_s* pubsub_multiple_listener_wait_timeout_S(size_t num_listeners, struct pubsub_listener_s** listeners, systime_t timeout) {
    chDbgCheckClassS();

    for (size_t i=0; i<num_listeners; i++) {
        pubsub_dbg_check_listener_initialized(listeners[i]);
    }

    // Check for immediately available messages
    for (size_t i=0; i<num_listeners; i++) {
        if (listeners && listeners[i] && listeners[i]->next_message) {
            return listeners[i];
        }
    }

    if (timeout == TIME_IMMEDIATE) {
        return NULL;
    }

    // Point listeners' waiting thread references to our thread
    thread_reference_t trp = NULL;
    for (size_t i=0; i<num_listeners; i++) {
        if (listeners && listeners[i]) {
            listeners[i]->waiting_thread_reference_ptr = &trp;
        }
    }

    // Wait for a listener to wake us up
    msg_t message = chThdSuspendTimeoutS(&trp, timeout);

    struct pubsub_listener_s* ret = NULL;
    if (message != MSG_TIMEOUT) {
        for (size_t i=0; i<num_listeners; i++) {
            if (listeners[i] == (void*)message) {
                ret = listeners[i];
            }
        }
    }

    // Set listeners' waiting thread references back to NULL
    for (size_t i=0; i<num_listeners; i++) {
        if (listeners && listeners[i]) {
            listeners[i]->waiting_thread_reference_ptr = NULL;
        }
    }

    return ret;
}

static void pubsub_delete_handler(void* delete_block) {

    // NOTE: this will be called during publishing in the publishing thread's context. The topic group will already be locked.
    struct pubsub_message_s* message_to_delete = delete_block;

    pubsub_dbg_check_topic_initialized(message_to_delete->topic);

    struct pubsub_listener_s* listener = message_to_delete->topic->listener_list_head;
    while (listener) {
        pubsub_dbg_check_listener_initialized(listener);

        if (listener->next_message == message_to_delete) {
            chMtxLock(&listener->mtx);
            if (listener->next_message == message_to_delete) {
                listener->next_message = message_to_delete->next_in_topic;
            }
            chMtxUnlock(&listener->mtx);
        }
        listener = listener->next;
    }

    if (message_to_delete->topic->message_list_tail == message_to_delete) {
        message_to_delete->topic->message_list_tail = NULL;
    }
}

#if CH_DBG_ENABLE_CHECKS && DBG_INIT_ORDER_CHECKS
static struct pubsub_topic_group_s* topic_group_init_check_head;
static struct pubsub_topic_s* topic_init_check_head;
static struct pubsub_listener_s* listener_init_check_head;

static void pubsub_dbg_set_topic_group_initialized(struct pubsub_topic_group_s* topic_group) {
    chSysLock();
    LINKED_LIST_APPEND(struct pubsub_topic_group_s, init_check_next, topic_group_init_check_head, topic_group);
    chSysUnlock();
}

static void pubsub_dbg_set_topic_initialized(struct pubsub_topic_s* topic) {
    chSysLock();
    LINKED_LIST_APPEND(struct pubsub_topic_s, init_check_next, topic_init_check_head, topic);
    chSysUnlock();
}

static void pubsub_dbg_set_listener_initialized(struct pubsub_listener_s* listener) {
    chSysLock();
    LINKED_LIST_APPEND(struct pubsub_listener_s, init_check_next, listener_init_check_head, listener);
    chSysUnlock();
}

static void pubsub_dbg_check_topic_group_initialized(struct pubsub_topic_group_s* topic_group) {
    bool result;
    LINKED_LIST_FIND(struct pubsub_topic_group_s, init_check_next, topic_group_init_check_head, topic_group, result)
    chDbgCheck(result);
}

static void pubsub_dbg_check_topic_initialized(struct pubsub_topic_s* topic) {
    bool result;
    LINKED_LIST_FIND(struct pubsub_topic_s, init_check_next, topic_init_check_head, topic, result)
    chDbgCheck(result);
}

static void pubsub_dbg_check_listener_initialized(struct pubsub_listener_s* listener) {
    bool result;
    LINKED_LIST_FIND(struct pubsub_listener_s, init_check_next, listener_init_check_head, listener, result)
    chDbgCheck(result);
}
#else
static void pubsub_dbg_set_topic_group_initialized(struct pubsub_topic_group_s* topic_group) {}
static void pubsub_dbg_set_topic_initialized(struct pubsub_topic_s* topic) {}
static void pubsub_dbg_set_listener_initialized(struct pubsub_listener_s* listener) {}
static void pubsub_dbg_check_topic_group_initialized(struct pubsub_topic_group_s* topic_group) {}
static void pubsub_dbg_check_topic_initialized(struct pubsub_topic_s* topic) {}
static void pubsub_dbg_check_listener_initialized(struct pubsub_listener_s* listener) {}
#endif
