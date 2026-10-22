#include "app.h"
#include "protocol.h"
#include "motion.h"
#include "machine_config.h"
#include "esp8266.h"
#include "main.h"
#include <stdio.h>

#define APP_TX_SLOTS 64u

typedef struct {
    uint16_t len;
    uint8_t  data[PROTO_MAX_FRAME];
} AppTxSlot;

static FrameParser s_parser;
static AppTxSlot  s_tx_queue[APP_TX_SLOTS];
static uint8_t    s_tx_head;
static uint8_t    s_tx_tail;
static uint8_t    s_tx_count;
static uint32_t   s_tx_retry_tick;
static uint16_t   s_last_accepted_seq;
static uint16_t   s_last_completed_seq;
static ProtoState s_state;
static bool       s_status_pending;

static void app_handle_frame(uint8_t cmd, const uint8_t *payload, uint16_t len);

static bool app_tx_push(uint8_t cmd, const uint8_t *payload, uint16_t len)
{
    if (s_tx_count >= APP_TX_SLOTS) return false;

    AppTxSlot *slot = &s_tx_queue[s_tx_tail];
    uint16_t frame_len = Frame_Encode(cmd, payload, len, slot->data, sizeof(slot->data));
    if (frame_len == 0u) return false;

    slot->len = frame_len;
    s_tx_tail = (uint8_t)((s_tx_tail + 1u) % APP_TX_SLOTS);
    s_tx_count++;
    return true;
}

static void app_tx_clear(void)
{
    s_tx_head = 0;
    s_tx_tail = 0;
    s_tx_count = 0;
}

static void app_tx_pump(void)
{
    if (s_tx_count == 0u) return;

    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - s_tx_retry_tick) < 10u) return;

    AppTxSlot *slot = &s_tx_queue[s_tx_head];
    if (Esp8266_SendData(slot->data, slot->len)) {
        s_tx_head = (uint8_t)((s_tx_head + 1u) % APP_TX_SLOTS);
        s_tx_count--;
    }
    s_tx_retry_tick = now;
}

static void app_send_ack(uint16_t seq, ProtoAckCode code)
{
    uint8_t payload[PROTO_ACK_PAYLOAD_LEN];
    Proto_WriteU16(&payload[0], seq);
    payload[2] = (uint8_t)code;
    app_tx_push(PROTO_CMD_ACK, payload, sizeof(payload));
}

static bool app_push_status_values(ProtoState state, uint8_t queue_free,
                                   const int32_t position[3], uint16_t seq)
{
    uint8_t payload[PROTO_STATUS_PAYLOAD_LEN];
    payload[0] = (uint8_t)state;
    payload[1] = queue_free;
    Proto_WriteU32(&payload[2], (uint32_t)position[0]);
    Proto_WriteU32(&payload[6], (uint32_t)position[1]);
    Proto_WriteU32(&payload[10], (uint32_t)position[2]);
    Proto_WriteU16(&payload[14], seq);
    return app_tx_push(PROTO_CMD_STATUS, payload, sizeof(payload));
}

static bool app_push_status_current(ProtoState state, uint16_t seq)
{
    const AxisState *x = Motion_GetAxis(AXIS_X);
    const AxisState *y = Motion_GetAxis(AXIS_Y);
    const AxisState *z = Motion_GetAxis(AXIS_Z);
    if (!x || !y || !z) return false;

    int32_t position[3] = { x->position, y->position, z->position };
    return app_push_status_values(state, Motion_QueueFree(), position, seq);
}

static bool app_push_done_status(const MotionDone *done)
{
    ProtoState state = done->more ? PROTO_STATE_MOVING : PROTO_STATE_IDLE;
    return app_push_status_values(state, Motion_QueueFree(), done->position, done->seq);
}

static void app_drain_done(void)
{
    while (s_tx_count < APP_TX_SLOTS) {
        MotionDone done;
        if (!Motion_PopDone(&done)) break;

        s_last_completed_seq = done.seq;

        if (!app_push_done_status(&done)) {
            printf("STATUS drop seq=%u\r\n", (unsigned)done.seq);
            break;
        }

        uint32_t rate = 0u;
        if (done.elapsed_ms > 0u) {
            rate = (uint32_t)(((uint64_t)done.steps * 1000u) / done.elapsed_ms);
        }

        printf("MOVE done seq=%u steps=%lu time=%lums rate=%lu steps/s pos=%ld,%ld,%ld\r\n",
               (unsigned)done.seq,
               (unsigned long)done.steps,
               (unsigned long)done.elapsed_ms,
               (unsigned long)rate,
               (long)done.position[AXIS_X],
               (long)done.position[AXIS_Y],
               (long)done.position[AXIS_Z]);
    }
}

static void app_handle_line(const uint8_t *payload, uint16_t len)
{
    if (len != PROTO_LINE_PAYLOAD_LEN) {
        app_send_ack(PROTO_SEQ_NONE, PROTO_ACK_BAD_LENGTH);
        printf("LINE bad length: %u\r\n", (unsigned)len);
        return;
    }

    int32_t x = (int32_t)Proto_ReadU32(&payload[0]);
    int32_t y = (int32_t)Proto_ReadU32(&payload[4]);
    int32_t z = (int32_t)Proto_ReadU32(&payload[8]);
    uint32_t feed = Proto_ReadU32(&payload[12]);
    uint16_t seq = Proto_ReadU16(&payload[16]);

    printf("LINE x=%ld y=%ld z=%ld feed=%lu seq=%u\r\n",
           (long)x, (long)y, (long)z, (unsigned long)feed, (unsigned)seq);

    if (s_state == PROTO_STATE_ESTOP) {
        app_send_ack(seq, PROTO_ACK_BUSY);
        return;
    }

    uint32_t max_feed = MACHINE_MAX_FEED_HZ;
    if (max_feed > MAX_STEP_HZ) max_feed = MAX_STEP_HZ;
    if (feed == 0u || feed > max_feed) {
        app_send_ack(seq, PROTO_ACK_BAD_PARAM);
        return;
    }

    if (!Motion_Enqueue(x, y, z, feed, seq)) {
        app_send_ack(seq, PROTO_ACK_QUEUE_FULL);
        return;
    }

    s_last_accepted_seq = seq;
    app_send_ack(seq, PROTO_ACK_OK);
}

static void app_handle_frame(uint8_t cmd, const uint8_t *payload, uint16_t len)
{
    switch (cmd) {
    case PROTO_CMD_LINE:
        app_handle_line(payload, len);
        break;

    case PROTO_CMD_STOP:
        if (len != 0u) {
            app_send_ack(PROTO_SEQ_NONE, PROTO_ACK_BAD_LENGTH);
            break;
        }
        app_tx_clear();
        Motion_Stop();
        s_state = PROTO_STATE_IDLE;
        printf("STOP\r\n");
        app_send_ack(PROTO_SEQ_NONE, PROTO_ACK_OK);
        s_status_pending = true;
        break;

    case PROTO_CMD_ESTOP:
        if (len != 0u) {
            app_send_ack(PROTO_SEQ_NONE, PROTO_ACK_BAD_LENGTH);
            break;
        }
        app_tx_clear();
        Motion_Stop();
        s_state = PROTO_STATE_ESTOP;
        printf("ESTOP\r\n");
        app_send_ack(PROTO_SEQ_NONE, PROTO_ACK_OK);
        s_status_pending = true;
        break;

    case PROTO_CMD_HOME:
        printf("HOME not implemented\r\n");
        app_send_ack(PROTO_SEQ_NONE, PROTO_ACK_UNSUPPORTED);
        break;

    case PROTO_CMD_QUERY:
        s_status_pending = true;
        break;

    default:
        printf("UNKNOWN cmd=0x%02X len=%u\r\n", (unsigned)cmd, (unsigned)len);
        app_send_ack(PROTO_SEQ_NONE, PROTO_ACK_UNSUPPORTED);
        break;
    }
}

void App_Init(void)
{
    FrameParser_Init(&s_parser, app_handle_frame);

    s_tx_head = 0;
    s_tx_tail = 0;
    s_tx_count = 0;
    s_tx_retry_tick = 0;
    s_last_accepted_seq = PROTO_SEQ_NONE;
    s_last_completed_seq = PROTO_SEQ_NONE;
    s_state = PROTO_STATE_IDLE;
    s_status_pending = false;
}

void App_OnLinkReset(void)
{
    FrameParser_Reset(&s_parser);
    app_tx_clear();
    s_tx_retry_tick = 0;
    s_last_accepted_seq = PROTO_SEQ_NONE;
    s_last_completed_seq = PROTO_SEQ_NONE;
    s_state = PROTO_STATE_IDLE;
    s_status_pending = false;
}

void App_OnTcpData(const uint8_t *data, uint16_t len)
{
    if (!data) return;
    for (uint16_t i = 0; i < len; i++) {
        FrameParser_Feed(&s_parser, data[i]);
    }
}

void App_Process(void)
{
    app_drain_done();

    if (s_status_pending && s_tx_count < APP_TX_SLOTS) {
        ProtoState state = s_state;
        if (state != PROTO_STATE_ESTOP && Motion_IsBusy()) {
            state = PROTO_STATE_MOVING;
        }
        uint16_t seq = (s_last_completed_seq != PROTO_SEQ_NONE)
                     ? s_last_completed_seq
                     : s_last_accepted_seq;

        if (app_push_status_current(state, seq)) {
            s_status_pending = false;
            printf("STATUS queued\r\n");
        }
    }

    app_tx_pump();
}
