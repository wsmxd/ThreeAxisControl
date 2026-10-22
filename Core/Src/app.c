#include "app.h"
#include "protocol.h"
#include "motion.h"
#include "esp8266.h"
#include "main.h"
#include <stdio.h>

#define APP_TX_SLOTS 4u

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
static uint16_t   s_last_seq;
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

static bool app_push_status(void)
{
    uint8_t payload[PROTO_STATUS_PAYLOAD_LEN];
    const AxisState *x = Motion_GetAxis(AXIS_X);
    const AxisState *y = Motion_GetAxis(AXIS_Y);
    const AxisState *z = Motion_GetAxis(AXIS_Z);

    if (!x || !y || !z) return false;

    ProtoState state = s_state;
    if (state != PROTO_STATE_ESTOP && Motion_IsBusy()) {
        state = PROTO_STATE_MOVING;
    }

    payload[0] = (uint8_t)state;
    payload[1] = PROTO_QUEUE_FREE_NONE;
    Proto_WriteU32(&payload[2], (uint32_t)x->position);
    Proto_WriteU32(&payload[6], (uint32_t)y->position);
    Proto_WriteU32(&payload[10], (uint32_t)z->position);
    Proto_WriteU16(&payload[14], s_last_seq);

    return app_tx_push(PROTO_CMD_STATUS, payload, sizeof(payload));
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

    s_last_seq = seq;

    printf("LINE x=%ld y=%ld z=%ld feed=%lu seq=%u\r\n",
           (long)x, (long)y, (long)z, (unsigned long)feed, (unsigned)seq);

    if (s_state == PROTO_STATE_ESTOP) {
        app_send_ack(seq, PROTO_ACK_BUSY);
        return;
    }

    if (feed == 0u || feed > MAX_STEP_HZ) {
        app_send_ack(seq, PROTO_ACK_BAD_PARAM);
        return;
    }

    app_send_ack(seq, PROTO_ACK_OK);
    s_status_pending = true;
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
        Motion_Stop();
        s_state = PROTO_STATE_IDLE;
        printf("STOP\r\n");
        app_send_ack(PROTO_SEQ_NONE, PROTO_ACK_OK);
        break;

    case PROTO_CMD_ESTOP:
        if (len != 0u) {
            app_send_ack(PROTO_SEQ_NONE, PROTO_ACK_BAD_LENGTH);
            break;
        }
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
    s_last_seq = PROTO_SEQ_NONE;
    s_state = PROTO_STATE_IDLE;
    s_status_pending = false;
}

void App_OnLinkReset(void)
{
    FrameParser_Reset(&s_parser);
    s_tx_head = 0;
    s_tx_tail = 0;
    s_tx_count = 0;
    s_tx_retry_tick = 0;
    s_last_seq = PROTO_SEQ_NONE;
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
    if (s_status_pending) {
        if (app_push_status()) {
            s_status_pending = false;
            printf("STATUS queued\r\n");
        }
    }
    app_tx_pump();
}
