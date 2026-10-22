#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PROTO_SOF1              0xAAu
#define PROTO_SOF2              0x55u
#define PROTO_MAX_PAYLOAD       64u
#define PROTO_HEADER_SIZE       5u
#define PROTO_CRC_SIZE          2u
#define PROTO_MAX_FRAME         (PROTO_HEADER_SIZE + PROTO_MAX_PAYLOAD + PROTO_CRC_SIZE)

#define PROTO_LINE_PAYLOAD_LEN   18u
#define PROTO_STATUS_PAYLOAD_LEN 16u
#define PROTO_ACK_PAYLOAD_LEN    3u

#define PROTO_SEQ_NONE          0xFFFFu
#define PROTO_QUEUE_FREE_NONE   0xFFu

typedef enum {
    PROTO_CMD_LINE    = 0x01,
    PROTO_CMD_STOP    = 0x02,
    PROTO_CMD_ESTOP   = 0x03,
    PROTO_CMD_HOME    = 0x04,
    PROTO_CMD_QUERY   = 0x05,
    PROTO_CMD_STATUS  = 0x81,
    PROTO_CMD_ACK     = 0x82,
} ProtoCmd;

typedef enum {
    PROTO_ACK_OK          = 0,
    PROTO_ACK_QUEUE_FULL  = 1,
    PROTO_ACK_BAD_LENGTH  = 2,
    PROTO_ACK_BAD_PARAM   = 3,
    PROTO_ACK_UNSUPPORTED = 4,
    PROTO_ACK_BUSY        = 5,
} ProtoAckCode;

typedef enum {
    PROTO_STATE_IDLE  = 0,
    PROTO_STATE_MOVING = 1,
    PROTO_STATE_ESTOP = 2,
    PROTO_STATE_ERROR = 3,
} ProtoState;

uint16_t Crc16CcittFalse(const uint8_t *data, uint16_t len);

uint16_t Proto_ReadU16(const uint8_t *p);
uint32_t Proto_ReadU32(const uint8_t *p);
void     Proto_WriteU16(uint8_t *p, uint16_t v);
void     Proto_WriteU32(uint8_t *p, uint32_t v);

uint16_t Frame_Encode(uint8_t cmd, const uint8_t *payload, uint16_t len,
                      uint8_t *out, uint16_t out_size);

typedef enum {
    FRAME_WAIT_SOF1 = 0,
    FRAME_WAIT_SOF2,
    FRAME_WAIT_CMD,
    FRAME_WAIT_LEN_LO,
    FRAME_WAIT_LEN_HI,
    FRAME_WAIT_DATA,
    FRAME_WAIT_CRC_LO,
    FRAME_WAIT_CRC_HI,
} FrameParseState;

typedef void (*FrameHandler)(uint8_t cmd, const uint8_t *payload, uint16_t len);

typedef struct {
    FrameParseState state;
    uint8_t  cmd;
    uint16_t len;
    uint16_t idx;
    uint16_t crc_calc;
    uint16_t crc_recv;
    uint8_t  payload[PROTO_MAX_PAYLOAD];
    FrameHandler handler;
    uint32_t frames_ok;
    uint32_t frames_bad;
} FrameParser;

void FrameParser_Init(FrameParser *parser, FrameHandler handler);
void FrameParser_Reset(FrameParser *parser);
void FrameParser_Feed(FrameParser *parser, uint8_t byte);

#ifdef __cplusplus
}
#endif

#endif
