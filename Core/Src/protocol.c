#include "protocol.h"
#include <string.h>

static uint16_t crc16_update(uint16_t crc, uint8_t byte)
{
    crc ^= (uint16_t)((uint16_t)byte << 8);
    for (uint8_t i = 0; i < 8u; i++) {
        if (crc & 0x8000u) {
            crc = (uint16_t)((uint16_t)(crc << 1) ^ 0x1021u);
        } else {
            crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

uint16_t Crc16CcittFalse(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    if (!data) return crc;
    for (uint16_t i = 0; i < len; i++) {
        crc = crc16_update(crc, data[i]);
    }
    return crc;
}

uint16_t Proto_ReadU16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

uint32_t Proto_ReadU32(const uint8_t *p)
{
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

void Proto_WriteU16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

void Proto_WriteU32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

uint16_t Frame_Encode(uint8_t cmd, const uint8_t *payload, uint16_t len,
                      uint8_t *out, uint16_t out_size)
{
    if (!out || len > PROTO_MAX_PAYLOAD) return 0;
    if (len > 0u && !payload) return 0;

    uint16_t total = (uint16_t)(PROTO_HEADER_SIZE + len + PROTO_CRC_SIZE);
    if (total > out_size) return 0;

    out[0] = PROTO_SOF1;
    out[1] = PROTO_SOF2;
    out[2] = cmd;
    Proto_WriteU16(&out[3], len);
    if (len > 0u) {
        memcpy(&out[5], payload, len);
    }

    uint16_t crc = Crc16CcittFalse(&out[2], (uint16_t)(3u + len));
    Proto_WriteU16(&out[5 + len], crc);
    return total;
}

void FrameParser_Reset(FrameParser *parser)
{
    if (!parser) return;
    parser->state = FRAME_WAIT_SOF1;
    parser->cmd = 0;
    parser->len = 0;
    parser->idx = 0;
    parser->crc_calc = 0;
    parser->crc_recv = 0;
}

void FrameParser_Init(FrameParser *parser, FrameHandler handler)
{
    if (!parser) return;
    memset(parser, 0, sizeof(*parser));
    parser->handler = handler;
    FrameParser_Reset(parser);
}

static void parser_abort(FrameParser *parser, uint8_t byte)
{
    parser->frames_bad++;
    parser->len = 0;
    parser->idx = 0;
    parser->state = (byte == PROTO_SOF1) ? FRAME_WAIT_SOF2 : FRAME_WAIT_SOF1;
}

void FrameParser_Feed(FrameParser *parser, uint8_t byte)
{
    if (!parser) return;

    switch (parser->state) {
    case FRAME_WAIT_SOF1:
        if (byte == PROTO_SOF1) parser->state = FRAME_WAIT_SOF2;
        break;

    case FRAME_WAIT_SOF2:
        if (byte == PROTO_SOF2) {
            parser->state = FRAME_WAIT_CMD;
        } else if (byte != PROTO_SOF1) {
            parser->state = FRAME_WAIT_SOF1;
        }
        break;

    case FRAME_WAIT_CMD:
        parser->cmd = byte;
        parser->crc_calc = crc16_update(0xFFFFu, byte);
        parser->state = FRAME_WAIT_LEN_LO;
        break;

    case FRAME_WAIT_LEN_LO:
        parser->len = byte;
        parser->crc_calc = crc16_update(parser->crc_calc, byte);
        parser->state = FRAME_WAIT_LEN_HI;
        break;

    case FRAME_WAIT_LEN_HI:
        parser->len |= (uint16_t)((uint16_t)byte << 8);
        parser->crc_calc = crc16_update(parser->crc_calc, byte);
        if (parser->len > PROTO_MAX_PAYLOAD) {
            parser_abort(parser, byte);
            break;
        }
        parser->idx = 0;
        parser->state = (parser->len == 0u) ? FRAME_WAIT_CRC_LO : FRAME_WAIT_DATA;
        break;

    case FRAME_WAIT_DATA:
        parser->payload[parser->idx++] = byte;
        parser->crc_calc = crc16_update(parser->crc_calc, byte);
        if (parser->idx >= parser->len) {
            parser->state = FRAME_WAIT_CRC_LO;
        }
        break;

    case FRAME_WAIT_CRC_LO:
        parser->crc_recv = byte;
        parser->state = FRAME_WAIT_CRC_HI;
        break;

    case FRAME_WAIT_CRC_HI:
        parser->crc_recv |= (uint16_t)((uint16_t)byte << 8);
        if (parser->crc_recv == parser->crc_calc) {
            uint8_t cmd = parser->cmd;
            uint16_t len = parser->len;
            FrameHandler handler = parser->handler;
            parser->frames_ok++;
            FrameParser_Reset(parser);
            if (handler) handler(cmd, parser->payload, len);
        } else {
            parser_abort(parser, byte);
        }
        break;
    }
}
