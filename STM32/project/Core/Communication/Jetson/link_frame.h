#ifndef LINK_FRAME_H
#define LINK_FRAME_H

#include <stdint.h>

/* USB 全速字节流上的一帧。布局：A5、版本 01、类型、长度、载荷、CRC-16/IBM 小端。
 * CRC 覆盖版本、类型、长度和载荷。最大载荷 8。长度 0 只是空载荷，不是失联。
 */

#define LINK_FRAME_SOF 0xA5U
#define LINK_FRAME_VERSION 0x01U
#define LINK_FRAME_MAX_PAYLOAD 8U
#define LINK_FRAME_MAX_BYTES (6U + LINK_FRAME_MAX_PAYLOAD)

typedef struct
{
    uint8_t type;
    uint8_t length;
    uint8_t data[LINK_FRAME_MAX_PAYLOAD];
} LinkFrame;

typedef struct
{
    uint8_t buffer[LINK_FRAME_MAX_BYTES];
    uint8_t count;
} LinkParser;

/** @brief 组一帧。载荷超过 8 字节返回 0，不写输出。 */
uint8_t LinkFrame_Encode(uint8_t type, const uint8_t *data, uint8_t length, uint8_t *out);

/** @brief 喂一个字节。凑齐且 CRC 正确时写出一帧并返回 1；半包返回 0。CRC 错则丢掉一个字节再同步。 */
uint8_t LinkParser_Feed(LinkParser *parser, uint8_t byte, LinkFrame *out);

#endif
