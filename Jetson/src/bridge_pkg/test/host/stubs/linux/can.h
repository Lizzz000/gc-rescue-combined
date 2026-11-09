#pragma once

// 生产协议头包含 <linux/can.h>。Windows host 没有这份内核头。
// 只提供编解码实际用到的类型和标志，布局不需要和 Linux 内核二进制兼容。
#include <stdint.h>

typedef uint32_t canid_t;

#define CAN_EFF_FLAG 0x80000000U
#define CAN_RTR_FLAG 0x40000000U
#define CAN_ERR_FLAG 0x20000000U
#define CAN_SFF_MASK 0x000007FFU

struct can_frame {
  canid_t can_id;
  uint8_t len;
  uint8_t pad0;
  uint8_t res0;
  uint8_t len8_dlc;
  uint8_t data[8];
};

struct can_filter {
  canid_t can_id;
  canid_t can_mask;
};
