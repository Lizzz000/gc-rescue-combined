#ifndef JETSON_CAN_H
#define JETSON_CAN_H

#include <stdint.h>

/* 来源：Jetson tools/source/can_codec.py 与 src/bridge_pkg/protocol/can_protocol.hpp。
 * STM32 接收快照由 CAN 中断写入、任务临界区读取。
 * received_ms 使用现有 1 kHz FreeRTOS 时基，不是总线事务编号。
 */
#define JETSON_CAN_HEARTBEAT_ID           0x180U
#define JETSON_CAN_TARGET_CENTER_ID       0x181U
#define JETSON_CAN_SAFETY_CENTER_ID       0x182U
#define JETSON_CAN_ARRIVAL_REPLY_ID       0x190U
#define JETSON_CAN_ROI_REPLY_ID           0x191U
#define JETSON_CAN_COMMAND_REPLY_ID       0x192U
#define JETSON_CAN_TURN_REQUEST_ID        0x193U
#define JETSON_CAN_VELOCITY_DLC           7U
/* 0x180 心跳：前 7 字节沿用速度帧布局，data[7] 为通用 pause（0/1）。 */
#define JETSON_CAN_HEARTBEAT_DLC          8U
#define JETSON_CAN_HEARTBEAT_PAUSE_BYTE   7U
#define JETSON_CAN_ARRIVAL_ID             0x200U
#define JETSON_CAN_TURN_COMPLETED_ID      0x203U
#define JETSON_CAN_SAFETY_ARRIVED_ID      0x204U
#define JETSON_CAN_START_SEARCH_ID        0x205U
#define JETSON_CAN_DELIVERY_COMPLETED_ID  0x206U
/* 0x208 到点：data[0] 为到点类型，由 0x192 命令 5（token=0）受理。 */
#define JETSON_CAN_POINT_REACHED_ID       0x208U
#define JETSON_CAN_POINT_GRASP            1U
#define JETSON_CAN_POINT_RELEASE          2U
#define JETSON_CAN_POINT_REPLY_COMMAND    5U

/* 速度载荷：valid:uint8 + vx/vy/wz:int16 小端，共 7 字节；手动 0x17F 即此载荷（DLC 7），0x180 心跳为此 7 字节加 data[7] pause（DLC 8）。 */
typedef struct
{
    uint8_t valid;       /**< 格式合法且有效字节为 1。 */
    int16_t vx, vy, wz;  /**< 原始定点值：vx/vy 为 0.001 m/s，wz 为 0.001 rad/s；无效时为 0。 */
} Jetson_CAN_Velocity_Frame_t;

typedef struct
{
    uint8_t valid;
    int16_t x, y;        /**< 图像归一化中心坐标乘 10000；不在协议层裁剪。 */
    uint32_t received_ms;
    uint32_t sequence;
    uint32_t source_frame; /**< 源帧时间的低 24 位毫秒，用于区分周期重发。 */
} Jetson_CAN_Center_t;

typedef struct
{
    uint8_t valid;       /**< 结构合法标记；accepted=0 仍是合法拒绝回复。 */
    uint8_t accepted;
    uint32_t received_ms;
    uint32_t sequence;
} Jetson_CAN_Arrival_Ack_t;

typedef struct
{
    uint8_t valid;
    uint8_t passed;
    uint8_t destination; /**< 0 未定，1 左区，2 右区；通过时必须为 1 或 2。 */
    uint32_t received_ms;
    uint32_t sequence;
} Jetson_CAN_ROI_Reply_t;

typedef struct
{
    uint8_t valid;       /**< 结构合法标记；accepted=0 仍是合法拒绝回复。 */
    uint8_t accepted;
    uint8_t command;     /**< 1 转向完成，2 安全区到达，3 重新搜索，4 投送完成，5 到点受理。 */
    uint32_t token;      /**< 0..UINT32_MAX 均合法；命令 3、5 必须为 0。 */
    uint32_t received_ms;
    uint32_t sequence;
} Jetson_CAN_Command_Reply_t;

typedef struct
{
    uint8_t valid;       /**< 合法请求字节固定为 1；本标记不直接授权底盘执行。 */
    uint8_t destination; /**< 1 左区，2 右区。 */
    uint32_t token;      /**< 本趟 ROI 请求号低 32 位，允许 0。 */
    uint32_t received_ms;
    uint32_t sequence;
} Jetson_CAN_Turn_Request_t;

typedef struct
{
    Jetson_CAN_Center_t target, safety;
    Jetson_CAN_Arrival_Ack_t arrival_ack;
    Jetson_CAN_ROI_Reply_t roi_reply;
    Jetson_CAN_Command_Reply_t command_reply; /**< 等待时按请求匹配并保留首个受理回执。 */
    Jetson_CAN_Turn_Request_t turn_to_safety;
    uint32_t rx_sequence; /**< 每条已知 ID 递增并自然回绕；各快照记录接收时的值。 */
    uint32_t heartbeat_received_ms; /**< 最近一条格式合法的 0x180 到达时刻，RTOS ms。 */
    uint32_t heartbeat_sequence;    /**< 最近一条格式合法的 0x180 的接收序号。 */
    uint8_t heartbeat_seen;         /**< 已收到格式合法的 0x180。 */
    uint8_t heartbeat_pause;        /**< 最近合法心跳的通用 pause；1 时不开始下一段运动。 */
    uint32_t mission_loss_sequence; /**< 空 0x180 失联标志计数，防止被紧随的心跳覆盖。 */
} Jetson_CAN_Data_t;

/**
 * @brief 清空 Jetson 接收状态；由任务在启动 CAN 接收前调用。
 * @return 无；所有有效标记、时间戳及序号清零。
 */
void Jetson_CAN_Init(void);

/**
 * @brief 判断速度帧格式是否合法：DLC 为 7、有效字节为 0 或 1。
 * @param[in] data 可读 len 字节；空指针视为不合法。
 * @param[in] len CAN DLC。
 * @return 合法返回 1，否则 0；valid=0 的周期帧也算合法。
 */
uint8_t Jetson_CAN_Is_Velocity_Frame(const uint8_t *data, uint8_t len);

/**
 * @brief 解码速度帧的有效位与三个定点分量，不做缩放和限幅。
 * @param[in] data 可读 len 字节；空指针按畸形帧处理。
 * @param[in] len CAN DLC。
 * @param[out] frame 非空；格式不合法或 valid=0 时整帧清零。
 * @return 无。
 */
void Jetson_CAN_Decode_Velocity(const uint8_t *data, uint8_t len, Jetson_CAN_Velocity_Frame_t *frame);

/**
 * @brief 在 CAN RX 中断内解析单帧；不调用底盘、不排队、不发送回复。
 * @param[in] id 已由 BSP 核验的 11 位标准数据帧 ID。
 * @param[in] data 可读 len 字节；空指针按对应 ID 的畸形帧处理。
 * @param[in] len CAN DLC，必须精确匹配协议；0x180 只认 DLC 8 心跳，空载荷表示任务失联。
 * @return 无；未知 ID（含手动 0x17F）不改变快照，已知坏帧按对应报文处理。
 */
void Jetson_CAN_Receive(uint16_t id, const uint8_t *data, uint8_t len);

/**
 * @brief 首次提交命令后设置 0x192 匹配，丢弃前一请求的回执快照。
 * @param[in] request_id 上行 0x203 至 0x206 或 0x208，传 0 结束等待并恢复最新帧快照。
 * @param[in] token 本趟 ROI 请求号低 32 位，0x205 与 0x208 固定按无 token 匹配。
 * @return 无；活动期只更新合法匹配帧，首个受理帧保留到本次等待结束。
 * @note 由自动任务在首次成功提交的同一临界区内设置，重试时不重新设置。
 */
void Jetson_CAN_Expect_Command_Reply(uint16_t request_id, uint32_t token);

/**
 * @brief 在任务临界区复制完整接收快照，屏蔽现有优先级 6 的 CAN RX 中断。
 * @param[out] out 非空时写入快照；空指针不执行复制。
 * @return 无；调用方负责时效检查及控制授权，不能把回复标记当运动许可。
 */
void Jetson_CAN_Get_Data(Jetson_CAN_Data_t *out);

/**
 * @brief 编码 STM32 上行运输请求的 CAN 载荷。
 * @param[in] id 运输请求的标准帧 ID。
 * @param[in] argument 0x203/0x204/0x206 为本趟 ROI 请求号低 32 位；0x208 为到点类型
 *            JETSON_CAN_POINT_GRASP 或 JETSON_CAN_POINT_RELEASE；0x200/0x205 不使用。
 * @param[out] data 非空时须能写 8 字节，DLC 以外清零以适配 HAL 的整缓冲读取。
 * @return 成功返回 DLC 1 或 5；未知 ID、非法到点类型或空指针返回 0，不修改缓冲区。
 */
uint8_t Jetson_CAN_Encode_Uplink(uint16_t id, uint32_t argument, uint8_t data[8]);

#endif
