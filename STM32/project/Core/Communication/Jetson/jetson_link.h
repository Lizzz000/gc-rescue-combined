#ifndef JETSON_CAN_H
#define JETSON_CAN_H

#include <stdint.h>

/* 板间类型号。快照由 USB 任务写入、控制任务临界区读取。
 * received_ms 使用现有 1 kHz FreeRTOS 时基，不是总线事务编号。
 * 自动固件忽略类型 2；类型 1 已删除。
 */
#define JETSON_CAN_TARGET_CENTER_ID       3U
#define JETSON_CAN_SAFETY_CENTER_ID       4U
#define JETSON_CAN_ARRIVAL_REPLY_ID       5U
#define JETSON_CAN_ROI_REPLY_ID           6U
#define JETSON_CAN_COMMAND_REPLY_ID       7U
#define JETSON_CAN_TURN_REQUEST_ID        8U
#define JETSON_CAN_ACTION_RESULT_ID       9U
#define JETSON_CAN_VELOCITY_DLC           7U
#define JETSON_CAN_ARRIVAL_ID             16U
#define JETSON_CAN_TURN_COMPLETED_ID      17U
#define JETSON_CAN_SAFETY_ARRIVED_ID      18U
#define JETSON_CAN_START_SEARCH_ID        19U
#define JETSON_CAN_DELIVERY_COMPLETED_ID  20U
/* 类型 21：kind + 非零 request_id。命令 5 回显编号，仅表示受理；类型 9 报动作终态。 */
#define JETSON_CAN_POINT_REACHED_ID       21U
#define JETSON_CAN_POINT_STARTUP          0U
#define JETSON_CAN_POINT_GRASP            1U
#define JETSON_CAN_POINT_RELEASE          2U
#define JETSON_CAN_POINT_REJECT_OPEN      3U
#define JETSON_CAN_POINT_REPLY_COMMAND    5U

/* 手动类型 2 的速度载荷：valid:uint8 + vx/vy/wz:int16 小端，共 7 字节。 */
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
    uint32_t sequence;
} Jetson_CAN_Arrival_Ack_t;

typedef struct
{
    uint8_t valid;
    uint8_t passed;
    uint8_t destination; /**< 0 未定，1 左区，2 右区；通过时必须为 1 或 2。 */
    uint32_t sequence;
} Jetson_CAN_ROI_Reply_t;

typedef struct
{
    uint8_t valid;       /**< 结构合法标记；accepted=0 仍是合法拒绝回复。 */
    uint8_t accepted;
    uint8_t command;     /**< 1 转向完成，2 安全区到达，3 重新搜索，4 投送完成，5 到点受理。 */
    uint32_t token;      /**< 命令 3/5 回显非零请求编号，其余为允许全范围的路线身份。 */
    uint32_t sequence;
} Jetson_CAN_Command_Reply_t;

typedef struct
{
    uint8_t valid;       /**< 合法请求字节固定为 1；本标记不直接授权底盘执行。 */
    uint8_t destination; /**< 1 左区，2 右区。 */
    uint32_t token;      /**< 本趟 ROI 请求号低 32 位，允许 0。 */
    uint32_t sequence;
} Jetson_CAN_Turn_Request_t;

typedef struct
{
    uint8_t valid;
    uint8_t kind;        /**< 0 启动张爪查询，1 夹取，2 释放，3 拒绝后张开。 */
    uint8_t success;     /**< 0 失败，1 成功；两者均为不可变终态。 */
    uint32_t request_id;
    uint32_t sequence;
} Jetson_CAN_Action_Result_t;

typedef struct
{
    Jetson_CAN_Center_t target, safety;
    Jetson_CAN_Arrival_Ack_t arrival_ack;
    Jetson_CAN_ROI_Reply_t roi_reply;
    Jetson_CAN_Command_Reply_t command_reply; /**< 等待时按请求匹配并保留首个受理回执。 */
    Jetson_CAN_Turn_Request_t turn_to_safety;
    Jetson_CAN_Action_Result_t action_result;
    uint32_t rx_sequence; /**< 每条已知下行类型递增并自然回绕；各快照记录接收时的值。 */
    uint32_t periodic_received_ms; /**< 最近格式合法的类型 3/4 到达时刻，RTOS ms；valid=0 也刷新。 */
    uint8_t periodic_received; /**< 已收到合法中心周期帧；与目标是否可见分开判断。 */
} Jetson_CAN_Data_t;

/**
 * @brief 清空 Jetson 接收状态；由控制任务在初始化自动流程前调用。
 * @return 无；所有有效标记、时间戳及序号清零。
 */
void Jetson_CAN_Init(void);

/**
 * @brief USB 会话中断时撤销周期流标记，旧会话快照不能授权重新启动。
 * @return 无；只清单字节标记，保留其余快照。USB 中断或其屏蔽临界区可调用。
 */
void Jetson_CAN_Invalidate_Periodic(void);

/**
 * @brief 判断速度载荷格式是否合法：长度为 7、有效字节为 0 或 1。
 * @param[in] data 可读 len 字节；空指针视为不合法。
 * @param[in] len USB 板间帧的载荷字节数。
 * @return 合法返回 1，否则 0；valid=0 的周期帧也算合法。
 */
uint8_t Jetson_CAN_Is_Velocity_Frame(const uint8_t *data, uint8_t len);

/**
 * @brief 解码速度帧的有效位与三个定点分量，不做缩放和限幅。
 * @param[in] data 可读 len 字节；空指针按畸形帧处理。
 * @param[in] len USB 板间帧的载荷字节数。
 * @param[out] frame 非空；格式不合法或 valid=0 时整帧清零。
 * @return 无。
 */
void Jetson_CAN_Decode_Velocity(const uint8_t *data, uint8_t len, Jetson_CAN_Velocity_Frame_t *frame);

/**
 * @brief 在 USB 任务内解析单帧；不调用底盘、不排队、不发送回复。
 * @param[in] type 下行类型号。类型 1、2 对本模块是未知类型。
 * @param[in] data 可读 len 字节；空指针按对应类型的畸形帧处理。
 * @param[in] len 载荷长度，必须精确匹配协议。
 * @return 无；未知类型不改变快照，已知坏帧按对应报文处理。
 */
void Jetson_CAN_Receive(uint16_t type, const uint8_t *data, uint8_t len);

/**
 * @brief 首次提交命令后设置类型 7 匹配，丢弃前一请求的回执快照。
 * @param[in] request_type 上行类型 17 至 20 或 21，传 0 结束等待并恢复最新帧快照。
 * @param[in] token 本趟 ROI 请求号或类型 19/21 非零请求编号。
 * @return 无；活动期只更新合法匹配帧，首个受理帧保留到本次等待结束。
 * @note 由自动任务在首次成功提交的同一临界区内设置，重试时不重新设置。
 */
void Jetson_CAN_Expect_Command_Reply(uint16_t request_type, uint32_t token);

/** @brief 首次成功提交动作时匹配 kind/id 并清旧终态；id=0 结束等待。
 * @note 与首次发送在同一临界区设置；重试不重置。成功、失败均保留首个匹配终态。
 */
void Jetson_CAN_Expect_Action_Result(uint8_t kind, uint32_t request_id);

/**
 * @brief 在任务临界区复制完整接收快照，防止复制期间 USB 接收任务更新数据。
 * @param[out] out 非空时写入快照；空指针不执行复制。
 * @return 无；调用方负责时效检查及控制授权，不能把回复标记当运动许可。
 */
void Jetson_CAN_Get_Data(Jetson_CAN_Data_t *out);

/**
 * @brief 编码 STM32 上行运输请求的载荷。
 * @param[in] type 上行类型号。
 * @param[in] argument 类型 17/18/20 为本趟 ROI 请求号；类型 19 为非零请求编号；类型 16 不使用。
 * @param[out] data 非空时须能写 8 字节，长度以外清零。
 * @return 成功返回长度 1 或 5；类型不在 16..20、类型 19 编号为 0 或空指针返回 0，不修改缓冲区。
 */
uint8_t Jetson_CAN_Encode_Uplink(uint16_t type, uint32_t argument, uint8_t data[8]);

/** @brief 编码类型 21，kind=0..3、request_id 非零；成功返回 5，否则 0且不改缓冲区。 */
uint8_t Jetson_CAN_Encode_Action_Request(uint8_t kind, uint32_t request_id, uint8_t data[8]);

#endif
