#pragma once
// MESSAGE STM32_F3_COMMAND PACKING

#define MAVLINK_MSG_ID_STM32_F3_COMMAND 500

MAVPACKED(
typedef struct __mavlink_stm32_f3_command_t {
 uint8_t command; /*< Command message sending from STM32_F3.*/
 uint8_t param; /*<  parameter for debug.*/
 char f3_log[100]; /*< Log message sending from f3.*/
}) mavlink_stm32_f3_command_t;

#define MAVLINK_MSG_ID_STM32_F3_COMMAND_LEN 102
#define MAVLINK_MSG_ID_STM32_F3_COMMAND_MIN_LEN 102
#define MAVLINK_MSG_ID_500_LEN 102
#define MAVLINK_MSG_ID_500_MIN_LEN 102

#define MAVLINK_MSG_ID_STM32_F3_COMMAND_CRC 79
#define MAVLINK_MSG_ID_500_CRC 79

#define MAVLINK_MSG_STM32_F3_COMMAND_FIELD_F3_LOG_LEN 100

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_STM32_F3_COMMAND { \
    500, \
    "STM32_F3_COMMAND", \
    3, \
    {  { "command", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_stm32_f3_command_t, command) }, \
         { "param", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_stm32_f3_command_t, param) }, \
         { "f3_log", NULL, MAVLINK_TYPE_CHAR, 100, 2, offsetof(mavlink_stm32_f3_command_t, f3_log) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_STM32_F3_COMMAND { \
    "STM32_F3_COMMAND", \
    3, \
    {  { "command", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_stm32_f3_command_t, command) }, \
         { "param", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_stm32_f3_command_t, param) }, \
         { "f3_log", NULL, MAVLINK_TYPE_CHAR, 100, 2, offsetof(mavlink_stm32_f3_command_t, f3_log) }, \
         } \
}
#endif

/**
 * @brief Pack a stm32_f3_command message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param command Command message sending from STM32_F3.
 * @param param  parameter for debug.
 * @param f3_log Log message sending from f3.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_stm32_f3_command_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t command, uint8_t param, const char *f3_log)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STM32_F3_COMMAND_LEN];
    _mav_put_uint8_t(buf, 0, command);
    _mav_put_uint8_t(buf, 1, param);
    _mav_put_char_array(buf, 2, f3_log, 100);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STM32_F3_COMMAND_LEN);
#else
    mavlink_stm32_f3_command_t packet;
    packet.command = command;
    packet.param = param;
    mav_array_memcpy(packet.f3_log, f3_log, sizeof(char)*100);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STM32_F3_COMMAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STM32_F3_COMMAND;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_STM32_F3_COMMAND_MIN_LEN, MAVLINK_MSG_ID_STM32_F3_COMMAND_LEN, MAVLINK_MSG_ID_STM32_F3_COMMAND_CRC);
}

/**
 * @brief Pack a stm32_f3_command message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param command Command message sending from STM32_F3.
 * @param param  parameter for debug.
 * @param f3_log Log message sending from f3.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_stm32_f3_command_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t command,uint8_t param,const char *f3_log)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STM32_F3_COMMAND_LEN];
    _mav_put_uint8_t(buf, 0, command);
    _mav_put_uint8_t(buf, 1, param);
    _mav_put_char_array(buf, 2, f3_log, 100);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STM32_F3_COMMAND_LEN);
#else
    mavlink_stm32_f3_command_t packet;
    packet.command = command;
    packet.param = param;
    mav_array_memcpy(packet.f3_log, f3_log, sizeof(char)*100);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STM32_F3_COMMAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STM32_F3_COMMAND;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_STM32_F3_COMMAND_MIN_LEN, MAVLINK_MSG_ID_STM32_F3_COMMAND_LEN, MAVLINK_MSG_ID_STM32_F3_COMMAND_CRC);
}

/**
 * @brief Encode a stm32_f3_command struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param stm32_f3_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_stm32_f3_command_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_stm32_f3_command_t* stm32_f3_command)
{
    return mavlink_msg_stm32_f3_command_pack(system_id, component_id, msg, stm32_f3_command->command, stm32_f3_command->param, stm32_f3_command->f3_log);
}

/**
 * @brief Encode a stm32_f3_command struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param stm32_f3_command C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_stm32_f3_command_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_stm32_f3_command_t* stm32_f3_command)
{
    return mavlink_msg_stm32_f3_command_pack_chan(system_id, component_id, chan, msg, stm32_f3_command->command, stm32_f3_command->param, stm32_f3_command->f3_log);
}

/**
 * @brief Send a stm32_f3_command message
 * @param chan MAVLink channel to send the message
 *
 * @param command Command message sending from STM32_F3.
 * @param param  parameter for debug.
 * @param f3_log Log message sending from f3.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_stm32_f3_command_send(mavlink_channel_t chan, uint8_t command, uint8_t param, const char *f3_log)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STM32_F3_COMMAND_LEN];
    _mav_put_uint8_t(buf, 0, command);
    _mav_put_uint8_t(buf, 1, param);
    _mav_put_char_array(buf, 2, f3_log, 100);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STM32_F3_COMMAND, buf, MAVLINK_MSG_ID_STM32_F3_COMMAND_MIN_LEN, MAVLINK_MSG_ID_STM32_F3_COMMAND_LEN, MAVLINK_MSG_ID_STM32_F3_COMMAND_CRC);
#else
    mavlink_stm32_f3_command_t packet;
    packet.command = command;
    packet.param = param;
    mav_array_memcpy(packet.f3_log, f3_log, sizeof(char)*100);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STM32_F3_COMMAND, (const char *)&packet, MAVLINK_MSG_ID_STM32_F3_COMMAND_MIN_LEN, MAVLINK_MSG_ID_STM32_F3_COMMAND_LEN, MAVLINK_MSG_ID_STM32_F3_COMMAND_CRC);
#endif
}

/**
 * @brief Send a stm32_f3_command message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_stm32_f3_command_send_struct(mavlink_channel_t chan, const mavlink_stm32_f3_command_t* stm32_f3_command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_stm32_f3_command_send(chan, stm32_f3_command->command, stm32_f3_command->param, stm32_f3_command->f3_log);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STM32_F3_COMMAND, (const char *)stm32_f3_command, MAVLINK_MSG_ID_STM32_F3_COMMAND_MIN_LEN, MAVLINK_MSG_ID_STM32_F3_COMMAND_LEN, MAVLINK_MSG_ID_STM32_F3_COMMAND_CRC);
#endif
}

#if MAVLINK_MSG_ID_STM32_F3_COMMAND_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_stm32_f3_command_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t command, uint8_t param, const char *f3_log)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, command);
    _mav_put_uint8_t(buf, 1, param);
    _mav_put_char_array(buf, 2, f3_log, 100);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STM32_F3_COMMAND, buf, MAVLINK_MSG_ID_STM32_F3_COMMAND_MIN_LEN, MAVLINK_MSG_ID_STM32_F3_COMMAND_LEN, MAVLINK_MSG_ID_STM32_F3_COMMAND_CRC);
#else
    mavlink_stm32_f3_command_t *packet = (mavlink_stm32_f3_command_t *)msgbuf;
    packet->command = command;
    packet->param = param;
    mav_array_memcpy(packet->f3_log, f3_log, sizeof(char)*100);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STM32_F3_COMMAND, (const char *)packet, MAVLINK_MSG_ID_STM32_F3_COMMAND_MIN_LEN, MAVLINK_MSG_ID_STM32_F3_COMMAND_LEN, MAVLINK_MSG_ID_STM32_F3_COMMAND_CRC);
#endif
}
#endif

#endif

// MESSAGE STM32_F3_COMMAND UNPACKING


/**
 * @brief Get field command from stm32_f3_command message
 *
 * @return Command message sending from STM32_F3.
 */
static inline uint8_t mavlink_msg_stm32_f3_command_get_command(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field param from stm32_f3_command message
 *
 * @return  parameter for debug.
 */
static inline uint8_t mavlink_msg_stm32_f3_command_get_param(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field f3_log from stm32_f3_command message
 *
 * @return Log message sending from f3.
 */
static inline uint16_t mavlink_msg_stm32_f3_command_get_f3_log(const mavlink_message_t* msg, char *f3_log)
{
    return _MAV_RETURN_char_array(msg, f3_log, 100,  2);
}

/**
 * @brief Decode a stm32_f3_command message into a struct
 *
 * @param msg The message to decode
 * @param stm32_f3_command C-struct to decode the message contents into
 */
static inline void mavlink_msg_stm32_f3_command_decode(const mavlink_message_t* msg, mavlink_stm32_f3_command_t* stm32_f3_command)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    stm32_f3_command->command = mavlink_msg_stm32_f3_command_get_command(msg);
    stm32_f3_command->param = mavlink_msg_stm32_f3_command_get_param(msg);
    mavlink_msg_stm32_f3_command_get_f3_log(msg, stm32_f3_command->f3_log);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_STM32_F3_COMMAND_LEN? msg->len : MAVLINK_MSG_ID_STM32_F3_COMMAND_LEN;
        memset(stm32_f3_command, 0, MAVLINK_MSG_ID_STM32_F3_COMMAND_LEN);
    memcpy(stm32_f3_command, _MAV_PAYLOAD(msg), len);
#endif
}
