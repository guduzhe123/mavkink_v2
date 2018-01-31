#pragma once
// MESSAGE STM32_F3_MOTOR_CURR PACKING

#define MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR 501

MAVPACKED(
typedef struct __mavlink_stm32_f3_motor_curr_t {
 float motor_curr[10]; /*< Current message of each motor.*/
}) mavlink_stm32_f3_motor_curr_t;

#define MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR_LEN 40
#define MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR_MIN_LEN 40
#define MAVLINK_MSG_ID_501_LEN 40
#define MAVLINK_MSG_ID_501_MIN_LEN 40

#define MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR_CRC 39
#define MAVLINK_MSG_ID_501_CRC 39

#define MAVLINK_MSG_STM32_F3_MOTOR_CURR_FIELD_MOTOR_CURR_LEN 10

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_STM32_F3_MOTOR_CURR { \
    501, \
    "STM32_F3_MOTOR_CURR", \
    1, \
    {  { "motor_curr", NULL, MAVLINK_TYPE_FLOAT, 10, 0, offsetof(mavlink_stm32_f3_motor_curr_t, motor_curr) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_STM32_F3_MOTOR_CURR { \
    "STM32_F3_MOTOR_CURR", \
    1, \
    {  { "motor_curr", NULL, MAVLINK_TYPE_FLOAT, 10, 0, offsetof(mavlink_stm32_f3_motor_curr_t, motor_curr) }, \
         } \
}
#endif

/**
 * @brief Pack a stm32_f3_motor_curr message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param motor_curr Current message of each motor.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_stm32_f3_motor_curr_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const float *motor_curr)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR_LEN];

    _mav_put_float_array(buf, 0, motor_curr, 10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR_LEN);
#else
    mavlink_stm32_f3_motor_curr_t packet;

    mav_array_memcpy(packet.motor_curr, motor_curr, sizeof(float)*10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR_MIN_LEN, MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR_LEN, MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR_CRC);
}

/**
 * @brief Pack a stm32_f3_motor_curr message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param motor_curr Current message of each motor.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_stm32_f3_motor_curr_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const float *motor_curr)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR_LEN];

    _mav_put_float_array(buf, 0, motor_curr, 10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR_LEN);
#else
    mavlink_stm32_f3_motor_curr_t packet;

    mav_array_memcpy(packet.motor_curr, motor_curr, sizeof(float)*10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR_MIN_LEN, MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR_LEN, MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR_CRC);
}

/**
 * @brief Encode a stm32_f3_motor_curr struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param stm32_f3_motor_curr C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_stm32_f3_motor_curr_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_stm32_f3_motor_curr_t* stm32_f3_motor_curr)
{
    return mavlink_msg_stm32_f3_motor_curr_pack(system_id, component_id, msg, stm32_f3_motor_curr->motor_curr);
}

/**
 * @brief Encode a stm32_f3_motor_curr struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param stm32_f3_motor_curr C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_stm32_f3_motor_curr_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_stm32_f3_motor_curr_t* stm32_f3_motor_curr)
{
    return mavlink_msg_stm32_f3_motor_curr_pack_chan(system_id, component_id, chan, msg, stm32_f3_motor_curr->motor_curr);
}

/**
 * @brief Send a stm32_f3_motor_curr message
 * @param chan MAVLink channel to send the message
 *
 * @param motor_curr Current message of each motor.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_stm32_f3_motor_curr_send(mavlink_channel_t chan, const float *motor_curr)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR_LEN];

    _mav_put_float_array(buf, 0, motor_curr, 10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR, buf, MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR_MIN_LEN, MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR_LEN, MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR_CRC);
#else
    mavlink_stm32_f3_motor_curr_t packet;

    mav_array_memcpy(packet.motor_curr, motor_curr, sizeof(float)*10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR, (const char *)&packet, MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR_MIN_LEN, MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR_LEN, MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR_CRC);
#endif
}

/**
 * @brief Send a stm32_f3_motor_curr message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_stm32_f3_motor_curr_send_struct(mavlink_channel_t chan, const mavlink_stm32_f3_motor_curr_t* stm32_f3_motor_curr)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_stm32_f3_motor_curr_send(chan, stm32_f3_motor_curr->motor_curr);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR, (const char *)stm32_f3_motor_curr, MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR_MIN_LEN, MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR_LEN, MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR_CRC);
#endif
}

#if MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_stm32_f3_motor_curr_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const float *motor_curr)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mav_put_float_array(buf, 0, motor_curr, 10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR, buf, MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR_MIN_LEN, MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR_LEN, MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR_CRC);
#else
    mavlink_stm32_f3_motor_curr_t *packet = (mavlink_stm32_f3_motor_curr_t *)msgbuf;

    mav_array_memcpy(packet->motor_curr, motor_curr, sizeof(float)*10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR, (const char *)packet, MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR_MIN_LEN, MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR_LEN, MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR_CRC);
#endif
}
#endif

#endif

// MESSAGE STM32_F3_MOTOR_CURR UNPACKING


/**
 * @brief Get field motor_curr from stm32_f3_motor_curr message
 *
 * @return Current message of each motor.
 */
static inline uint16_t mavlink_msg_stm32_f3_motor_curr_get_motor_curr(const mavlink_message_t* msg, float *motor_curr)
{
    return _MAV_RETURN_float_array(msg, motor_curr, 10,  0);
}

/**
 * @brief Decode a stm32_f3_motor_curr message into a struct
 *
 * @param msg The message to decode
 * @param stm32_f3_motor_curr C-struct to decode the message contents into
 */
static inline void mavlink_msg_stm32_f3_motor_curr_decode(const mavlink_message_t* msg, mavlink_stm32_f3_motor_curr_t* stm32_f3_motor_curr)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_stm32_f3_motor_curr_get_motor_curr(msg, stm32_f3_motor_curr->motor_curr);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR_LEN? msg->len : MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR_LEN;
        memset(stm32_f3_motor_curr, 0, MAVLINK_MSG_ID_STM32_F3_MOTOR_CURR_LEN);
    memcpy(stm32_f3_motor_curr, _MAV_PAYLOAD(msg), len);
#endif
}
