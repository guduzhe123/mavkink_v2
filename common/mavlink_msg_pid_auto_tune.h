#pragma once
// MESSAGE PID_AUTO_TUNE PACKING

#define MAVLINK_MSG_ID_PID_AUTO_TUNE 600

MAVPACKED(
typedef struct __mavlink_pid_auto_tune_t {
 float j_pid_tune; /*< Judgment of pid performance.*/
 float e_rate; /*< error of rate.*/
 float e_ang; /*< error of angle.*/
 float control_u; /*< control output.*/
 float error_ang_bet; /*< error of two angles of previous and current  .*/
 float ang_act; /*< angle of current.*/
 float angle_sp; /*< angle of setpoint.*/
 float ang_pre; /*< angle of previous.*/
}) mavlink_pid_auto_tune_t;

#define MAVLINK_MSG_ID_PID_AUTO_TUNE_LEN 32
#define MAVLINK_MSG_ID_PID_AUTO_TUNE_MIN_LEN 32
#define MAVLINK_MSG_ID_600_LEN 32
#define MAVLINK_MSG_ID_600_MIN_LEN 32

#define MAVLINK_MSG_ID_PID_AUTO_TUNE_CRC 44
#define MAVLINK_MSG_ID_600_CRC 44



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PID_AUTO_TUNE { \
    600, \
    "PID_AUTO_TUNE", \
    8, \
    {  { "j_pid_tune", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_pid_auto_tune_t, j_pid_tune) }, \
         { "e_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_pid_auto_tune_t, e_rate) }, \
         { "e_ang", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_pid_auto_tune_t, e_ang) }, \
         { "control_u", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_pid_auto_tune_t, control_u) }, \
         { "error_ang_bet", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_pid_auto_tune_t, error_ang_bet) }, \
         { "ang_act", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_pid_auto_tune_t, ang_act) }, \
         { "angle_sp", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_pid_auto_tune_t, angle_sp) }, \
         { "ang_pre", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_pid_auto_tune_t, ang_pre) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PID_AUTO_TUNE { \
    "PID_AUTO_TUNE", \
    8, \
    {  { "j_pid_tune", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_pid_auto_tune_t, j_pid_tune) }, \
         { "e_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_pid_auto_tune_t, e_rate) }, \
         { "e_ang", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_pid_auto_tune_t, e_ang) }, \
         { "control_u", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_pid_auto_tune_t, control_u) }, \
         { "error_ang_bet", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_pid_auto_tune_t, error_ang_bet) }, \
         { "ang_act", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_pid_auto_tune_t, ang_act) }, \
         { "angle_sp", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_pid_auto_tune_t, angle_sp) }, \
         { "ang_pre", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_pid_auto_tune_t, ang_pre) }, \
         } \
}
#endif

/**
 * @brief Pack a pid_auto_tune message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param j_pid_tune Judgment of pid performance.
 * @param e_rate error of rate.
 * @param e_ang error of angle.
 * @param control_u control output.
 * @param error_ang_bet error of two angles of previous and current  .
 * @param ang_act angle of current.
 * @param angle_sp angle of setpoint.
 * @param ang_pre angle of previous.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pid_auto_tune_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float j_pid_tune, float e_rate, float e_ang, float control_u, float error_ang_bet, float ang_act, float angle_sp, float ang_pre)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PID_AUTO_TUNE_LEN];
    _mav_put_float(buf, 0, j_pid_tune);
    _mav_put_float(buf, 4, e_rate);
    _mav_put_float(buf, 8, e_ang);
    _mav_put_float(buf, 12, control_u);
    _mav_put_float(buf, 16, error_ang_bet);
    _mav_put_float(buf, 20, ang_act);
    _mav_put_float(buf, 24, angle_sp);
    _mav_put_float(buf, 28, ang_pre);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PID_AUTO_TUNE_LEN);
#else
    mavlink_pid_auto_tune_t packet;
    packet.j_pid_tune = j_pid_tune;
    packet.e_rate = e_rate;
    packet.e_ang = e_ang;
    packet.control_u = control_u;
    packet.error_ang_bet = error_ang_bet;
    packet.ang_act = ang_act;
    packet.angle_sp = angle_sp;
    packet.ang_pre = ang_pre;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PID_AUTO_TUNE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PID_AUTO_TUNE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PID_AUTO_TUNE_MIN_LEN, MAVLINK_MSG_ID_PID_AUTO_TUNE_LEN, MAVLINK_MSG_ID_PID_AUTO_TUNE_CRC);
}

/**
 * @brief Pack a pid_auto_tune message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param j_pid_tune Judgment of pid performance.
 * @param e_rate error of rate.
 * @param e_ang error of angle.
 * @param control_u control output.
 * @param error_ang_bet error of two angles of previous and current  .
 * @param ang_act angle of current.
 * @param angle_sp angle of setpoint.
 * @param ang_pre angle of previous.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pid_auto_tune_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float j_pid_tune,float e_rate,float e_ang,float control_u,float error_ang_bet,float ang_act,float angle_sp,float ang_pre)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PID_AUTO_TUNE_LEN];
    _mav_put_float(buf, 0, j_pid_tune);
    _mav_put_float(buf, 4, e_rate);
    _mav_put_float(buf, 8, e_ang);
    _mav_put_float(buf, 12, control_u);
    _mav_put_float(buf, 16, error_ang_bet);
    _mav_put_float(buf, 20, ang_act);
    _mav_put_float(buf, 24, angle_sp);
    _mav_put_float(buf, 28, ang_pre);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PID_AUTO_TUNE_LEN);
#else
    mavlink_pid_auto_tune_t packet;
    packet.j_pid_tune = j_pid_tune;
    packet.e_rate = e_rate;
    packet.e_ang = e_ang;
    packet.control_u = control_u;
    packet.error_ang_bet = error_ang_bet;
    packet.ang_act = ang_act;
    packet.angle_sp = angle_sp;
    packet.ang_pre = ang_pre;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PID_AUTO_TUNE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PID_AUTO_TUNE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PID_AUTO_TUNE_MIN_LEN, MAVLINK_MSG_ID_PID_AUTO_TUNE_LEN, MAVLINK_MSG_ID_PID_AUTO_TUNE_CRC);
}

/**
 * @brief Encode a pid_auto_tune struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param pid_auto_tune C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pid_auto_tune_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_pid_auto_tune_t* pid_auto_tune)
{
    return mavlink_msg_pid_auto_tune_pack(system_id, component_id, msg, pid_auto_tune->j_pid_tune, pid_auto_tune->e_rate, pid_auto_tune->e_ang, pid_auto_tune->control_u, pid_auto_tune->error_ang_bet, pid_auto_tune->ang_act, pid_auto_tune->angle_sp, pid_auto_tune->ang_pre);
}

/**
 * @brief Encode a pid_auto_tune struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pid_auto_tune C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pid_auto_tune_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_pid_auto_tune_t* pid_auto_tune)
{
    return mavlink_msg_pid_auto_tune_pack_chan(system_id, component_id, chan, msg, pid_auto_tune->j_pid_tune, pid_auto_tune->e_rate, pid_auto_tune->e_ang, pid_auto_tune->control_u, pid_auto_tune->error_ang_bet, pid_auto_tune->ang_act, pid_auto_tune->angle_sp, pid_auto_tune->ang_pre);
}

/**
 * @brief Send a pid_auto_tune message
 * @param chan MAVLink channel to send the message
 *
 * @param j_pid_tune Judgment of pid performance.
 * @param e_rate error of rate.
 * @param e_ang error of angle.
 * @param control_u control output.
 * @param error_ang_bet error of two angles of previous and current  .
 * @param ang_act angle of current.
 * @param angle_sp angle of setpoint.
 * @param ang_pre angle of previous.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_pid_auto_tune_send(mavlink_channel_t chan, float j_pid_tune, float e_rate, float e_ang, float control_u, float error_ang_bet, float ang_act, float angle_sp, float ang_pre)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PID_AUTO_TUNE_LEN];
    _mav_put_float(buf, 0, j_pid_tune);
    _mav_put_float(buf, 4, e_rate);
    _mav_put_float(buf, 8, e_ang);
    _mav_put_float(buf, 12, control_u);
    _mav_put_float(buf, 16, error_ang_bet);
    _mav_put_float(buf, 20, ang_act);
    _mav_put_float(buf, 24, angle_sp);
    _mav_put_float(buf, 28, ang_pre);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PID_AUTO_TUNE, buf, MAVLINK_MSG_ID_PID_AUTO_TUNE_MIN_LEN, MAVLINK_MSG_ID_PID_AUTO_TUNE_LEN, MAVLINK_MSG_ID_PID_AUTO_TUNE_CRC);
#else
    mavlink_pid_auto_tune_t packet;
    packet.j_pid_tune = j_pid_tune;
    packet.e_rate = e_rate;
    packet.e_ang = e_ang;
    packet.control_u = control_u;
    packet.error_ang_bet = error_ang_bet;
    packet.ang_act = ang_act;
    packet.angle_sp = angle_sp;
    packet.ang_pre = ang_pre;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PID_AUTO_TUNE, (const char *)&packet, MAVLINK_MSG_ID_PID_AUTO_TUNE_MIN_LEN, MAVLINK_MSG_ID_PID_AUTO_TUNE_LEN, MAVLINK_MSG_ID_PID_AUTO_TUNE_CRC);
#endif
}

/**
 * @brief Send a pid_auto_tune message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_pid_auto_tune_send_struct(mavlink_channel_t chan, const mavlink_pid_auto_tune_t* pid_auto_tune)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_pid_auto_tune_send(chan, pid_auto_tune->j_pid_tune, pid_auto_tune->e_rate, pid_auto_tune->e_ang, pid_auto_tune->control_u, pid_auto_tune->error_ang_bet, pid_auto_tune->ang_act, pid_auto_tune->angle_sp, pid_auto_tune->ang_pre);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PID_AUTO_TUNE, (const char *)pid_auto_tune, MAVLINK_MSG_ID_PID_AUTO_TUNE_MIN_LEN, MAVLINK_MSG_ID_PID_AUTO_TUNE_LEN, MAVLINK_MSG_ID_PID_AUTO_TUNE_CRC);
#endif
}

#if MAVLINK_MSG_ID_PID_AUTO_TUNE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_pid_auto_tune_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float j_pid_tune, float e_rate, float e_ang, float control_u, float error_ang_bet, float ang_act, float angle_sp, float ang_pre)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, j_pid_tune);
    _mav_put_float(buf, 4, e_rate);
    _mav_put_float(buf, 8, e_ang);
    _mav_put_float(buf, 12, control_u);
    _mav_put_float(buf, 16, error_ang_bet);
    _mav_put_float(buf, 20, ang_act);
    _mav_put_float(buf, 24, angle_sp);
    _mav_put_float(buf, 28, ang_pre);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PID_AUTO_TUNE, buf, MAVLINK_MSG_ID_PID_AUTO_TUNE_MIN_LEN, MAVLINK_MSG_ID_PID_AUTO_TUNE_LEN, MAVLINK_MSG_ID_PID_AUTO_TUNE_CRC);
#else
    mavlink_pid_auto_tune_t *packet = (mavlink_pid_auto_tune_t *)msgbuf;
    packet->j_pid_tune = j_pid_tune;
    packet->e_rate = e_rate;
    packet->e_ang = e_ang;
    packet->control_u = control_u;
    packet->error_ang_bet = error_ang_bet;
    packet->ang_act = ang_act;
    packet->angle_sp = angle_sp;
    packet->ang_pre = ang_pre;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PID_AUTO_TUNE, (const char *)packet, MAVLINK_MSG_ID_PID_AUTO_TUNE_MIN_LEN, MAVLINK_MSG_ID_PID_AUTO_TUNE_LEN, MAVLINK_MSG_ID_PID_AUTO_TUNE_CRC);
#endif
}
#endif

#endif

// MESSAGE PID_AUTO_TUNE UNPACKING


/**
 * @brief Get field j_pid_tune from pid_auto_tune message
 *
 * @return Judgment of pid performance.
 */
static inline float mavlink_msg_pid_auto_tune_get_j_pid_tune(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field e_rate from pid_auto_tune message
 *
 * @return error of rate.
 */
static inline float mavlink_msg_pid_auto_tune_get_e_rate(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field e_ang from pid_auto_tune message
 *
 * @return error of angle.
 */
static inline float mavlink_msg_pid_auto_tune_get_e_ang(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field control_u from pid_auto_tune message
 *
 * @return control output.
 */
static inline float mavlink_msg_pid_auto_tune_get_control_u(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field error_ang_bet from pid_auto_tune message
 *
 * @return error of two angles of previous and current  .
 */
static inline float mavlink_msg_pid_auto_tune_get_error_ang_bet(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field ang_act from pid_auto_tune message
 *
 * @return angle of current.
 */
static inline float mavlink_msg_pid_auto_tune_get_ang_act(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field angle_sp from pid_auto_tune message
 *
 * @return angle of setpoint.
 */
static inline float mavlink_msg_pid_auto_tune_get_angle_sp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field ang_pre from pid_auto_tune message
 *
 * @return angle of previous.
 */
static inline float mavlink_msg_pid_auto_tune_get_ang_pre(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Decode a pid_auto_tune message into a struct
 *
 * @param msg The message to decode
 * @param pid_auto_tune C-struct to decode the message contents into
 */
static inline void mavlink_msg_pid_auto_tune_decode(const mavlink_message_t* msg, mavlink_pid_auto_tune_t* pid_auto_tune)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    pid_auto_tune->j_pid_tune = mavlink_msg_pid_auto_tune_get_j_pid_tune(msg);
    pid_auto_tune->e_rate = mavlink_msg_pid_auto_tune_get_e_rate(msg);
    pid_auto_tune->e_ang = mavlink_msg_pid_auto_tune_get_e_ang(msg);
    pid_auto_tune->control_u = mavlink_msg_pid_auto_tune_get_control_u(msg);
    pid_auto_tune->error_ang_bet = mavlink_msg_pid_auto_tune_get_error_ang_bet(msg);
    pid_auto_tune->ang_act = mavlink_msg_pid_auto_tune_get_ang_act(msg);
    pid_auto_tune->angle_sp = mavlink_msg_pid_auto_tune_get_angle_sp(msg);
    pid_auto_tune->ang_pre = mavlink_msg_pid_auto_tune_get_ang_pre(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PID_AUTO_TUNE_LEN? msg->len : MAVLINK_MSG_ID_PID_AUTO_TUNE_LEN;
        memset(pid_auto_tune, 0, MAVLINK_MSG_ID_PID_AUTO_TUNE_LEN);
    memcpy(pid_auto_tune, _MAV_PAYLOAD(msg), len);
#endif
}
