#pragma once
// MESSAGE MABUR_CAL_APPLIED PACKING

#define MAVLINK_MSG_ID_MABUR_CAL_APPLIED 60004


typedef struct __mavlink_mabur_cal_applied_t {
 uint32_t nonce; /*<  Echoes the apply's nonce.*/
 uint8_t bw; /*<  Echoes the bandwidth the apply targeted.*/
 uint8_t ok; /*<  1 if the table was adopted, 0 otherwise.*/
 char reason[50]; /*<  Human-readable refusal reason, empty on success.*/
} mavlink_mabur_cal_applied_t;

#define MAVLINK_MSG_ID_MABUR_CAL_APPLIED_LEN 56
#define MAVLINK_MSG_ID_MABUR_CAL_APPLIED_MIN_LEN 56
#define MAVLINK_MSG_ID_60004_LEN 56
#define MAVLINK_MSG_ID_60004_MIN_LEN 56

#define MAVLINK_MSG_ID_MABUR_CAL_APPLIED_CRC 99
#define MAVLINK_MSG_ID_60004_CRC 99

#define MAVLINK_MSG_MABUR_CAL_APPLIED_FIELD_REASON_LEN 50

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MABUR_CAL_APPLIED { \
    60004, \
    "MABUR_CAL_APPLIED", \
    4, \
    {  { "nonce", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_mabur_cal_applied_t, nonce) }, \
         { "bw", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_mabur_cal_applied_t, bw) }, \
         { "ok", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_mabur_cal_applied_t, ok) }, \
         { "reason", NULL, MAVLINK_TYPE_CHAR, 50, 6, offsetof(mavlink_mabur_cal_applied_t, reason) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MABUR_CAL_APPLIED { \
    "MABUR_CAL_APPLIED", \
    4, \
    {  { "nonce", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_mabur_cal_applied_t, nonce) }, \
         { "bw", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_mabur_cal_applied_t, bw) }, \
         { "ok", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_mabur_cal_applied_t, ok) }, \
         { "reason", NULL, MAVLINK_TYPE_CHAR, 50, 6, offsetof(mavlink_mabur_cal_applied_t, reason) }, \
         } \
}
#endif

/**
 * @brief Pack a mabur_cal_applied message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param nonce  Echoes the apply's nonce.
 * @param bw  Echoes the bandwidth the apply targeted.
 * @param ok  1 if the table was adopted, 0 otherwise.
 * @param reason  Human-readable refusal reason, empty on success.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mabur_cal_applied_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t nonce, uint8_t bw, uint8_t ok, const char *reason)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MABUR_CAL_APPLIED_LEN];
    _mav_put_uint32_t(buf, 0, nonce);
    _mav_put_uint8_t(buf, 4, bw);
    _mav_put_uint8_t(buf, 5, ok);
    _mav_put_char_array(buf, 6, reason, 50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MABUR_CAL_APPLIED_LEN);
#else
    mavlink_mabur_cal_applied_t packet;
    packet.nonce = nonce;
    packet.bw = bw;
    packet.ok = ok;
    mav_array_assign_char(packet.reason, reason, 50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MABUR_CAL_APPLIED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MABUR_CAL_APPLIED;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MABUR_CAL_APPLIED_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_APPLIED_LEN, MAVLINK_MSG_ID_MABUR_CAL_APPLIED_CRC);
}

/**
 * @brief Pack a mabur_cal_applied message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param nonce  Echoes the apply's nonce.
 * @param bw  Echoes the bandwidth the apply targeted.
 * @param ok  1 if the table was adopted, 0 otherwise.
 * @param reason  Human-readable refusal reason, empty on success.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mabur_cal_applied_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t nonce, uint8_t bw, uint8_t ok, const char *reason)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MABUR_CAL_APPLIED_LEN];
    _mav_put_uint32_t(buf, 0, nonce);
    _mav_put_uint8_t(buf, 4, bw);
    _mav_put_uint8_t(buf, 5, ok);
    _mav_put_char_array(buf, 6, reason, 50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MABUR_CAL_APPLIED_LEN);
#else
    mavlink_mabur_cal_applied_t packet;
    packet.nonce = nonce;
    packet.bw = bw;
    packet.ok = ok;
    mav_array_memcpy(packet.reason, reason, sizeof(char)*50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MABUR_CAL_APPLIED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MABUR_CAL_APPLIED;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MABUR_CAL_APPLIED_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_APPLIED_LEN, MAVLINK_MSG_ID_MABUR_CAL_APPLIED_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MABUR_CAL_APPLIED_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_APPLIED_LEN);
#endif
}

/**
 * @brief Pack a mabur_cal_applied message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param nonce  Echoes the apply's nonce.
 * @param bw  Echoes the bandwidth the apply targeted.
 * @param ok  1 if the table was adopted, 0 otherwise.
 * @param reason  Human-readable refusal reason, empty on success.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mabur_cal_applied_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t nonce,uint8_t bw,uint8_t ok,const char *reason)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MABUR_CAL_APPLIED_LEN];
    _mav_put_uint32_t(buf, 0, nonce);
    _mav_put_uint8_t(buf, 4, bw);
    _mav_put_uint8_t(buf, 5, ok);
    _mav_put_char_array(buf, 6, reason, 50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MABUR_CAL_APPLIED_LEN);
#else
    mavlink_mabur_cal_applied_t packet;
    packet.nonce = nonce;
    packet.bw = bw;
    packet.ok = ok;
    mav_array_assign_char(packet.reason, reason, 50);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MABUR_CAL_APPLIED_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MABUR_CAL_APPLIED;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MABUR_CAL_APPLIED_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_APPLIED_LEN, MAVLINK_MSG_ID_MABUR_CAL_APPLIED_CRC);
}

/**
 * @brief Encode a mabur_cal_applied struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mabur_cal_applied C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mabur_cal_applied_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mabur_cal_applied_t* mabur_cal_applied)
{
    return mavlink_msg_mabur_cal_applied_pack(system_id, component_id, msg, mabur_cal_applied->nonce, mabur_cal_applied->bw, mabur_cal_applied->ok, mabur_cal_applied->reason);
}

/**
 * @brief Encode a mabur_cal_applied struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mabur_cal_applied C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mabur_cal_applied_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mabur_cal_applied_t* mabur_cal_applied)
{
    return mavlink_msg_mabur_cal_applied_pack_chan(system_id, component_id, chan, msg, mabur_cal_applied->nonce, mabur_cal_applied->bw, mabur_cal_applied->ok, mabur_cal_applied->reason);
}

/**
 * @brief Encode a mabur_cal_applied struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param mabur_cal_applied C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mabur_cal_applied_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_mabur_cal_applied_t* mabur_cal_applied)
{
    return mavlink_msg_mabur_cal_applied_pack_status(system_id, component_id, _status, msg,  mabur_cal_applied->nonce, mabur_cal_applied->bw, mabur_cal_applied->ok, mabur_cal_applied->reason);
}

/**
 * @brief Send a mabur_cal_applied message
 * @param chan MAVLink channel to send the message
 *
 * @param nonce  Echoes the apply's nonce.
 * @param bw  Echoes the bandwidth the apply targeted.
 * @param ok  1 if the table was adopted, 0 otherwise.
 * @param reason  Human-readable refusal reason, empty on success.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mabur_cal_applied_send(mavlink_channel_t chan, uint32_t nonce, uint8_t bw, uint8_t ok, const char *reason)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MABUR_CAL_APPLIED_LEN];
    _mav_put_uint32_t(buf, 0, nonce);
    _mav_put_uint8_t(buf, 4, bw);
    _mav_put_uint8_t(buf, 5, ok);
    _mav_put_char_array(buf, 6, reason, 50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_CAL_APPLIED, buf, MAVLINK_MSG_ID_MABUR_CAL_APPLIED_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_APPLIED_LEN, MAVLINK_MSG_ID_MABUR_CAL_APPLIED_CRC);
#else
    mavlink_mabur_cal_applied_t packet;
    packet.nonce = nonce;
    packet.bw = bw;
    packet.ok = ok;
    mav_array_assign_char(packet.reason, reason, 50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_CAL_APPLIED, (const char *)&packet, MAVLINK_MSG_ID_MABUR_CAL_APPLIED_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_APPLIED_LEN, MAVLINK_MSG_ID_MABUR_CAL_APPLIED_CRC);
#endif
}

/**
 * @brief Send a mabur_cal_applied message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mabur_cal_applied_send_struct(mavlink_channel_t chan, const mavlink_mabur_cal_applied_t* mabur_cal_applied)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mabur_cal_applied_send(chan, mabur_cal_applied->nonce, mabur_cal_applied->bw, mabur_cal_applied->ok, mabur_cal_applied->reason);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_CAL_APPLIED, (const char *)mabur_cal_applied, MAVLINK_MSG_ID_MABUR_CAL_APPLIED_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_APPLIED_LEN, MAVLINK_MSG_ID_MABUR_CAL_APPLIED_CRC);
#endif
}

#if MAVLINK_MSG_ID_MABUR_CAL_APPLIED_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mabur_cal_applied_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t nonce, uint8_t bw, uint8_t ok, const char *reason)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, nonce);
    _mav_put_uint8_t(buf, 4, bw);
    _mav_put_uint8_t(buf, 5, ok);
    _mav_put_char_array(buf, 6, reason, 50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_CAL_APPLIED, buf, MAVLINK_MSG_ID_MABUR_CAL_APPLIED_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_APPLIED_LEN, MAVLINK_MSG_ID_MABUR_CAL_APPLIED_CRC);
#else
    mavlink_mabur_cal_applied_t *packet = (mavlink_mabur_cal_applied_t *)msgbuf;
    packet->nonce = nonce;
    packet->bw = bw;
    packet->ok = ok;
    mav_array_assign_char(packet->reason, reason, 50);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_CAL_APPLIED, (const char *)packet, MAVLINK_MSG_ID_MABUR_CAL_APPLIED_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_APPLIED_LEN, MAVLINK_MSG_ID_MABUR_CAL_APPLIED_CRC);
#endif
}
#endif

#endif

// MESSAGE MABUR_CAL_APPLIED UNPACKING


/**
 * @brief Get field nonce from mabur_cal_applied message
 *
 * @return  Echoes the apply's nonce.
 */
static inline uint32_t mavlink_msg_mabur_cal_applied_get_nonce(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field bw from mabur_cal_applied message
 *
 * @return  Echoes the bandwidth the apply targeted.
 */
static inline uint8_t mavlink_msg_mabur_cal_applied_get_bw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field ok from mabur_cal_applied message
 *
 * @return  1 if the table was adopted, 0 otherwise.
 */
static inline uint8_t mavlink_msg_mabur_cal_applied_get_ok(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field reason from mabur_cal_applied message
 *
 * @return  Human-readable refusal reason, empty on success.
 */
static inline uint16_t mavlink_msg_mabur_cal_applied_get_reason(const mavlink_message_t* msg, char *reason)
{
    return _MAV_RETURN_char_array(msg, reason, 50,  6);
}

/**
 * @brief Decode a mabur_cal_applied message into a struct
 *
 * @param msg The message to decode
 * @param mabur_cal_applied C-struct to decode the message contents into
 */
static inline void mavlink_msg_mabur_cal_applied_decode(const mavlink_message_t* msg, mavlink_mabur_cal_applied_t* mabur_cal_applied)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mabur_cal_applied->nonce = mavlink_msg_mabur_cal_applied_get_nonce(msg);
    mabur_cal_applied->bw = mavlink_msg_mabur_cal_applied_get_bw(msg);
    mabur_cal_applied->ok = mavlink_msg_mabur_cal_applied_get_ok(msg);
    mavlink_msg_mabur_cal_applied_get_reason(msg, mabur_cal_applied->reason);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MABUR_CAL_APPLIED_LEN? msg->len : MAVLINK_MSG_ID_MABUR_CAL_APPLIED_LEN;
        memset(mabur_cal_applied, 0, MAVLINK_MSG_ID_MABUR_CAL_APPLIED_LEN);
    memcpy(mabur_cal_applied, _MAV_PAYLOAD(msg), len);
#endif
}
