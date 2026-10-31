#pragma once
// MESSAGE MABUR_CAL_ACK PACKING

#define MAVLINK_MSG_ID_MABUR_CAL_ACK 60001


typedef struct __mavlink_mabur_cal_ack_t {
 uint32_t nonce; /*<  Echoes the sweep's nonce.*/
 uint16_t start_in_ms; /*<  Milliseconds until the sweep starts, if accepted.*/
 uint8_t seq; /*<  Echoes the sweep's seq.*/
 uint8_t result; /*<  Accept/refuse result.*/
 uint8_t flags; /*<  Stamp flag mask the drone will fly for this sweep.*/
} mavlink_mabur_cal_ack_t;

#define MAVLINK_MSG_ID_MABUR_CAL_ACK_LEN 9
#define MAVLINK_MSG_ID_MABUR_CAL_ACK_MIN_LEN 9
#define MAVLINK_MSG_ID_60001_LEN 9
#define MAVLINK_MSG_ID_60001_MIN_LEN 9

#define MAVLINK_MSG_ID_MABUR_CAL_ACK_CRC 87
#define MAVLINK_MSG_ID_60001_CRC 87



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MABUR_CAL_ACK { \
    60001, \
    "MABUR_CAL_ACK", \
    5, \
    {  { "nonce", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_mabur_cal_ack_t, nonce) }, \
         { "seq", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_mabur_cal_ack_t, seq) }, \
         { "result", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_mabur_cal_ack_t, result) }, \
         { "start_in_ms", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_mabur_cal_ack_t, start_in_ms) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_mabur_cal_ack_t, flags) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MABUR_CAL_ACK { \
    "MABUR_CAL_ACK", \
    5, \
    {  { "nonce", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_mabur_cal_ack_t, nonce) }, \
         { "seq", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_mabur_cal_ack_t, seq) }, \
         { "result", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_mabur_cal_ack_t, result) }, \
         { "start_in_ms", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_mabur_cal_ack_t, start_in_ms) }, \
         { "flags", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_mabur_cal_ack_t, flags) }, \
         } \
}
#endif

/**
 * @brief Pack a mabur_cal_ack message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param nonce  Echoes the sweep's nonce.
 * @param seq  Echoes the sweep's seq.
 * @param result  Accept/refuse result.
 * @param start_in_ms  Milliseconds until the sweep starts, if accepted.
 * @param flags  Stamp flag mask the drone will fly for this sweep.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mabur_cal_ack_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t nonce, uint8_t seq, uint8_t result, uint16_t start_in_ms, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MABUR_CAL_ACK_LEN];
    _mav_put_uint32_t(buf, 0, nonce);
    _mav_put_uint16_t(buf, 4, start_in_ms);
    _mav_put_uint8_t(buf, 6, seq);
    _mav_put_uint8_t(buf, 7, result);
    _mav_put_uint8_t(buf, 8, flags);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MABUR_CAL_ACK_LEN);
#else
    mavlink_mabur_cal_ack_t packet;
    packet.nonce = nonce;
    packet.start_in_ms = start_in_ms;
    packet.seq = seq;
    packet.result = result;
    packet.flags = flags;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MABUR_CAL_ACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MABUR_CAL_ACK;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MABUR_CAL_ACK_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_ACK_LEN, MAVLINK_MSG_ID_MABUR_CAL_ACK_CRC);
}

/**
 * @brief Pack a mabur_cal_ack message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param nonce  Echoes the sweep's nonce.
 * @param seq  Echoes the sweep's seq.
 * @param result  Accept/refuse result.
 * @param start_in_ms  Milliseconds until the sweep starts, if accepted.
 * @param flags  Stamp flag mask the drone will fly for this sweep.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mabur_cal_ack_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t nonce, uint8_t seq, uint8_t result, uint16_t start_in_ms, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MABUR_CAL_ACK_LEN];
    _mav_put_uint32_t(buf, 0, nonce);
    _mav_put_uint16_t(buf, 4, start_in_ms);
    _mav_put_uint8_t(buf, 6, seq);
    _mav_put_uint8_t(buf, 7, result);
    _mav_put_uint8_t(buf, 8, flags);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MABUR_CAL_ACK_LEN);
#else
    mavlink_mabur_cal_ack_t packet;
    packet.nonce = nonce;
    packet.start_in_ms = start_in_ms;
    packet.seq = seq;
    packet.result = result;
    packet.flags = flags;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MABUR_CAL_ACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MABUR_CAL_ACK;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MABUR_CAL_ACK_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_ACK_LEN, MAVLINK_MSG_ID_MABUR_CAL_ACK_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MABUR_CAL_ACK_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_ACK_LEN);
#endif
}

/**
 * @brief Pack a mabur_cal_ack message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param nonce  Echoes the sweep's nonce.
 * @param seq  Echoes the sweep's seq.
 * @param result  Accept/refuse result.
 * @param start_in_ms  Milliseconds until the sweep starts, if accepted.
 * @param flags  Stamp flag mask the drone will fly for this sweep.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mabur_cal_ack_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t nonce,uint8_t seq,uint8_t result,uint16_t start_in_ms,uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MABUR_CAL_ACK_LEN];
    _mav_put_uint32_t(buf, 0, nonce);
    _mav_put_uint16_t(buf, 4, start_in_ms);
    _mav_put_uint8_t(buf, 6, seq);
    _mav_put_uint8_t(buf, 7, result);
    _mav_put_uint8_t(buf, 8, flags);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MABUR_CAL_ACK_LEN);
#else
    mavlink_mabur_cal_ack_t packet;
    packet.nonce = nonce;
    packet.start_in_ms = start_in_ms;
    packet.seq = seq;
    packet.result = result;
    packet.flags = flags;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MABUR_CAL_ACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MABUR_CAL_ACK;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MABUR_CAL_ACK_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_ACK_LEN, MAVLINK_MSG_ID_MABUR_CAL_ACK_CRC);
}

/**
 * @brief Encode a mabur_cal_ack struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mabur_cal_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mabur_cal_ack_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mabur_cal_ack_t* mabur_cal_ack)
{
    return mavlink_msg_mabur_cal_ack_pack(system_id, component_id, msg, mabur_cal_ack->nonce, mabur_cal_ack->seq, mabur_cal_ack->result, mabur_cal_ack->start_in_ms, mabur_cal_ack->flags);
}

/**
 * @brief Encode a mabur_cal_ack struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mabur_cal_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mabur_cal_ack_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mabur_cal_ack_t* mabur_cal_ack)
{
    return mavlink_msg_mabur_cal_ack_pack_chan(system_id, component_id, chan, msg, mabur_cal_ack->nonce, mabur_cal_ack->seq, mabur_cal_ack->result, mabur_cal_ack->start_in_ms, mabur_cal_ack->flags);
}

/**
 * @brief Encode a mabur_cal_ack struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param mabur_cal_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mabur_cal_ack_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_mabur_cal_ack_t* mabur_cal_ack)
{
    return mavlink_msg_mabur_cal_ack_pack_status(system_id, component_id, _status, msg,  mabur_cal_ack->nonce, mabur_cal_ack->seq, mabur_cal_ack->result, mabur_cal_ack->start_in_ms, mabur_cal_ack->flags);
}

/**
 * @brief Send a mabur_cal_ack message
 * @param chan MAVLink channel to send the message
 *
 * @param nonce  Echoes the sweep's nonce.
 * @param seq  Echoes the sweep's seq.
 * @param result  Accept/refuse result.
 * @param start_in_ms  Milliseconds until the sweep starts, if accepted.
 * @param flags  Stamp flag mask the drone will fly for this sweep.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mabur_cal_ack_send(mavlink_channel_t chan, uint32_t nonce, uint8_t seq, uint8_t result, uint16_t start_in_ms, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MABUR_CAL_ACK_LEN];
    _mav_put_uint32_t(buf, 0, nonce);
    _mav_put_uint16_t(buf, 4, start_in_ms);
    _mav_put_uint8_t(buf, 6, seq);
    _mav_put_uint8_t(buf, 7, result);
    _mav_put_uint8_t(buf, 8, flags);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_CAL_ACK, buf, MAVLINK_MSG_ID_MABUR_CAL_ACK_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_ACK_LEN, MAVLINK_MSG_ID_MABUR_CAL_ACK_CRC);
#else
    mavlink_mabur_cal_ack_t packet;
    packet.nonce = nonce;
    packet.start_in_ms = start_in_ms;
    packet.seq = seq;
    packet.result = result;
    packet.flags = flags;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_CAL_ACK, (const char *)&packet, MAVLINK_MSG_ID_MABUR_CAL_ACK_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_ACK_LEN, MAVLINK_MSG_ID_MABUR_CAL_ACK_CRC);
#endif
}

/**
 * @brief Send a mabur_cal_ack message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mabur_cal_ack_send_struct(mavlink_channel_t chan, const mavlink_mabur_cal_ack_t* mabur_cal_ack)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mabur_cal_ack_send(chan, mabur_cal_ack->nonce, mabur_cal_ack->seq, mabur_cal_ack->result, mabur_cal_ack->start_in_ms, mabur_cal_ack->flags);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_CAL_ACK, (const char *)mabur_cal_ack, MAVLINK_MSG_ID_MABUR_CAL_ACK_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_ACK_LEN, MAVLINK_MSG_ID_MABUR_CAL_ACK_CRC);
#endif
}

#if MAVLINK_MSG_ID_MABUR_CAL_ACK_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mabur_cal_ack_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t nonce, uint8_t seq, uint8_t result, uint16_t start_in_ms, uint8_t flags)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, nonce);
    _mav_put_uint16_t(buf, 4, start_in_ms);
    _mav_put_uint8_t(buf, 6, seq);
    _mav_put_uint8_t(buf, 7, result);
    _mav_put_uint8_t(buf, 8, flags);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_CAL_ACK, buf, MAVLINK_MSG_ID_MABUR_CAL_ACK_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_ACK_LEN, MAVLINK_MSG_ID_MABUR_CAL_ACK_CRC);
#else
    mavlink_mabur_cal_ack_t *packet = (mavlink_mabur_cal_ack_t *)msgbuf;
    packet->nonce = nonce;
    packet->start_in_ms = start_in_ms;
    packet->seq = seq;
    packet->result = result;
    packet->flags = flags;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_CAL_ACK, (const char *)packet, MAVLINK_MSG_ID_MABUR_CAL_ACK_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_ACK_LEN, MAVLINK_MSG_ID_MABUR_CAL_ACK_CRC);
#endif
}
#endif

#endif

// MESSAGE MABUR_CAL_ACK UNPACKING


/**
 * @brief Get field nonce from mabur_cal_ack message
 *
 * @return  Echoes the sweep's nonce.
 */
static inline uint32_t mavlink_msg_mabur_cal_ack_get_nonce(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field seq from mabur_cal_ack message
 *
 * @return  Echoes the sweep's seq.
 */
static inline uint8_t mavlink_msg_mabur_cal_ack_get_seq(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field result from mabur_cal_ack message
 *
 * @return  Accept/refuse result.
 */
static inline uint8_t mavlink_msg_mabur_cal_ack_get_result(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field start_in_ms from mabur_cal_ack message
 *
 * @return  Milliseconds until the sweep starts, if accepted.
 */
static inline uint16_t mavlink_msg_mabur_cal_ack_get_start_in_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field flags from mabur_cal_ack message
 *
 * @return  Stamp flag mask the drone will fly for this sweep.
 */
static inline uint8_t mavlink_msg_mabur_cal_ack_get_flags(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Decode a mabur_cal_ack message into a struct
 *
 * @param msg The message to decode
 * @param mabur_cal_ack C-struct to decode the message contents into
 */
static inline void mavlink_msg_mabur_cal_ack_decode(const mavlink_message_t* msg, mavlink_mabur_cal_ack_t* mabur_cal_ack)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mabur_cal_ack->nonce = mavlink_msg_mabur_cal_ack_get_nonce(msg);
    mabur_cal_ack->start_in_ms = mavlink_msg_mabur_cal_ack_get_start_in_ms(msg);
    mabur_cal_ack->seq = mavlink_msg_mabur_cal_ack_get_seq(msg);
    mabur_cal_ack->result = mavlink_msg_mabur_cal_ack_get_result(msg);
    mabur_cal_ack->flags = mavlink_msg_mabur_cal_ack_get_flags(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MABUR_CAL_ACK_LEN? msg->len : MAVLINK_MSG_ID_MABUR_CAL_ACK_LEN;
        memset(mabur_cal_ack, 0, MAVLINK_MSG_ID_MABUR_CAL_ACK_LEN);
    memcpy(mabur_cal_ack, _MAV_PAYLOAD(msg), len);
#endif
}
