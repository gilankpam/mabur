#pragma once
// MESSAGE MABUR_CAL_SWEEP_DONE PACKING

#define MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE 60002


typedef struct __mavlink_mabur_cal_sweep_done_t {
 uint32_t nonce; /*<  Echoes the sweep's nonce.*/
 uint32_t offered[8]; /*<  Frames offered per cell.*/
 uint8_t seq; /*<  Echoes the sweep's seq.*/
 uint8_t blocked_pct[8]; /*<  Per rate, the share of the capacity dwell (percent) the drone's sender spent blocked on the device; 0 for a power sweep.*/
} mavlink_mabur_cal_sweep_done_t;

#define MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_LEN 45
#define MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_MIN_LEN 45
#define MAVLINK_MSG_ID_60002_LEN 45
#define MAVLINK_MSG_ID_60002_MIN_LEN 45

#define MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_CRC 127
#define MAVLINK_MSG_ID_60002_CRC 127

#define MAVLINK_MSG_MABUR_CAL_SWEEP_DONE_FIELD_OFFERED_LEN 8
#define MAVLINK_MSG_MABUR_CAL_SWEEP_DONE_FIELD_BLOCKED_PCT_LEN 8

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MABUR_CAL_SWEEP_DONE { \
    60002, \
    "MABUR_CAL_SWEEP_DONE", \
    4, \
    {  { "nonce", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_mabur_cal_sweep_done_t, nonce) }, \
         { "seq", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_mabur_cal_sweep_done_t, seq) }, \
         { "offered", NULL, MAVLINK_TYPE_UINT32_T, 8, 4, offsetof(mavlink_mabur_cal_sweep_done_t, offered) }, \
         { "blocked_pct", NULL, MAVLINK_TYPE_UINT8_T, 8, 37, offsetof(mavlink_mabur_cal_sweep_done_t, blocked_pct) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MABUR_CAL_SWEEP_DONE { \
    "MABUR_CAL_SWEEP_DONE", \
    4, \
    {  { "nonce", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_mabur_cal_sweep_done_t, nonce) }, \
         { "seq", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_mabur_cal_sweep_done_t, seq) }, \
         { "offered", NULL, MAVLINK_TYPE_UINT32_T, 8, 4, offsetof(mavlink_mabur_cal_sweep_done_t, offered) }, \
         { "blocked_pct", NULL, MAVLINK_TYPE_UINT8_T, 8, 37, offsetof(mavlink_mabur_cal_sweep_done_t, blocked_pct) }, \
         } \
}
#endif

/**
 * @brief Pack a mabur_cal_sweep_done message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param nonce  Echoes the sweep's nonce.
 * @param seq  Echoes the sweep's seq.
 * @param offered  Frames offered per cell.
 * @param blocked_pct  Per rate, the share of the capacity dwell (percent) the drone's sender spent blocked on the device; 0 for a power sweep.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mabur_cal_sweep_done_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t nonce, uint8_t seq, const uint32_t *offered, const uint8_t *blocked_pct)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_LEN];
    _mav_put_uint32_t(buf, 0, nonce);
    _mav_put_uint8_t(buf, 36, seq);
    _mav_put_uint32_t_array(buf, 4, offered, 8);
    _mav_put_uint8_t_array(buf, 37, blocked_pct, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_LEN);
#else
    mavlink_mabur_cal_sweep_done_t packet;
    packet.nonce = nonce;
    packet.seq = seq;
    mav_array_assign_uint32_t(packet.offered, offered, 8);
    mav_array_assign_uint8_t(packet.blocked_pct, blocked_pct, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_LEN, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_CRC);
}

/**
 * @brief Pack a mabur_cal_sweep_done message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param nonce  Echoes the sweep's nonce.
 * @param seq  Echoes the sweep's seq.
 * @param offered  Frames offered per cell.
 * @param blocked_pct  Per rate, the share of the capacity dwell (percent) the drone's sender spent blocked on the device; 0 for a power sweep.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mabur_cal_sweep_done_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t nonce, uint8_t seq, const uint32_t *offered, const uint8_t *blocked_pct)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_LEN];
    _mav_put_uint32_t(buf, 0, nonce);
    _mav_put_uint8_t(buf, 36, seq);
    _mav_put_uint32_t_array(buf, 4, offered, 8);
    _mav_put_uint8_t_array(buf, 37, blocked_pct, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_LEN);
#else
    mavlink_mabur_cal_sweep_done_t packet;
    packet.nonce = nonce;
    packet.seq = seq;
    mav_array_memcpy(packet.offered, offered, sizeof(uint32_t)*8);
    mav_array_memcpy(packet.blocked_pct, blocked_pct, sizeof(uint8_t)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_LEN, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_LEN);
#endif
}

/**
 * @brief Pack a mabur_cal_sweep_done message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param nonce  Echoes the sweep's nonce.
 * @param seq  Echoes the sweep's seq.
 * @param offered  Frames offered per cell.
 * @param blocked_pct  Per rate, the share of the capacity dwell (percent) the drone's sender spent blocked on the device; 0 for a power sweep.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mabur_cal_sweep_done_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t nonce,uint8_t seq,const uint32_t *offered,const uint8_t *blocked_pct)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_LEN];
    _mav_put_uint32_t(buf, 0, nonce);
    _mav_put_uint8_t(buf, 36, seq);
    _mav_put_uint32_t_array(buf, 4, offered, 8);
    _mav_put_uint8_t_array(buf, 37, blocked_pct, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_LEN);
#else
    mavlink_mabur_cal_sweep_done_t packet;
    packet.nonce = nonce;
    packet.seq = seq;
    mav_array_assign_uint32_t(packet.offered, offered, 8);
    mav_array_assign_uint8_t(packet.blocked_pct, blocked_pct, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_LEN, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_CRC);
}

/**
 * @brief Encode a mabur_cal_sweep_done struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mabur_cal_sweep_done C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mabur_cal_sweep_done_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mabur_cal_sweep_done_t* mabur_cal_sweep_done)
{
    return mavlink_msg_mabur_cal_sweep_done_pack(system_id, component_id, msg, mabur_cal_sweep_done->nonce, mabur_cal_sweep_done->seq, mabur_cal_sweep_done->offered, mabur_cal_sweep_done->blocked_pct);
}

/**
 * @brief Encode a mabur_cal_sweep_done struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mabur_cal_sweep_done C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mabur_cal_sweep_done_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mabur_cal_sweep_done_t* mabur_cal_sweep_done)
{
    return mavlink_msg_mabur_cal_sweep_done_pack_chan(system_id, component_id, chan, msg, mabur_cal_sweep_done->nonce, mabur_cal_sweep_done->seq, mabur_cal_sweep_done->offered, mabur_cal_sweep_done->blocked_pct);
}

/**
 * @brief Encode a mabur_cal_sweep_done struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param mabur_cal_sweep_done C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mabur_cal_sweep_done_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_mabur_cal_sweep_done_t* mabur_cal_sweep_done)
{
    return mavlink_msg_mabur_cal_sweep_done_pack_status(system_id, component_id, _status, msg,  mabur_cal_sweep_done->nonce, mabur_cal_sweep_done->seq, mabur_cal_sweep_done->offered, mabur_cal_sweep_done->blocked_pct);
}

/**
 * @brief Send a mabur_cal_sweep_done message
 * @param chan MAVLink channel to send the message
 *
 * @param nonce  Echoes the sweep's nonce.
 * @param seq  Echoes the sweep's seq.
 * @param offered  Frames offered per cell.
 * @param blocked_pct  Per rate, the share of the capacity dwell (percent) the drone's sender spent blocked on the device; 0 for a power sweep.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mabur_cal_sweep_done_send(mavlink_channel_t chan, uint32_t nonce, uint8_t seq, const uint32_t *offered, const uint8_t *blocked_pct)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_LEN];
    _mav_put_uint32_t(buf, 0, nonce);
    _mav_put_uint8_t(buf, 36, seq);
    _mav_put_uint32_t_array(buf, 4, offered, 8);
    _mav_put_uint8_t_array(buf, 37, blocked_pct, 8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE, buf, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_LEN, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_CRC);
#else
    mavlink_mabur_cal_sweep_done_t packet;
    packet.nonce = nonce;
    packet.seq = seq;
    mav_array_assign_uint32_t(packet.offered, offered, 8);
    mav_array_assign_uint8_t(packet.blocked_pct, blocked_pct, 8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE, (const char *)&packet, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_LEN, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_CRC);
#endif
}

/**
 * @brief Send a mabur_cal_sweep_done message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mabur_cal_sweep_done_send_struct(mavlink_channel_t chan, const mavlink_mabur_cal_sweep_done_t* mabur_cal_sweep_done)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mabur_cal_sweep_done_send(chan, mabur_cal_sweep_done->nonce, mabur_cal_sweep_done->seq, mabur_cal_sweep_done->offered, mabur_cal_sweep_done->blocked_pct);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE, (const char *)mabur_cal_sweep_done, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_LEN, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_CRC);
#endif
}

#if MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mabur_cal_sweep_done_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t nonce, uint8_t seq, const uint32_t *offered, const uint8_t *blocked_pct)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, nonce);
    _mav_put_uint8_t(buf, 36, seq);
    _mav_put_uint32_t_array(buf, 4, offered, 8);
    _mav_put_uint8_t_array(buf, 37, blocked_pct, 8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE, buf, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_LEN, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_CRC);
#else
    mavlink_mabur_cal_sweep_done_t *packet = (mavlink_mabur_cal_sweep_done_t *)msgbuf;
    packet->nonce = nonce;
    packet->seq = seq;
    mav_array_assign_uint32_t(packet->offered, offered, 8);
    mav_array_assign_uint8_t(packet->blocked_pct, blocked_pct, 8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE, (const char *)packet, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_LEN, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_CRC);
#endif
}
#endif

#endif

// MESSAGE MABUR_CAL_SWEEP_DONE UNPACKING


/**
 * @brief Get field nonce from mabur_cal_sweep_done message
 *
 * @return  Echoes the sweep's nonce.
 */
static inline uint32_t mavlink_msg_mabur_cal_sweep_done_get_nonce(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field seq from mabur_cal_sweep_done message
 *
 * @return  Echoes the sweep's seq.
 */
static inline uint8_t mavlink_msg_mabur_cal_sweep_done_get_seq(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  36);
}

/**
 * @brief Get field offered from mabur_cal_sweep_done message
 *
 * @return  Frames offered per cell.
 */
static inline uint16_t mavlink_msg_mabur_cal_sweep_done_get_offered(const mavlink_message_t* msg, uint32_t *offered)
{
    return _MAV_RETURN_uint32_t_array(msg, offered, 8,  4);
}

/**
 * @brief Get field blocked_pct from mabur_cal_sweep_done message
 *
 * @return  Per rate, the share of the capacity dwell (percent) the drone's sender spent blocked on the device; 0 for a power sweep.
 */
static inline uint16_t mavlink_msg_mabur_cal_sweep_done_get_blocked_pct(const mavlink_message_t* msg, uint8_t *blocked_pct)
{
    return _MAV_RETURN_uint8_t_array(msg, blocked_pct, 8,  37);
}

/**
 * @brief Decode a mabur_cal_sweep_done message into a struct
 *
 * @param msg The message to decode
 * @param mabur_cal_sweep_done C-struct to decode the message contents into
 */
static inline void mavlink_msg_mabur_cal_sweep_done_decode(const mavlink_message_t* msg, mavlink_mabur_cal_sweep_done_t* mabur_cal_sweep_done)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mabur_cal_sweep_done->nonce = mavlink_msg_mabur_cal_sweep_done_get_nonce(msg);
    mavlink_msg_mabur_cal_sweep_done_get_offered(msg, mabur_cal_sweep_done->offered);
    mabur_cal_sweep_done->seq = mavlink_msg_mabur_cal_sweep_done_get_seq(msg);
    mavlink_msg_mabur_cal_sweep_done_get_blocked_pct(msg, mabur_cal_sweep_done->blocked_pct);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_LEN? msg->len : MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_LEN;
        memset(mabur_cal_sweep_done, 0, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_LEN);
    memcpy(mabur_cal_sweep_done, _MAV_PAYLOAD(msg), len);
#endif
}
