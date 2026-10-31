#pragma once
// MESSAGE MABUR_CAL_ABORT PACKING

#define MAVLINK_MSG_ID_MABUR_CAL_ABORT 60005


typedef struct __mavlink_mabur_cal_abort_t {
 uint32_t nonce; /*<  Nonce of the sweep to cancel.*/
} mavlink_mabur_cal_abort_t;

#define MAVLINK_MSG_ID_MABUR_CAL_ABORT_LEN 4
#define MAVLINK_MSG_ID_MABUR_CAL_ABORT_MIN_LEN 4
#define MAVLINK_MSG_ID_60005_LEN 4
#define MAVLINK_MSG_ID_60005_MIN_LEN 4

#define MAVLINK_MSG_ID_MABUR_CAL_ABORT_CRC 204
#define MAVLINK_MSG_ID_60005_CRC 204



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MABUR_CAL_ABORT { \
    60005, \
    "MABUR_CAL_ABORT", \
    1, \
    {  { "nonce", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_mabur_cal_abort_t, nonce) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MABUR_CAL_ABORT { \
    "MABUR_CAL_ABORT", \
    1, \
    {  { "nonce", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_mabur_cal_abort_t, nonce) }, \
         } \
}
#endif

/**
 * @brief Pack a mabur_cal_abort message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param nonce  Nonce of the sweep to cancel.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mabur_cal_abort_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t nonce)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MABUR_CAL_ABORT_LEN];
    _mav_put_uint32_t(buf, 0, nonce);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MABUR_CAL_ABORT_LEN);
#else
    mavlink_mabur_cal_abort_t packet;
    packet.nonce = nonce;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MABUR_CAL_ABORT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MABUR_CAL_ABORT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MABUR_CAL_ABORT_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_ABORT_LEN, MAVLINK_MSG_ID_MABUR_CAL_ABORT_CRC);
}

/**
 * @brief Pack a mabur_cal_abort message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param nonce  Nonce of the sweep to cancel.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mabur_cal_abort_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t nonce)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MABUR_CAL_ABORT_LEN];
    _mav_put_uint32_t(buf, 0, nonce);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MABUR_CAL_ABORT_LEN);
#else
    mavlink_mabur_cal_abort_t packet;
    packet.nonce = nonce;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MABUR_CAL_ABORT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MABUR_CAL_ABORT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MABUR_CAL_ABORT_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_ABORT_LEN, MAVLINK_MSG_ID_MABUR_CAL_ABORT_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MABUR_CAL_ABORT_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_ABORT_LEN);
#endif
}

/**
 * @brief Pack a mabur_cal_abort message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param nonce  Nonce of the sweep to cancel.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mabur_cal_abort_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t nonce)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MABUR_CAL_ABORT_LEN];
    _mav_put_uint32_t(buf, 0, nonce);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MABUR_CAL_ABORT_LEN);
#else
    mavlink_mabur_cal_abort_t packet;
    packet.nonce = nonce;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MABUR_CAL_ABORT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MABUR_CAL_ABORT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MABUR_CAL_ABORT_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_ABORT_LEN, MAVLINK_MSG_ID_MABUR_CAL_ABORT_CRC);
}

/**
 * @brief Encode a mabur_cal_abort struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mabur_cal_abort C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mabur_cal_abort_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mabur_cal_abort_t* mabur_cal_abort)
{
    return mavlink_msg_mabur_cal_abort_pack(system_id, component_id, msg, mabur_cal_abort->nonce);
}

/**
 * @brief Encode a mabur_cal_abort struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mabur_cal_abort C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mabur_cal_abort_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mabur_cal_abort_t* mabur_cal_abort)
{
    return mavlink_msg_mabur_cal_abort_pack_chan(system_id, component_id, chan, msg, mabur_cal_abort->nonce);
}

/**
 * @brief Encode a mabur_cal_abort struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param mabur_cal_abort C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mabur_cal_abort_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_mabur_cal_abort_t* mabur_cal_abort)
{
    return mavlink_msg_mabur_cal_abort_pack_status(system_id, component_id, _status, msg,  mabur_cal_abort->nonce);
}

/**
 * @brief Send a mabur_cal_abort message
 * @param chan MAVLink channel to send the message
 *
 * @param nonce  Nonce of the sweep to cancel.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mabur_cal_abort_send(mavlink_channel_t chan, uint32_t nonce)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MABUR_CAL_ABORT_LEN];
    _mav_put_uint32_t(buf, 0, nonce);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_CAL_ABORT, buf, MAVLINK_MSG_ID_MABUR_CAL_ABORT_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_ABORT_LEN, MAVLINK_MSG_ID_MABUR_CAL_ABORT_CRC);
#else
    mavlink_mabur_cal_abort_t packet;
    packet.nonce = nonce;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_CAL_ABORT, (const char *)&packet, MAVLINK_MSG_ID_MABUR_CAL_ABORT_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_ABORT_LEN, MAVLINK_MSG_ID_MABUR_CAL_ABORT_CRC);
#endif
}

/**
 * @brief Send a mabur_cal_abort message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mabur_cal_abort_send_struct(mavlink_channel_t chan, const mavlink_mabur_cal_abort_t* mabur_cal_abort)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mabur_cal_abort_send(chan, mabur_cal_abort->nonce);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_CAL_ABORT, (const char *)mabur_cal_abort, MAVLINK_MSG_ID_MABUR_CAL_ABORT_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_ABORT_LEN, MAVLINK_MSG_ID_MABUR_CAL_ABORT_CRC);
#endif
}

#if MAVLINK_MSG_ID_MABUR_CAL_ABORT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mabur_cal_abort_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t nonce)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, nonce);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_CAL_ABORT, buf, MAVLINK_MSG_ID_MABUR_CAL_ABORT_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_ABORT_LEN, MAVLINK_MSG_ID_MABUR_CAL_ABORT_CRC);
#else
    mavlink_mabur_cal_abort_t *packet = (mavlink_mabur_cal_abort_t *)msgbuf;
    packet->nonce = nonce;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_CAL_ABORT, (const char *)packet, MAVLINK_MSG_ID_MABUR_CAL_ABORT_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_ABORT_LEN, MAVLINK_MSG_ID_MABUR_CAL_ABORT_CRC);
#endif
}
#endif

#endif

// MESSAGE MABUR_CAL_ABORT UNPACKING


/**
 * @brief Get field nonce from mabur_cal_abort message
 *
 * @return  Nonce of the sweep to cancel.
 */
static inline uint32_t mavlink_msg_mabur_cal_abort_get_nonce(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Decode a mabur_cal_abort message into a struct
 *
 * @param msg The message to decode
 * @param mabur_cal_abort C-struct to decode the message contents into
 */
static inline void mavlink_msg_mabur_cal_abort_decode(const mavlink_message_t* msg, mavlink_mabur_cal_abort_t* mabur_cal_abort)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mabur_cal_abort->nonce = mavlink_msg_mabur_cal_abort_get_nonce(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MABUR_CAL_ABORT_LEN? msg->len : MAVLINK_MSG_ID_MABUR_CAL_ABORT_LEN;
        memset(mabur_cal_abort, 0, MAVLINK_MSG_ID_MABUR_CAL_ABORT_LEN);
    memcpy(mabur_cal_abort, _MAV_PAYLOAD(msg), len);
#endif
}
