#pragma once
// MESSAGE MABUR_IDENT_REQ PACKING

#define MAVLINK_MSG_ID_MABUR_IDENT_REQ 60006


typedef struct __mavlink_mabur_ident_req_t {
 uint8_t pad; /*<  Unused; MAVLink needs one field.*/
} mavlink_mabur_ident_req_t;

#define MAVLINK_MSG_ID_MABUR_IDENT_REQ_LEN 1
#define MAVLINK_MSG_ID_MABUR_IDENT_REQ_MIN_LEN 1
#define MAVLINK_MSG_ID_60006_LEN 1
#define MAVLINK_MSG_ID_60006_MIN_LEN 1

#define MAVLINK_MSG_ID_MABUR_IDENT_REQ_CRC 129
#define MAVLINK_MSG_ID_60006_CRC 129



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MABUR_IDENT_REQ { \
    60006, \
    "MABUR_IDENT_REQ", \
    1, \
    {  { "pad", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_mabur_ident_req_t, pad) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MABUR_IDENT_REQ { \
    "MABUR_IDENT_REQ", \
    1, \
    {  { "pad", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_mabur_ident_req_t, pad) }, \
         } \
}
#endif

/**
 * @brief Pack a mabur_ident_req message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param pad  Unused; MAVLink needs one field.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mabur_ident_req_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t pad)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MABUR_IDENT_REQ_LEN];
    _mav_put_uint8_t(buf, 0, pad);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MABUR_IDENT_REQ_LEN);
#else
    mavlink_mabur_ident_req_t packet;
    packet.pad = pad;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MABUR_IDENT_REQ_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MABUR_IDENT_REQ;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MABUR_IDENT_REQ_MIN_LEN, MAVLINK_MSG_ID_MABUR_IDENT_REQ_LEN, MAVLINK_MSG_ID_MABUR_IDENT_REQ_CRC);
}

/**
 * @brief Pack a mabur_ident_req message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param pad  Unused; MAVLink needs one field.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mabur_ident_req_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t pad)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MABUR_IDENT_REQ_LEN];
    _mav_put_uint8_t(buf, 0, pad);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MABUR_IDENT_REQ_LEN);
#else
    mavlink_mabur_ident_req_t packet;
    packet.pad = pad;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MABUR_IDENT_REQ_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MABUR_IDENT_REQ;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MABUR_IDENT_REQ_MIN_LEN, MAVLINK_MSG_ID_MABUR_IDENT_REQ_LEN, MAVLINK_MSG_ID_MABUR_IDENT_REQ_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MABUR_IDENT_REQ_MIN_LEN, MAVLINK_MSG_ID_MABUR_IDENT_REQ_LEN);
#endif
}

/**
 * @brief Pack a mabur_ident_req message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pad  Unused; MAVLink needs one field.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mabur_ident_req_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t pad)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MABUR_IDENT_REQ_LEN];
    _mav_put_uint8_t(buf, 0, pad);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MABUR_IDENT_REQ_LEN);
#else
    mavlink_mabur_ident_req_t packet;
    packet.pad = pad;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MABUR_IDENT_REQ_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MABUR_IDENT_REQ;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MABUR_IDENT_REQ_MIN_LEN, MAVLINK_MSG_ID_MABUR_IDENT_REQ_LEN, MAVLINK_MSG_ID_MABUR_IDENT_REQ_CRC);
}

/**
 * @brief Encode a mabur_ident_req struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mabur_ident_req C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mabur_ident_req_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mabur_ident_req_t* mabur_ident_req)
{
    return mavlink_msg_mabur_ident_req_pack(system_id, component_id, msg, mabur_ident_req->pad);
}

/**
 * @brief Encode a mabur_ident_req struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mabur_ident_req C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mabur_ident_req_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mabur_ident_req_t* mabur_ident_req)
{
    return mavlink_msg_mabur_ident_req_pack_chan(system_id, component_id, chan, msg, mabur_ident_req->pad);
}

/**
 * @brief Encode a mabur_ident_req struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param mabur_ident_req C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mabur_ident_req_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_mabur_ident_req_t* mabur_ident_req)
{
    return mavlink_msg_mabur_ident_req_pack_status(system_id, component_id, _status, msg,  mabur_ident_req->pad);
}

/**
 * @brief Send a mabur_ident_req message
 * @param chan MAVLink channel to send the message
 *
 * @param pad  Unused; MAVLink needs one field.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mabur_ident_req_send(mavlink_channel_t chan, uint8_t pad)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MABUR_IDENT_REQ_LEN];
    _mav_put_uint8_t(buf, 0, pad);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_IDENT_REQ, buf, MAVLINK_MSG_ID_MABUR_IDENT_REQ_MIN_LEN, MAVLINK_MSG_ID_MABUR_IDENT_REQ_LEN, MAVLINK_MSG_ID_MABUR_IDENT_REQ_CRC);
#else
    mavlink_mabur_ident_req_t packet;
    packet.pad = pad;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_IDENT_REQ, (const char *)&packet, MAVLINK_MSG_ID_MABUR_IDENT_REQ_MIN_LEN, MAVLINK_MSG_ID_MABUR_IDENT_REQ_LEN, MAVLINK_MSG_ID_MABUR_IDENT_REQ_CRC);
#endif
}

/**
 * @brief Send a mabur_ident_req message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mabur_ident_req_send_struct(mavlink_channel_t chan, const mavlink_mabur_ident_req_t* mabur_ident_req)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mabur_ident_req_send(chan, mabur_ident_req->pad);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_IDENT_REQ, (const char *)mabur_ident_req, MAVLINK_MSG_ID_MABUR_IDENT_REQ_MIN_LEN, MAVLINK_MSG_ID_MABUR_IDENT_REQ_LEN, MAVLINK_MSG_ID_MABUR_IDENT_REQ_CRC);
#endif
}

#if MAVLINK_MSG_ID_MABUR_IDENT_REQ_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mabur_ident_req_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t pad)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, pad);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_IDENT_REQ, buf, MAVLINK_MSG_ID_MABUR_IDENT_REQ_MIN_LEN, MAVLINK_MSG_ID_MABUR_IDENT_REQ_LEN, MAVLINK_MSG_ID_MABUR_IDENT_REQ_CRC);
#else
    mavlink_mabur_ident_req_t *packet = (mavlink_mabur_ident_req_t *)msgbuf;
    packet->pad = pad;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_IDENT_REQ, (const char *)packet, MAVLINK_MSG_ID_MABUR_IDENT_REQ_MIN_LEN, MAVLINK_MSG_ID_MABUR_IDENT_REQ_LEN, MAVLINK_MSG_ID_MABUR_IDENT_REQ_CRC);
#endif
}
#endif

#endif

// MESSAGE MABUR_IDENT_REQ UNPACKING


/**
 * @brief Get field pad from mabur_ident_req message
 *
 * @return  Unused; MAVLink needs one field.
 */
static inline uint8_t mavlink_msg_mabur_ident_req_get_pad(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a mabur_ident_req message into a struct
 *
 * @param msg The message to decode
 * @param mabur_ident_req C-struct to decode the message contents into
 */
static inline void mavlink_msg_mabur_ident_req_decode(const mavlink_message_t* msg, mavlink_mabur_ident_req_t* mabur_ident_req)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mabur_ident_req->pad = mavlink_msg_mabur_ident_req_get_pad(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MABUR_IDENT_REQ_LEN? msg->len : MAVLINK_MSG_ID_MABUR_IDENT_REQ_LEN;
        memset(mabur_ident_req, 0, MAVLINK_MSG_ID_MABUR_IDENT_REQ_LEN);
    memcpy(mabur_ident_req, _MAV_PAYLOAD(msg), len);
#endif
}
