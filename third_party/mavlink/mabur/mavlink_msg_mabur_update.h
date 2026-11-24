#pragma once
// MESSAGE MABUR_UPDATE PACKING

#define MAVLINK_MSG_ID_MABUR_UPDATE 60008


typedef struct __mavlink_mabur_update_t {
 uint8_t cmd; /*<  Command.*/
 uint8_t sha256[32]; /*<  SHA-256 of the uploaded binary (START).*/
} mavlink_mabur_update_t;

#define MAVLINK_MSG_ID_MABUR_UPDATE_LEN 33
#define MAVLINK_MSG_ID_MABUR_UPDATE_MIN_LEN 33
#define MAVLINK_MSG_ID_60008_LEN 33
#define MAVLINK_MSG_ID_60008_MIN_LEN 33

#define MAVLINK_MSG_ID_MABUR_UPDATE_CRC 127
#define MAVLINK_MSG_ID_60008_CRC 127

#define MAVLINK_MSG_MABUR_UPDATE_FIELD_SHA256_LEN 32

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MABUR_UPDATE { \
    60008, \
    "MABUR_UPDATE", \
    2, \
    {  { "cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_mabur_update_t, cmd) }, \
         { "sha256", NULL, MAVLINK_TYPE_UINT8_T, 32, 1, offsetof(mavlink_mabur_update_t, sha256) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MABUR_UPDATE { \
    "MABUR_UPDATE", \
    2, \
    {  { "cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_mabur_update_t, cmd) }, \
         { "sha256", NULL, MAVLINK_TYPE_UINT8_T, 32, 1, offsetof(mavlink_mabur_update_t, sha256) }, \
         } \
}
#endif

/**
 * @brief Pack a mabur_update message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param cmd  Command.
 * @param sha256  SHA-256 of the uploaded binary (START).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mabur_update_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t cmd, const uint8_t *sha256)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MABUR_UPDATE_LEN];
    _mav_put_uint8_t(buf, 0, cmd);
    _mav_put_uint8_t_array(buf, 1, sha256, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MABUR_UPDATE_LEN);
#else
    mavlink_mabur_update_t packet;
    packet.cmd = cmd;
    mav_array_assign_uint8_t(packet.sha256, sha256, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MABUR_UPDATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MABUR_UPDATE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MABUR_UPDATE_MIN_LEN, MAVLINK_MSG_ID_MABUR_UPDATE_LEN, MAVLINK_MSG_ID_MABUR_UPDATE_CRC);
}

/**
 * @brief Pack a mabur_update message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param cmd  Command.
 * @param sha256  SHA-256 of the uploaded binary (START).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mabur_update_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t cmd, const uint8_t *sha256)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MABUR_UPDATE_LEN];
    _mav_put_uint8_t(buf, 0, cmd);
    _mav_put_uint8_t_array(buf, 1, sha256, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MABUR_UPDATE_LEN);
#else
    mavlink_mabur_update_t packet;
    packet.cmd = cmd;
    mav_array_memcpy(packet.sha256, sha256, sizeof(uint8_t)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MABUR_UPDATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MABUR_UPDATE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MABUR_UPDATE_MIN_LEN, MAVLINK_MSG_ID_MABUR_UPDATE_LEN, MAVLINK_MSG_ID_MABUR_UPDATE_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MABUR_UPDATE_MIN_LEN, MAVLINK_MSG_ID_MABUR_UPDATE_LEN);
#endif
}

/**
 * @brief Pack a mabur_update message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param cmd  Command.
 * @param sha256  SHA-256 of the uploaded binary (START).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mabur_update_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t cmd,const uint8_t *sha256)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MABUR_UPDATE_LEN];
    _mav_put_uint8_t(buf, 0, cmd);
    _mav_put_uint8_t_array(buf, 1, sha256, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MABUR_UPDATE_LEN);
#else
    mavlink_mabur_update_t packet;
    packet.cmd = cmd;
    mav_array_assign_uint8_t(packet.sha256, sha256, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MABUR_UPDATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MABUR_UPDATE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MABUR_UPDATE_MIN_LEN, MAVLINK_MSG_ID_MABUR_UPDATE_LEN, MAVLINK_MSG_ID_MABUR_UPDATE_CRC);
}

/**
 * @brief Encode a mabur_update struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mabur_update C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mabur_update_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mabur_update_t* mabur_update)
{
    return mavlink_msg_mabur_update_pack(system_id, component_id, msg, mabur_update->cmd, mabur_update->sha256);
}

/**
 * @brief Encode a mabur_update struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mabur_update C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mabur_update_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mabur_update_t* mabur_update)
{
    return mavlink_msg_mabur_update_pack_chan(system_id, component_id, chan, msg, mabur_update->cmd, mabur_update->sha256);
}

/**
 * @brief Encode a mabur_update struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param mabur_update C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mabur_update_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_mabur_update_t* mabur_update)
{
    return mavlink_msg_mabur_update_pack_status(system_id, component_id, _status, msg,  mabur_update->cmd, mabur_update->sha256);
}

/**
 * @brief Send a mabur_update message
 * @param chan MAVLink channel to send the message
 *
 * @param cmd  Command.
 * @param sha256  SHA-256 of the uploaded binary (START).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mabur_update_send(mavlink_channel_t chan, uint8_t cmd, const uint8_t *sha256)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MABUR_UPDATE_LEN];
    _mav_put_uint8_t(buf, 0, cmd);
    _mav_put_uint8_t_array(buf, 1, sha256, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_UPDATE, buf, MAVLINK_MSG_ID_MABUR_UPDATE_MIN_LEN, MAVLINK_MSG_ID_MABUR_UPDATE_LEN, MAVLINK_MSG_ID_MABUR_UPDATE_CRC);
#else
    mavlink_mabur_update_t packet;
    packet.cmd = cmd;
    mav_array_assign_uint8_t(packet.sha256, sha256, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_UPDATE, (const char *)&packet, MAVLINK_MSG_ID_MABUR_UPDATE_MIN_LEN, MAVLINK_MSG_ID_MABUR_UPDATE_LEN, MAVLINK_MSG_ID_MABUR_UPDATE_CRC);
#endif
}

/**
 * @brief Send a mabur_update message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mabur_update_send_struct(mavlink_channel_t chan, const mavlink_mabur_update_t* mabur_update)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mabur_update_send(chan, mabur_update->cmd, mabur_update->sha256);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_UPDATE, (const char *)mabur_update, MAVLINK_MSG_ID_MABUR_UPDATE_MIN_LEN, MAVLINK_MSG_ID_MABUR_UPDATE_LEN, MAVLINK_MSG_ID_MABUR_UPDATE_CRC);
#endif
}

#if MAVLINK_MSG_ID_MABUR_UPDATE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mabur_update_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t cmd, const uint8_t *sha256)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, cmd);
    _mav_put_uint8_t_array(buf, 1, sha256, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_UPDATE, buf, MAVLINK_MSG_ID_MABUR_UPDATE_MIN_LEN, MAVLINK_MSG_ID_MABUR_UPDATE_LEN, MAVLINK_MSG_ID_MABUR_UPDATE_CRC);
#else
    mavlink_mabur_update_t *packet = (mavlink_mabur_update_t *)msgbuf;
    packet->cmd = cmd;
    mav_array_assign_uint8_t(packet->sha256, sha256, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_UPDATE, (const char *)packet, MAVLINK_MSG_ID_MABUR_UPDATE_MIN_LEN, MAVLINK_MSG_ID_MABUR_UPDATE_LEN, MAVLINK_MSG_ID_MABUR_UPDATE_CRC);
#endif
}
#endif

#endif

// MESSAGE MABUR_UPDATE UNPACKING


/**
 * @brief Get field cmd from mabur_update message
 *
 * @return  Command.
 */
static inline uint8_t mavlink_msg_mabur_update_get_cmd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field sha256 from mabur_update message
 *
 * @return  SHA-256 of the uploaded binary (START).
 */
static inline uint16_t mavlink_msg_mabur_update_get_sha256(const mavlink_message_t* msg, uint8_t *sha256)
{
    return _MAV_RETURN_uint8_t_array(msg, sha256, 32,  1);
}

/**
 * @brief Decode a mabur_update message into a struct
 *
 * @param msg The message to decode
 * @param mabur_update C-struct to decode the message contents into
 */
static inline void mavlink_msg_mabur_update_decode(const mavlink_message_t* msg, mavlink_mabur_update_t* mabur_update)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mabur_update->cmd = mavlink_msg_mabur_update_get_cmd(msg);
    mavlink_msg_mabur_update_get_sha256(msg, mabur_update->sha256);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MABUR_UPDATE_LEN? msg->len : MAVLINK_MSG_ID_MABUR_UPDATE_LEN;
        memset(mabur_update, 0, MAVLINK_MSG_ID_MABUR_UPDATE_LEN);
    memcpy(mabur_update, _MAV_PAYLOAD(msg), len);
#endif
}
