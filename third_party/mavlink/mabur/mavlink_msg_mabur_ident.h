#pragma once
// MESSAGE MABUR_IDENT PACKING

#define MAVLINK_MSG_ID_MABUR_IDENT 60007


typedef struct __mavlink_mabur_ident_t {
 char version[32]; /*<  Build version.*/
 char board[48]; /*<  Board name.*/
 char chip[16]; /*<  Radio chip name.*/
} mavlink_mabur_ident_t;

#define MAVLINK_MSG_ID_MABUR_IDENT_LEN 96
#define MAVLINK_MSG_ID_MABUR_IDENT_MIN_LEN 96
#define MAVLINK_MSG_ID_60007_LEN 96
#define MAVLINK_MSG_ID_60007_MIN_LEN 96

#define MAVLINK_MSG_ID_MABUR_IDENT_CRC 157
#define MAVLINK_MSG_ID_60007_CRC 157

#define MAVLINK_MSG_MABUR_IDENT_FIELD_VERSION_LEN 32
#define MAVLINK_MSG_MABUR_IDENT_FIELD_BOARD_LEN 48
#define MAVLINK_MSG_MABUR_IDENT_FIELD_CHIP_LEN 16

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MABUR_IDENT { \
    60007, \
    "MABUR_IDENT", \
    3, \
    {  { "version", NULL, MAVLINK_TYPE_CHAR, 32, 0, offsetof(mavlink_mabur_ident_t, version) }, \
         { "board", NULL, MAVLINK_TYPE_CHAR, 48, 32, offsetof(mavlink_mabur_ident_t, board) }, \
         { "chip", NULL, MAVLINK_TYPE_CHAR, 16, 80, offsetof(mavlink_mabur_ident_t, chip) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MABUR_IDENT { \
    "MABUR_IDENT", \
    3, \
    {  { "version", NULL, MAVLINK_TYPE_CHAR, 32, 0, offsetof(mavlink_mabur_ident_t, version) }, \
         { "board", NULL, MAVLINK_TYPE_CHAR, 48, 32, offsetof(mavlink_mabur_ident_t, board) }, \
         { "chip", NULL, MAVLINK_TYPE_CHAR, 16, 80, offsetof(mavlink_mabur_ident_t, chip) }, \
         } \
}
#endif

/**
 * @brief Pack a mabur_ident message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param version  Build version.
 * @param board  Board name.
 * @param chip  Radio chip name.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mabur_ident_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const char *version, const char *board, const char *chip)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MABUR_IDENT_LEN];

    _mav_put_char_array(buf, 0, version, 32);
    _mav_put_char_array(buf, 32, board, 48);
    _mav_put_char_array(buf, 80, chip, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MABUR_IDENT_LEN);
#else
    mavlink_mabur_ident_t packet;

    mav_array_assign_char(packet.version, version, 32);
    mav_array_assign_char(packet.board, board, 48);
    mav_array_assign_char(packet.chip, chip, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MABUR_IDENT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MABUR_IDENT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MABUR_IDENT_MIN_LEN, MAVLINK_MSG_ID_MABUR_IDENT_LEN, MAVLINK_MSG_ID_MABUR_IDENT_CRC);
}

/**
 * @brief Pack a mabur_ident message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param version  Build version.
 * @param board  Board name.
 * @param chip  Radio chip name.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mabur_ident_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               const char *version, const char *board, const char *chip)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MABUR_IDENT_LEN];

    _mav_put_char_array(buf, 0, version, 32);
    _mav_put_char_array(buf, 32, board, 48);
    _mav_put_char_array(buf, 80, chip, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MABUR_IDENT_LEN);
#else
    mavlink_mabur_ident_t packet;

    mav_array_memcpy(packet.version, version, sizeof(char)*32);
    mav_array_memcpy(packet.board, board, sizeof(char)*48);
    mav_array_memcpy(packet.chip, chip, sizeof(char)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MABUR_IDENT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MABUR_IDENT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MABUR_IDENT_MIN_LEN, MAVLINK_MSG_ID_MABUR_IDENT_LEN, MAVLINK_MSG_ID_MABUR_IDENT_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MABUR_IDENT_MIN_LEN, MAVLINK_MSG_ID_MABUR_IDENT_LEN);
#endif
}

/**
 * @brief Pack a mabur_ident message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param version  Build version.
 * @param board  Board name.
 * @param chip  Radio chip name.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mabur_ident_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const char *version,const char *board,const char *chip)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MABUR_IDENT_LEN];

    _mav_put_char_array(buf, 0, version, 32);
    _mav_put_char_array(buf, 32, board, 48);
    _mav_put_char_array(buf, 80, chip, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MABUR_IDENT_LEN);
#else
    mavlink_mabur_ident_t packet;

    mav_array_assign_char(packet.version, version, 32);
    mav_array_assign_char(packet.board, board, 48);
    mav_array_assign_char(packet.chip, chip, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MABUR_IDENT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MABUR_IDENT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MABUR_IDENT_MIN_LEN, MAVLINK_MSG_ID_MABUR_IDENT_LEN, MAVLINK_MSG_ID_MABUR_IDENT_CRC);
}

/**
 * @brief Encode a mabur_ident struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mabur_ident C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mabur_ident_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mabur_ident_t* mabur_ident)
{
    return mavlink_msg_mabur_ident_pack(system_id, component_id, msg, mabur_ident->version, mabur_ident->board, mabur_ident->chip);
}

/**
 * @brief Encode a mabur_ident struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mabur_ident C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mabur_ident_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mabur_ident_t* mabur_ident)
{
    return mavlink_msg_mabur_ident_pack_chan(system_id, component_id, chan, msg, mabur_ident->version, mabur_ident->board, mabur_ident->chip);
}

/**
 * @brief Encode a mabur_ident struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param mabur_ident C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mabur_ident_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_mabur_ident_t* mabur_ident)
{
    return mavlink_msg_mabur_ident_pack_status(system_id, component_id, _status, msg,  mabur_ident->version, mabur_ident->board, mabur_ident->chip);
}

/**
 * @brief Send a mabur_ident message
 * @param chan MAVLink channel to send the message
 *
 * @param version  Build version.
 * @param board  Board name.
 * @param chip  Radio chip name.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mabur_ident_send(mavlink_channel_t chan, const char *version, const char *board, const char *chip)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MABUR_IDENT_LEN];

    _mav_put_char_array(buf, 0, version, 32);
    _mav_put_char_array(buf, 32, board, 48);
    _mav_put_char_array(buf, 80, chip, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_IDENT, buf, MAVLINK_MSG_ID_MABUR_IDENT_MIN_LEN, MAVLINK_MSG_ID_MABUR_IDENT_LEN, MAVLINK_MSG_ID_MABUR_IDENT_CRC);
#else
    mavlink_mabur_ident_t packet;

    mav_array_assign_char(packet.version, version, 32);
    mav_array_assign_char(packet.board, board, 48);
    mav_array_assign_char(packet.chip, chip, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_IDENT, (const char *)&packet, MAVLINK_MSG_ID_MABUR_IDENT_MIN_LEN, MAVLINK_MSG_ID_MABUR_IDENT_LEN, MAVLINK_MSG_ID_MABUR_IDENT_CRC);
#endif
}

/**
 * @brief Send a mabur_ident message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mabur_ident_send_struct(mavlink_channel_t chan, const mavlink_mabur_ident_t* mabur_ident)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mabur_ident_send(chan, mabur_ident->version, mabur_ident->board, mabur_ident->chip);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_IDENT, (const char *)mabur_ident, MAVLINK_MSG_ID_MABUR_IDENT_MIN_LEN, MAVLINK_MSG_ID_MABUR_IDENT_LEN, MAVLINK_MSG_ID_MABUR_IDENT_CRC);
#endif
}

#if MAVLINK_MSG_ID_MABUR_IDENT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mabur_ident_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const char *version, const char *board, const char *chip)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mav_put_char_array(buf, 0, version, 32);
    _mav_put_char_array(buf, 32, board, 48);
    _mav_put_char_array(buf, 80, chip, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_IDENT, buf, MAVLINK_MSG_ID_MABUR_IDENT_MIN_LEN, MAVLINK_MSG_ID_MABUR_IDENT_LEN, MAVLINK_MSG_ID_MABUR_IDENT_CRC);
#else
    mavlink_mabur_ident_t *packet = (mavlink_mabur_ident_t *)msgbuf;

    mav_array_assign_char(packet->version, version, 32);
    mav_array_assign_char(packet->board, board, 48);
    mav_array_assign_char(packet->chip, chip, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_IDENT, (const char *)packet, MAVLINK_MSG_ID_MABUR_IDENT_MIN_LEN, MAVLINK_MSG_ID_MABUR_IDENT_LEN, MAVLINK_MSG_ID_MABUR_IDENT_CRC);
#endif
}
#endif

#endif

// MESSAGE MABUR_IDENT UNPACKING


/**
 * @brief Get field version from mabur_ident message
 *
 * @return  Build version.
 */
static inline uint16_t mavlink_msg_mabur_ident_get_version(const mavlink_message_t* msg, char *version)
{
    return _MAV_RETURN_char_array(msg, version, 32,  0);
}

/**
 * @brief Get field board from mabur_ident message
 *
 * @return  Board name.
 */
static inline uint16_t mavlink_msg_mabur_ident_get_board(const mavlink_message_t* msg, char *board)
{
    return _MAV_RETURN_char_array(msg, board, 48,  32);
}

/**
 * @brief Get field chip from mabur_ident message
 *
 * @return  Radio chip name.
 */
static inline uint16_t mavlink_msg_mabur_ident_get_chip(const mavlink_message_t* msg, char *chip)
{
    return _MAV_RETURN_char_array(msg, chip, 16,  80);
}

/**
 * @brief Decode a mabur_ident message into a struct
 *
 * @param msg The message to decode
 * @param mabur_ident C-struct to decode the message contents into
 */
static inline void mavlink_msg_mabur_ident_decode(const mavlink_message_t* msg, mavlink_mabur_ident_t* mabur_ident)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mabur_ident_get_version(msg, mabur_ident->version);
    mavlink_msg_mabur_ident_get_board(msg, mabur_ident->board);
    mavlink_msg_mabur_ident_get_chip(msg, mabur_ident->chip);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MABUR_IDENT_LEN? msg->len : MAVLINK_MSG_ID_MABUR_IDENT_LEN;
        memset(mabur_ident, 0, MAVLINK_MSG_ID_MABUR_IDENT_LEN);
    memcpy(mabur_ident, _MAV_PAYLOAD(msg), len);
#endif
}
