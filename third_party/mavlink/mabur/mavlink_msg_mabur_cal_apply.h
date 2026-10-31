#pragma once
// MESSAGE MABUR_CAL_APPLY PACKING

#define MAVLINK_MSG_ID_MABUR_CAL_APPLY 60003


typedef struct __mavlink_mabur_cal_apply_t {
 uint32_t nonce; /*<  Session nonce for this apply.*/
 uint16_t eff[8]; /*<  air_clock efficiency per rung.*/
 uint8_t seq; /*<  Seq of the sweep that produced this table.*/
 uint8_t bw; /*<  Channel bandwidth (MHz) the table applies to.*/
 uint8_t has; /*<  Bitmask of which wall/efficiency slots are present.*/
 int8_t walls[8]; /*<  Rel power-index wall per rung.*/
 int8_t legacy; /*<  Wall for legacy/control frames (radio.legacy_wall_rel), from the 20 MHz mcs0 wall; -128 = leave.*/
} mavlink_mabur_cal_apply_t;

#define MAVLINK_MSG_ID_MABUR_CAL_APPLY_LEN 32
#define MAVLINK_MSG_ID_MABUR_CAL_APPLY_MIN_LEN 32
#define MAVLINK_MSG_ID_60003_LEN 32
#define MAVLINK_MSG_ID_60003_MIN_LEN 32

#define MAVLINK_MSG_ID_MABUR_CAL_APPLY_CRC 48
#define MAVLINK_MSG_ID_60003_CRC 48

#define MAVLINK_MSG_MABUR_CAL_APPLY_FIELD_EFF_LEN 8
#define MAVLINK_MSG_MABUR_CAL_APPLY_FIELD_WALLS_LEN 8

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MABUR_CAL_APPLY { \
    60003, \
    "MABUR_CAL_APPLY", \
    7, \
    {  { "nonce", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_mabur_cal_apply_t, nonce) }, \
         { "seq", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_mabur_cal_apply_t, seq) }, \
         { "bw", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_mabur_cal_apply_t, bw) }, \
         { "has", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_mabur_cal_apply_t, has) }, \
         { "walls", NULL, MAVLINK_TYPE_INT8_T, 8, 23, offsetof(mavlink_mabur_cal_apply_t, walls) }, \
         { "legacy", NULL, MAVLINK_TYPE_INT8_T, 0, 31, offsetof(mavlink_mabur_cal_apply_t, legacy) }, \
         { "eff", NULL, MAVLINK_TYPE_UINT16_T, 8, 4, offsetof(mavlink_mabur_cal_apply_t, eff) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MABUR_CAL_APPLY { \
    "MABUR_CAL_APPLY", \
    7, \
    {  { "nonce", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_mabur_cal_apply_t, nonce) }, \
         { "seq", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_mabur_cal_apply_t, seq) }, \
         { "bw", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_mabur_cal_apply_t, bw) }, \
         { "has", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_mabur_cal_apply_t, has) }, \
         { "walls", NULL, MAVLINK_TYPE_INT8_T, 8, 23, offsetof(mavlink_mabur_cal_apply_t, walls) }, \
         { "legacy", NULL, MAVLINK_TYPE_INT8_T, 0, 31, offsetof(mavlink_mabur_cal_apply_t, legacy) }, \
         { "eff", NULL, MAVLINK_TYPE_UINT16_T, 8, 4, offsetof(mavlink_mabur_cal_apply_t, eff) }, \
         } \
}
#endif

/**
 * @brief Pack a mabur_cal_apply message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param nonce  Session nonce for this apply.
 * @param seq  Seq of the sweep that produced this table.
 * @param bw  Channel bandwidth (MHz) the table applies to.
 * @param has  Bitmask of which wall/efficiency slots are present.
 * @param walls  Rel power-index wall per rung.
 * @param legacy  Wall for legacy/control frames (radio.legacy_wall_rel), from the 20 MHz mcs0 wall; -128 = leave.
 * @param eff  air_clock efficiency per rung.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mabur_cal_apply_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t nonce, uint8_t seq, uint8_t bw, uint8_t has, const int8_t *walls, int8_t legacy, const uint16_t *eff)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MABUR_CAL_APPLY_LEN];
    _mav_put_uint32_t(buf, 0, nonce);
    _mav_put_uint8_t(buf, 20, seq);
    _mav_put_uint8_t(buf, 21, bw);
    _mav_put_uint8_t(buf, 22, has);
    _mav_put_int8_t(buf, 31, legacy);
    _mav_put_uint16_t_array(buf, 4, eff, 8);
    _mav_put_int8_t_array(buf, 23, walls, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MABUR_CAL_APPLY_LEN);
#else
    mavlink_mabur_cal_apply_t packet;
    packet.nonce = nonce;
    packet.seq = seq;
    packet.bw = bw;
    packet.has = has;
    packet.legacy = legacy;
    mav_array_assign_uint16_t(packet.eff, eff, 8);
    mav_array_assign_int8_t(packet.walls, walls, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MABUR_CAL_APPLY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MABUR_CAL_APPLY;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MABUR_CAL_APPLY_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_APPLY_LEN, MAVLINK_MSG_ID_MABUR_CAL_APPLY_CRC);
}

/**
 * @brief Pack a mabur_cal_apply message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param nonce  Session nonce for this apply.
 * @param seq  Seq of the sweep that produced this table.
 * @param bw  Channel bandwidth (MHz) the table applies to.
 * @param has  Bitmask of which wall/efficiency slots are present.
 * @param walls  Rel power-index wall per rung.
 * @param legacy  Wall for legacy/control frames (radio.legacy_wall_rel), from the 20 MHz mcs0 wall; -128 = leave.
 * @param eff  air_clock efficiency per rung.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mabur_cal_apply_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t nonce, uint8_t seq, uint8_t bw, uint8_t has, const int8_t *walls, int8_t legacy, const uint16_t *eff)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MABUR_CAL_APPLY_LEN];
    _mav_put_uint32_t(buf, 0, nonce);
    _mav_put_uint8_t(buf, 20, seq);
    _mav_put_uint8_t(buf, 21, bw);
    _mav_put_uint8_t(buf, 22, has);
    _mav_put_int8_t(buf, 31, legacy);
    _mav_put_uint16_t_array(buf, 4, eff, 8);
    _mav_put_int8_t_array(buf, 23, walls, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MABUR_CAL_APPLY_LEN);
#else
    mavlink_mabur_cal_apply_t packet;
    packet.nonce = nonce;
    packet.seq = seq;
    packet.bw = bw;
    packet.has = has;
    packet.legacy = legacy;
    mav_array_memcpy(packet.eff, eff, sizeof(uint16_t)*8);
    mav_array_memcpy(packet.walls, walls, sizeof(int8_t)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MABUR_CAL_APPLY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MABUR_CAL_APPLY;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MABUR_CAL_APPLY_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_APPLY_LEN, MAVLINK_MSG_ID_MABUR_CAL_APPLY_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MABUR_CAL_APPLY_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_APPLY_LEN);
#endif
}

/**
 * @brief Pack a mabur_cal_apply message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param nonce  Session nonce for this apply.
 * @param seq  Seq of the sweep that produced this table.
 * @param bw  Channel bandwidth (MHz) the table applies to.
 * @param has  Bitmask of which wall/efficiency slots are present.
 * @param walls  Rel power-index wall per rung.
 * @param legacy  Wall for legacy/control frames (radio.legacy_wall_rel), from the 20 MHz mcs0 wall; -128 = leave.
 * @param eff  air_clock efficiency per rung.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mabur_cal_apply_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t nonce,uint8_t seq,uint8_t bw,uint8_t has,const int8_t *walls,int8_t legacy,const uint16_t *eff)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MABUR_CAL_APPLY_LEN];
    _mav_put_uint32_t(buf, 0, nonce);
    _mav_put_uint8_t(buf, 20, seq);
    _mav_put_uint8_t(buf, 21, bw);
    _mav_put_uint8_t(buf, 22, has);
    _mav_put_int8_t(buf, 31, legacy);
    _mav_put_uint16_t_array(buf, 4, eff, 8);
    _mav_put_int8_t_array(buf, 23, walls, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MABUR_CAL_APPLY_LEN);
#else
    mavlink_mabur_cal_apply_t packet;
    packet.nonce = nonce;
    packet.seq = seq;
    packet.bw = bw;
    packet.has = has;
    packet.legacy = legacy;
    mav_array_assign_uint16_t(packet.eff, eff, 8);
    mav_array_assign_int8_t(packet.walls, walls, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MABUR_CAL_APPLY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MABUR_CAL_APPLY;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MABUR_CAL_APPLY_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_APPLY_LEN, MAVLINK_MSG_ID_MABUR_CAL_APPLY_CRC);
}

/**
 * @brief Encode a mabur_cal_apply struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mabur_cal_apply C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mabur_cal_apply_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mabur_cal_apply_t* mabur_cal_apply)
{
    return mavlink_msg_mabur_cal_apply_pack(system_id, component_id, msg, mabur_cal_apply->nonce, mabur_cal_apply->seq, mabur_cal_apply->bw, mabur_cal_apply->has, mabur_cal_apply->walls, mabur_cal_apply->legacy, mabur_cal_apply->eff);
}

/**
 * @brief Encode a mabur_cal_apply struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mabur_cal_apply C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mabur_cal_apply_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mabur_cal_apply_t* mabur_cal_apply)
{
    return mavlink_msg_mabur_cal_apply_pack_chan(system_id, component_id, chan, msg, mabur_cal_apply->nonce, mabur_cal_apply->seq, mabur_cal_apply->bw, mabur_cal_apply->has, mabur_cal_apply->walls, mabur_cal_apply->legacy, mabur_cal_apply->eff);
}

/**
 * @brief Encode a mabur_cal_apply struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param mabur_cal_apply C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mabur_cal_apply_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_mabur_cal_apply_t* mabur_cal_apply)
{
    return mavlink_msg_mabur_cal_apply_pack_status(system_id, component_id, _status, msg,  mabur_cal_apply->nonce, mabur_cal_apply->seq, mabur_cal_apply->bw, mabur_cal_apply->has, mabur_cal_apply->walls, mabur_cal_apply->legacy, mabur_cal_apply->eff);
}

/**
 * @brief Send a mabur_cal_apply message
 * @param chan MAVLink channel to send the message
 *
 * @param nonce  Session nonce for this apply.
 * @param seq  Seq of the sweep that produced this table.
 * @param bw  Channel bandwidth (MHz) the table applies to.
 * @param has  Bitmask of which wall/efficiency slots are present.
 * @param walls  Rel power-index wall per rung.
 * @param legacy  Wall for legacy/control frames (radio.legacy_wall_rel), from the 20 MHz mcs0 wall; -128 = leave.
 * @param eff  air_clock efficiency per rung.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mabur_cal_apply_send(mavlink_channel_t chan, uint32_t nonce, uint8_t seq, uint8_t bw, uint8_t has, const int8_t *walls, int8_t legacy, const uint16_t *eff)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MABUR_CAL_APPLY_LEN];
    _mav_put_uint32_t(buf, 0, nonce);
    _mav_put_uint8_t(buf, 20, seq);
    _mav_put_uint8_t(buf, 21, bw);
    _mav_put_uint8_t(buf, 22, has);
    _mav_put_int8_t(buf, 31, legacy);
    _mav_put_uint16_t_array(buf, 4, eff, 8);
    _mav_put_int8_t_array(buf, 23, walls, 8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_CAL_APPLY, buf, MAVLINK_MSG_ID_MABUR_CAL_APPLY_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_APPLY_LEN, MAVLINK_MSG_ID_MABUR_CAL_APPLY_CRC);
#else
    mavlink_mabur_cal_apply_t packet;
    packet.nonce = nonce;
    packet.seq = seq;
    packet.bw = bw;
    packet.has = has;
    packet.legacy = legacy;
    mav_array_assign_uint16_t(packet.eff, eff, 8);
    mav_array_assign_int8_t(packet.walls, walls, 8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_CAL_APPLY, (const char *)&packet, MAVLINK_MSG_ID_MABUR_CAL_APPLY_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_APPLY_LEN, MAVLINK_MSG_ID_MABUR_CAL_APPLY_CRC);
#endif
}

/**
 * @brief Send a mabur_cal_apply message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mabur_cal_apply_send_struct(mavlink_channel_t chan, const mavlink_mabur_cal_apply_t* mabur_cal_apply)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mabur_cal_apply_send(chan, mabur_cal_apply->nonce, mabur_cal_apply->seq, mabur_cal_apply->bw, mabur_cal_apply->has, mabur_cal_apply->walls, mabur_cal_apply->legacy, mabur_cal_apply->eff);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_CAL_APPLY, (const char *)mabur_cal_apply, MAVLINK_MSG_ID_MABUR_CAL_APPLY_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_APPLY_LEN, MAVLINK_MSG_ID_MABUR_CAL_APPLY_CRC);
#endif
}

#if MAVLINK_MSG_ID_MABUR_CAL_APPLY_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mabur_cal_apply_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t nonce, uint8_t seq, uint8_t bw, uint8_t has, const int8_t *walls, int8_t legacy, const uint16_t *eff)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, nonce);
    _mav_put_uint8_t(buf, 20, seq);
    _mav_put_uint8_t(buf, 21, bw);
    _mav_put_uint8_t(buf, 22, has);
    _mav_put_int8_t(buf, 31, legacy);
    _mav_put_uint16_t_array(buf, 4, eff, 8);
    _mav_put_int8_t_array(buf, 23, walls, 8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_CAL_APPLY, buf, MAVLINK_MSG_ID_MABUR_CAL_APPLY_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_APPLY_LEN, MAVLINK_MSG_ID_MABUR_CAL_APPLY_CRC);
#else
    mavlink_mabur_cal_apply_t *packet = (mavlink_mabur_cal_apply_t *)msgbuf;
    packet->nonce = nonce;
    packet->seq = seq;
    packet->bw = bw;
    packet->has = has;
    packet->legacy = legacy;
    mav_array_assign_uint16_t(packet->eff, eff, 8);
    mav_array_assign_int8_t(packet->walls, walls, 8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_CAL_APPLY, (const char *)packet, MAVLINK_MSG_ID_MABUR_CAL_APPLY_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_APPLY_LEN, MAVLINK_MSG_ID_MABUR_CAL_APPLY_CRC);
#endif
}
#endif

#endif

// MESSAGE MABUR_CAL_APPLY UNPACKING


/**
 * @brief Get field nonce from mabur_cal_apply message
 *
 * @return  Session nonce for this apply.
 */
static inline uint32_t mavlink_msg_mabur_cal_apply_get_nonce(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field seq from mabur_cal_apply message
 *
 * @return  Seq of the sweep that produced this table.
 */
static inline uint8_t mavlink_msg_mabur_cal_apply_get_seq(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field bw from mabur_cal_apply message
 *
 * @return  Channel bandwidth (MHz) the table applies to.
 */
static inline uint8_t mavlink_msg_mabur_cal_apply_get_bw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  21);
}

/**
 * @brief Get field has from mabur_cal_apply message
 *
 * @return  Bitmask of which wall/efficiency slots are present.
 */
static inline uint8_t mavlink_msg_mabur_cal_apply_get_has(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  22);
}

/**
 * @brief Get field walls from mabur_cal_apply message
 *
 * @return  Rel power-index wall per rung.
 */
static inline uint16_t mavlink_msg_mabur_cal_apply_get_walls(const mavlink_message_t* msg, int8_t *walls)
{
    return _MAV_RETURN_int8_t_array(msg, walls, 8,  23);
}

/**
 * @brief Get field legacy from mabur_cal_apply message
 *
 * @return  Wall for legacy/control frames (radio.legacy_wall_rel), from the 20 MHz mcs0 wall; -128 = leave.
 */
static inline int8_t mavlink_msg_mabur_cal_apply_get_legacy(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  31);
}

/**
 * @brief Get field eff from mabur_cal_apply message
 *
 * @return  air_clock efficiency per rung.
 */
static inline uint16_t mavlink_msg_mabur_cal_apply_get_eff(const mavlink_message_t* msg, uint16_t *eff)
{
    return _MAV_RETURN_uint16_t_array(msg, eff, 8,  4);
}

/**
 * @brief Decode a mabur_cal_apply message into a struct
 *
 * @param msg The message to decode
 * @param mabur_cal_apply C-struct to decode the message contents into
 */
static inline void mavlink_msg_mabur_cal_apply_decode(const mavlink_message_t* msg, mavlink_mabur_cal_apply_t* mabur_cal_apply)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mabur_cal_apply->nonce = mavlink_msg_mabur_cal_apply_get_nonce(msg);
    mavlink_msg_mabur_cal_apply_get_eff(msg, mabur_cal_apply->eff);
    mabur_cal_apply->seq = mavlink_msg_mabur_cal_apply_get_seq(msg);
    mabur_cal_apply->bw = mavlink_msg_mabur_cal_apply_get_bw(msg);
    mabur_cal_apply->has = mavlink_msg_mabur_cal_apply_get_has(msg);
    mavlink_msg_mabur_cal_apply_get_walls(msg, mabur_cal_apply->walls);
    mabur_cal_apply->legacy = mavlink_msg_mabur_cal_apply_get_legacy(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MABUR_CAL_APPLY_LEN? msg->len : MAVLINK_MSG_ID_MABUR_CAL_APPLY_LEN;
        memset(mabur_cal_apply, 0, MAVLINK_MSG_ID_MABUR_CAL_APPLY_LEN);
    memcpy(mabur_cal_apply, _MAV_PAYLOAD(msg), len);
#endif
}
