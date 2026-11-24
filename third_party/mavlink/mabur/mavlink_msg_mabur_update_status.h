#pragma once
// MESSAGE MABUR_UPDATE_STATUS PACKING

#define MAVLINK_MSG_ID_MABUR_UPDATE_STATUS 60009


typedef struct __mavlink_mabur_update_status_t {
 uint8_t state; /*<  Update state.*/
 uint8_t reason; /*<  Reason for refused or rolled back.*/
 uint8_t trial_s; /*<  Seconds left in the trial.*/
} mavlink_mabur_update_status_t;

#define MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_LEN 3
#define MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_MIN_LEN 3
#define MAVLINK_MSG_ID_60009_LEN 3
#define MAVLINK_MSG_ID_60009_MIN_LEN 3

#define MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_CRC 113
#define MAVLINK_MSG_ID_60009_CRC 113



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MABUR_UPDATE_STATUS { \
    60009, \
    "MABUR_UPDATE_STATUS", \
    3, \
    {  { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_mabur_update_status_t, state) }, \
         { "reason", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_mabur_update_status_t, reason) }, \
         { "trial_s", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_mabur_update_status_t, trial_s) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MABUR_UPDATE_STATUS { \
    "MABUR_UPDATE_STATUS", \
    3, \
    {  { "state", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_mabur_update_status_t, state) }, \
         { "reason", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_mabur_update_status_t, reason) }, \
         { "trial_s", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_mabur_update_status_t, trial_s) }, \
         } \
}
#endif

/**
 * @brief Pack a mabur_update_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param state  Update state.
 * @param reason  Reason for refused or rolled back.
 * @param trial_s  Seconds left in the trial.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mabur_update_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t state, uint8_t reason, uint8_t trial_s)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_LEN];
    _mav_put_uint8_t(buf, 0, state);
    _mav_put_uint8_t(buf, 1, reason);
    _mav_put_uint8_t(buf, 2, trial_s);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_LEN);
#else
    mavlink_mabur_update_status_t packet;
    packet.state = state;
    packet.reason = reason;
    packet.trial_s = trial_s;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MABUR_UPDATE_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_MIN_LEN, MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_LEN, MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_CRC);
}

/**
 * @brief Pack a mabur_update_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param state  Update state.
 * @param reason  Reason for refused or rolled back.
 * @param trial_s  Seconds left in the trial.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mabur_update_status_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t state, uint8_t reason, uint8_t trial_s)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_LEN];
    _mav_put_uint8_t(buf, 0, state);
    _mav_put_uint8_t(buf, 1, reason);
    _mav_put_uint8_t(buf, 2, trial_s);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_LEN);
#else
    mavlink_mabur_update_status_t packet;
    packet.state = state;
    packet.reason = reason;
    packet.trial_s = trial_s;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MABUR_UPDATE_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_MIN_LEN, MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_LEN, MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_MIN_LEN, MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_LEN);
#endif
}

/**
 * @brief Pack a mabur_update_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param state  Update state.
 * @param reason  Reason for refused or rolled back.
 * @param trial_s  Seconds left in the trial.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mabur_update_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t state,uint8_t reason,uint8_t trial_s)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_LEN];
    _mav_put_uint8_t(buf, 0, state);
    _mav_put_uint8_t(buf, 1, reason);
    _mav_put_uint8_t(buf, 2, trial_s);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_LEN);
#else
    mavlink_mabur_update_status_t packet;
    packet.state = state;
    packet.reason = reason;
    packet.trial_s = trial_s;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MABUR_UPDATE_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_MIN_LEN, MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_LEN, MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_CRC);
}

/**
 * @brief Encode a mabur_update_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mabur_update_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mabur_update_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mabur_update_status_t* mabur_update_status)
{
    return mavlink_msg_mabur_update_status_pack(system_id, component_id, msg, mabur_update_status->state, mabur_update_status->reason, mabur_update_status->trial_s);
}

/**
 * @brief Encode a mabur_update_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mabur_update_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mabur_update_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mabur_update_status_t* mabur_update_status)
{
    return mavlink_msg_mabur_update_status_pack_chan(system_id, component_id, chan, msg, mabur_update_status->state, mabur_update_status->reason, mabur_update_status->trial_s);
}

/**
 * @brief Encode a mabur_update_status struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param mabur_update_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mabur_update_status_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_mabur_update_status_t* mabur_update_status)
{
    return mavlink_msg_mabur_update_status_pack_status(system_id, component_id, _status, msg,  mabur_update_status->state, mabur_update_status->reason, mabur_update_status->trial_s);
}

/**
 * @brief Send a mabur_update_status message
 * @param chan MAVLink channel to send the message
 *
 * @param state  Update state.
 * @param reason  Reason for refused or rolled back.
 * @param trial_s  Seconds left in the trial.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mabur_update_status_send(mavlink_channel_t chan, uint8_t state, uint8_t reason, uint8_t trial_s)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_LEN];
    _mav_put_uint8_t(buf, 0, state);
    _mav_put_uint8_t(buf, 1, reason);
    _mav_put_uint8_t(buf, 2, trial_s);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_UPDATE_STATUS, buf, MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_MIN_LEN, MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_LEN, MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_CRC);
#else
    mavlink_mabur_update_status_t packet;
    packet.state = state;
    packet.reason = reason;
    packet.trial_s = trial_s;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_UPDATE_STATUS, (const char *)&packet, MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_MIN_LEN, MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_LEN, MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_CRC);
#endif
}

/**
 * @brief Send a mabur_update_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mabur_update_status_send_struct(mavlink_channel_t chan, const mavlink_mabur_update_status_t* mabur_update_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mabur_update_status_send(chan, mabur_update_status->state, mabur_update_status->reason, mabur_update_status->trial_s);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_UPDATE_STATUS, (const char *)mabur_update_status, MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_MIN_LEN, MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_LEN, MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mabur_update_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t state, uint8_t reason, uint8_t trial_s)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, state);
    _mav_put_uint8_t(buf, 1, reason);
    _mav_put_uint8_t(buf, 2, trial_s);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_UPDATE_STATUS, buf, MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_MIN_LEN, MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_LEN, MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_CRC);
#else
    mavlink_mabur_update_status_t *packet = (mavlink_mabur_update_status_t *)msgbuf;
    packet->state = state;
    packet->reason = reason;
    packet->trial_s = trial_s;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_UPDATE_STATUS, (const char *)packet, MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_MIN_LEN, MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_LEN, MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE MABUR_UPDATE_STATUS UNPACKING


/**
 * @brief Get field state from mabur_update_status message
 *
 * @return  Update state.
 */
static inline uint8_t mavlink_msg_mabur_update_status_get_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field reason from mabur_update_status message
 *
 * @return  Reason for refused or rolled back.
 */
static inline uint8_t mavlink_msg_mabur_update_status_get_reason(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field trial_s from mabur_update_status message
 *
 * @return  Seconds left in the trial.
 */
static inline uint8_t mavlink_msg_mabur_update_status_get_trial_s(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a mabur_update_status message into a struct
 *
 * @param msg The message to decode
 * @param mabur_update_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_mabur_update_status_decode(const mavlink_message_t* msg, mavlink_mabur_update_status_t* mabur_update_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mabur_update_status->state = mavlink_msg_mabur_update_status_get_state(msg);
    mabur_update_status->reason = mavlink_msg_mabur_update_status_get_reason(msg);
    mabur_update_status->trial_s = mavlink_msg_mabur_update_status_get_trial_s(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_LEN? msg->len : MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_LEN;
        memset(mabur_update_status, 0, MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_LEN);
    memcpy(mabur_update_status, _MAV_PAYLOAD(msg), len);
#endif
}
