#pragma once
// MESSAGE MABUR_CAL_SWEEP PACKING

#define MAVLINK_MSG_ID_MABUR_CAL_SWEEP 60000


typedef struct __mavlink_mabur_cal_sweep_t {
 uint32_t nonce; /*<  Session nonce; the drone echoes it on every reply for this sweep.*/
 uint16_t frames_per_cell; /*<  Frames transmitted per power/verify cell.*/
 uint16_t settle_ms; /*<  Settle time before sampling a cell, in milliseconds.*/
 uint16_t gap_us; /*<  Gap between frames within a cell, in microseconds.*/
 uint16_t dwell_ms; /*<  Dwell time per capacity point, in milliseconds.*/
 uint8_t seq; /*<  The sweep's position in the run; the drone is idempotent by (nonce, seq).*/
 uint8_t kind; /*<  Which sweep to run.*/
 uint8_t bw; /*<  Channel bandwidth (MHz) the sweep flies at.*/
 int8_t idx_lo; /*<  Low end of the rel power index range swept.*/
 int8_t idx_hi; /*<  High end of the rel power index range swept.*/
 uint8_t mcs_mask; /*<  Bitmask of MCS indices the capacity sweep probes.*/
} mavlink_mabur_cal_sweep_t;

#define MAVLINK_MSG_ID_MABUR_CAL_SWEEP_LEN 18
#define MAVLINK_MSG_ID_MABUR_CAL_SWEEP_MIN_LEN 18
#define MAVLINK_MSG_ID_60000_LEN 18
#define MAVLINK_MSG_ID_60000_MIN_LEN 18

#define MAVLINK_MSG_ID_MABUR_CAL_SWEEP_CRC 193
#define MAVLINK_MSG_ID_60000_CRC 193



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MABUR_CAL_SWEEP { \
    60000, \
    "MABUR_CAL_SWEEP", \
    11, \
    {  { "nonce", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_mabur_cal_sweep_t, nonce) }, \
         { "seq", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_mabur_cal_sweep_t, seq) }, \
         { "kind", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_mabur_cal_sweep_t, kind) }, \
         { "bw", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_mabur_cal_sweep_t, bw) }, \
         { "frames_per_cell", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_mabur_cal_sweep_t, frames_per_cell) }, \
         { "settle_ms", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_mabur_cal_sweep_t, settle_ms) }, \
         { "gap_us", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_mabur_cal_sweep_t, gap_us) }, \
         { "dwell_ms", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_mabur_cal_sweep_t, dwell_ms) }, \
         { "idx_lo", NULL, MAVLINK_TYPE_INT8_T, 0, 15, offsetof(mavlink_mabur_cal_sweep_t, idx_lo) }, \
         { "idx_hi", NULL, MAVLINK_TYPE_INT8_T, 0, 16, offsetof(mavlink_mabur_cal_sweep_t, idx_hi) }, \
         { "mcs_mask", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_mabur_cal_sweep_t, mcs_mask) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MABUR_CAL_SWEEP { \
    "MABUR_CAL_SWEEP", \
    11, \
    {  { "nonce", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_mabur_cal_sweep_t, nonce) }, \
         { "seq", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_mabur_cal_sweep_t, seq) }, \
         { "kind", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_mabur_cal_sweep_t, kind) }, \
         { "bw", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_mabur_cal_sweep_t, bw) }, \
         { "frames_per_cell", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_mabur_cal_sweep_t, frames_per_cell) }, \
         { "settle_ms", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_mabur_cal_sweep_t, settle_ms) }, \
         { "gap_us", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_mabur_cal_sweep_t, gap_us) }, \
         { "dwell_ms", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_mabur_cal_sweep_t, dwell_ms) }, \
         { "idx_lo", NULL, MAVLINK_TYPE_INT8_T, 0, 15, offsetof(mavlink_mabur_cal_sweep_t, idx_lo) }, \
         { "idx_hi", NULL, MAVLINK_TYPE_INT8_T, 0, 16, offsetof(mavlink_mabur_cal_sweep_t, idx_hi) }, \
         { "mcs_mask", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_mabur_cal_sweep_t, mcs_mask) }, \
         } \
}
#endif

/**
 * @brief Pack a mabur_cal_sweep message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param nonce  Session nonce; the drone echoes it on every reply for this sweep.
 * @param seq  The sweep's position in the run; the drone is idempotent by (nonce, seq).
 * @param kind  Which sweep to run.
 * @param bw  Channel bandwidth (MHz) the sweep flies at.
 * @param frames_per_cell  Frames transmitted per power/verify cell.
 * @param settle_ms  Settle time before sampling a cell, in milliseconds.
 * @param gap_us  Gap between frames within a cell, in microseconds.
 * @param dwell_ms  Dwell time per capacity point, in milliseconds.
 * @param idx_lo  Low end of the rel power index range swept.
 * @param idx_hi  High end of the rel power index range swept.
 * @param mcs_mask  Bitmask of MCS indices the capacity sweep probes.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mabur_cal_sweep_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t nonce, uint8_t seq, uint8_t kind, uint8_t bw, uint16_t frames_per_cell, uint16_t settle_ms, uint16_t gap_us, uint16_t dwell_ms, int8_t idx_lo, int8_t idx_hi, uint8_t mcs_mask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MABUR_CAL_SWEEP_LEN];
    _mav_put_uint32_t(buf, 0, nonce);
    _mav_put_uint16_t(buf, 4, frames_per_cell);
    _mav_put_uint16_t(buf, 6, settle_ms);
    _mav_put_uint16_t(buf, 8, gap_us);
    _mav_put_uint16_t(buf, 10, dwell_ms);
    _mav_put_uint8_t(buf, 12, seq);
    _mav_put_uint8_t(buf, 13, kind);
    _mav_put_uint8_t(buf, 14, bw);
    _mav_put_int8_t(buf, 15, idx_lo);
    _mav_put_int8_t(buf, 16, idx_hi);
    _mav_put_uint8_t(buf, 17, mcs_mask);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_LEN);
#else
    mavlink_mabur_cal_sweep_t packet;
    packet.nonce = nonce;
    packet.frames_per_cell = frames_per_cell;
    packet.settle_ms = settle_ms;
    packet.gap_us = gap_us;
    packet.dwell_ms = dwell_ms;
    packet.seq = seq;
    packet.kind = kind;
    packet.bw = bw;
    packet.idx_lo = idx_lo;
    packet.idx_hi = idx_hi;
    packet.mcs_mask = mcs_mask;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MABUR_CAL_SWEEP;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_LEN, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_CRC);
}

/**
 * @brief Pack a mabur_cal_sweep message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param nonce  Session nonce; the drone echoes it on every reply for this sweep.
 * @param seq  The sweep's position in the run; the drone is idempotent by (nonce, seq).
 * @param kind  Which sweep to run.
 * @param bw  Channel bandwidth (MHz) the sweep flies at.
 * @param frames_per_cell  Frames transmitted per power/verify cell.
 * @param settle_ms  Settle time before sampling a cell, in milliseconds.
 * @param gap_us  Gap between frames within a cell, in microseconds.
 * @param dwell_ms  Dwell time per capacity point, in milliseconds.
 * @param idx_lo  Low end of the rel power index range swept.
 * @param idx_hi  High end of the rel power index range swept.
 * @param mcs_mask  Bitmask of MCS indices the capacity sweep probes.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mabur_cal_sweep_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint32_t nonce, uint8_t seq, uint8_t kind, uint8_t bw, uint16_t frames_per_cell, uint16_t settle_ms, uint16_t gap_us, uint16_t dwell_ms, int8_t idx_lo, int8_t idx_hi, uint8_t mcs_mask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MABUR_CAL_SWEEP_LEN];
    _mav_put_uint32_t(buf, 0, nonce);
    _mav_put_uint16_t(buf, 4, frames_per_cell);
    _mav_put_uint16_t(buf, 6, settle_ms);
    _mav_put_uint16_t(buf, 8, gap_us);
    _mav_put_uint16_t(buf, 10, dwell_ms);
    _mav_put_uint8_t(buf, 12, seq);
    _mav_put_uint8_t(buf, 13, kind);
    _mav_put_uint8_t(buf, 14, bw);
    _mav_put_int8_t(buf, 15, idx_lo);
    _mav_put_int8_t(buf, 16, idx_hi);
    _mav_put_uint8_t(buf, 17, mcs_mask);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_LEN);
#else
    mavlink_mabur_cal_sweep_t packet;
    packet.nonce = nonce;
    packet.frames_per_cell = frames_per_cell;
    packet.settle_ms = settle_ms;
    packet.gap_us = gap_us;
    packet.dwell_ms = dwell_ms;
    packet.seq = seq;
    packet.kind = kind;
    packet.bw = bw;
    packet.idx_lo = idx_lo;
    packet.idx_hi = idx_hi;
    packet.mcs_mask = mcs_mask;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MABUR_CAL_SWEEP;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_LEN, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_LEN);
#endif
}

/**
 * @brief Pack a mabur_cal_sweep message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param nonce  Session nonce; the drone echoes it on every reply for this sweep.
 * @param seq  The sweep's position in the run; the drone is idempotent by (nonce, seq).
 * @param kind  Which sweep to run.
 * @param bw  Channel bandwidth (MHz) the sweep flies at.
 * @param frames_per_cell  Frames transmitted per power/verify cell.
 * @param settle_ms  Settle time before sampling a cell, in milliseconds.
 * @param gap_us  Gap between frames within a cell, in microseconds.
 * @param dwell_ms  Dwell time per capacity point, in milliseconds.
 * @param idx_lo  Low end of the rel power index range swept.
 * @param idx_hi  High end of the rel power index range swept.
 * @param mcs_mask  Bitmask of MCS indices the capacity sweep probes.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mabur_cal_sweep_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t nonce,uint8_t seq,uint8_t kind,uint8_t bw,uint16_t frames_per_cell,uint16_t settle_ms,uint16_t gap_us,uint16_t dwell_ms,int8_t idx_lo,int8_t idx_hi,uint8_t mcs_mask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MABUR_CAL_SWEEP_LEN];
    _mav_put_uint32_t(buf, 0, nonce);
    _mav_put_uint16_t(buf, 4, frames_per_cell);
    _mav_put_uint16_t(buf, 6, settle_ms);
    _mav_put_uint16_t(buf, 8, gap_us);
    _mav_put_uint16_t(buf, 10, dwell_ms);
    _mav_put_uint8_t(buf, 12, seq);
    _mav_put_uint8_t(buf, 13, kind);
    _mav_put_uint8_t(buf, 14, bw);
    _mav_put_int8_t(buf, 15, idx_lo);
    _mav_put_int8_t(buf, 16, idx_hi);
    _mav_put_uint8_t(buf, 17, mcs_mask);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_LEN);
#else
    mavlink_mabur_cal_sweep_t packet;
    packet.nonce = nonce;
    packet.frames_per_cell = frames_per_cell;
    packet.settle_ms = settle_ms;
    packet.gap_us = gap_us;
    packet.dwell_ms = dwell_ms;
    packet.seq = seq;
    packet.kind = kind;
    packet.bw = bw;
    packet.idx_lo = idx_lo;
    packet.idx_hi = idx_hi;
    packet.mcs_mask = mcs_mask;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MABUR_CAL_SWEEP;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_LEN, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_CRC);
}

/**
 * @brief Encode a mabur_cal_sweep struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mabur_cal_sweep C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mabur_cal_sweep_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mabur_cal_sweep_t* mabur_cal_sweep)
{
    return mavlink_msg_mabur_cal_sweep_pack(system_id, component_id, msg, mabur_cal_sweep->nonce, mabur_cal_sweep->seq, mabur_cal_sweep->kind, mabur_cal_sweep->bw, mabur_cal_sweep->frames_per_cell, mabur_cal_sweep->settle_ms, mabur_cal_sweep->gap_us, mabur_cal_sweep->dwell_ms, mabur_cal_sweep->idx_lo, mabur_cal_sweep->idx_hi, mabur_cal_sweep->mcs_mask);
}

/**
 * @brief Encode a mabur_cal_sweep struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mabur_cal_sweep C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mabur_cal_sweep_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mabur_cal_sweep_t* mabur_cal_sweep)
{
    return mavlink_msg_mabur_cal_sweep_pack_chan(system_id, component_id, chan, msg, mabur_cal_sweep->nonce, mabur_cal_sweep->seq, mabur_cal_sweep->kind, mabur_cal_sweep->bw, mabur_cal_sweep->frames_per_cell, mabur_cal_sweep->settle_ms, mabur_cal_sweep->gap_us, mabur_cal_sweep->dwell_ms, mabur_cal_sweep->idx_lo, mabur_cal_sweep->idx_hi, mabur_cal_sweep->mcs_mask);
}

/**
 * @brief Encode a mabur_cal_sweep struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param mabur_cal_sweep C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mabur_cal_sweep_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_mabur_cal_sweep_t* mabur_cal_sweep)
{
    return mavlink_msg_mabur_cal_sweep_pack_status(system_id, component_id, _status, msg,  mabur_cal_sweep->nonce, mabur_cal_sweep->seq, mabur_cal_sweep->kind, mabur_cal_sweep->bw, mabur_cal_sweep->frames_per_cell, mabur_cal_sweep->settle_ms, mabur_cal_sweep->gap_us, mabur_cal_sweep->dwell_ms, mabur_cal_sweep->idx_lo, mabur_cal_sweep->idx_hi, mabur_cal_sweep->mcs_mask);
}

/**
 * @brief Send a mabur_cal_sweep message
 * @param chan MAVLink channel to send the message
 *
 * @param nonce  Session nonce; the drone echoes it on every reply for this sweep.
 * @param seq  The sweep's position in the run; the drone is idempotent by (nonce, seq).
 * @param kind  Which sweep to run.
 * @param bw  Channel bandwidth (MHz) the sweep flies at.
 * @param frames_per_cell  Frames transmitted per power/verify cell.
 * @param settle_ms  Settle time before sampling a cell, in milliseconds.
 * @param gap_us  Gap between frames within a cell, in microseconds.
 * @param dwell_ms  Dwell time per capacity point, in milliseconds.
 * @param idx_lo  Low end of the rel power index range swept.
 * @param idx_hi  High end of the rel power index range swept.
 * @param mcs_mask  Bitmask of MCS indices the capacity sweep probes.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mabur_cal_sweep_send(mavlink_channel_t chan, uint32_t nonce, uint8_t seq, uint8_t kind, uint8_t bw, uint16_t frames_per_cell, uint16_t settle_ms, uint16_t gap_us, uint16_t dwell_ms, int8_t idx_lo, int8_t idx_hi, uint8_t mcs_mask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MABUR_CAL_SWEEP_LEN];
    _mav_put_uint32_t(buf, 0, nonce);
    _mav_put_uint16_t(buf, 4, frames_per_cell);
    _mav_put_uint16_t(buf, 6, settle_ms);
    _mav_put_uint16_t(buf, 8, gap_us);
    _mav_put_uint16_t(buf, 10, dwell_ms);
    _mav_put_uint8_t(buf, 12, seq);
    _mav_put_uint8_t(buf, 13, kind);
    _mav_put_uint8_t(buf, 14, bw);
    _mav_put_int8_t(buf, 15, idx_lo);
    _mav_put_int8_t(buf, 16, idx_hi);
    _mav_put_uint8_t(buf, 17, mcs_mask);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_CAL_SWEEP, buf, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_LEN, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_CRC);
#else
    mavlink_mabur_cal_sweep_t packet;
    packet.nonce = nonce;
    packet.frames_per_cell = frames_per_cell;
    packet.settle_ms = settle_ms;
    packet.gap_us = gap_us;
    packet.dwell_ms = dwell_ms;
    packet.seq = seq;
    packet.kind = kind;
    packet.bw = bw;
    packet.idx_lo = idx_lo;
    packet.idx_hi = idx_hi;
    packet.mcs_mask = mcs_mask;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_CAL_SWEEP, (const char *)&packet, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_LEN, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_CRC);
#endif
}

/**
 * @brief Send a mabur_cal_sweep message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mabur_cal_sweep_send_struct(mavlink_channel_t chan, const mavlink_mabur_cal_sweep_t* mabur_cal_sweep)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mabur_cal_sweep_send(chan, mabur_cal_sweep->nonce, mabur_cal_sweep->seq, mabur_cal_sweep->kind, mabur_cal_sweep->bw, mabur_cal_sweep->frames_per_cell, mabur_cal_sweep->settle_ms, mabur_cal_sweep->gap_us, mabur_cal_sweep->dwell_ms, mabur_cal_sweep->idx_lo, mabur_cal_sweep->idx_hi, mabur_cal_sweep->mcs_mask);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_CAL_SWEEP, (const char *)mabur_cal_sweep, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_LEN, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_CRC);
#endif
}

#if MAVLINK_MSG_ID_MABUR_CAL_SWEEP_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by reusing
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mabur_cal_sweep_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t nonce, uint8_t seq, uint8_t kind, uint8_t bw, uint16_t frames_per_cell, uint16_t settle_ms, uint16_t gap_us, uint16_t dwell_ms, int8_t idx_lo, int8_t idx_hi, uint8_t mcs_mask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, nonce);
    _mav_put_uint16_t(buf, 4, frames_per_cell);
    _mav_put_uint16_t(buf, 6, settle_ms);
    _mav_put_uint16_t(buf, 8, gap_us);
    _mav_put_uint16_t(buf, 10, dwell_ms);
    _mav_put_uint8_t(buf, 12, seq);
    _mav_put_uint8_t(buf, 13, kind);
    _mav_put_uint8_t(buf, 14, bw);
    _mav_put_int8_t(buf, 15, idx_lo);
    _mav_put_int8_t(buf, 16, idx_hi);
    _mav_put_uint8_t(buf, 17, mcs_mask);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_CAL_SWEEP, buf, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_LEN, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_CRC);
#else
    mavlink_mabur_cal_sweep_t *packet = (mavlink_mabur_cal_sweep_t *)msgbuf;
    packet->nonce = nonce;
    packet->frames_per_cell = frames_per_cell;
    packet->settle_ms = settle_ms;
    packet->gap_us = gap_us;
    packet->dwell_ms = dwell_ms;
    packet->seq = seq;
    packet->kind = kind;
    packet->bw = bw;
    packet->idx_lo = idx_lo;
    packet->idx_hi = idx_hi;
    packet->mcs_mask = mcs_mask;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MABUR_CAL_SWEEP, (const char *)packet, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_MIN_LEN, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_LEN, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_CRC);
#endif
}
#endif

#endif

// MESSAGE MABUR_CAL_SWEEP UNPACKING


/**
 * @brief Get field nonce from mabur_cal_sweep message
 *
 * @return  Session nonce; the drone echoes it on every reply for this sweep.
 */
static inline uint32_t mavlink_msg_mabur_cal_sweep_get_nonce(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field seq from mabur_cal_sweep message
 *
 * @return  The sweep's position in the run; the drone is idempotent by (nonce, seq).
 */
static inline uint8_t mavlink_msg_mabur_cal_sweep_get_seq(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field kind from mabur_cal_sweep message
 *
 * @return  Which sweep to run.
 */
static inline uint8_t mavlink_msg_mabur_cal_sweep_get_kind(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Get field bw from mabur_cal_sweep message
 *
 * @return  Channel bandwidth (MHz) the sweep flies at.
 */
static inline uint8_t mavlink_msg_mabur_cal_sweep_get_bw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  14);
}

/**
 * @brief Get field frames_per_cell from mabur_cal_sweep message
 *
 * @return  Frames transmitted per power/verify cell.
 */
static inline uint16_t mavlink_msg_mabur_cal_sweep_get_frames_per_cell(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field settle_ms from mabur_cal_sweep message
 *
 * @return  Settle time before sampling a cell, in milliseconds.
 */
static inline uint16_t mavlink_msg_mabur_cal_sweep_get_settle_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field gap_us from mabur_cal_sweep message
 *
 * @return  Gap between frames within a cell, in microseconds.
 */
static inline uint16_t mavlink_msg_mabur_cal_sweep_get_gap_us(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field dwell_ms from mabur_cal_sweep message
 *
 * @return  Dwell time per capacity point, in milliseconds.
 */
static inline uint16_t mavlink_msg_mabur_cal_sweep_get_dwell_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  10);
}

/**
 * @brief Get field idx_lo from mabur_cal_sweep message
 *
 * @return  Low end of the rel power index range swept.
 */
static inline int8_t mavlink_msg_mabur_cal_sweep_get_idx_lo(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  15);
}

/**
 * @brief Get field idx_hi from mabur_cal_sweep message
 *
 * @return  High end of the rel power index range swept.
 */
static inline int8_t mavlink_msg_mabur_cal_sweep_get_idx_hi(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  16);
}

/**
 * @brief Get field mcs_mask from mabur_cal_sweep message
 *
 * @return  Bitmask of MCS indices the capacity sweep probes.
 */
static inline uint8_t mavlink_msg_mabur_cal_sweep_get_mcs_mask(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Decode a mabur_cal_sweep message into a struct
 *
 * @param msg The message to decode
 * @param mabur_cal_sweep C-struct to decode the message contents into
 */
static inline void mavlink_msg_mabur_cal_sweep_decode(const mavlink_message_t* msg, mavlink_mabur_cal_sweep_t* mabur_cal_sweep)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mabur_cal_sweep->nonce = mavlink_msg_mabur_cal_sweep_get_nonce(msg);
    mabur_cal_sweep->frames_per_cell = mavlink_msg_mabur_cal_sweep_get_frames_per_cell(msg);
    mabur_cal_sweep->settle_ms = mavlink_msg_mabur_cal_sweep_get_settle_ms(msg);
    mabur_cal_sweep->gap_us = mavlink_msg_mabur_cal_sweep_get_gap_us(msg);
    mabur_cal_sweep->dwell_ms = mavlink_msg_mabur_cal_sweep_get_dwell_ms(msg);
    mabur_cal_sweep->seq = mavlink_msg_mabur_cal_sweep_get_seq(msg);
    mabur_cal_sweep->kind = mavlink_msg_mabur_cal_sweep_get_kind(msg);
    mabur_cal_sweep->bw = mavlink_msg_mabur_cal_sweep_get_bw(msg);
    mabur_cal_sweep->idx_lo = mavlink_msg_mabur_cal_sweep_get_idx_lo(msg);
    mabur_cal_sweep->idx_hi = mavlink_msg_mabur_cal_sweep_get_idx_hi(msg);
    mabur_cal_sweep->mcs_mask = mavlink_msg_mabur_cal_sweep_get_mcs_mask(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MABUR_CAL_SWEEP_LEN? msg->len : MAVLINK_MSG_ID_MABUR_CAL_SWEEP_LEN;
        memset(mabur_cal_sweep, 0, MAVLINK_MSG_ID_MABUR_CAL_SWEEP_LEN);
    memcpy(mabur_cal_sweep, _MAV_PAYLOAD(msg), len);
#endif
}
