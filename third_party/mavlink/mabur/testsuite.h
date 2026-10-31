/** @file
 *    @brief MAVLink comm protocol testsuite generated from mabur.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef MABUR_TESTSUITE_H
#define MABUR_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_mabur(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_common(system_id, component_id, last_msg);
    mavlink_test_mabur(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_mabur_cal_sweep(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MABUR_CAL_SWEEP >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_mabur_cal_sweep_t packet_in = {
        963497464,17443,17547,17651,17755,41,108,175,242,53,120
    };
    mavlink_mabur_cal_sweep_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.nonce = packet_in.nonce;
        packet1.frames_per_cell = packet_in.frames_per_cell;
        packet1.settle_ms = packet_in.settle_ms;
        packet1.gap_us = packet_in.gap_us;
        packet1.dwell_ms = packet_in.dwell_ms;
        packet1.seq = packet_in.seq;
        packet1.kind = packet_in.kind;
        packet1.bw = packet_in.bw;
        packet1.idx_lo = packet_in.idx_lo;
        packet1.idx_hi = packet_in.idx_hi;
        packet1.mcs_mask = packet_in.mcs_mask;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MABUR_CAL_SWEEP_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MABUR_CAL_SWEEP_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_cal_sweep_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mabur_cal_sweep_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_cal_sweep_pack(system_id, component_id, &msg , packet1.nonce , packet1.seq , packet1.kind , packet1.bw , packet1.frames_per_cell , packet1.settle_ms , packet1.gap_us , packet1.dwell_ms , packet1.idx_lo , packet1.idx_hi , packet1.mcs_mask );
    mavlink_msg_mabur_cal_sweep_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_cal_sweep_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.nonce , packet1.seq , packet1.kind , packet1.bw , packet1.frames_per_cell , packet1.settle_ms , packet1.gap_us , packet1.dwell_ms , packet1.idx_lo , packet1.idx_hi , packet1.mcs_mask );
    mavlink_msg_mabur_cal_sweep_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_mabur_cal_sweep_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_cal_sweep_send(MAVLINK_COMM_1 , packet1.nonce , packet1.seq , packet1.kind , packet1.bw , packet1.frames_per_cell , packet1.settle_ms , packet1.gap_us , packet1.dwell_ms , packet1.idx_lo , packet1.idx_hi , packet1.mcs_mask );
    mavlink_msg_mabur_cal_sweep_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MABUR_CAL_SWEEP") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MABUR_CAL_SWEEP) != NULL);
#endif
}

static void mavlink_test_mabur_cal_ack(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MABUR_CAL_ACK >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_mabur_cal_ack_t packet_in = {
        963497464,17443,151,218,29
    };
    mavlink_mabur_cal_ack_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.nonce = packet_in.nonce;
        packet1.start_in_ms = packet_in.start_in_ms;
        packet1.seq = packet_in.seq;
        packet1.result = packet_in.result;
        packet1.flags = packet_in.flags;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MABUR_CAL_ACK_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MABUR_CAL_ACK_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_cal_ack_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mabur_cal_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_cal_ack_pack(system_id, component_id, &msg , packet1.nonce , packet1.seq , packet1.result , packet1.start_in_ms , packet1.flags );
    mavlink_msg_mabur_cal_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_cal_ack_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.nonce , packet1.seq , packet1.result , packet1.start_in_ms , packet1.flags );
    mavlink_msg_mabur_cal_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_mabur_cal_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_cal_ack_send(MAVLINK_COMM_1 , packet1.nonce , packet1.seq , packet1.result , packet1.start_in_ms , packet1.flags );
    mavlink_msg_mabur_cal_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MABUR_CAL_ACK") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MABUR_CAL_ACK) != NULL);
#endif
}

static void mavlink_test_mabur_cal_sweep_done(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_mabur_cal_sweep_done_t packet_in = {
        963497464,{ 963497672, 963497673, 963497674, 963497675, 963497676, 963497677, 963497678, 963497679 },113,{ 180, 181, 182, 183, 184, 185, 186, 187 }
    };
    mavlink_mabur_cal_sweep_done_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.nonce = packet_in.nonce;
        packet1.seq = packet_in.seq;
        
        mav_array_memcpy(packet1.offered, packet_in.offered, sizeof(uint32_t)*8);
        mav_array_memcpy(packet1.blocked_pct, packet_in.blocked_pct, sizeof(uint8_t)*8);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_cal_sweep_done_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mabur_cal_sweep_done_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_cal_sweep_done_pack(system_id, component_id, &msg , packet1.nonce , packet1.seq , packet1.offered , packet1.blocked_pct );
    mavlink_msg_mabur_cal_sweep_done_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_cal_sweep_done_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.nonce , packet1.seq , packet1.offered , packet1.blocked_pct );
    mavlink_msg_mabur_cal_sweep_done_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_mabur_cal_sweep_done_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_cal_sweep_done_send(MAVLINK_COMM_1 , packet1.nonce , packet1.seq , packet1.offered , packet1.blocked_pct );
    mavlink_msg_mabur_cal_sweep_done_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MABUR_CAL_SWEEP_DONE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MABUR_CAL_SWEEP_DONE) != NULL);
#endif
}

static void mavlink_test_mabur_cal_apply(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MABUR_CAL_APPLY >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_mabur_cal_apply_t packet_in = {
        963497464,{ 17443, 17444, 17445, 17446, 17447, 17448, 17449, 17450 },65,132,199,{ 10, 11, 12, 13, 14, 15, 16, 17 },34
    };
    mavlink_mabur_cal_apply_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.nonce = packet_in.nonce;
        packet1.seq = packet_in.seq;
        packet1.bw = packet_in.bw;
        packet1.has = packet_in.has;
        packet1.legacy = packet_in.legacy;
        
        mav_array_memcpy(packet1.eff, packet_in.eff, sizeof(uint16_t)*8);
        mav_array_memcpy(packet1.walls, packet_in.walls, sizeof(int8_t)*8);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MABUR_CAL_APPLY_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MABUR_CAL_APPLY_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_cal_apply_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mabur_cal_apply_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_cal_apply_pack(system_id, component_id, &msg , packet1.nonce , packet1.seq , packet1.bw , packet1.has , packet1.walls , packet1.legacy , packet1.eff );
    mavlink_msg_mabur_cal_apply_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_cal_apply_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.nonce , packet1.seq , packet1.bw , packet1.has , packet1.walls , packet1.legacy , packet1.eff );
    mavlink_msg_mabur_cal_apply_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_mabur_cal_apply_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_cal_apply_send(MAVLINK_COMM_1 , packet1.nonce , packet1.seq , packet1.bw , packet1.has , packet1.walls , packet1.legacy , packet1.eff );
    mavlink_msg_mabur_cal_apply_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MABUR_CAL_APPLY") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MABUR_CAL_APPLY) != NULL);
#endif
}

static void mavlink_test_mabur_cal_applied(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MABUR_CAL_APPLIED >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_mabur_cal_applied_t packet_in = {
        963497464,17,84,151,"HIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZABCD"
    };
    mavlink_mabur_cal_applied_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.nonce = packet_in.nonce;
        packet1.seq = packet_in.seq;
        packet1.bw = packet_in.bw;
        packet1.ok = packet_in.ok;
        
        mav_array_memcpy(packet1.reason, packet_in.reason, sizeof(char)*50);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MABUR_CAL_APPLIED_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MABUR_CAL_APPLIED_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_cal_applied_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mabur_cal_applied_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_cal_applied_pack(system_id, component_id, &msg , packet1.nonce , packet1.seq , packet1.bw , packet1.ok , packet1.reason );
    mavlink_msg_mabur_cal_applied_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_cal_applied_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.nonce , packet1.seq , packet1.bw , packet1.ok , packet1.reason );
    mavlink_msg_mabur_cal_applied_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_mabur_cal_applied_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_cal_applied_send(MAVLINK_COMM_1 , packet1.nonce , packet1.seq , packet1.bw , packet1.ok , packet1.reason );
    mavlink_msg_mabur_cal_applied_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MABUR_CAL_APPLIED") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MABUR_CAL_APPLIED) != NULL);
#endif
}

static void mavlink_test_mabur_cal_abort(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MABUR_CAL_ABORT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_mabur_cal_abort_t packet_in = {
        963497464
    };
    mavlink_mabur_cal_abort_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.nonce = packet_in.nonce;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MABUR_CAL_ABORT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MABUR_CAL_ABORT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_cal_abort_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mabur_cal_abort_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_cal_abort_pack(system_id, component_id, &msg , packet1.nonce );
    mavlink_msg_mabur_cal_abort_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_cal_abort_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.nonce );
    mavlink_msg_mabur_cal_abort_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_mabur_cal_abort_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_cal_abort_send(MAVLINK_COMM_1 , packet1.nonce );
    mavlink_msg_mabur_cal_abort_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MABUR_CAL_ABORT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MABUR_CAL_ABORT) != NULL);
#endif
}

static void mavlink_test_mabur(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_mabur_cal_sweep(system_id, component_id, last_msg);
    mavlink_test_mabur_cal_ack(system_id, component_id, last_msg);
    mavlink_test_mabur_cal_sweep_done(system_id, component_id, last_msg);
    mavlink_test_mabur_cal_apply(system_id, component_id, last_msg);
    mavlink_test_mabur_cal_applied(system_id, component_id, last_msg);
    mavlink_test_mabur_cal_abort(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MABUR_TESTSUITE_H
