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
        963497464,17443,17547,29,96,163,230,41
    };
    mavlink_mabur_cal_sweep_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.nonce = packet_in.nonce;
        packet1.settle_ms = packet_in.settle_ms;
        packet1.dwell_ms = packet_in.dwell_ms;
        packet1.seq = packet_in.seq;
        packet1.kind = packet_in.kind;
        packet1.bw = packet_in.bw;
        packet1.channel = packet_in.channel;
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
    mavlink_msg_mabur_cal_sweep_pack(system_id, component_id, &msg , packet1.nonce , packet1.seq , packet1.kind , packet1.bw , packet1.channel , packet1.settle_ms , packet1.dwell_ms , packet1.mcs_mask );
    mavlink_msg_mabur_cal_sweep_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_cal_sweep_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.nonce , packet1.seq , packet1.kind , packet1.bw , packet1.channel , packet1.settle_ms , packet1.dwell_ms , packet1.mcs_mask );
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
    mavlink_msg_mabur_cal_sweep_send(MAVLINK_COMM_1 , packet1.nonce , packet1.seq , packet1.kind , packet1.bw , packet1.channel , packet1.settle_ms , packet1.dwell_ms , packet1.mcs_mask );
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
        963497464,{ 963497672, 963497673, 963497674, 963497675, 963497676, 963497677, 963497678, 963497679 },19107,247,{ 58, 59, 60, 61, 62, 63, 64, 65 },82,149,216,{ 27, 28, 29, 30, 31, 32, 33, 34 },51
    };
    mavlink_mabur_cal_sweep_done_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.nonce = packet_in.nonce;
        packet1.floor_evm_x10 = packet_in.floor_evm_x10;
        packet1.seq = packet_in.seq;
        packet1.wall_result = packet_in.wall_result;
        packet1.channel = packet_in.channel;
        packet1.bw = packet_in.bw;
        packet1.anchor = packet_in.anchor;
        
        mav_array_memcpy(packet1.offered, packet_in.offered, sizeof(uint32_t)*8);
        mav_array_memcpy(packet1.blocked_pct, packet_in.blocked_pct, sizeof(uint8_t)*8);
        mav_array_memcpy(packet1.walls, packet_in.walls, sizeof(int8_t)*8);
        
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
    mavlink_msg_mabur_cal_sweep_done_pack(system_id, component_id, &msg , packet1.nonce , packet1.seq , packet1.offered , packet1.blocked_pct , packet1.wall_result , packet1.channel , packet1.bw , packet1.walls , packet1.floor_evm_x10 , packet1.anchor );
    mavlink_msg_mabur_cal_sweep_done_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_cal_sweep_done_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.nonce , packet1.seq , packet1.offered , packet1.blocked_pct , packet1.wall_result , packet1.channel , packet1.bw , packet1.walls , packet1.floor_evm_x10 , packet1.anchor );
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
    mavlink_msg_mabur_cal_sweep_done_send(MAVLINK_COMM_1 , packet1.nonce , packet1.seq , packet1.offered , packet1.blocked_pct , packet1.wall_result , packet1.channel , packet1.bw , packet1.walls , packet1.floor_evm_x10 , packet1.anchor );
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
        963497464,{ 17443, 17444, 17445, 17446, 17447, 17448, 17449, 17450 },65,132
    };
    mavlink_mabur_cal_apply_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.nonce = packet_in.nonce;
        packet1.seq = packet_in.seq;
        packet1.bw = packet_in.bw;
        
        mav_array_memcpy(packet1.eff, packet_in.eff, sizeof(uint16_t)*8);
        
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
    mavlink_msg_mabur_cal_apply_pack(system_id, component_id, &msg , packet1.nonce , packet1.seq , packet1.bw , packet1.eff );
    mavlink_msg_mabur_cal_apply_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_cal_apply_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.nonce , packet1.seq , packet1.bw , packet1.eff );
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
    mavlink_msg_mabur_cal_apply_send(MAVLINK_COMM_1 , packet1.nonce , packet1.seq , packet1.bw , packet1.eff );
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

static void mavlink_test_mabur_ident_req(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MABUR_IDENT_REQ >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_mabur_ident_req_t packet_in = {
        5
    };
    mavlink_mabur_ident_req_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.pad = packet_in.pad;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MABUR_IDENT_REQ_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MABUR_IDENT_REQ_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_ident_req_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mabur_ident_req_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_ident_req_pack(system_id, component_id, &msg , packet1.pad );
    mavlink_msg_mabur_ident_req_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_ident_req_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.pad );
    mavlink_msg_mabur_ident_req_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_mabur_ident_req_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_ident_req_send(MAVLINK_COMM_1 , packet1.pad );
    mavlink_msg_mabur_ident_req_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MABUR_IDENT_REQ") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MABUR_IDENT_REQ) != NULL);
#endif
}

static void mavlink_test_mabur_ident(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MABUR_IDENT >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_mabur_ident_t packet_in = {
        "ABCDEFGHIJKLMNOPQRSTUVWXYZABCDE","GHIJKLMNOPQRSTUVWXYZABCDEFGHIJKLMNOPQRSTUVWXYZA","CDEFGHIJKLMNOPQ"
    };
    mavlink_mabur_ident_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        mav_array_memcpy(packet1.version, packet_in.version, sizeof(char)*32);
        mav_array_memcpy(packet1.board, packet_in.board, sizeof(char)*48);
        mav_array_memcpy(packet1.chip, packet_in.chip, sizeof(char)*16);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MABUR_IDENT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MABUR_IDENT_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_ident_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mabur_ident_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_ident_pack(system_id, component_id, &msg , packet1.version , packet1.board , packet1.chip );
    mavlink_msg_mabur_ident_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_ident_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.version , packet1.board , packet1.chip );
    mavlink_msg_mabur_ident_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_mabur_ident_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_ident_send(MAVLINK_COMM_1 , packet1.version , packet1.board , packet1.chip );
    mavlink_msg_mabur_ident_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MABUR_IDENT") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MABUR_IDENT) != NULL);
#endif
}

static void mavlink_test_mabur_update(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MABUR_UPDATE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_mabur_update_t packet_in = {
        5,{ 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103 }
    };
    mavlink_mabur_update_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.cmd = packet_in.cmd;
        
        mav_array_memcpy(packet1.sha256, packet_in.sha256, sizeof(uint8_t)*32);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MABUR_UPDATE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MABUR_UPDATE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_update_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mabur_update_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_update_pack(system_id, component_id, &msg , packet1.cmd , packet1.sha256 );
    mavlink_msg_mabur_update_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_update_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.cmd , packet1.sha256 );
    mavlink_msg_mabur_update_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_mabur_update_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_update_send(MAVLINK_COMM_1 , packet1.cmd , packet1.sha256 );
    mavlink_msg_mabur_update_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MABUR_UPDATE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MABUR_UPDATE) != NULL);
#endif
}

static void mavlink_test_mabur_update_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MABUR_UPDATE_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_mabur_update_status_t packet_in = {
        5,72,139
    };
    mavlink_mabur_update_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.state = packet_in.state;
        packet1.reason = packet_in.reason;
        packet1.trial_s = packet_in.trial_s;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MABUR_UPDATE_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_update_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_mabur_update_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_update_status_pack(system_id, component_id, &msg , packet1.state , packet1.reason , packet1.trial_s );
    mavlink_msg_mabur_update_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_update_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.state , packet1.reason , packet1.trial_s );
    mavlink_msg_mabur_update_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_mabur_update_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_mabur_update_status_send(MAVLINK_COMM_1 , packet1.state , packet1.reason , packet1.trial_s );
    mavlink_msg_mabur_update_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MABUR_UPDATE_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MABUR_UPDATE_STATUS) != NULL);
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
    mavlink_test_mabur_ident_req(system_id, component_id, last_msg);
    mavlink_test_mabur_ident(system_id, component_id, last_msg);
    mavlink_test_mabur_update(system_id, component_id, last_msg);
    mavlink_test_mabur_update_status(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MABUR_TESTSUITE_H
