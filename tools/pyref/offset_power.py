"""Mabur-owned DEFAULT_PROFILE_TABLE pwr_offset_qdb column, 2026-07-17.
DIVERGES from devourer rc_proto.DEFAULT_PROFILE_TABLE's pwr_idx (TXAGC
63/48/32/20/8): these are bench-tunable qdB offsets, 0 = full legal power
(max legal offset is ZERO; wall-equalized diffs already park every rate at
wall - margin at offset 0). Mirrors common/src/profile.cpp verbatim.

The rest of this module (linear offset-qdB gain model, the model-driven
link-table/controller port) was removed 2026-07-27 (SDD ladder-controller
Task 5): the gs/src model-era sources it mirrored were all deleted,
superseded by the measured-loss ladder controller
(gs/src/ladder_controller.h), and this constant was the only piece any
surviving generator script still imported."""

PROFILE_TABLE_PWR_OFFSET_QDB = [0, -4, -8, -12, -16]
