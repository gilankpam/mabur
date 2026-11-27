# The mabur MAVLink dialect

`mabur.xml` is the mabur dialect: ten messages (ids 60000-60009) in two
groups. The calibration messages (60000-60005) carry the wall and
capacity sweeps between the ground calibration tool and the drone over the
existing MAVLink tunnel; their enums are `MABUR_CAL_KIND`,
`MABUR_CAL_ACK_RESULT` and `MABUR_CAL_WALL_RESULT`. The update messages (60006-60009,
`MABUR_IDENT_REQ`, `MABUR_IDENT`, `MABUR_UPDATE`, `MABUR_UPDATE_STATUS`)
carry the drone's identity and the over-the-air update's commands and
status; their enums are `MABUR_UPDATE_CMD`, `MABUR_UPDATE_STATE` and
`MABUR_UPDATE_REASON`. It includes `common.xml`, so it is a superset --
every common message still packs and decodes under it. The message table
is the XML itself; read `mabur.xml` for field names, types and enums.

**Ids 60006-60009 are frozen:** their ids, names, field names and base
fields never change, because a GS and a drone at different versions rely
on them to update the drone. New fields come only as MAVLink 2
extensions; new enum values are allowed. `tests/test_ota_mav.cpp`
(`frozen_ota_goldens`) pins the bytes. The calibration messages are not
frozen (`docs/link-pairing.md` "Frozen handshake").

`common.xml`, `minimal.xml` and `standard.xml` beside it are copied
unmodified from the pymavlink package used to generate, so the included
common dialect matches the generated output exactly.

The generated C headers live in `third_party/mavlink/` (see its
`VERSION` file for the pymavlink version, the regenerate command, and
the one hand-applied patch to `protocol.h` that the regenerate needs
reapplied).
