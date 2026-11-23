# The mabur MAVLink dialect

`mabur.xml` is the mabur calibration dialect: ten messages (ids
60000-60009): six carry the power/verify/capacity calibration sweep
between the ground calibration tool and the drone over the existing
MAVLink tunnel, four (60006-60009) carry the update lane's identity and
update status. It includes `common.xml`, so it is a superset -- every
common message still packs and decodes under it. The message table is
the XML itself; read `mabur.xml` for field names, types and the
`MABUR_CAL_KIND` / `MABUR_CAL_ACK_RESULT` enums.

`common.xml`, `minimal.xml` and `standard.xml` beside it are copied
unmodified from the pymavlink package used to generate, so the included
common dialect matches the generated output exactly.

The generated C headers live in `third_party/mavlink/` (see its
`VERSION` file for the pymavlink version, the regenerate command, and
the one hand-applied patch to `protocol.h` that the regenerate needs
reapplied).
