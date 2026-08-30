"""SBI (Sub-Block Integrity) ver 1 packer for test vector generation.
Override of devourer's fec_subblock for latency-accounting: adds q_ms and enc_us duration fields.
Ver 0 is hard-rejected (flag-day deploy, no compatibility).
"""
import struct
from typing import List

SBI_MAGIC = 0xF5B0
SBI_VER = 1
SBI_HDR_STRUCT = "<HBBHBHH"  # MAGIC, VER, STREAM_ID, BLOCK_PAYLOAD, N_BLOCKS, Q_MS, ENC_US
SBI_HDR_LEN = struct.calcsize(SBI_HDR_STRUCT)  # 11 bytes
SBI_Q_MS_OFF = 7
SBI_ENC_US_OFF = 9


class SubBlockPacker:
    """Ver 1 SBI packer: 11-byte header with q_ms and enc_us latency fields.
    Replaces devourer's version for test vector generation only."""

    def __init__(self, block_payload: int, blocks_per_body: int, stream_id: int = 0):
        self.block_payload = block_payload
        self.blocks_per_body = blocks_per_body
        self.stream_id = stream_id
        self.pending: List[bytes] = []

    def block_stride(self) -> int:
        """Per-block wire size: crc16 (2 bytes) + block_payload."""
        return 2 + self.block_payload

    def add(self, env: bytes) -> List[bytes]:
        """Add one FEC envelope. Returns completed bodies when blocks_per_body accumulate."""
        if len(env) != self.block_payload:
            return []
        self.pending.append(bytes(env))
        out = []
        while len(self.pending) >= self.blocks_per_body:
            batch = self.pending[: self.blocks_per_body]
            self.pending = self.pending[self.blocks_per_body :]
            out.append(self._build_body(batch))
        return out

    def flush(self) -> List[bytes]:
        """Emit final body with pending envelopes. Returns empty if nothing pending."""
        if not self.pending:
            return []
        out = self._build_body(self.pending)
        self.pending = []
        return [out]

    def _build_body(self, batch: List[bytes]) -> bytes:
        """Build one SBI body from a batch of envelopes."""
        out = bytearray()

        # Header: <u16 MAGIC LE, u8 ver, u8 stream_id, u16 block_payload LE, u8 n_blocks, u16 q_ms LE, u16 enc_us LE>
        # q_ms and enc_us are initialized as zero placeholders.
        out.extend(
            struct.pack(
                SBI_HDR_STRUCT,
                SBI_MAGIC,
                SBI_VER,
                self.stream_id,
                self.block_payload,
                len(batch),
                0,  # q_ms placeholder
                0,  # enc_us placeholder
            )
        )

        # Append sub-blocks: <u16 crc16 LE, payload>
        for env in batch:
            crc = self._crc16_ccitt(env)
            out.extend(struct.pack("<H", crc))
            out.extend(env)

        return bytes(out)

    @staticmethod
    def _crc16_ccitt(data: bytes) -> int:
        """CRC16-CCITT (same as devourer's implementation)."""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                crc <<= 1
                if crc & 0x10000:
                    crc ^= 0x1021
                crc &= 0xFFFF
        return crc


def unpack(body: bytes, block_payload: int) -> dict:
    """Unpack an SBI body into surviving sub-blocks (ver 1).
    Returns: {survivors: list of valid payloads, n_blocks, n_failed, header_ok, stream_id, q_ms, enc_us}
    """
    result = {
        "survivors": [],
        "n_blocks": 0,
        "n_failed": 0,
        "header_ok": False,
        "stream_id": 0,
        "q_ms": 0,
        "enc_us": 0,
    }

    if block_payload <= 0 or len(body) < SBI_HDR_LEN:
        return result

    magic, ver, stream_id, hdr_bp, n_blocks, q_ms, enc_us = struct.unpack_from(
        SBI_HDR_STRUCT, body
    )

    result["stream_id"] = stream_id
    result["header_ok"] = (
        magic == SBI_MAGIC and ver == SBI_VER and hdr_bp == block_payload
    )
    if result["header_ok"]:
        result["q_ms"] = q_ms
        result["enc_us"] = enc_us

    region = body[SBI_HDR_LEN :]
    stride = 2 + block_payload
    result["n_blocks"] = len(region) // stride

    for i in range(result["n_blocks"]):
        off = i * stride
        crc_field = struct.unpack_from("<H", region[off : off + 2])[0]
        payload = region[off + 2 : off + 2 + block_payload]
        if len(payload) == block_payload:
            computed_crc = SubBlockPacker._crc16_ccitt(payload)
            if crc_field == computed_crc:
                result["survivors"].append(payload)
            else:
                result["n_failed"] += 1

    return result


def peek_stream_id(body: bytes) -> int:
    """Peek stream_id from SBI header (ver 1), or -1 on invalid header."""
    if len(body) < SBI_HDR_LEN:
        return -1
    magic, ver, stream_id, _, _, _, _ = struct.unpack_from(SBI_HDR_STRUCT, body)
    if magic != SBI_MAGIC or ver != SBI_VER:
        return -1
    return stream_id
