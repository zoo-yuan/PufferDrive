"""Minimal reader for Apollo cyber record files (``*.record.*`` / ``*.bag``).

Why this exists
---------------
The rino data lake is stored as Apollo cyber records, not ROS bags
(`cyber_recorder info` works on every file). Neither `cyber_py3` nor any
generated ``*_pb2.py`` is available in this workspace, and the repo's Python
data tooling (`data_utils/`) could not read records at all. This module is the
front door for the bag -> scene pipeline: it parses the record container and the
few fields we actually need, without protobuf codegen.

Format (verified against the writer/reader in
``apollo-1804/cyber/record/file/{record_file_writer.h,record_file_reader.cc}``)
---------------------------------------------------------------------------
::

    [Section(16B)] [Header proto (N bytes)] [zero-padding to 2048B]
    then, repeated until EOF:
        [Section(16B)] [payload (size bytes)]
    Section  = struct { int32 type; int32 pad; int64 size; }   # little-endian
    type 0 = header, 1 = chunk_header, 2 = chunk_body, 3 = index, 4 = channel
    chunk_body payload (compress != NONE):
        [uint64 raw_size] [compressed bytes]

The file header's protobuf fields we use: ``compress`` (3), ``message_number``
(11), ``is_complete`` (13). Message payloads are decoded with a tiny wire-format
walker; field numbers are taken from the local Apollo protos:

- ``apollo.localization.LocalizationEstimate``: pose = 2, measurement_time = 4
- ``apollo.localization.Pose``: position = 1, linear_velocity = 3,
  linear_acceleration = 4, angular_velocity = 5, heading = 6
- ``apollo.common.Point3D`` / ``PointENU``: x = 1, y = 2, z = 3 (double)

Usage
-----
    from data_utils.apollo_record import iter_messages, decode_localization_pose

    for channel, t_ns, payload in iter_messages(path, channels={"/apollo/localization/pose"}):
        sample = decode_localization_pose(payload)

Fail-fast: unknown section types, truncated sections, unsupported wire types and
missing compression modules raise ``RecordFormatError`` instead of guessing.
"""

from __future__ import annotations

import bz2
import struct
from typing import Dict, Iterator, Iterable, Optional, Tuple

HEADER_LENGTH_BYTES = 2048
SECTION_STRUCT = struct.Struct("<i4xq")

SECTION_HEADER = 0
SECTION_CHUNK_HEADER = 1
SECTION_CHUNK_BODY = 2
SECTION_INDEX = 3
SECTION_CHANNEL = 4

COMPRESS_NONE = 0
COMPRESS_BZ2 = 1
COMPRESS_LZ4 = 2
COMPRESS_ZSTD = 3

WIRE_VARINT = 0
WIRE_FIXED64 = 1
WIRE_LEN = 2
WIRE_FIXED32 = 5

DEFAULT_POSE_CHANNELS = (
    "/apollo/localization/pose",
    "/apollo/localization/wr_fusion_pose",
)


class RecordFormatError(RuntimeError):
    """Raised when a record file does not match the documented layout."""


def _read_varint(buf: bytes, offset: int) -> Tuple[int, int]:
    result = 0
    shift = 0
    while True:
        if offset >= len(buf):
            raise RecordFormatError("truncated varint")
        byte = buf[offset]
        offset += 1
        result |= (byte & 0x7F) << shift
        if not byte & 0x80:
            return result, offset
        shift += 7
        if shift > 63:
            raise RecordFormatError("varint longer than 64 bits")


def iter_fields(buf: bytes) -> Iterator[Tuple[int, int, object]]:
    """Yield ``(field_number, wire_type, value)`` for one protobuf message.

    ``value`` is an int for varint fields, a ``bytes`` slice for length-delimited
    and fixed-width fields.
    """
    offset = 0
    end = len(buf)
    while offset < end:
        key, offset = _read_varint(buf, offset)
        field_number = key >> 3
        wire_type = key & 0x07
        if wire_type == WIRE_VARINT:
            value, offset = _read_varint(buf, offset)
        elif wire_type == WIRE_FIXED64:
            if offset + 8 > end:
                raise RecordFormatError("truncated fixed64 field")
            value = buf[offset : offset + 8]
            offset += 8
        elif wire_type == WIRE_LEN:
            length, offset = _read_varint(buf, offset)
            if offset + length > end:
                raise RecordFormatError("truncated length-delimited field")
            value = buf[offset : offset + length]
            offset += length
        elif wire_type == WIRE_FIXED32:
            if offset + 4 > end:
                raise RecordFormatError("truncated fixed32 field")
            value = buf[offset : offset + 4]
            offset += 4
        else:
            raise RecordFormatError(f"unsupported wire type {wire_type}")
        yield field_number, wire_type, value


def _find_field(buf: bytes, field_number: int) -> Optional[object]:
    found = None
    for number, _wire, value in iter_fields(buf):
        if number == field_number:
            found = value
    return found


def _find_double(buf: bytes, field_number: int) -> float:
    raw = _find_field(buf, field_number)
    if raw is None:
        return float("nan")
    if not isinstance(raw, bytes) or len(raw) != 8:
        raise RecordFormatError(f"field {field_number} is not a double")
    return struct.unpack("<d", raw)[0]


def _find_uint(buf: bytes, field_number: int) -> int:
    raw = _find_field(buf, field_number)
    return 0 if raw is None else int(raw)


def _point3d(buf: bytes) -> Dict[str, float]:
    return {
        "x": _find_double(buf, 1),
        "y": _find_double(buf, 2),
        "z": _find_double(buf, 3),
    }


def _decompress(payload: bytes, compress_type: int, raw_size_hint: int) -> bytes:
    if compress_type == COMPRESS_NONE:
        return payload
    if len(payload) < 8:
        raise RecordFormatError("compressed chunk body shorter than raw_size prefix")
    raw_size = struct.unpack("<Q", payload[:8])[0]
    if raw_size_hint and raw_size != raw_size_hint:
        raise RecordFormatError(
            f"chunk raw_size mismatch: prefix={raw_size} header={raw_size_hint}"
        )
    body = payload[8:]
    if compress_type == COMPRESS_BZ2:
        return bz2.decompress(body)
    if compress_type == COMPRESS_LZ4:
        try:
            import lz4.block  # type: ignore
        except ImportError as exc:  # pragma: no cover - depends on environment
            raise RecordFormatError(
                "LZ4-compressed record: pip install lz4 to read it"
            ) from exc
        return lz4.block.decompress(body, uncompressed_size=raw_size)
    if compress_type == COMPRESS_ZSTD:
        try:
            import zstandard  # type: ignore
        except ImportError as exc:  # pragma: no cover - depends on environment
            raise RecordFormatError(
                "ZSTD-compressed record: pip install zstandard to read it"
            ) from exc
        return zstandard.ZstdDecompressor().decompress(body, max_output_size=raw_size)
    raise RecordFormatError(f"unknown compress type {compress_type}")


def read_header(path: str) -> Dict[str, object]:
    """Parse the record file header and leave the stream positioned at the first chunk."""
    with open(path, "rb") as stream:
        section_type, section_size = SECTION_STRUCT.unpack(
            stream.read(SECTION_STRUCT.size)
        )
        if section_type != SECTION_HEADER:
            raise RecordFormatError(
                f"{path}: first section is {section_type}, expected header"
            )
        if not 0 <= section_size <= HEADER_LENGTH_BYTES:
            raise RecordFormatError(f"{path}: header size {section_size} out of range")
        payload = stream.read(section_size)
        if len(payload) != section_size:
            raise RecordFormatError(f"{path}: truncated header")
    return {
        "major_version": _find_uint(payload, 1),
        "minor_version": _find_uint(payload, 2),
        "compress": _find_uint(payload, 3),
        "chunk_interval": _find_uint(payload, 4),
        "index_position": _find_uint(payload, 6),
        "chunk_number": _find_uint(payload, 7),
        "channel_number": _find_uint(payload, 8),
        "begin_time_ns": _find_uint(payload, 9),
        "end_time_ns": _find_uint(payload, 10),
        "message_number": _find_uint(payload, 11),
        "is_complete": bool(_find_uint(payload, 13)),
    }


def _iter_chunk_messages(body: bytes) -> Iterator[Tuple[str, int, bytes]]:
    for field_number, wire_type, value in iter_fields(body):
        if field_number != 1 or wire_type != WIRE_LEN:
            continue
        channel = _find_field(value, 1)
        if not isinstance(channel, bytes):
            raise RecordFormatError("single message without channel name")
        timestamp = _find_uint(value, 2)
        content = _find_field(value, 3)
        if not isinstance(content, bytes):
            raise RecordFormatError("single message without content")
        yield channel.decode("utf-8"), timestamp, content


def iter_messages(
    path: str,
    channels: Optional[Iterable[str]] = None,
    check_message_number: bool = True,
) -> Iterator[Tuple[str, int, bytes]]:
    """Yield ``(channel_name, time_ns, content_bytes)`` for one record file."""
    wanted = None if channels is None else set(channels)
    header = read_header(path)
    compress_type = int(header["compress"])
    chunk_raw_size = 0
    seen = 0

    with open(path, "rb") as stream:
        stream.seek(SECTION_STRUCT.size + HEADER_LENGTH_BYTES)
        while True:
            section_header = stream.read(SECTION_STRUCT.size)
            if not section_header:
                break
            if len(section_header) != SECTION_STRUCT.size:
                raise RecordFormatError(f"{path}: truncated section header")
            section_type, section_size = SECTION_STRUCT.unpack(section_header)
            if section_size < 0:
                raise RecordFormatError(f"{path}: negative section size {section_size}")
            payload = stream.read(section_size)
            if len(payload) != section_size:
                raise RecordFormatError(f"{path}: truncated section payload")

            if section_type == SECTION_CHUNK_HEADER:
                chunk_raw_size = _find_uint(payload, 4)
            elif section_type == SECTION_CHUNK_BODY:
                body = _decompress(payload, compress_type, chunk_raw_size)
                for channel, timestamp, content in _iter_chunk_messages(body):
                    seen += 1
                    if wanted is None or channel in wanted:
                        yield channel, timestamp, content
            elif section_type in (SECTION_INDEX, SECTION_CHANNEL):
                continue
            else:
                raise RecordFormatError(f"{path}: unknown section type {section_type}")

    if check_message_number and seen != int(header["message_number"]):
        raise RecordFormatError(
            f"{path}: parsed {seen} messages, header says {header['message_number']}"
        )


def list_channels(path: str) -> Dict[str, int]:
    """Return ``{channel: message_count}`` without decoding payloads."""
    counts: Dict[str, int] = {}
    for channel, _timestamp, _content in iter_messages(path):
        counts[channel] = counts.get(channel, 0) + 1
    return counts


def decode_localization_pose(payload: bytes) -> Dict[str, float]:
    """Decode the ego pose fields needed for executed-dynamics statistics."""
    pose = _find_field(payload, 2)
    if not isinstance(pose, bytes):
        raise RecordFormatError("LocalizationEstimate without pose")

    position = _find_field(pose, 1) or b""
    velocity = _find_field(pose, 3) or b""
    acceleration = _find_field(pose, 4) or b""
    angular_velocity = _find_field(pose, 5) or b""

    velocity_xyz = _point3d(velocity)
    acceleration_xyz = _point3d(acceleration)
    angular_velocity_xyz = _point3d(angular_velocity)
    return {
        "measurement_time_s": _find_double(payload, 4),
        "x_m": _find_double(position, 1),
        "y_m": _find_double(position, 2),
        "heading_rad": _find_double(pose, 6),
        "vx_mps": velocity_xyz["x"],
        "vy_mps": velocity_xyz["y"],
        "vz_mps": velocity_xyz["z"],
        "ax_mps2": acceleration_xyz["x"],
        "ay_mps2": acceleration_xyz["y"],
        "az_mps2": acceleration_xyz["z"],
        "yaw_rate_radps": angular_velocity_xyz["z"],
    }


def decode_header(payload: bytes) -> Dict[str, object]:
    """Decode ``apollo.common.Header`` (timestamp_sec = 1 ... radar_timestamp = 6)."""
    module_name = _find_field(payload, 2)
    return {
        "timestamp_sec": _find_double(payload, 1),
        "module_name": module_name.decode("utf-8") if isinstance(module_name, bytes) else "",
        "sequence_num": _find_uint(payload, 3),
        "lidar_timestamp_ns": _find_uint(payload, 4),
        "camera_timestamp_ns": _find_uint(payload, 5),
        "radar_timestamp_ns": _find_uint(payload, 6),
    }


_PERCEPTION_OBSTACLE_FIELDS = {
    "id": 1,
    "theta": 3,
    "length": 5,
    "width": 6,
    "height": 7,
    "tracking_time_s": 9,
    "type": 10,
    "timestamp_s": 11,
}


def decode_perception_obstacle(payload: bytes) -> Dict[str, object]:
    obstacle: Dict[str, object] = {}
    for name, number in _PERCEPTION_OBSTACLE_FIELDS.items():
        obstacle[name] = _find_uint(payload, number) if name in ("id", "type") else _find_double(payload, number)
    position = _find_field(payload, 2)
    velocity = _find_field(payload, 4)
    position_xyz = _point3d(position) if isinstance(position, bytes) else {"x": float("nan"), "y": float("nan"), "z": float("nan")}
    velocity_xyz = _point3d(velocity) if isinstance(velocity, bytes) else {"x": float("nan"), "y": float("nan"), "z": float("nan")}
    obstacle["x_m"] = position_xyz["x"]
    obstacle["y_m"] = position_xyz["y"]
    obstacle["vx_mps"] = velocity_xyz["x"]
    obstacle["vy_mps"] = velocity_xyz["y"]
    return obstacle


def decode_perception_obstacles(payload: bytes) -> Dict[str, object]:
    """Decode ``apollo.perception.PerceptionObstacles`` (header = 2, obstacles = 1)."""
    header = _find_field(payload, 2)
    obstacles = []
    for field_number, wire_type, value in iter_fields(payload):
        if field_number == 1 and wire_type == WIRE_LEN:
            obstacles.append(decode_perception_obstacle(value))
    return {
        "header": decode_header(header) if isinstance(header, bytes) else {},
        "obstacles": obstacles,
        "obj_source": _find_uint(payload, 8),
    }


def _decode_path_point(payload: bytes) -> Dict[str, float]:
    return {
        "x_m": _find_double(payload, 1),
        "y_m": _find_double(payload, 2),
        "theta_rad": _find_double(payload, 4),
        "kappa_1pm": _find_double(payload, 5),
        "s_m": _find_double(payload, 6),
    }


def _decode_trajectory_point(payload: bytes) -> Dict[str, float]:
    path_point = _find_field(payload, 1)
    decoded = _decode_path_point(path_point) if isinstance(path_point, bytes) else {}
    decoded.update(
        {
            "v_mps": _find_double(payload, 2),
            "a_mps2": _find_double(payload, 3),
            "relative_time_s": _find_double(payload, 4),
            "steer_rad": _find_double(payload, 6),
        }
    )
    return decoded


def decode_planning(payload: bytes) -> Dict[str, object]:
    """Decode the parts of ``apollo.planning.ADCTrajectory`` we measure."""
    header = _find_field(payload, 1)
    points = []
    for field_number, wire_type, value in iter_fields(payload):
        if field_number == 12 and wire_type == WIRE_LEN:
            points.append(_decode_trajectory_point(value))
    latency_stats = _find_field(payload, 15)
    latency = {}
    if isinstance(latency_stats, bytes):
        latency = {
            "total_time_ms": _find_double(latency_stats, 1),
            "init_frame_time_ms": _find_double(latency_stats, 3),
            "perception_latency_ms": _find_double(latency_stats, 4),
        }
    return {
        "header": decode_header(header) if isinstance(header, bytes) else {},
        "trajectory_points": points,
        "latency_stats": latency,
    }


def decode_control(payload: bytes) -> Dict[str, object]:
    """Decode ``apollo.control.ControlCommand`` (header = 1)."""
    header = _find_field(payload, 1)
    return {
        "header": decode_header(header) if isinstance(header, bytes) else {},
        "throttle_percent": _find_double(payload, 3),
        "brake_percent": _find_double(payload, 4),
        "steering_target_percent": _find_double(payload, 7),
        "speed_mps": _find_double(payload, 9),
        "acceleration_mps2": _find_double(payload, 10),
    }
