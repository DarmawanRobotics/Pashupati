import json
import struct

import numpy as np

PORT = 9870
MAGIC = b'PSH1'
STREAM_CAMERA = 1
STREAM_LIDAR = 2
STREAM_ROUTE = 3
HEADER = struct.Struct('<4sBIHH')  # magic, stream, frame, chunk index, chunk count
CHUNK_PAYLOAD = 1400


def encode_json(message: dict) -> bytes:
    """Serialise a control/telemetry message."""
    return json.dumps(message, separators=(',', ':')).encode()


def decode_json(data: bytes):
    """Parse a JSON datagram, or None when it is not valid JSON or not an object."""
    try:
        message = json.loads(data.decode())
    except (UnicodeDecodeError, ValueError):
        return None
    return message if isinstance(message, dict) and 't' in message else None


def chunk(stream: int, frame: int, payload: bytes, size: int = CHUNK_PAYLOAD) -> list[bytes]:
    """Split a binary payload into datagrams small enough to avoid IP fragmentation."""
    count = max(1, -(-len(payload) // size))
    return [
        HEADER.pack(MAGIC, stream, frame & 0xFFFFFFFF, i, count)
        + payload[i * size:(i + 1) * size]
        for i in range(count)
    ]


class Reassembler:
    """Rebuild chunked payloads per stream, dropping a frame as soon as a newer one starts."""

    def __init__(self):
        self._frames: dict[int, tuple[int, int, dict[int, bytes]]] = {}

    def push(self, datagram: bytes):
        """Add one datagram; return (stream, frame, payload) when a frame is complete."""
        if len(datagram) < HEADER.size or datagram[:4] != MAGIC:
            return None
        _, stream, frame, index, count = HEADER.unpack_from(datagram)
        current = self._frames.get(stream)
        if current is None or current[0] != frame:
            if current is not None and frame < current[0] and current[0] - frame < 1 << 31:
                return None
            current = (frame, count, {})
            self._frames[stream] = current
        current[2][index] = datagram[HEADER.size:]
        if len(current[2]) < current[1]:
            return None
        del self._frames[stream]
        return stream, frame, b''.join(current[2][i] for i in range(current[1]))


def pack_points(xyz: np.ndarray) -> bytes:
    """Quantise points (metres) to int16 centimetres, 6 bytes per point."""
    q = np.clip(np.round(xyz * 100.0), -32768, 32767).astype('<i2')
    return q.tobytes()


def unpack_points(payload: bytes) -> np.ndarray:
    """Decode points packed by pack_points."""
    return np.frombuffer(payload, dtype='<i2').reshape(-1, 3).astype(np.float32) / 100.0


def voxel_downsample(xyz: np.ndarray, voxel: float, max_points: int) -> np.ndarray:
    """Keep one point per voxel, then subsample to max_points."""
    if xyz.shape[0] == 0:
        return xyz
    keys = np.floor(xyz / voxel).astype(np.int32)
    _, first = np.unique(keys, axis=0, return_index=True)
    out = xyz[np.sort(first)]
    if out.shape[0] > max_points:
        out = out[np.linspace(0, out.shape[0] - 1, max_points).astype(int)]
    return out


def simplify_route(points: list, max_points: int) -> list:
    """Evenly subsample a route to at most max_points, keeping both ends."""
    if len(points) <= max_points:
        return points
    idx = np.linspace(0, len(points) - 1, max_points).round().astype(int)
    return [points[i] for i in idx]
