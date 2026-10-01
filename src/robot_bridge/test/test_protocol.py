import random

import numpy as np
from robot_bridge import protocol


def test_chunks_reassemble_out_of_order():
    """Chunked payloads survive reordering."""
    payload = bytes(random.getrandbits(8) for _ in range(9000))
    datagrams = protocol.chunk(protocol.STREAM_CAMERA, 5, payload)
    assert all(len(d) <= protocol.HEADER.size + protocol.CHUNK_PAYLOAD for d in datagrams)
    random.shuffle(datagrams)
    r = protocol.Reassembler()
    done = [x for d in datagrams if (x := r.push(d))]
    assert done == [(protocol.STREAM_CAMERA, 5, payload)]


def test_incomplete_frame_is_dropped_for_a_newer_one():
    """A lost chunk only costs that frame."""
    r = protocol.Reassembler()
    for d in protocol.chunk(1, 1, bytes(5000))[:-1]:
        assert r.push(d) is None
    assert r.push(protocol.chunk(1, 2, b'ok')[0]) == (1, 2, b'ok')


def test_points_roundtrip_within_a_centimetre():
    """Packing to int16 cm keeps points within 5 mm."""
    xyz = np.random.uniform(-20, 20, (500, 3)).astype(np.float32)
    assert np.abs(protocol.unpack_points(protocol.pack_points(xyz)) - xyz).max() <= 0.005 + 1e-6


def test_voxel_downsample_caps_points():
    """Downsampling keeps at most max_points."""
    xyz = np.random.uniform(-5, 5, (20000, 3)).astype(np.float32)
    assert protocol.voxel_downsample(xyz, 0.1, 1000).shape[0] == 1000
