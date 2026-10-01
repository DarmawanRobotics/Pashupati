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


def test_discovery_beacon_and_seq():
    """A discover probe gets a beacon without the token; states carry an increasing seq."""
    import socket

    import stubgen  # noqa: F401
    from robot_bridge import teleop_udp_node as node_module

    node = node_module.TeleopUdpNode.__new__(node_module.TeleopUdpNode)
    node._robot_name, node._token, node._client, node._client_seen = 'l1w-01', 'secret', None, 0.0
    node._client_timeout, node._client_name, node._seq = 1.0, '', 0
    node._state = {'mode': 'remote', 'battery': {'pct': 80}}
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.bind(('127.0.0.1', 0))
    node._sock = server
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(('127.0.0.1', 0))
    probe.settimeout(1)
    node.reply_beacon(probe.getsockname())
    beacon = protocol.decode_json(probe.recv(2048))
    expected = {
        't': 'beacon',
        'name': 'l1w-01',
        'secured': True,
        'in_use_by': '',
        'mode': 'remote',
    }
    assert {k: beacon[k] for k in expected} == expected and beacon['battery'] == 80
    sent = []
    node._client, node.pose = ('127.0.0.1', 1), lambda: None
    node.send_json = sent.append
    node.send_state()
    node.send_state()
    assert [m['seq'] for m in sent] == [1, 2]
