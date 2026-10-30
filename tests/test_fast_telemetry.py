"""fw21 compact/checked telemetry: no stale or fabricated safety data."""
from __future__ import annotations

import threading

import pytest

from robot_control import slam


@pytest.fixture
def backend():
    return slam.load_backend()


def checked(backend, payload, *, full=False):
    checksum = backend.crc16(payload.encode("ascii"))
    return f"{payload}{' crc=' if full else ' '}{checksum:04X}"


def fast_values():
    # fw, angle/target (0.01°), status, drive, max, Servo, RPMx10, Hz, PPR,
    # edges, recent, brake, neutral ID, neutral lock, live ID, live seq, uptime.
    status = 0x2000 | 0x1000 | 0x80 | 0x40 | 0x20 | 0x08 | 0x02 | 0x01 | 0x300
    return [21, -1250, 4500, status, -20, 100, 36, 600, 90, 90,
            1234, 1, 42, 0, 0, 42, 7, 12345]


def fast_line(backend, values=None):
    payload = "@F " + " ".join(str(value) for value in (values or fast_values()))
    return checked(backend, payload)


def test_compact_fields_match_the_checked_legacy_snapshot(backend):
    short = fast_line(backend)
    long = checked(backend,
        "@TLM deg=-12.50 tgt=45.00 moving=1 cal=1 cycle=0 calib=0 opto=1 "
        "home=opto err=3 drive=-20 fw=21 pot=1 pm=1 tach=0 dmax=100 sv=36 duty=200 "
        "sc_rpm=60.0 sc_hz=90 sc_ppr=90 sc_count=1234 sc_recent=1 brake_id=42 "
        "neutral_id=0 neutral_lock=0 live_id=42 live_seq=7 t=12345", full=True)
    assert backend.crc16(b"123456789") == 0x4B37
    parsed = backend.parse_master_line(short, module="FR")
    assert parsed == backend.parse_master_line(long, module="FR")
    assert parsed["deg"] == -12.5 and parsed["moving"] and parsed["scRecent"]
    assert parsed["brakeId"] == parsed["liveId"] == 42 and parsed["liveSeq"] == 7
    assert len(short) < len(long) / 2
    assert backend.parse_master_line(short) is None  # role always comes from selected port


def test_any_single_bit_corruption_or_truncation_does_not_refresh_the_state(backend):
    original = fast_line(backend)
    source = object.__new__(backend.MasterSource)
    source.lock = threading.Lock()
    source.frames = {}
    assert source.feed(original, now=10.0, module="FL")
    baseline = source.frames["FL"]
    data = original.encode("ascii")
    for index in range(len(data)):
        for bit in range(8):
            altered = bytearray(data)
            altered[index] ^= 1 << bit
            assert not source.feed(altered.decode("latin-1"), now=20.0, module="FL"), (index, bit)
            assert source.frames["FL"] == baseline
    for end in range(len(original)):
        assert not source.feed(original[:end], now=20.0, module="FL")
        assert source.frames["FL"] == baseline


@pytest.mark.parametrize("index,bad", [
    (0, 20), (0, 256), (1, 2**31), (2, -(2**31)-1), (3, 0), (3, 0x40),
    (4, 101), (5, 0), (5, 101), (6, 181), (7, 2**32), (8, -1),
    (8, 65536), (9, 0), (9, 1001), (10, 2**32), (11, 2),
    (12, -1), (13, 0x7FFFFFFF), (14, 2), (15, 0x7FFFFFFF),
    (16, -1), (17, 2**32),
])
def test_valid_crc_cannot_make_impossible_values_valid(backend, index, bad):
    values = fast_values()
    values[index] = bad
    assert backend.parse_master_line(fast_line(backend, values), module="FL") is None


def test_v21_full_frame_requires_valid_checksum_but_old_frames_remain_supported(backend):
    base = ("@TLM deg=0 tgt=0 moving=0 cal=1 drive=0 fw=21 dmax=100 "
            "brake_id=42 neutral_id=0 neutral_lock=0 live_id=0 live_seq=0")
    valid = checked(backend, base, full=True)
    assert backend.parse_master_line(valid, module="FL")["brakeId"] == 42
    assert backend.parse_master_line(base, module="FL") is None
    assert backend.parse_master_line(valid.replace("brake_id=42", "brake_id=43"), module="FL") is None
    assert backend.parse_master_line(valid.replace("fw=21", "fw=20"), module="FL") is None
    assert backend.parse_master_line(base.replace("fw=21", "fw=20"), module="FL")["brakeId"] == 42
    assert backend.parse_master_line(valid + " extra=1", module="FL") is None
    for index in range(len(valid)):
        for bit in range(8):
            altered = bytearray(valid.encode("ascii"))
            altered[index] ^= 1 << bit
            assert backend.parse_master_line(altered.decode("latin-1"), module="FL") is None, (index, bit)
