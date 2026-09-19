"""
test_serial_bridge.py -- Unit tests for PGK Hardware Serial Bridge
"""

import numpy as np
from embedded.serial_bridge import compute_checksum, verify_checksum, PGKSerialBridge


def test_compute_checksum():
    payload = "PGK,1.000,500.00,0.00,100.00,500.00,0.00,0.00,0.00,0.00,-9.81,100.00"
    chk = compute_checksum(payload)
    assert len(chk) == 2
    assert all(c in "0123456789ABCDEF" for c in chk)


def test_verify_checksum_valid():
    payload = "PGK,1.000,500.00,0.00,100.00,500.00,0.00,0.00,0.00,0.00,-9.81,100.00"
    chk = compute_checksum(payload)
    packet = f"${payload}*{chk}\n"
    assert verify_checksum(packet) is True


def test_verify_checksum_corrupted():
    packet = "$PGK,1.000,500.00,0.00,100.00,500.00,0.00,0.00,0.00,0.00,-9.81,100.00*FF\n"
    assert verify_checksum(packet) is False


def test_serial_bridge_status_initially_disconnected():
    bridge = PGKSerialBridge(port=None)
    status = bridge.get_status()
    assert status["connected"] is False
    assert status["tx_count"] == 0
    assert status["rx_count"] == 0
