"""Communication module - Network transport and messaging.

This module owns ALL communication-related functionality:
- NetworkAbc: Abstract network interface
- Network implementations (WiFi, Serial, MAVLink)
- Message types and serialization
- Message filtering (dedup + TTL)
- Listener interfaces

Usage:
    from navpy.modules.comm import NetworkAbc, create_network
    from navpy.modules.comm.messages import MsgABC, MsgSerializer
    from navpy.modules.comm.message_filter import MessageFilter
"""
from navpy.modules.comm.network_abc import NetworkAbc
from navpy.modules.comm.network_factory import create_network
from navpy.modules.comm.network_wifi import NetworkWifi
from navpy.modules.comm.network_serial import NetworkSerial
from navpy.modules.comm.network_mavlink import NetworkMavlink
from navpy.modules.comm.listener_abc import ListenerAbc
from navpy.modules.comm.message_filter import (
    MessageFilter,
    DedupCache,
    ClockOffsetEstimator,
    FilterMetrics,
)

__all__ = [
    "NetworkAbc",
    "create_network",
    "NetworkWifi",
    "NetworkSerial",
    "NetworkMavlink",
    "ListenerAbc",
    "MessageFilter",
    "DedupCache",
    "ClockOffsetEstimator",
    "FilterMetrics",
]

