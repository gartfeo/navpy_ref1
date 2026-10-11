"""Communication module - Network transport and messaging.

This module owns ALL communication-related functionality:
- NetworkAbc: Abstract network interface
- Network implementations (WiFi, Serial, MAVLink)
- Message types and serialization
- Message filtering (dedup + TTL)
- Listener interfaces
"""
