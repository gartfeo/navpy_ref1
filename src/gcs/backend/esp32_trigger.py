"""
ESP32 Trigger Client

HTTP client to communicate with ESP32 launch controller for 5V relay triggers.
"""

import requests
from typing import Optional
from dataclasses import dataclass


@dataclass
class TriggerResult:
    """Result of a trigger operation."""
    success: bool
    channel: int
    message: str
    duration_ms: int = 0


class Esp32TriggerClient:
    """HTTP client for ESP32 launch controller."""

    def __init__(self, host: str, port: int = 80, timeout: float = 5.0):
        """
        Initialize ESP32 trigger client.

        Args:
            host: ESP32 IP address or hostname
            port: HTTP port (default 80)
            timeout: Request timeout in seconds
        """
        # Strip http:// or https:// prefix if present
        host = host.removeprefix("http://").removeprefix("https://").rstrip("/")
        self.host = host
        self.port = port
        self.timeout = timeout
        self.base_url = f"http://{host}:{port}"

    def health_check(self) -> bool:
        """
        Check if ESP32 is reachable.

        Returns:
            True if ESP32 responds to health check
        """
        try:
            response = requests.get(
                f"{self.base_url}/health",
                timeout=self.timeout
            )
            return response.status_code == 200
        except requests.RequestException:
            return False

    def trigger_channel(self, channel: int) -> TriggerResult:
        """
        Trigger 5V on specific channel.

        Args:
            channel: Channel number (1-indexed)

        Returns:
            TriggerResult with success status and details
        """
        try:
            response = requests.get(
                f"{self.base_url}/launch",
                params={"channel": channel},
                timeout=self.timeout
            )

            if response.status_code == 200:
                data = response.json()
                return TriggerResult(
                    success=True,
                    channel=channel,
                    message="Trigger successful",
                    duration_ms=data.get("duration_ms", 500)
                )
            elif response.status_code == 400:
                data = response.json()
                return TriggerResult(
                    success=False,
                    channel=channel,
                    message=data.get("error", "Invalid request")
                )
            else:
                return TriggerResult(
                    success=False,
                    channel=channel,
                    message=f"HTTP error: {response.status_code}"
                )

        except requests.Timeout:
            return TriggerResult(
                success=False,
                channel=channel,
                message="Request timeout"
            )
        except requests.ConnectionError:
            return TriggerResult(
                success=False,
                channel=channel,
                message="Connection failed"
            )
        except requests.RequestException as e:
            return TriggerResult(
                success=False,
                channel=channel,
                message=str(e)
            )

    def get_status(self) -> Optional[dict]:
        """
        Get ESP32 status.

        Returns:
            Status dict or None if request fails
        """
        try:
            response = requests.get(
                f"{self.base_url}/status",
                timeout=self.timeout
            )
            if response.status_code == 200:
                return response.json()
            return None
        except requests.RequestException:
            return None

    def reset_all(self) -> bool:
        """
        Reset all channels to OFF.

        Returns:
            True if reset successful
        """
        try:
            response = requests.get(
                f"{self.base_url}/reset",
                timeout=self.timeout
            )
            return response.status_code == 200
        except requests.RequestException:
            return False
