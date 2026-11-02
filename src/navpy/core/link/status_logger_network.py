from typing import Optional

from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.config.logger_args import LoggerArgs, LogStatusDest
from navpy.core.link.log_status_msg import LogStatusMsg
from navpy.core.ports.network_abc import NetworkAbc
from navpy.core.ports.vehicle_contract import IVehicle
from navpy.core.kernel.logging.status_logger import IStatusLogger

# Send time a deferred flush may spend, besides its pacing sleeps, before it
# leaves the rest pending. The flush runs on the NAV-exit thread and each send
# may wait for the bus lock (vehicle_messaging.STATUS_TEXT_SEND_WAIT_S), so the
# flush adds at most this plus one send's wait, however many texts are queued.
DEFERRED_FLUSH_SEND_BUDGET_S = 0.1


class GroundStationLoggerNetwork(IStatusLogger):
    def __init__(self, sys_id: int, args: LoggerArgs, ground_station_id):
        self.sys_id = sys_id
        self.args = args
        self.network: Optional[NetworkAbc] = None
        self.vehicle = None
        self.ground_station_id = ground_station_id

        self._defer_status_texts = False
        self._pending_status_texts: list[str] = []

    def set_vehicle(self, vehicle: IVehicle):
        if not LogStatusDest.DRONE in self.args.status_dest:
            return
        sys_id = vehicle.get_parameter("SYSID_THISMAV")
        if vehicle.target_system != sys_id:
            raise ValueError(f'Vehicle ID: {vehicle.target_system} does not match source system: {sys_id}')
        self.vehicle = vehicle
        self.args.set_vehicle(vehicle)

    def set_network(self, network: NetworkAbc):
        if not LogStatusDest.NETWORK in self.args.status_dest:
            return

        self.network = network

    def send_log(self, msg: str, dest=None):
        if self.network and not self.network.is_closed and (dest is None or dest == LogStatusDest.NETWORK):
            self.network.broadcast(LogStatusMsg(self.sys_id, msg))
        if self.vehicle and (dest is None or dest == LogStatusDest.DRONE):
            if self._defer_status_texts:
                self._pending_status_texts.append(msg)
                return
            if self._pending_status_texts:
                # Left by a budgeted NAV-exit flush: send them first, in order,
                # here on the logger worker rather than the navigation thread.
                self._pending_status_texts.append(msg)
                self._flush_pending_log(budget_s=None, stop_on_drop=True)
                return
            self.vehicle.send_status_text(msg)

    def defer_status_texts(self, enable: bool, *, flush: bool = True) -> None:
        if not self.args.defer_status_logs:
            return

        self._defer_status_texts = enable
        if not enable and flush:
            self._flush_pending_log(
                budget_s=DEFERRED_FLUSH_SEND_BUDGET_S, stop_on_drop=True,
            )

    def _flush_pending_log(
        self,
        *,
        budget_s: float | None,
        stop_on_drop: bool,
    ) -> None:
        """Send the pending texts in order on the caller's thread.

        It stops once its sends took ``budget_s`` (the NAV-exit flush, on the
        navigation thread) or, with ``stop_on_drop``, after the first text not
        sent (dropped and logged by the vehicle); the rest stays pending for
        the next status text or close.
        """
        logs = self._pending_status_texts
        rest: list[str] = []
        spent_s = 0.0
        for index, l in enumerate(logs):
            started_s = rt.monotonic()
            sent = self.vehicle.send_status_text(l)
            spent_s += rt.monotonic() - started_s
            if (sent is False and stop_on_drop) or (
                budget_s is not None and spent_s >= budget_s
            ):
                rest = logs[index + 1:]
                break
            rt.sleep(0.01)
        self._pending_status_texts = rest

    def close(self):
        """Send every pending text; each one not sent is logged by the vehicle."""
        if self._pending_status_texts and not self._defer_status_texts:
            self._flush_pending_log(budget_s=None, stop_on_drop=False)

    @classmethod
    def create(cls, sys_id, logger_args: LoggerArgs, ground_station_id=0):
        return cls(sys_id, logger_args, ground_station_id)
