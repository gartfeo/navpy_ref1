from typing import Optional

from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.config.logger_args import LoggerArgs, LogStatusDest
from navpy.core.link.log_status_msg import LogStatusMsg
from navpy.core.ports.network_abc import NetworkAbc
from navpy.core.ports.vehicle_contract import IVehicle
from navpy.core.kernel.logging.status_logger import IStatusLogger

# Time one drain of pending texts may take, pacing sleeps included, before it
# leaves the rest pending. Each send may wait for the bus lock
# (vehicle_messaging.STATUS_TEXT_SEND_WAIT_S), so a drain takes at most this plus
# one send's wait and one pacing sleep, however many texts are queued. That bounds the NAV-exit flush
# on the navigation thread and, on the logger worker, how long a drain holds the
# status output lock that the navigation thread's defer call takes.
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
                # Left by a budgeted drain: send them first, in order, here on
                # the logger worker; the rest waits for the next text or close.
                self._pending_status_texts.append(msg)
                self._flush_pending_log(
                    budget_s=DEFERRED_FLUSH_SEND_BUDGET_S, stop_on_drop=True,
                )
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
    ) -> list[str]:
        """Send the pending texts in order on the caller's thread; return those not sent.

        It stops once it has taken ``budget_s``, pacing included, or, with ``stop_on_drop``,
        after the first text not sent (dropped and logged by the vehicle); the
        rest stays pending for the next status text or close.
        """
        logs = self._pending_status_texts
        rest: list[str] = []
        unsent: list[str] = []
        started_s = rt.monotonic()
        for index, l in enumerate(logs):
            if index:
                rt.sleep(0.01)
            sent = self.vehicle.send_status_text(l)
            spent_s = rt.monotonic() - started_s
            if sent is False:
                unsent.append(l)
            if (sent is False and stop_on_drop) or (
                budget_s is not None and spent_s >= budget_s
            ):
                rest = logs[index + 1:]
                break
        self._pending_status_texts = rest
        return unsent

    def close(self) -> list[str]:
        """Send every pending text; return those not sent.

        The vehicle's own drop warning goes through the logger worker, which
        has stopped by now, so the caller logs the returned texts itself.
        """
        if self._pending_status_texts and not self._defer_status_texts:
            return self._flush_pending_log(budget_s=None, stop_on_drop=False)
        return []

    @classmethod
    def create(cls, sys_id, logger_args: LoggerArgs, ground_station_id=0):
        return cls(sys_id, logger_args, ground_station_id)
