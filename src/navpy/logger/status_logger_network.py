import time
from typing import Optional

from navpy.args.logger_args import LoggerArgs, LogStatusDest
from navpy.modules.comm.messages.log_status_msg import LogStatusMsg
from navpy.modules.comm.network_abc import NetworkAbc
from navpy.modules.vehicle.vehicle_interface import IVehicle
from navpy.logger.status_logger import IStatusLogger


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
            self.vehicle.send_status_text(msg)

    def defer_status_texts(self, enable: bool, *, flush: bool = True) -> None:
        if not self.args.defer_status_logs:
            return

        self._defer_status_texts = enable
        if not enable and flush:
            self._flush_pending_log()

    def _flush_pending_log(self) -> None:
        logs = self._pending_status_texts
        self._pending_status_texts = []
        if len(logs) == 0:
            return

        for l in logs:
            self.vehicle.send_status_text(l)
            time.sleep(0.01)

    def close(self):
        if self._pending_status_texts and not self._defer_status_texts:
            self._flush_pending_log()

    @classmethod
    def create(cls, sys_id, logger_args: LoggerArgs, ground_station_id=0):
        return cls(sys_id, logger_args, ground_station_id)
