from abc import ABC, abstractmethod


class IStatusLogger(ABC):
    @abstractmethod
    def send_log(self, msg: str, dest=None):
        raise NotImplementedError

    @abstractmethod
    def defer_status_texts(self, enable: bool, *, flush: bool = True) -> None:
        raise NotImplementedError

    @abstractmethod
    def close(self):
        raise NotImplementedError
