from abc import abstractmethod

from navpy.core.link.msg_abc import MsgABC


class ListenerAbc:
    @abstractmethod
    def on_message(self, message: MsgABC):
        pass
