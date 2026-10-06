from abc import abstractmethod

from navpy.modules.comm.messages.msg_abc import MsgABC


class ListenerAbc:
    @abstractmethod
    def on_message(self, message: MsgABC):
        pass
