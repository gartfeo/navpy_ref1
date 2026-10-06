from abc import ABC, abstractmethod

from navpy.modules.comm.messages.msg_abc import MsgABC, MsgSerializer


class SerializerAbc(ABC):
    @abstractmethod
    def serialize_message(self, message) -> str:
        pass

    @abstractmethod
    def deserialize_message(self, encrypted_msg: str) -> MsgABC:
        pass


class JsonSerializer(SerializerAbc):
    def serialize_message(self, message: MsgABC) -> str:
        return MsgSerializer.to_json(message)

    def deserialize_message(self, encrypted_msg: str) -> MsgABC:
        return MsgSerializer.from_json(encrypted_msg)
