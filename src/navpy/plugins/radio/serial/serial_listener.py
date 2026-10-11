from abc import abstractmethod


class ISerialListener:
    @abstractmethod
    def receive_packet(self, received_data):
        pass
