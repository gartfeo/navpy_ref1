import json
import threading
from abc import ABC, abstractmethod
from typing import Dict, Type, Any, Optional, TYPE_CHECKING

from navpy.modules.comm.messages.types import MsgType

if TYPE_CHECKING:
    from navpy.modules.comm.messages.msg_meta import MsgMeta


class MsgDataAbc(ABC):
    """
    Abstract base class for message data.
    """

    @abstractmethod
    def to_dict(self) -> Dict[str, Any]:
        """
        Serializes the message data to a dictionary.
        """
        pass

    @classmethod
    @abstractmethod
    def from_dict(cls, data: Dict[str, Any]) -> "MsgDataAbc":
        """
        Deserializes the message data from a dictionary.
        """
        pass


class MsgABC(ABC):
    """
    Abstract base class for messages.

    Supports optional metadata (MsgMeta) for deduplication and TTL:
    - boot_id: Random ID identifying sender process instance
    - msg_seq: Monotonic sequence number per sender
    - time_ms: Sender timestamp in milliseconds
    - ttl_ms: Time-to-live in milliseconds

    Metadata is optional for backward compatibility with older messages.
    """

    def __init__(self, sender_id: int, receiver_id: Optional[int] = None,
                 meta: Optional["MsgMeta"] = None):
        # IDs may come from command line arguments which are parsed as strings.
        # Cast them to ``int`` to avoid type errors when creating MAVLink
        # messages where ``struct.pack`` expects integers.
        self.sender_id: int = int(sender_id)
        self.receiver_id: Optional[int] = int(receiver_id) if receiver_id is not None else None
        self.meta: Optional["MsgMeta"] = meta

    @abstractmethod
    def to_dict(self) -> Dict[str, Any]:
        """
        Serializes the message to a dictionary.

        Includes meta fields if metadata is present.
        """
        data = {
            "mt": self.msg_type().name,
            "sid": self.sender_id,
            "rid": self.receiver_id,
        }
        if self.meta is not None:
            data.update(self.meta.to_dict())
        return data

    def set_meta(self, meta: "MsgMeta") -> "MsgABC":
        """
        Set metadata on the message (fluent interface).

        Args:
            meta: The metadata to attach

        Returns:
            Self for chaining
        """
        self.meta = meta
        return self

    def set_meta_from_provider(self) -> "MsgABC":
        """
        Set metadata from the global MsgMetaProvider using TTL defaults.

        Returns:
            Self for chaining
        """
        from navpy.modules.comm.messages.msg_meta import MsgMetaProvider
        provider = MsgMetaProvider.get_instance()
        self.meta = provider.create_meta_for_type(self.msg_type())
        return self

    def has_meta(self) -> bool:
        """Returns True if this message has metadata attached."""
        return self.meta is not None

    def get_msg_uid(self) -> Optional[tuple]:
        """
        Get full unique ID for deduplication: (sender_id, boot_id, msg_seq).

        Returns None if no metadata is present.
        """
        if self.meta is None:
            return None
        return self.meta.get_full_uid(self.sender_id)

    @classmethod
    @abstractmethod
    def from_dict(cls, data: Dict[str, Any]) -> "MsgABC":
        """
        Deserializes the message from a dictionary.
        """
        raise NotImplementedError("Subclasses must implement from_dict method.")

    @classmethod
    @abstractmethod
    def msg_type(cls) -> MsgType:
        """
        Returns the message type of the message data.
        """
        raise NotImplementedError("Subclasses must implement msg_type method.")

    @classmethod
    @abstractmethod
    def mav_id(cls) -> int:
        """
        Returns the MAVLink message ID for this message.
        This is used to map the message to its corresponding MAVLink class.
        """
        raise NotImplementedError("Subclasses must implement mav_id method.")

    @abstractmethod
    def to_mavlink(self):
        """Serialize this message to a MAVLink packet."""
        raise NotImplementedError

    @classmethod
    def from_mavlink(cls, mav_msg: Any) -> "MsgABC":
        """Create a message instance from a MAVLink packet."""
        raise NotImplementedError("Subclasses must implement from_mavlink method.")

    @classmethod
    def msg_class(cls):
        """
        Returns the class of the message.
        This is used to register the message class in the registry.
        """
        return cls


class MsgRegistry:
    """
    Registry for message types and their corresponding classes.
    """

    _registry_lock = threading.Lock()
    _registry: Dict[MsgType, Type[MsgABC]] = {}
    _mav_registry: Dict[int, Type[MsgABC]] = {}

    @classmethod
    def register(cls, msg_class_type: Type[MsgABC]) -> None:
        """
        Registers a message class with a message type.
        """
        with cls._registry_lock:
            msg_class = msg_class_type.msg_class()

            # Only register by msg_type if it's not UNKNOWN (allows multiple UNKNOWN mocks)
            msg_type = msg_class_type.msg_type()
            if msg_type != MsgType.UNKNOWN:
                if cls._registry.get(msg_type) is not None:
                    raise RuntimeError(f"Duplicate message type registration: {msg_type}")
                cls._registry[msg_class.msg_type()] = msg_class_type

            mav_id = msg_class.mav_id()
            if cls._mav_registry.get(mav_id) is not None:
                raise RuntimeError(f"Duplicate MAVLink ID registration: {mav_id}")
            cls._mav_registry[msg_class.mav_id()] = msg_class_type

    @classmethod
    def get_class(cls, msg_type: MsgType) -> Optional[Type[MsgABC]]:
        """
        Retrieves the message class associated with a message type.
        """
        with cls._registry_lock:
            return cls._registry.get(msg_type)

    @classmethod
    def get_class_by_mav_id(cls, mav_id: int) -> Optional[Type[MsgABC]]:
        """
        Retrieves the message class associated with a MAVLink message ID.
        """
        with cls._registry_lock:
            return cls._mav_registry.get(mav_id)

    @classmethod
    def has_mav_id(cls, mav_id: int) -> bool:
        """
        Checks if a MAVLink message ID is registered.
        """
        with cls._registry_lock:
            return mav_id in cls._mav_registry


def register_msg():
    """
    Decorator to register a message class with the message registry.
    """

    def decorator(cls: Type[MsgABC]) -> Type[MsgABC]:
        MsgRegistry.register(cls)
        return cls

    return decorator


class MsgSerializer:
    """
    Serializer and deserializer for messages.
    """

    @staticmethod
    def to_json(obj: MsgABC) -> str:
        """
        Serializes a message object to a JSON string.
        """
        return json.dumps(obj.to_dict())

    @staticmethod
    def from_json(json_str: str) -> MsgABC:
        """
        Deserializes a JSON string to a message object.

        Also parses and attaches metadata (MsgMeta) if present in the dict.
        """
        from navpy.modules.comm.messages.msg_meta import MsgMeta

        dict_obj = json.loads(json_str)
        msg_type_str = dict_obj.get("mt", "UNKNOWN")
        try:
            msg_type = MsgType[msg_type_str]
        except KeyError:
            msg_type = MsgType.UNKNOWN

        msg_class = MsgRegistry.get_class(msg_type)
        if msg_class is None:
            raise ValueError(f"UNKNOWN message type: {msg_type}. {json_str}")

        msg = msg_class.from_dict(dict_obj)

        # Parse and attach metadata if present (backward compatible)
        meta = MsgMeta.from_dict(dict_obj)
        if meta is not None:
            msg.meta = meta

        return msg
