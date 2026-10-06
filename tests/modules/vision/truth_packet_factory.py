"""Real generated MAVLink packets for source-capture tests."""

from pymavlink.dialects.v20 import ardupilotmega as mavlink


def truth_packet(stamp=123456, *, value=1.0, sequence=0, signed=False):
    sender = mavlink.MAVLink(None, srcSystem=17, srcComponent=1)
    sender.seq = sequence
    if signed:
        sender.signing.secret_key = bytes(range(32))
        sender.signing.sign_outgoing = True
        sender.signing.link_id = 3
        sender.signing.timestamp = 12345
    message = mavlink.MAVLink_sim_state_message(
        *([value] * 21), lat_int=400000000, lon_int=440000000, time_us=stamp,
    )
    wire = message.pack(sender)
    receiver = mavlink.MAVLink(None)
    return receiver.parse_char(wire)
