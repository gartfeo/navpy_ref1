from navpy.modules.common.models.attitude import Attitude


def get_euler_by_sequence(att: Attitude, seq):
    att_map = {
        'Z': att.yaw,
        'Y': att.pitch,
        'X': att.roll,
        'x': att.roll,
        'y': att.pitch,
        'z': att.yaw,
    }
    return [att_map[seq[0]], att_map[seq[1]], att_map[seq[2]]]


def get_att_by_sequence(euler, seq):
    seq_lower = seq.lower()
    att_map = {
        seq_lower[0]: euler[0],
        seq_lower[1]: euler[1],
        seq_lower[2]: euler[2],
    }

    return Attitude(att_map['y'], att_map['z'], att_map['x'])
