class MissionItem:
    def __init__(self, seq=0, frame=0, command=0, param1=0, param2=0, param3=0, param4=0, x=0, y=0, z=0):
        self.seq = seq
        self.frame = frame
        self.command = command
        self.param1 = param1
        self.param2 = param2
        self.param3 = param3
        self.param4 = param4
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        return f"MissionItem(seq={self.seq}, frame={self.frame}, command={self.command}, param1={self.param1}, param2={self.param2}, param3={self.param3}, param4={self.param4}, x={self.x}, y={self.y}, z={self.z})"
