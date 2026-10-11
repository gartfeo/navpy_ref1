class UasArgs:
    def __init__(self, uas_seq='ZYX', degrees=True, utm_x=0, utm_y=0, utm_z=0):
        self.uas_seq = uas_seq
        self.degrees = degrees
        self.utm_x = utm_x
        self.utm_y = utm_y
        self.utm_z = utm_z

        # self.refresh()

    def refresh(self):
        self.uas_seq = 'ZYX'
        self.degrees = True
        #
        # self.setup_att = setup_att
        # self.setup_angle_seq = setup_angle_seq
        # self.setup_angle_degrees = setup_angle_degrees
        #
        # self.t_g_to_uas = np.array([[setup_dist_x], [setup_dist_y], [setup_dist_z]])
        self.utm_x = 0
        self.utm_y = 0
        self.utm_z = 0
