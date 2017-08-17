class RobotBhpnDrakeConnection:

    def __init__(self, bhpn_robot_conf):
        self.bhpn_robot_conf = bhpn_robot_conf.copy()

    def get_bhpn_robot_conf(self, drake_robot_conf):
        raise NotImplementedError

    def interpolate_drake_robot_confs(self, bhpn_robot_conf):
        raise NotImplementedError

    def get_joint_list(self, bhpn_robot_conf):
        raise NotImplementedError

    def get_joint_list_names(self):
        raise NotImplementedError

    def get_hands_to_end_effectors(self):
        raise NotImplementedError

    def get_drake_continuous_joint_indices(self):
        raise NotImplementedError

    def get_gripped_objects(self, contact_results, objects_to_check):
        raise NotImplementedError

    def maintain_pick_conf(self, bhpn_robot_conf, hand):
        raise NotImplementedError

    def pick(
            self,
            start_conf,
            targe_conf,
            hand,
            obj,
            bhpn_drake_interface_obj):
        raise NotImplementedError

    def place(
            self,
            start_conf,
            target_conf,
            hand,
            obj,
            bhpn_drake_interface_obj):
        raise NotImplementedError
