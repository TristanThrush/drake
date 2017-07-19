class RobotBhpnDrakeConnection:

    def __init__(self, bhpnRobotConf):
        self.bhpnRobotConf = bhpnRobotConf.copy()

    def getBhpnRobotConf(self, drakeRobotConf):
        raise NotImplementedError

    def getDrakeRobotConf(self, bhpnRobotConf):
        raise NotImplementedError

    def getJointList(self, bhpnRobotConf):
        raise NotImplementedError

    def getJointListNames(self):
        raise NotImplementedError

    def getUrdfPath(self):
        raise NotImplementedError

    def getNumJoints(self):
        raise NotImplementedError

    def getMoveThreshold(self):
        raise NotImplementedError

    def getHandsToEndEffectors(self):
        raise NotImplementedError
    
    def getContinuousJointListIndices(self):
        raise NotImplementedError

    def getGrippedObjects(self, contactResults, objectsToCheck):
        raise NotImplementedError

    def pick(self, startConf, targetConf, hand, obj, bhpnDrakeInterfaceObj):
        raise NotImplementedError

    def place(self, startConf, targetConf, hand, obj, bhpnDrakeInterfaceObj):
        raise NotImplementedError
