import os
import threading
from geometry import shapes

interface_path = '/home/tristanthrush/FakeDesktop/spartan/drake/drake/examples/bhpn_drake_interface/'

interface_build_path = '/home/tristanthrush/FakeDesktop/spartan/build/drake/drake/examples/bhpn_drake_interface/'

def startDrakeSimulation(exp, curU):
    
    command = interface_build_path + 'simulation '

    command += '/examples/PR2/pr2_fixed.urdf 0 0 0 0 0 0 true '

    for obj in exp.fixPoses:
        bhpnToDrakeObject(obj, curU)        
	command += '/examples/bhpn_drake_interface/object_conversion_utils/generated_drake_objects/' + obj + '.off.urdf '
	objDrakePose = convertToDrakePose(exp.fixPoses[obj])
        for value in objDrakePose:
            command += str(value) + ' '
        command += 'true '

    for obj in exp.movePoses:
        bhpnToDrakeObject(obj, curU)
        command += '/examples/bhpn_drake_interface/object_conversion_utils/generated_drake_objects/' + obj + '.off.urdf '
	objDrakePose = convertToDrakePose(exp.movePoses[obj])
        for value in objDrakePose:
            command += str(value) + ' '
        command += 'false '
            
    drake_simulation = threading.Thread(target=os.system, args=[command])
    drake_simulation.daemon = True
    drake_simulation.start()

def convertToDrakePose(pose):
    return pose.x, pose.y, pose.z, 0, 0, pose.theta

def bhpnToDrakeObject(obj, curU):
    path_to_objects = interface_path + 'object_conversion_utils/'
    shapes.writeOff(curU.getObjectTypeData(obj, 'constructor')()[0], path_to_objects + 'generated_bhpn_objects/' + obj + '.off')
    os.system(path_to_objects + 'bhpn_to_drake_object.py ' + path_to_objects + 'generated_bhpn_objects/' + obj + '.off')


