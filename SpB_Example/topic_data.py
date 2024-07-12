from mqtt_spb_wrapper import MqttSpbEntityDevice as device
import rospy
import json
import os

# bring in common ros types
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import Pose, PoseStamped, Wrench, WrenchStamped
# from audio_common_msgs import AudioInfo, AudioData
from industrial_msgs.msg import RobotStatus


class AliasMap:
    Next_Server = 0
    Rebirth = 1
    Reboot = 2
    Dataset = 3
    fts = 4
    pose = 5
    joint = 6
    robotStatus = 7
    audioInfo = 8
    audioData = 9
    image = 10


script_dir = os.path.dirname(os.path.realpath(__file__))
config_path = os.path.join(script_dir, 'config.json')
with open(config_path, 'r') as file:
    config = json.load(file)

mqtt = config.get('mqtt')
attributes = mqtt.get('attributes')
commands = mqtt.get('commands')
robot_names = mqtt.get('robot_names')
telemetry_template = mqtt.get('telemetry_template')
telemetry = {
    f"{robot_name}.{key}": value for robot_name in robot_names for key, value in telemetry_template.items()
}
telemetry["defaultData"] = ""


def publishData(device:device, ros_topic:str, ros_type:str, instanceName:str):
    # Publish the data added from the functions
    try:
        result = device.publish_data()
        if result:
            rospy.loginfo(
                # Success
                f"Published message: {ros_topic} of type: {ros_type} from instance: {instanceName}")
        else:
            rospy.loginfo(
                f"Failed to publish message: {ros_topic}. Result:  {result}from instance: {instanceName}")
    except Exception as e:
        rospy.loginfo(
            f"Exception while publishing message: {ros_topic}. Exception: {str(e)}")


def audioinfo_to_bytearray(device, ros_data, rostopic):
    data = ros_data
    instanceName = rostopic.split('/')[0]
    print(
        f"tag path should be {instanceName}.audioinfo.channels and has a topic of: {rostopic}")
    # header setup
    device.data.set_value(instanceName+".audioinfo.channels", data.channels)
    device.data.set_value(
        instanceName+".audioinfo.sample_rate", data.sample_rate)
    device.data.set_value(
        instanceName+".audioinfo.sample_format", data.sample_format)
    device.data.set_value(instanceName+".audioinfo.bitrate", data.bitrate)
    device.data.set_value(instanceName+".audioinfo.codec", data.codec)
    publishData(device=device, ros_topic="audioinfo", ros_type="audioinfo", instanceName=instanceName)


def audiodata_to_bytearray(device, ros_data, rostopic):
    data = ros_data
    instanceName = rostopic.split('/')[0]
    print(
        f"tag path should be {instanceName}.image.seq and has a topic of: {rostopic}")
    device.data.set_value(instanceName+".audio.data", data.data)
    publishData(device=device, ros_topic="audiodata", ros_type="audiodata", instanceName=instanceName)



def default_payload(device, ros_data, rostopic):
    data = ros_data
    instanceName = rostopic.split('/')[0]
    print(
        f"tag path should be {instanceName} (default data) and has a topic of: {rostopic}")
    device.data.set_value("defaultData", str(data))
    # Publish the data added from the functions
    publishData(device=device, ros_topic="defaultData", ros_type="String", instanceName=instanceName)


def image_to_bytearray(device, ros_data, rostopic):
    if isinstance(ros_data, Image):
        data = ros_data
        instanceName = rostopic.split('/')[0]
        print(
            f"tag path should be {instanceName}.image.seq and has a topic of: {rostopic}")
        # header setup
        device.data.set_value(instanceName+".image.seq", data.header.seq)
        device.data.set_value(
            instanceName+".image.stamp.secs", data.header.stamp.secs)
        device.data.set_value(
            instanceName+".image.stamp.nsecs", data.header.stamp.nsecs)
        # Image data setup
        device.data.set_value(instanceName+".image.height", data.height)
        device.data.set_value(instanceName+".image.width", data.width)
        device.data.set_value(instanceName+".image.encoding", data.encoding)
        device.data.set_value(
            instanceName+".image.is_bigendian", data.is_bigendian)
        device.data.set_value(instanceName+".image.step", data.step)
        device.data.set_value(instanceName+".image.data", data.data)
        publishData(device=device, ros_topic="image", ros_type="Image", instanceName=instanceName)
    else:
        print(f"Data is not of type Image, it is of type {type(ros_data)}")


def jointstate_to_bytearray(device, ros_data, rostopic):
    if isinstance(ros_data, JointState):
        data = ros_data
        instanceName = rostopic.split('/')[0]
        print(
            f"tag path should be {instanceName}.joint.seq and has a topic of: {rostopic}")
        # header setup
        device.data.set_value(instanceName+".joint.seq", data.header.seq)
        device.data.set_value(
            instanceName+".joint.stamp.secs", data.header.stamp.secs)
        device.data.set_value(
            instanceName+".joint.stamp.nsecs", data.header.stamp.nsecs)
        # Joint states setup
        # We start the loop from 1 because we want the joint numbering to start from 1.
        # However, Python lists are 0-indexed, so we need to subtract 1 when accessing 
        # the elements of data.name, data.position, and data.velocity.
        for i in range(1, len(data.name) + 1):  
            device.data.set_value(
                f"{instanceName}.joint.{i}.name", data.name[i-1])  # Subtract 1 from i to get the correct index for data.name
            device.data.set_value(
                f"{instanceName}.joint.{i}.position", data.position[i-1])  # Subtract 1 from i to get the correct index for data.position
            device.data.set_value(
                f"{instanceName}.joint.{i}.velocity", data.velocity[i-1])  # Subtract 1 from i to get the correct index for data.velocity
            # device.data.set_value(f"{instanceName}.joint.{i}.effort", data.velocity[i-1]) # Empty array, not used
        publishData(device=device, ros_topic="jointstate", ros_type="JointState", instanceName=instanceName)
    else:
        print(
            f"Data is not of type JointState, it is of type {type(ros_data)}")

def posestamped_to_bytearray(device, ros_data, rostopic):
    if isinstance(ros_data, PoseStamped):
        data = ros_data
        instanceName = rostopic.split('/')[0]
        print(
            f"tag path should be {instanceName}.position.seq and has a topic of: {rostopic}")
        # header setup
        device.data.set_value(instanceName+".position.seq", data.header.seq)
        device.data.set_value(
            instanceName+".position.stamp.secs", data.header.stamp.secs)
        device.data.set_value(
            instanceName+".position.stamp.nsecs", data.header.stamp.nsecs)
        device.data.set_value(instanceName+".position.x", data.position.x)
        device.data.set_value(instanceName+".position.y", data.position.y)
        device.data.set_value(instanceName+".position.z", data.position.z)
        device.data.set_value(
            instanceName+".orientation.x", data.orientation.x)
        device.data.set_value(
            instanceName+".orientation.y", data.orientation.y)
        device.data.set_value(
            instanceName+".orientation.z", data.orientation.z)
        device.data.set_value(
            instanceName+".orientation.w", data.orientation.w)
        publishData(device=device, ros_topic="pose", ros_type="PoseStamped", instanceName=instanceName)
    else:
        print(
            f"Data is not of type PoseStamped, it is of type {type(ros_data)}")


def robotstatus_to_bytearray(device, ros_data, rostopic):
    if isinstance(ros_data, RobotStatus):
        data = ros_data
        instanceName = rostopic.split('/')[0]
        print(
            f"tag path should be {instanceName}.robot.seq and has a topic of: {rostopic}")
        device.data.set_value(instanceName+".robot.seq", data.header.seq)
        device.data.set_value(
            instanceName+".robot.stamp.secs", data.header.stamp.secs)
        device.data.set_value(
            instanceName+".robot.stamp.nsecs", data.header.stamp.nsecs)
        device.data.set_value(instanceName+".robot.status.mode", data.mode.val)
        device.data.set_value(
            instanceName+".robot.status.e_stopped", data.e_stopped.val)
        device.data.set_value(instanceName+".robot.status.drives_powered",
                              data.drives_powered.val)
        device.data.set_value(instanceName+".robot.status.motion_possible",
                              data.motion_possible.val)
        device.data.set_value(
            instanceName+".robot.status.in_motion", data.in_motion.val)
        device.data.set_value(
            instanceName+".robot.status.in_error", data.in_error.val)
        device.data.set_value(
            instanceName+".robot.status.error_code", data.error_code)
        publishData(device=device, ros_topic="robotStatus", ros_type="robotStatus", instanceName=instanceName)
    else:
        print(
            f"Data is not of type RobotStatus, it is of type {type(ros_data)}")


def wrenchstamped_to_bytearray(device, ros_data, rostopic):
    if isinstance(ros_data, WrenchStamped):
        data = ros_data
        instanceName = rostopic.split('/')[0]
        print(
            f"tag path should be {instanceName}.fts.seq and has a topic of: {rostopic}")
        device.data.set_value(instanceName+".fts.seq", data.header.seq)
        device.data.set_value(
            instanceName+".fts.stamp.secs", data.header.stamp.secs)
        device.data.set_value(
            instanceName+".fts.stamp.nsecs", data.header.stamp.nsecs)
        device.data.set_value(instanceName+".fts.force.x", data.wrench.force.x)
        device.data.set_value(instanceName+".fts.force.y", data.wrench.force.y)
        device.data.set_value(instanceName+".fts.force.z", data.wrench.force.z)
        device.data.set_value(
            instanceName+".fts.torque.x", data.wrench.torque.x)
        device.data.set_value(
            instanceName+".fts.torque.y", data.wrench.torque.y)
        device.data.set_value(
            instanceName+".fts.torque.z", data.wrench.torque.z)
        publishData(device=device, ros_topic="fts", ros_type="wrenchStamped", instanceName=instanceName)
    else:
        print(
            f"Data is not of type WrenchStamped, it is of type {type(ros_data)}")
