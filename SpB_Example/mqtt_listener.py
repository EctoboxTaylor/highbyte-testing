#!/usr/bin/env python3
""" 
######################################################################
# ROS1 TESTING COMMANDS
######################################################################
# TEST BAG PLAYBACK
docker exec -it ros1_talker bash
stdbuf -o L rosbag play /root/bag_files/2024-02-14-13-43-54.bag --loop

#TEST RUNNING SCRIPT (LISTENER)
docker exec -it ros1_listener bash
rosrun mqtt_transmitter mqtt_listener.py
"""

######################################################################
# Main package imports
######################################################################
import sparkplug_b as sparkplug
import sparkplug_b_pb2
from sparkplug_b import MetricDataType, addMetric
from roslib.message import get_message_class
from mqtt_spb_wrapper import *
import paho.mqtt.client as mqtt
import rospy
import time
import json
import topic_data  # Used to lookup functions in topic data from the config file
import os
import signal
import sys
######################################################################
# Bring in my python data types to build
######################################################################
# ROS IMPORT TYPES
from topic_data import attributes, commands, telemetry, default_payload

# Get the directory of the current script
script_dir = os.path.dirname(os.path.realpath(__file__))
config_path = os.path.join(script_dir, 'config.json')
with open(config_path, 'r') as file:
    config = json.load(file)

# Get variables from the configuration file
mqtt_config = config.get('mqtt')
broker = mqtt_config.get('broker_address')
port = mqtt_config.get('broker_port')
username = mqtt_config.get('username')
password = mqtt_config.get('password')
GroupId = mqtt_config.get('GroupId')
NodeName = mqtt_config.get('NodeName')
DeviceName = mqtt_config.get('DeviceName')
publishSleep50Hz = mqtt_config.get('publishSleep50Hz')
# func_mapping = config['func_mapping']

# Create a dictionary lookup of ROS topics to MQTT topics
topics = mqtt_config.get('topics')
topic_mappings = {topic: details.get('mqtt_topic')
                  for topic, details in topics.items()}
ros_types = {topic: details.get('ros_type')
             for topic, details in topics.items()}
func_mappings = {topic: details.get('function')
                 for topic, details in topics.items()}

rospy.loginfo(
    f"topic_mappings: {topic_mappings}, ros_types: {ros_types}, func_mappings: {func_mappings}")
print(
    f"topic_mappings: {topic_mappings}, ros_types: {ros_types}, func_mappings: {func_mappings}")


# WRAPPER SETUP AREA -----------------------------------------------
_DEBUG = False   # Enable debug messages

# Sparkplug B parameters
_config_spb_group_name = mqtt_config.get('GroupId')
_config_spb_eon_name = mqtt_config.get('NodeName')
_config_spb_eon_device_name = mqtt_config.get('DeviceName')
# MQTT Configuration
_config_mqtt_topic = "spBv1.0/#"    # Topic to listen
_config_mqtt_host = mqtt_config.get('broker_address')
_config_mqtt_port = int(mqtt_config.get('broker_port'))
_config_mqtt_user = mqtt_config.get('username')
_config_mqtt_pass = mqtt_config.get('password')
_config_mqtt_tls_enabled = False
# _config_mqtt_tls_ca = os.environ.get("MQTT_TLS_CA", "")
# _config_mqtt_tls_cert = os.environ.get("MQTT_TLS_CERT", "")
# _config_mqtt_tls_key = os.environ.get("MQTT_TLS_KEY", "")


def callback_command(payload):
    """
        Callback function for received commands events.
    """
    print("DEVICE received CMD: %s" % payload)
    # Parse commands
    for cmd in payload['metrics']:
        # Parse fields
        name = cmd["name"]
        value = cmd["value"]
        # Parse commands
        if name == "ping" and value:  # Ping command
            # Send response
            device.data.set_value("ping", True)
            device.publish_data()
            # print("  CMD Ping received - Sending response")
        # Parse commands
        if name == "Node Control/Next Server" and value:  # Ping command
            print("'Node Control/Next Server' is not implemented in this example")
        if name == "Node Control/Rebirth" and value:  # Ping command
            device.data.set_value("Node Control/Rebirth", True)
            device.publish_birth()
        if name == "Node Control/Reboot" and value:  # Ping command
            device.data.set_value("Node Control/Reboot", True)
            device.publish_birth()


def callback_message(topic, payload):
    """
        Callback function for received messages events
    """
    print("Received MESSAGE: %s - %s" % (topic, payload))


global device, _connected
# Create the spB entity object
device = MqttSpbEntityDevice(_config_spb_group_name,
                             _config_spb_eon_name,
                             _config_spb_eon_device_name,
                             _DEBUG)

# Configure callbacks
device.on_message = callback_message    # Received messages
device.on_command = callback_command    # Callback for received commands

# The below items are imported from the topic_data.py file
# Console print the device data and fill device fields
print("--- ATTRIBUTES")
for k in attributes:
    print("  %s - %s" % (k, str(attributes[k])))
    device.attributes.set_value(k, attributes[k])

print("--- COMMANDS")
for k in commands:
    print("  %s - %s" % (k, str(commands[k])))
    device.commands.set_value(k, commands[k])

print("--- TELEMETRY")
for k in telemetry:
    print("  %s - %s" % (k, str(telemetry[k])))
    device.data.set_value(k, telemetry[k])

# Connect to the broker.
_connected = device.is_connected()
while not _connected:
    print("Connecting to data broker %s:%d ..." %
          (_config_mqtt_host, _config_mqtt_port))
    _connected = device.connect(_config_mqtt_host,
                                _config_mqtt_port,
                                _config_mqtt_user,
                                _config_mqtt_pass,
                                _config_mqtt_tls_enabled)
    # ,
    # _config_mqtt_tls_ca,
    # _config_mqtt_tls_cert,
    # _config_mqtt_tls_key)
    if not _connected:
        print("  Error, could not connect. Trying again in a few seconds ...")
        time.sleep(3)

device.publish_birth()  # Send birth message


############ OLD LOGIC FROM ORIGINAL SPB PACKAGE ##########################

def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    if device.is_connected():
        device.disconnect()
        rospy.loginfo("Disconnedted from MQTT Broker (Program Intterupted)")
    sys.exit(0)


def sub_callback(ros_data, rostopic):  # ROS TOPIC SUBSCRIBER CALLBACK
    # This function is called whenever a new message is received on the ROS topic
    # It publishes the received message to the MQTT broker
    if device.is_connected():
        if rostopic in topic_mappings:
            func_name = func_mappings.get(rostopic, "default_payload")
            func = getattr(topic_data, func_name)
            func(device, ros_data, rostopic)
        else:
            rospy.loginfo(
                f"Topic does not exist in config.json: {rostopic}")
        time.sleep(publishSleep50Hz)  # Target of 50Hz (50messages/sec)
    else:
        rospy.logerr(f"Client Not connected, cannot publish.")


def verify_master_node():
    """
    Function to verify if connected to a master node.
    Returns:
        bool: True if connected to a master node, False otherwise.
    """
    master_node = rospy.get_param('/run_id', default=None)
    if master_node is not None:
        return True
    else:
        return False

# ********************************************************************************************************************
# DO NOT EDIT BELOW THIS LINE - HANDLES NODE STARTUP AND RUNNING THE LISTENER
# ********************************************************************************************************************


def setup_subscribers():
    """
    We need to check for active topics and create subscribers for each topic that matches our configuration file.
    """
    # Set up the subscribers for each topic
    for rosType, mqtt_topic in topic_mappings.items():
        ros_type = ros_types.get(rosType)
        ros_msg_type = get_message_class(ros_type)
        if ros_msg_type is not None:
            rospy.loginfo(
                f"creating subscription for ros_msg_type: {ros_msg_type}, rosTopic: {rosType}, mqtt_topic: {mqtt_topic}")
            rospy.Subscriber(rosType, ros_msg_type,
                             sub_callback, callback_args=(mqtt_topic))
        else:
            rospy.logerr(
                f"Failed to get message class for type '{ros_type}'.")


def start_listener():
    master = verify_master_node()
    if master:
        rospy.init_node('mqtt_listener', anonymous=True)
        rospy.loginfo('MQTT Listener Node Started')
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTSTP, signal_handler)
        setup_subscribers()  # setup_subscribers()
        rospy.spin()  # Keep the program running until it's stopped


if __name__ == '__main__':
    start_listener()
