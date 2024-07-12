#!/usr/bin/python3
# -*- coding: utf-8 -*-

import context  # Ensures paho is in PYTHONPATH
import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish


def on_message_msgs(mosq, obj, msg):
    # This callback will only be called for messages with topics that match
    # $SYS/broker/messages/#
    print("MESSAGES: " + msg.topic + " " +
          str(msg.qos) + " " + str(msg.payload))


def on_message_bytes(mosq, obj, msg):
    # This callback will only be called for messages with topics that match
    # $SYS/broker/bytes/#
    print("BYTES: " + msg.topic + " " + str(msg.qos) + " " + str(msg.payload))


def on_message(mosq, obj, msg):
    # This callback will be called for messages that we receive that do not
    # match any patterns defined in topic specific callbacks, i.e. in this case
    # those messages that do not have topics $SYS/broker/messages/# nor
    # $SYS/broker/bytes/#
    print(msg.topic + " " + str(msg.qos) + " " + str(msg.payload))


mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

# Add message callbacks that will only trigger on a specific subscription match.
mqttc.message_callback_add("$SYS/broker/messages/#", on_message_msgs)
mqttc.message_callback_add("$SYS/broker/bytes/#", on_message_bytes)
mqttc.on_message = on_message
mqttc.connect("mqtt.eclipseprojects.io", 1883, 60)
mqttc.subscribe("$SYS/#", 0)


msgs = [{'topic': "paho/test/multiple", 'payload': "multiple 1"},
        ("paho/test/multiple", "multiple 2", 0, False)]
publish.multiple(msgs, hostname="mqtt.eclipseprojects.io")

mqttc.loop_forever()
