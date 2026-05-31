import paho.mqtt.client as mqtt

import threading

from mqtt_websocket_bridge import (
    run_websocket_server,
    send_to_websocket
)

BROKER = "mqtt.ics.ele.tue.nl"

# Topics
ROBOT_83_SEND = "/pynqbridge/83/send"
ROBOT_83_RECV = "/pynqbridge/83/recv"
ROBOT_81_SEND = "/pynqbridge/81/send"
ROBOT_81_RECV = "/pynqbridge/81/recv"

client83 = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client81 = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

def parse_protocol_message(payload_str):
    """Parses comma-separated protocol messages."""
    parts = payload_str.split(',')
    print(parts)
    
    if parts[0] == "SENSOR_DATA" and len(parts) >= 9:
        bot_id = parts[1]
        dist   = parts[5]
        color  = parts[7]
        temp   = parts[8]
        return f"Robot {bot_id} | Temp: {temp}C | Dist: {dist}mm | Color: {color}"
    
    elif parts[0] == "EVENT":
        bot_id = parts[1]
        event_type = parts[2]
        return f"*** EVENT from {bot_id}: {event_type} ***"
        
    return f"Raw: {payload_str}"

def forward_to_websocket(robot_id, payload_str):
    """
    Send raw MQTT data to websocket clients.
    """
    send_to_websocket(payload_str)

    print(f"Sent to websocket from robot {robot_id}")


def on_message_83(client, userdata, message):
    payload_str = message.payload.decode().strip()
    # 1. Process and print for the laptop console
    print(parse_protocol_message(payload_str))

    forward_to_websocket("83", payload_str)
    
    # 2. Forward the raw bytes to the other robot
    client81.publish(ROBOT_81_RECV, message.payload)
    print("Forwarded 83 → 81")

def on_message_81(client, userdata, message):
    payload_str = message.payload.decode().strip()
    # 1. Process and print for the laptop console
    print(parse_protocol_message(payload_str))

    forward_to_websocket("81", payload_str)
    
    # 2. Forward the raw bytes to the other robot
    client83.publish(ROBOT_83_RECV, message.payload)
    print("Forwarded 81 → 83")

# Connection Callbacks
def on_connect(client, userdata, flags, reason_code, properties=None):
    topic = ROBOT_81_SEND if client == client81 else ROBOT_83_SEND
    if reason_code == 0:
        print(f"SUCCESS: Connected to {topic}")
        client.subscribe(topic)
    else:
        print(f"ERROR: Connection failed with code {reason_code}")

# Setup Clients
client83.username_pw_set("robot_83_1", "GsPO5eY7")
client83.on_connect = on_connect
client83.on_message = on_message_83

client81.username_pw_set("robot_81_1", "hZyDS0OF")
client81.on_connect = on_connect
client81.on_message = on_message_81

ws_thread = threading.Thread(
    target=run_websocket_server,
    daemon=True
)

ws_thread.start()

print("Connecting both robots...")
client83.connect(BROKER, 1883, 60)
client81.connect(BROKER, 1883, 60)

client83.loop_start()
client81.loop_start()

print("CSV Relay + Websocket running... Press CTRL+C to stop")
try:
    while True:
        pass
except KeyboardInterrupt:
    print("Stopping...")
    client83.loop_stop()
    client81.loop_stop()
