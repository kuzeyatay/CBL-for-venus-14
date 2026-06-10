import threading
import paho.mqtt.client as mqtt

from mqtt_websocket_bridge import (
    run_websocket_server,
    send_to_websocket,
    set_gui_message_handler,
)

BROKER = "mqtt.ics.ele.tue.nl"

ROBOT_83_SEND = "/pynqbridge/83/send"
ROBOT_83_RECV = "/pynqbridge/83/recv"
ROBOT_81_SEND = "/pynqbridge/81/send"
ROBOT_81_RECV = "/pynqbridge/81/recv"

client83 = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client81 = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)


# ---------------------------------------------------------------------------
# GUI → robots (two-way)
# ---------------------------------------------------------------------------
def on_gui_command(message: str):
    """Forward commands sent from the GUI to both robots."""
    print(f"[GUI CMD] Sending to robots: {message}")
    client81.publish(ROBOT_81_RECV, message)
    client83.publish(ROBOT_83_RECV, message)


set_gui_message_handler(on_gui_command)


# ---------------------------------------------------------------------------
# Robots → GUI
# ---------------------------------------------------------------------------
def parse_protocol_message(payload_str):
    parts = payload_str.split(',')
    print(parts)

    if parts[0] == "SENSOR_DATA" and len(parts) >= 9:
        bot_id = parts[1]
        dist   = parts[5]
        color  = parts[7]
        temp   = parts[8]
        return f"Robot {bot_id} | Temp: {temp}C | Dist: {dist}mm | Color: {color}"

    elif parts[0] == "EVENT":
        bot_id     = parts[1]
        event_type = parts[2]
        return f"*** EVENT from {bot_id}: {event_type} ***"

    return f"Raw: {payload_str}"


def on_message_83(client, userdata, message):
    payload_str = message.payload.decode().strip()
    print(parse_protocol_message(payload_str))
    send_to_websocket(payload_str)


def on_message_81(client, userdata, message):
    payload_str = message.payload.decode().strip()
    print(parse_protocol_message(payload_str))
    send_to_websocket(payload_str)


# ---------------------------------------------------------------------------
# MQTT connection callbacks
# ---------------------------------------------------------------------------
def on_connect(client, userdata, flags, reason_code, properties=None):
    topic = ROBOT_81_SEND if client == client81 else ROBOT_83_SEND
    if reason_code == 0:
        print(f"SUCCESS: Connected to {topic}")
        client.subscribe(topic)
    else:
        print(f"ERROR: Connection failed with code {reason_code}")


client83.username_pw_set("robot_83_1", "GsPO5eY7")
client83.on_connect = on_connect
client83.on_message = on_message_83

client81.username_pw_set("robot_81_1", "hZyDS0OF")
client81.on_connect = on_connect
client81.on_message = on_message_81


# ---------------------------------------------------------------------------
# Start
# ---------------------------------------------------------------------------
ws_thread = threading.Thread(target=run_websocket_server, daemon=True)
ws_thread.start()

print("Connecting both robots...")
client83.connect(BROKER, 1883, 60)
client81.connect(BROKER, 1883, 60)

client83.loop_start()
client81.loop_start()

print("CSV Relay + WebSocket running... Press CTRL+C to stop")
try:
    while True:
        pass
except KeyboardInterrupt:
    print("Stopping...")
    client83.loop_stop()
    client81.loop_stop()
