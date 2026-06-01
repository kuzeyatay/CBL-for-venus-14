import asyncio
import logging
import socket
import websockets

# Show websockets handshake details so we can see why connections fail
logging.basicConfig(level=logging.INFO, format='%(asctime)s %(name)s %(levelname)s %(message)s')
logging.getLogger('websockets').setLevel(logging.DEBUG)

connected_clients = set()
_on_gui_message = None
_loop = None


def set_gui_message_handler(callback):
    global _on_gui_message
    _on_gui_message = callback


async def _handler(websocket, path=None):  # path= keeps compat with websockets < 10
    connected_clients.add(websocket)
    addr = websocket.remote_address
    print(f"[WS] GUI client connected from {addr}")
    try:
        async for message in websocket:
            print(f"[WS] GUI → robots: {message}")
            if _on_gui_message:
                _on_gui_message(message)
    except Exception as e:
        print(f"[WS] Handler error: {e}")
    finally:
        connected_clients.discard(websocket)
        print(f"[WS] GUI client disconnected from {addr}")


async def _broadcast(message):
    if not connected_clients:
        return
    await asyncio.gather(
        *[client.send(message) for client in list(connected_clients)],
        return_exceptions=True,
    )


async def _serve():
    global _loop
    _loop = asyncio.get_running_loop()

    # Pass None as host so Python binds on BOTH IPv4 (0.0.0.0) and
    # IPv6 (::) — fixes the common Windows issue where the browser
    # resolves "localhost" to ::1 but the server only listens on IPv4.
    server = await websockets.serve(
        _handler,
        host=None,
        port=8765,
        compression=None,       # disable permessage-deflate
        ping_interval=None,     # let the browser handle keep-alive
    )

    # Print every address the server is actually bound to
    for sock in server.sockets:
        addr = sock.getsockname()
        fam = "IPv6" if sock.family == socket.AF_INET6 else "IPv4"
        print(f"[WS] Listening on {fam} {addr[0]}:{addr[1]}")

    print("[WS] WebSocket server ready — waiting for GUI connection…")
    await asyncio.Future()   # run forever


def send_to_websocket(message):
    if _loop is not None and _loop.is_running():
        asyncio.run_coroutine_threadsafe(_broadcast(message), _loop)


def run_websocket_server():
    asyncio.run(_serve())
