"""
Quick standalone WebSocket echo server on port 8765.
Run this, then open the GUI — if it connects, the issue is in pc_recieve_ws.py.
If it still fails, the issue is network/firewall/browser.
"""
import asyncio
import socket
import websockets

async def echo(ws):
    print(f"[TEST] client connected: {ws.remote_address}")
    async for msg in ws:
        print(f"[TEST] received: {msg}")
        await ws.send(f"ECHO: {msg}")

async def main():
    server = await websockets.serve(echo, host=None, port=8765, compression=None)
    for s in server.sockets:
        fam = "IPv6" if s.family == socket.AF_INET6 else "IPv4"
        print(f"[TEST] listening {fam} port {s.getsockname()[1]}")
    print("[TEST] ready — open the GUI now")
    await asyncio.Future()

asyncio.run(main())
