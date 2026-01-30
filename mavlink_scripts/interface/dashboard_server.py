
import asyncio
import websockets
import json
import logging
import http.server
import socketserver
import threading
import os

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("DashboardServer")

# Configuration
HTTP_PORT = 8000
WS_PORT = 8765

# Store connected clients
clients = set()

async def ws_handler(websocket):
    """
    Handle incoming WebSocket connections.
    """
    clients.add(websocket)
    logger.info(f"WS Client connected. Total: {len(clients)}")
    
    try:
        async for message in websocket:
            # Broadcast message to all other clients
            if clients:
                disconnected_clients = set()
                for client in clients:
                    if client != websocket:
                        try:
                            await client.send(message)
                        except websockets.exceptions.ConnectionClosed:
                            disconnected_clients.add(client)
                
                for client in disconnected_clients:
                    clients.remove(client)
                    
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        clients.remove(websocket)
        logger.info(f"WS Client disconnected. Total: {len(clients)}")

def run_http_server():
    """
    Run a simple HTTP server to serve dashboard.html
    """
    web_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(web_dir) # Serve from this directory
    
    Handler = http.server.SimpleHTTPRequestHandler
    with socketserver.TCPServer(("", HTTP_PORT), Handler) as httpd:
        logger.info(f"Serving HTTP at http://localhost:{HTTP_PORT}")
        httpd.serve_forever()

async def main():
    # Start HTTP Server in a separate thread
    http_thread = threading.Thread(target=run_http_server, daemon=True)
    http_thread.start()
    
    # Start WebSocket Server
    logger.info(f"Serving WebSockets at ws://localhost:{WS_PORT}")
    async with websockets.serve(ws_handler, "0.0.0.0", WS_PORT):
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("Server stopped.")
