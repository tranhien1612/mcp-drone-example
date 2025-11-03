import json
from mcp.server.fastmcp import FastMCP
from mcp.server import Server
from mcp.server.sse import SseServerTransport

from starlette.applications import Starlette
from starlette.routing import Mount, Route
from starlette.requests import Request

import uvicorn

from drone import *

# MCPClient ---SSE over HTTP---> UvicornServer ---> ASGI Protocol Bridge ---> Starlette App ---> FastMCP Server

mcp = FastMCP("drone-control-mcp-server")

drone = DroneController()

@mcp.tool()
def add_connection(connection_ip: str, port: int) -> str:
    return drone.connect(connection_ip, port)

@mcp.tool()
def remove_connection() -> str:
    return drone.disconnect()

@mcp.tool()
def arm() -> str:
    return drone.arm()

@mcp.tool()
def disarm() -> str:
    return drone.disarm()

@mcp.tool()
def takeoff(altitude: float = 10.0) -> str:
    return drone.takeoff(altitude)

@mcp.tool()
def landing() -> str:
    return drone.land()

@mcp.tool()
def get_drone_status() -> str:
    return json.dumps(drone.get_info(), indent=2)

@mcp.tool()
def change_flight_mode(mode_name: str) -> str:
    return drone.change_mode(mode_name)

@mcp.tool()
def goto_location(latitude: float, longitude: float, altitude: float) -> str:
    return drone.goto_location(latitude, longitude, altitude)


# Create Starlette app to expose the tools via HTTP (using SEE)
def create_starlette_app(mcp_server: Server, *, debug: bool = False) -> Starlette:
    sse = SseServerTransport("/messages/")

    async def handle_sse(request: Request) -> None:
        async with sse.connect_sse(
            request.scope,
            request.receive,
            request._send,
        ) as (read_stream, write_stream):
            await mcp_server.run(
                read_stream,
                write_stream,
                mcp_server.create_initialization_options(),
            )
    return Starlette(
        debug=debug,
        routes=[
            Route("/sse", endpoint=handle_sse),
            Mount("/messages/", app=sse.handle_post_message),
        ],
    )

if __name__ == "__main__":
    mcp_server = mcp._mcp_server

    import argparse
    parser = argparse.ArgumentParser(description="Run MCP SSE-based server")
    parser.add_argument("--host", default='0.0.0.0',help="Host to bind the server to")
    parser.add_argument("--port", type=int, default=8081, help="Port tolisten on")
    args = parser.parse_args()

    starlette_app = create_starlette_app(mcp_server, debug=True)
    uvicorn.run(starlette_app, host=args.host, port=args.port)

                
