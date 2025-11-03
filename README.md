# mcp-drone-example
## Install lib
```
python>=3.10
uv
mcp
```

## Create project and install lib
### Client
```
uv init client
uv venv
.venv\Scripts\activate

uv add mcp python-dotenv google-genai
```

### Server
```
uv init server
uv venv
.venv\Scripts\activate

uv add mcp python-dotenv google-genai pymavlink
```

## Run
### Server
```
uv run server.py
```
### Client
```
uv run client.py http://localhost:8081/sse
```

