import os
import asyncio
import sys
import json
from typing import Optional

from mcp import ClientSession
from mcp.client.sse import sse_client

from google import genai
from google.genai import types
from google.genai.types import Tool, FunctionDeclaration
from google.genai.types import GenerateContentConfig

from dotenv import load_dotenv
load_dotenv()

class MCPClient:
    def __init__(self):
        self.seeion: Optional[ClientSession] = None
        self._streams_context = None
        self._session_context = None
        gemini_api_key = os.getenv("GENAI_API_KEY")

        if not gemini_api_key:
            raise ValueError("GENAI_API_KEY environment variable is not set.")
        
        self.genai_client = genai.Client(api_key=gemini_api_key)

    async def connect_to_sse_server(self, server_url: str):
        self._streams_context = sse_client(server_url)
        streams = await self._streams_context.__aenter__()

        self._session_context = ClientSession(*streams)
        self.session: ClientSession = await self._session_context.__aenter__()

        await self.session.initialize()
        print("Initialized SSE Client Session.")
        print("Listing tools...")
        response = await self.session.list_tools()
        tools = response.tools
        print("Connected to server with tools:", [tool.name for tool in tools])

        self.function_declarations = convert_mcp_tools_to_gemini(tools)
    
    async def cleanup(self):
        if self._session_context:
            await self._session_context.__aexit__(None, None, None)
        if self._streams_context:
            await self._streams_context.__aexit__(None, None, None)
    
    async def process_query(self, query: str) -> str:
        # Format user input as structured content object for gemini
        user_prompt_content = types.Content(
            role='user',
            parts=[types.Part.from_text(text=query)]
        )
        # Send the query to the Gemini model
        response = self.genai_client.models.generate_content(
            model="gemini-2.0-flash-001",
            contents=[user_prompt_content],
            config=types.GenerateContentConfig(
                tools=self.function_declarations,
            ),
        )

        final_text = []
        # Process each candiate response from Gemini
        for candidate in response.candidates:
            if candidate.content.parts:
                for part in candidate.content.parts:
                    if isinstance(part, types.Part):
                        if part.function_call:
                            tool_name = part.function_call.name
                            tool_args = part.function_call.args
                            print(f"\n[Gemini requested tool call: {tool_name} with args: {tool_args}]")

                            try:
                                result = await self.session.call_tool(tool_name, tool_args)

                                print("Tool raw result:", result)
                                if hasattr(result, "isError") and result.isError:
                                    # Extract error message text if present
                                    text_value = ""
                                    if hasattr(result, "content") and result.content:
                                        text_value = getattr(result.content[0], "text", "")
                                    raise RuntimeError(text_value or "Unknown MCP tool error")

                                # Extract actual return text
                                if hasattr(result, "content") and result.content:
                                    text_value = getattr(result.content[0], "text", "")
                                else:
                                    text_value = str(result)

                                function_response = {"result": text_value}

                                # function_response = {"result": result.result}
                            except Exception as e:
                                print("Tool call failed:", e)
                                function_response = {"error": str(e)}
                            
                            function_response_part = types.Part.from_function_response(
                                name=tool_name,
                                response=function_response
                            )

                            function_response_content = types.Content(
                                role='tool',
                                parts=[function_response_part]
                            )

                            # Send tool execution result back to Gemini
                            response = self.genai_client.models.generate_content(
                                model="gemini-2.0-flash-001",
                                contents=[
                                    user_prompt_content, 
                                    part, 
                                    function_response_content,
                                ],
                                config=types.GenerateContentConfig(
                                    tools=self.function_declarations,
                                ),
                            )

                            final_text.append(response.candidates[0].content.parts[0].text)
                        else:
                            final_text.append(part.text)
                
        return "\n".join(final_text)

    async def chat_loop(self):
        print("\nMCP Client started! Type 'quit' to exit.")
        while True:
            query = input("\nQuery: ").strip()
            if query.lower() == 'quit':
                print("Exiting MCP Client.")
                break

            response = await self.process_query(query)
            print("\nGemini:", response)



def clean_schema(schema):
    if isinstance(schema, dict):
        schema.pop("title", None) # remove title if present

        # Recursively clean nested properties
        if "properties" in schema and isinstance(schema["properties"], dict):
            for key in schema["properties"]:
                schema["properties"][key] = clean_schema(schema["properties"][key])
    
    return schema

def convert_mcp_tools_to_gemini(mcp_tools):
    gemini_tools = []

    for tool in mcp_tools:
        parameters = clean_schema(tool.inputSchema)

        function_declaration = FunctionDeclaration(
            name=tool.name,
            description=tool.description,
            parameters=parameters
        )

        # Wrap in a Tool object
        gemini_tool = Tool(function_declarations=[function_declaration])
        gemini_tools.append(gemini_tool)

    return gemini_tools

async def main():
    if len(sys.argv) < 2:
        print("Usage: python client.py <server_url>")
        sys.exit(1)

    client = MCPClient()
    try:
        await client.connect_to_sse_server(sys.argv[1])
        await client.chat_loop()
    finally:
        await client.cleanup()


if __name__ == "__main__":
    asyncio.run(main())

# uv add mcp python-dotenv google-genai
# run: uv run client.py ./servers/sample_server.py 
