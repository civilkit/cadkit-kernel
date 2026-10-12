// MCP stdio entry: run with `node packages/mcp.mjs` (build first).
// env: CK_WASM overrides the kernel wasm path.
import { StdioServerTransport } from '@modelcontextprotocol/sdk/server/stdio.js';
import { createMcpServer } from './dist/session/src/mcp-server.js';

const { server } = createMcpServer();
await server.connect(new StdioServerTransport());
