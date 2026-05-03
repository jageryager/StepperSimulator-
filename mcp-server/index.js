/**
 * StepperSimulator MCP Server
 *
 * Listens to the StepperSimulator Arduino sketch streaming on COM7 (115200 baud).
 *
 * The sketch emits a status line every 2 seconds in this format:
 *   Motor steps: ,<sub>, | Encoder: ,<enc>, | MotorPos: ,<motorPos>, |
 *   EncPos: ,<encPos>, | Z: ,<HIGH/low>, | MotorDegrees: ,<degrees>
 *
 * Two MCP tools are exposed:
 *   get_encoder_angle  — returns the raw encoder count  (long integer)
 *   get_position_angle — returns the motor angle in degrees (float, 2 dp)
 *
 * The server keeps the port open for the lifetime of the process.
 * Tool calls return the most-recently received values immediately, or wait
 * up to WAIT_TIMEOUT_MS for the next status line if the cache is empty.
 */

import { Server }              from "@modelcontextprotocol/sdk/server/index.js";
import { StdioServerTransport } from "@modelcontextprotocol/sdk/server/stdio.js";
import {
  CallToolRequestSchema,
  ListToolsRequestSchema,
} from "@modelcontextprotocol/sdk/types.js";
import { SerialPort }          from "serialport";
import { ReadlineParser }      from "@serialport/parser-readline";

// ─── Configuration ────────────────────────────────────────────────────────────
const PORT_PATH      = "COM7";
const BAUD_RATE      = 115200;
const WAIT_TIMEOUT_MS = 5000;   // max wait for a fresh status line

// ─── Serial Port Setup ────────────────────────────────────────────────────────
const port   = new SerialPort({ path: PORT_PATH, baudRate: BAUD_RATE, autoOpen: false });
const parser = port.pipe(new ReadlineParser({ delimiter: "\n" }));

/**
 * Cached values from the most recent status line.
 * null = not yet received.
 */
let cache = {
  encoderCounts: null,   // long integer
  motorDegrees:  null,   // float
  timestamp:     0,      // Date.now() when this was last updated
};

/**
 * Pending resolvers waiting for the next status update.
 * Each entry is called as soon as a new status line arrives.
 */
const pendingWaiters = [];

// Status line example:
//   Motor steps: ,-204800, | Encoder: ,-10000, | MotorPos: ,9, | EncPos: ,9, | Z: ,HIGH, | MotorDegrees: ,360.00
const STATUS_RE = /Encoder:\s*,(-?\d+),.*MotorDegrees:\s*,(-?[\d.]+)/;

parser.on("data", (rawLine) => {
  const line = rawLine.toString().trim();
  const m    = STATUS_RE.exec(line);
  if (!m) return;   // not a status line (e.g. banner or jitter report)

  cache.encoderCounts = parseInt(m[1], 10);
  cache.motorDegrees  = parseFloat(m[2]);
  cache.timestamp     = Date.now();

  // Wake all waiters
  for (const resolve of pendingWaiters.splice(0)) {
    resolve(cache);
  }
});

port.on("error", (err) => {
  process.stderr.write(`[serial] error: ${err.message}\n`);
  // Reject any lingering waiters
  for (const resolve of pendingWaiters.splice(0)) {
    resolve(null);
  }
});

/** Open the port (idempotent). */
function openPort() {
  return new Promise((resolve, reject) => {
    if (port.isOpen) return resolve();
    port.open((err) => (err ? reject(err) : resolve()));
  });
}

/**
 * Return the current cache immediately if populated, otherwise wait up to
 * WAIT_TIMEOUT_MS for the next status line.
 * Rejects with an Error on timeout.
 */
function getLatestStatus() {
  if (cache.encoderCounts !== null) {
    return Promise.resolve(cache);
  }

  return new Promise((resolve, reject) => {
    const timer = setTimeout(() => {
      const idx = pendingWaiters.indexOf(resolve);
      if (idx !== -1) pendingWaiters.splice(idx, 1);
      reject(new Error(`Timeout: no status line received from ${PORT_PATH} within ${WAIT_TIMEOUT_MS} ms`));
    }, WAIT_TIMEOUT_MS);

    pendingWaiters.push((c) => {
      clearTimeout(timer);
      if (c === null) {
        reject(new Error("Serial port error while waiting for status."));
      } else {
        resolve(c);
      }
    });
  });
}

// ─── MCP Server ───────────────────────────────────────────────────────────────
const server = new Server(
  { name: "stepper-simulator", version: "1.0.0" },
  { capabilities: { tools: {} } }
);

server.setRequestHandler(ListToolsRequestSchema, async () => ({
  tools: [
    {
      name: "get_encoder_angle",
      description:
        "Read the current raw encoder count from the StepperSimulator on COM7. " +
        "Returns the signed integer encoder count (10 000 counts per output-shaft revolution).",
      inputSchema: { type: "object", properties: {}, required: [] },
    },
    {
      name: "get_position_angle",
      description:
        "Read the current motor position angle in degrees from the StepperSimulator on COM7. " +
        "Returns a floating-point value representing the motor shaft angle (0–360°, wrapping).",
      inputSchema: { type: "object", properties: {}, required: [] },
    },
  ],
}));

server.setRequestHandler(CallToolRequestSchema, async (request) => {
  const { name } = request.params;

  if (name !== "get_encoder_angle" && name !== "get_position_angle") {
    throw new Error(`Unknown tool: ${name}`);
  }

  await openPort();
  const status = await getLatestStatus();

  if (name === "get_encoder_angle") {
    return {
      content: [
        {
          type: "text",
          text: `Encoder count: ${status.encoderCounts} counts`,
        },
      ],
    };
  }

  // get_position_angle
  return {
    content: [
      {
        type: "text",
        text: `Position angle: ${status.motorDegrees.toFixed(2)}°`,
      },
    ],
  };
});

// ─── Start ────────────────────────────────────────────────────────────────────
async function main() {
  process.stderr.write(`[stepper-mcp] Opening ${PORT_PATH} at ${BAUD_RATE} baud…\n`);
  await openPort();
  process.stderr.write(`[stepper-mcp] Serial port open — listening for status lines.\n`);
  process.stderr.write(`[stepper-mcp] Starting MCP server on stdio.\n`);

  const transport = new StdioServerTransport();
  await server.connect(transport);
}

main().catch((err) => {
  process.stderr.write(`[stepper-mcp] Fatal: ${err.message}\n`);
  process.exit(1);
});
