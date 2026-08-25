#!/usr/bin/env node
// Opens a serial port, autobauds against a WT901B (tries 115200, 9600,
// 460800 in turn, 8N1), and prints the first 10 valid frames it reads
// to stdout as hex ("55 53 ...").
//
// Usage: node read-hwt901b.js /dev/ttyUSB0
//
// Requires: npm install serialport

const { SerialPort } = require('serialport');

const FRAME_LEN = 11;
const CANDIDATE_BAUDS = [115200, 9600, 460800]; // 115200 recommended, tried first
const DETECT_TIMEOUT_MS = 1000;
const LINES_TO_PRINT = 10;

const path = process.argv[2];
if (!path) {
  console.error('Usage: node read-hwt901b.js <serial-port-path>');
  process.exit(1);
}

// checksum = low byte of the sum of the first 10 bytes of an 11-byte frame
function isValidFrame(buf) {
  if (buf.length !== FRAME_LEN || buf[0] !== 0x55) return false;
  let sum = 0;
  for (let i = 0; i < 10; i++) sum = (sum + buf[i]) & 0xff;
  return sum === buf[10];
}

function openPort(baudRate) {
  return new Promise((resolve, reject) => {
    const port = new SerialPort(
      { path, baudRate, dataBits: 8, parity: 'none', stopBits: 1, autoOpen: false },
      (err) => {}
    );
    port.open((err) => (err ? reject(err) : resolve(port)));
  });
}

// Buffers incoming bytes, syncing on 0x55, and resolves the first
// complete frame that passes the checksum check within timeoutMs (as a
// Buffer), or null if none arrives in time.
function waitForValidFrame(port, timeoutMs) {
  return new Promise((resolve) => {
    let buf = Buffer.alloc(0);
    let done = false;

    const onData = (chunk) => {
      buf = Buffer.concat([buf, chunk]);
      // Drop leading bytes until a 0x55 header (or the buffer is empty).
      while (buf.length > 0 && buf[0] !== 0x55) buf = buf.subarray(1);
      while (buf.length >= FRAME_LEN) {
        const frame = buf.subarray(0, FRAME_LEN);
        buf = buf.subarray(FRAME_LEN);
        if (isValidFrame(frame)) {
          if (!done) {
            done = true;
            clearTimeout(timer);
            port.off('data', onData);
            resolve(Buffer.from(frame));
          }
          return;
        }
        while (buf.length > 0 && buf[0] !== 0x55) buf = buf.subarray(1);
      }
    };

    const timer = setTimeout(() => {
      if (!done) {
        done = true;
        port.off('data', onData);
        resolve(null);
      }
    }, timeoutMs);

    port.on('data', onData);
  });
}

function formatFrame(frame) {
  return [...frame].map((b) => b.toString(16).padStart(2, '0')).join(' ');
}

// Reads exactly `count` valid frames from an already-open, already-
// baud-matched port and prints each as a hex line.
function printFrames(port, count) {
  return new Promise((resolve) => {
    let buf = Buffer.alloc(0);
    let printed = 0;

    const onData = (chunk) => {
      buf = Buffer.concat([buf, chunk]);
      while (buf.length > 0 && buf[0] !== 0x55) buf = buf.subarray(1);
      while (buf.length >= FRAME_LEN && printed < count) {
        const frame = buf.subarray(0, FRAME_LEN);
        buf = buf.subarray(FRAME_LEN);
        if (isValidFrame(frame)) {
          console.log(formatFrame(frame));
          printed++;
        }
        while (buf.length > 0 && buf[0] !== 0x55) buf = buf.subarray(1);
      }
      if (printed >= count) {
        port.off('data', onData);
        resolve();
      }
    };

    port.on('data', onData);
  });
}

async function main() {
  for (const baudRate of CANDIDATE_BAUDS) {
    let port;
    try {
      port = await openPort(baudRate);
    } catch (err) {
      console.error(`Failed to open ${path} at ${baudRate}: ${err.message}`);
      process.exit(1);
    }

    console.error(`Trying ${baudRate} baud...`);
    const firstFrame = await waitForValidFrame(port, DETECT_TIMEOUT_MS);
    if (firstFrame) {
      console.error(`Locked at ${baudRate} baud. First ${LINES_TO_PRINT} frames:`);
      console.log(formatFrame(firstFrame));
      if (LINES_TO_PRINT > 1) await printFrames(port, LINES_TO_PRINT - 1);
      port.close();
      return;
    }

    await new Promise((resolve) => port.close(resolve));
  }

  console.error(`No valid WT901B data seen at any of: ${CANDIDATE_BAUDS.join(', ')}`);
  process.exit(1);
}

main();
