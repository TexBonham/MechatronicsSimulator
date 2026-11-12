// Facade protocol client (ADR 0015 proposed; FAC-003, FAC-019, FAC-020). Envelope, version, message ids,
// request/response matching and tick_batch decoding. No domain logic (FAC-016).

export const PROTOCOL_VERSION = 1;

export class FacadeError extends Error {
  constructor(payload) {
    super(payload.message);
    this.code = payload.code;
    this.requirementId = payload.requirement_id;
    this.payload = payload;
  }
}

// FAC-019: header names dtype float64, little-endian, the sample count and the byte length of the one binary frame
// that follows: (1 + series) x samples x 8 bytes, time column first.
export function decodeTickBatch(header, buffer) {
  if (header.dtype !== "float64" || header.byte_order !== "little") {
    throw new Error(`tick_batch: unsupported dtype ${header.dtype} ${header.byte_order}`);
  }
  const width = 1 + header.series.length;
  const expected = width * header.sample_count * 8;
  if (header.byte_length !== expected || buffer.byteLength !== expected) {
    throw new Error(`tick_batch: ${buffer.byteLength} bytes, header says ${header.byte_length}, expected ${expected}`);
  }
  const view = new DataView(buffer);
  const columns = [];
  for (let column = 0; column < width; column += 1) {
    const values = new Float64Array(header.sample_count);
    for (let sample = 0; sample < header.sample_count; sample += 1) {
      values[sample] = view.getFloat64((column * header.sample_count + sample) * 8, true);
    }
    columns.push(values);
  }
  const series = {};
  header.series.forEach((name, index) => {
    series[name] = columns[index + 1];
  });
  return { runId: header.run_id, sequence: header.sequence, times: columns[0], series };
}

export class FacadeClient {
  constructor() {
    this.socket = null;
    this.nextId = 1;
    this.pending = new Map();
    this.listeners = [];
    this.batchWaiters = [];
    this.header = null;
  }

  connect(url) {
    return new Promise((resolve, reject) => {
      const socket = new WebSocket(url);
      socket.binaryType = "arraybuffer";
      socket.addEventListener("open", () => resolve(this));
      socket.addEventListener("error", () => reject(new Error("cannot reach the facade service")));
      socket.addEventListener("close", (event) => this.closed(event));
      socket.addEventListener("message", (event) => this.receive(event.data));
      this.socket = socket;
    });
  }

  onEvent(listener) {
    this.listeners.push(listener);
  }

  // `message` is { kind, payload }; resolves with the `ok` result, rejects with a FacadeError on `error`.
  request(message) {
    const id = `c${this.nextId}`;
    this.nextId += 1;
    const envelope = { version: PROTOCOL_VERSION, message_id: id, kind: message.kind, payload: message.payload };
    return new Promise((resolve, reject) => {
      this.pending.set(id, { resolve, reject });
      this.socket.send(JSON.stringify(envelope));
    });
  }

  // A replay subscription answers `ok`, then (when the window holds samples) one tick_batch for the run.
  async window(runId, series, tStart, tEnd, points) {
    const result = await this.request({
      kind: "subscribe_series",
      payload: { run_id: runId, series, t_start: tStart, t_end: tEnd, points },
    });
    if (result.sample_count === 0) {
      return { runId, sequence: 0, times: new Float64Array(0), series: {}, units: result.units };
    }
    const batch = await new Promise((resolve, reject) => {
      this.batchWaiters.push({ runId, resolve, reject });
    });
    batch.units = result.units;
    return batch;
  }

  receive(data) {
    if (data instanceof ArrayBuffer) {
      this.binary(data);
      return;
    }
    const message = JSON.parse(data);
    if (message.version !== PROTOCOL_VERSION) {
      this.emit({ localError: `unsupported protocol version ${message.version}` });
      return;
    }
    if (message.kind === "ok") {
      this.settle(message.payload.request_id, (entry) => entry.resolve(message.payload.result));
    } else if (message.kind === "error") {
      if (message.payload.request_id === null || !this.pending.has(message.payload.request_id)) {
        this.emit(message);
        return;
      }
      this.settle(message.payload.request_id, (entry) => entry.reject(new FacadeError(message.payload)));
    } else if (message.kind === "tick_batch") {
      this.header = message;
    } else if (message.kind === "run_progress" || message.kind === "run_ended" || message.kind === "timeline_event") {
      this.emit(message);
    }
  }

  binary(buffer) {
    const header = this.header;
    this.header = null;
    if (header === null) {
      this.emit({ localError: "binary frame without a tick_batch header" });
      return;
    }
    let batch = null;
    try {
      batch = decodeTickBatch(header.payload, buffer);
    } catch (error) {
      this.emit({ localError: error.message });
      return;
    }
    const waiter = this.batchWaiters.findIndex((entry) => entry.runId === batch.runId);
    if (waiter >= 0) {
      const [entry] = this.batchWaiters.splice(waiter, 1);
      entry.resolve(batch);
      return;
    }
    this.emit({ ...header, batch });
  }

  settle(id, action) {
    const entry = this.pending.get(id);
    if (entry === undefined) {
      return;
    }
    this.pending.delete(id);
    action(entry);
  }

  emit(message) {
    for (const listener of this.listeners) {
      listener(message);
    }
  }

  closed(event) {
    const error = new Error(`facade connection closed (${event.code})`);
    for (const entry of this.pending.values()) {
      entry.reject(error);
    }
    this.pending.clear();
    for (const entry of this.batchWaiters) {
      entry.reject(error);
    }
    this.batchWaiters = [];
    this.emit({ localError: error.message });
  }
}
