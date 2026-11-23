# Rfm9X::Rfm9XSim

`Rfm9XSim` is a passive test component that attaches the
`Rfm9XManager` asynchronous SPI request/completion interface to
`Rfm9XSimModel`, a plain C++ model of the LoRa register subset used by the
manager. The model is also used directly by the manager's unit tests.
This is a test article, not flight software or a complete radio simulator.

## Interfaces and scheduling

| Port | Purpose |
| --- | --- |
| `spiWriteReadAsync` | Execute one simulated SPI request |
| `spiWriteReadAsyncComplete` | Return the request buffers and status |
| `run` | Deliver one pending SPI completion |
| `airDataIn` / `airDataReturnOut` | Inject received bytes and return the input buffer |
| `airDataOut` | Send a simulated transmitted payload to a byte-stream peer |
| `allocate` / `deallocate` | Supply temporary buffers for the transmitted payload |
| `airReady` | Driver-ready notification, currently ignored |

An SPI transaction is evaluated when requested; its completion is deferred
until `run`. Only one completion can be pending. Connect both the manager and
the simulator to a scheduler, and ensure the manager has at most one SPI
request outstanding. `SimSubtopology` connects the SPI pair but does not
provide scheduler, air-interface, or buffer-manager connections.

The SPI input is guarded, but `run` and `airDataIn` are synchronous and do
not share that guard. The deployment must serialize these handlers and the
model's users; this component does not provide general concurrent access
protection.

## Model behavior and limits

- The model implements the SPI address/write flag, register bursts, a
  256-byte FIFO with an incrementing FIFO pointer, and the LoRa register
  subset needed for configuration, packet TX/RX, IRQ clearing, and status.
  `RegVersion` at `0x42` reads `0x12`. Reset initializes modeled register
  values, including an intentional LoRa standby starting mode; it is not a
  complete power-on emulation.
- SPI byte exchanges advance a synthetic clock. One air-byte step occurs per
  two SPI byte clocks, subject to configurable mode-settle delays. TX takes
  a number of air steps proportional to payload size, asserts `TxDone`, and
  queues the payload. RX similarly advances a staged packet and sets
  `RxDone`, optionally with `PayloadCrcError`. These timings are not LoRa
  time-on-air calculations and stop advancing when SPI activity stops.
- The model buffers at most 2048 incoming bytes, discarding the oldest bytes
  on overflow. It packetizes buffered bytes into chunks up to
  `MAX_PAYLOAD_LENGTH`; a byte stream does **not** preserve original sender
  packet boundaries. Up to eight transmitted packets await retrieval; new
  transmitted packets are not queued if that queue is full.
- `Rfm9XSim` sends each retrieved TX payload through `airDataOut` using a
  temporary allocated buffer, which it deallocates after the synchronous
  call. A missing output drops the packet; an undersized allocation emits
  `SimAllocationFailed`. A connected byte-stream peer must consume the
  buffer during the call.
- Test controls can simulate an absent radio, busy channel, stalled TxDone,
  mode-settle delays, corrupt frames, and RSSI/SNR readbacks. No modulation,
  propagation, realistic RF timing, collisions, or actual CRC calculation is
  modeled. The simulator cannot prove hardware behavior or OTA GDS framing.

## Verification

The standalone model tests cover register access, FIFO behavior, packet
transmission/reception, and fault controls. Manager tests exercise the model
through the manager's SPI request/completion contract.
