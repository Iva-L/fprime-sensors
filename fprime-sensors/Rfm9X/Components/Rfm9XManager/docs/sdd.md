# Rfm9X::Rfm9XManager

## Purpose and scope

`Rfm9XManager` is a passive F Prime `Svc.Com` adapter for an SX1276/8-based
RFM9X LoRa radio. Each input buffer is one raw RF packet; it does not frame,
segment, reassemble, or echo application data. A deployment must supply any
framing, routing, and ground gateway needed for an F Prime-over-radio link.

The manager polls the radio over asynchronous SPI and uses LoRa explicit
headers, payload CRC, an eight-symbol preamble, and a shared 256-byte FIFO.
Packets contain 1 through `MAX_PAYLOAD_LENGTH` bytes (255 by default). Both
FIFO base addresses are set to zero: this half-duplex design uses the FIFO
for one direction at a time. TX fills the FIFO in standby before requesting
TX; RX reads the completed packet using `FifoRxCurrentAddr`. There is no
FSK-style FIFO streaming.

| File | Responsibility |
| --- | --- |
| `Rfm9XManager.fpp` | Ports, commands, parameters, events, and telemetry |
| `Rfm9XManager.cpp` | Lifecycle, RX, queueing, buffer ownership, and F Prime handlers |
| `Rfm9XHelpers.cpp` | SPI request construction, TX state machine, RSSI/SNR conversion |
| `Rfm9XRadio.hpp` | Register constants and parameter-to-register mappings |
| `Rfm9XManagerCfg/Rfm9XManagerCfg.fpp` | Compile-time limits and timing defaults |

## Configuration and operator interface

| Parameter | Library default | Supported values |
| --- | --- | --- |
| `SPREADING_FACTOR` | `SF7` | `SF7`-`SF12`; SF6 requires unsupported implicit-header handling |
| `BANDWIDTH` | `BW_125_KHZ` | 7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250, or 500 kHz |
| `CODING_RATE` | `CR_4_5` | 4/5, 4/6, 4/7, 4/8 |
| `FREQUENCY_HZ` | 434000000 | U32 carrier frequency, converted using a 32 MHz crystal |
| `TX_POWER` | `DBM_13` | 5, 10, 13, 17, or 20 dBm on PA_BOOST |

These are component defaults, not a guarantee of valid operation for every
board or frequency. Select supported radio hardware and a locally permitted
frequency and power. Match the peer's LoRa configuration. The manager converts
frequency to register values but does not validate the hardware's supported
band or regulatory limits. Low-data-rate optimization is enabled when the
configured symbol period exceeds 16 ms. For `DBM_20`, the configuration
uses the high-power PA DAC and increased overcurrent limit.

`TRANSMIT` enables or disables transmission; disabling leaves reception
enabled. `RESET` starts the optional reset/power-enable pulse and
reinitialization, aborting an active transmission. Parameter updates while
ready trigger reconfiguration. Parameter persistence is a deployment concern.
There is no manager `SEND_STRING` command; test commands belong in a
deployment-specific component.

Events are `ConfigurationFailed`, `SendFailed`, `RadioReady`,
`ResetInitiated`, and `AllocationFailed`. `SendFailed` distinguishes invalid
size, radio fault, local queue full, and not-ready reasons. Telemetry includes
`RadioState`, `PacketsTransmitted`, `PacketsReceived`, `RxCrcErrors`,
`LastPacketRssi`, and `LastPacketSnr`. The current RSSI conversion is
`-137 + RegPktRssiValue` dBm; SNR is the signed register value divided by
four. Verify the RSSI conversion for the operating band and hardware before
using it for link-budget decisions.

## Deployment and buffer contract

1. Connect the asynchronous SPI request and completion ports to a driver that
   completes every request. Connect `resetGpio` if reset or power control is
   available; the output drives LOW then HIGH. Adjust reset hold and settle
   times for the actual reset circuit or power switch.
2. Connect `run` to a periodic scheduler, call `configure(tickPeriodMs)`
   before scheduling, and load the FPP parameters. The constructor assumes a
   1 ms period until `configure` is called. Millisecond settings are rounded
   up to whole ticks.
3. Connect the `Svc.Com` data ports and RX `allocate`/`deallocate` ports.
   The caller retains TX buffer contents until `dataReturnOut` returns the
   buffer and its frame context. RX buffers allocated by the manager are
   delivered on `dataOut` and must be returned through `dataReturnIn`.
4. Choose the TX backpressure mode by wiring `comStatusOut`. When connected,
   an unavailable radio returns the input buffer and reports Com failure for
   an upstream retry path; readiness/resumption is signaled through Com status.
   When unconnected, the manager locally holds pending buffers up to
   `PENDING_TX_DEPTH`, preserving FIFO order. A full hold queue returns the
   buffer and reports `QUEUE_FULL`. Held invalid-size buffers are rejected
   when they reach the head of the queue.

The reference hardware and simulation subtopologies wire the SPI pair only;
the deployment must also connect scheduling, parameters, buffers, and the
appropriate communications or test component. The current STM32 and SAMD
reference deployments use UART for GDS and raw radio packets for testing;
they do not provide an OTA GDS gateway.

The implementation stages SPI, event, telemetry, and status outputs for
dispatch outside its radio lock. The staged-output batch and simulator require
serialized handler execution as in the supported cyclic-executive
deployments; the mutex alone is not a guarantee that arbitrary concurrent
port invocations are safe. Preserve the one-outstanding-SPI-request contract
and serialize callers when adapting the component to another execution model.
After `RESET` abandons an outstanding request, its completion must arrive
before a new request starts: callbacks have no transaction identifier with
which to distinguish an old completion from a new one.

## State and timing

After parameters are loaded, `run` advances optional reset assertion, hold,
and settle, then reads `RegVersion` (`0x42`, expected `0x12`). It enters LoRa
mode through sleep, writes the configuration table, clears IRQs, and enters
continuous RX. A failed detection or configuration is reported and retried.
`RadioReady` indicates configuration completed, but a successful modem-status
read is still required before a send can start.

TX and RX advance as bounded SPI steps. TX polls `TxDone` and attempts RX
recovery on SPI failure or timeout. RX polls modem status and IRQ flags,
drops and counts CRC-error packets, reads RSSI/SNR and packet length, then
reads and delivers the FIFO payload. `PacketsReceived` counts successful FIFO
reads, even if a subsequent buffer allocation fails.

The cached `SignalDetected` bit blocks TX until a successful status sample
indicates no signal. It is reset to busy on startup, reset, reconfiguration,
or status-read failure. `RxOngoing` alone is not used as a gate because it
was observed set while idle. This cached signal check and the holdoffs are
best-effort pacing, not collision-free channel assessment, listen-before-talk,
or evidence of regulatory compliance.

`Rfm9X.ManagerCfg` defaults are 5000 ms TX-done timeout, 500 ms post-RX and
post-TX holdoffs, 1000 ms RX-holdoff starvation bound, 1 ms reset hold, 6 ms
reset settle, 255-byte maximum payload, and eight local pending-TX entries.
The local hold path observes post-TX pacing but ignores the post-RX holdoff;
the Com retry path observes both. The TX timeout is advanced by completed
status polls, so scheduler and SPI completion latency affect elapsed time.
Tune the timeout for the selected LoRa profile and maximum payload; the
default is not a guarantee for every spreading-factor/bandwidth combination.

## Verification

The manager unit tests use the [register model](../../Rfm9XSim/docs/sdd.md)
to exercise initialization, register mapping, TX/RX, ownership, queueing,
reset, CRC errors, status gating, and failures. The simulator is not RF
hardware and cannot validate signal integrity, timing, power, or range.
The reference projects have reported a physical raw-string echo using earlier
firmware; repeat hardware testing after firmware or topology changes.
