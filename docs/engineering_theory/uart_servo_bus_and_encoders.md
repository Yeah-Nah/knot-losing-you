# UART, Servo Bus, and Servo Encoders — Reference

Covers serial communication fundamentals for the Waveshare UGV Rover pan-tilt system.
Focus: what UART is, what TX and RX pins are, how a half-duplex single-wire servo bus differs
from ordinary serial, what a servo bus is and why it exists, what a servo encoder is, and how
all of these pieces fit together to explain the pan-servo feedback failure investigated in this
project.

---

## 1. Bits and Bytes: Sending Data Between Chips

Computers talk to peripherals by sending binary data — streams of 0s and 1s. Physically, these
are represented as voltage levels on a wire (for example, 3.3 V = 1, 0 V = 0 on a
3.3 V logic-level system).

The central question is: how does the receiver know where one bit ends and the next begins?
There are two broad answers.

**Synchronous serial** uses a separate *clock* wire. The transmitter toggles a clock signal at
a fixed rate, and the receiver samples the data wire on every clock edge. Both sides stay
in sync because they share the clock.

**Asynchronous serial** has no clock wire. Both sides instead agree in advance on a speed
(the **baud rate**, measured in symbols per second). The transmitter and receiver each run their
own internal clock at that agreed speed. A start bit and a stop bit frame each byte so the
receiver knows when to start and stop counting.

UART is the dominant asynchronous serial standard.

---

## 2. UART — Universal Asynchronous Receiver-Transmitter

### What It Is

UART is a hardware module built into almost every microcontroller (including the ESP32). It
converts a byte into a serial sequence of voltage pulses on a wire, and reconstitutes received
pulses back into bytes. The programmer writes one byte; the UART hardware handles all the timing.

The key parameters agreed by both sides before communication begins:

| Parameter | Typical values | What it controls |
|---|---|---|
| Baud rate | 9600, 115200, 1000000 | Bit duration = 1 / baud |
| Data bits | 8 | Payload size per frame |
| Stop bits | 1 or 2 | Framing delimiter at end |
| Parity | None, Even, Odd | Optional error check bit |

For this project, two UART connections exist at different speeds:

- **Raspberry Pi → ESP32:** 115200 baud, used for JSON command-response protocol.
- **ESP32 → ST3215 servo bus:** 1,000,000 baud (1 Mbps), used for the binary SCServo protocol.

### The TX and RX Pins

Every UART connection uses two signal wires:

- **TX (Transmit):** the wire on which this device *sends* data.
- **RX (Receive):** the wire on which this device *listens* for data.

The fundamental wiring rule is:

$$\text{TX}_A \rightarrow \text{RX}_B \qquad \text{TX}_B \rightarrow \text{RX}_A$$

That is, the transmit pin of device A connects to the receive pin of device B, and vice versa.
This is called a **crossed** or **null-modem** connection. Connecting TX to TX by mistake is a
common hardware error — both devices would be transmitting on the same wire, neither would
receive anything.

A UART frame for a single byte looks like:

```
Idle  Start  D0  D1  D2  D3  D4  D5  D6  D7  Stop  Idle
 1      0    ..  ..  ..  ..  ..  ..  ..  ..    1     1
```

The line sits at 1 (idle/high) between frames. The start bit is always 0 (low), which the
receiver uses to detect the beginning of a new byte. Eight data bits follow, LSB first, then
one or more stop bits return the line to idle.

### FIFO Buffers

The UART hardware contains small first-in-first-out (FIFO) buffers — small queues of bytes —
on both the TX and RX sides.

- When software calls a write function, bytes go into the **TX FIFO**. The hardware drains this
  queue autonomously, sending bytes at the agreed baud rate.
- When bytes arrive on the RX pin, the hardware places them in the **RX FIFO**. Software reads
  them out when ready.

A critical consequence: **writing a byte to the TX FIFO and the byte actually departing the wire
are not the same event.** The write call returns immediately once the byte is in the FIFO. The
wire transmission follows at the hardware's own pace. This distinction matters greatly for the
servo bus discussed in Section 4.

---

## 3. Full-Duplex vs Half-Duplex

These terms describe the direction(s) in which data can flow simultaneously.

### Full-Duplex

Two separate wires (TX and RX) allow both sides to transmit at the same time. The Raspberry Pi
to ESP32 link in this project is full-duplex: the Pi can send a JSON command while simultaneously
the ESP32 is sending a telemetry response back.

```
Pi TX ──────────────────────→ ESP32 RX
Pi RX ←────────────────────── ESP32 TX
```

Data flows independently in both directions on independent wires.

### Half-Duplex

Only one side can transmit at a time. Both devices share the same wire for both transmit and
receive. This is common when:

- The protocol requires a request-before-response discipline (only one side speaks at a time).
- Reducing wire count matters (one wire instead of two).
- The physical bus is a broadcast medium shared by many devices.

To operate correctly in half-duplex, a device must switch between transmit mode and receive
mode. While transmitting, it drives the wire. After transmitting, it releases the wire and
listens. If it fails to release the wire (or releases it too slowly), it will either prevent
the remote device's response from reaching it, or read its own outgoing transmission back
as if it were an incoming response.

This switching behaviour is exactly where the ST3215 servo bus failure in this project occurs.
See Section 6.

---

## 4. What Is a Servo Bus?

### Traditional PWM Servos

The simplest hobby servos (steering servos, simple robot joints) use a single **PWM signal**
(Pulse Width Modulation) for control. The controller sends a pulse of variable width — typically
1000–2000 µs — once every 20 ms. The servo's internal circuit measures the pulse width and
moves to the corresponding angle.

PWM control has significant limitations at scale:

| Limitation | Effect |
|---|---|
| One wire per servo | 12 servos = 12 control wires |
| No feedback path | No way to read back actual position |
| No error reporting | A stalled or jammed servo is invisible to the controller |
| No configuration | Speed, acceleration limits cannot be set over the wire |

### Servo Bus

A **servo bus** replaces the individual per-servo wires with a single shared serial link. Every
servo on the bus has a **servo ID** — a unique integer address. The controller sends a packet
that includes the target servo's ID; only that servo responds or acts on the packet.

```
Controller ─── single bus wire ─── Servo ID 1
                                └── Servo ID 2
                                └── Servo ID 3
                                └── ...
```

Benefits over PWM:

- **One wire for all servos.** Wiring complexity is dramatically reduced.
- **Bidirectional.** The controller can read servo state (position, speed, temperature, load, voltage).
- **Configurable.** Operating mode, limits, and PID parameters can be written over the bus.
- **Addressable.** Each servo has an ID; multiple servos on one wire do not interfere as long as
  the protocol is respected.

The Waveshare UGV Rover uses **Feetech ST3215 bus servos**, which communicate using the
**SCServo (SC-series Serial Servo) protocol** over a half-duplex TTL UART at 1 Mbps.

---

## 5. How the SCServo Protocol Works

The ST3215 uses a binary packet protocol (not human-readable text). Every packet has the
following structure:

| Byte position | Content |
|---|---|
| 0 | `0xFF` (header) |
| 1 | `0xFF` (header) |
| 2 | Servo ID (1–253; 0xFE = broadcast) |
| 3 | Length (number of bytes that follow, not counting header/ID/length) |
| 4 | Instruction code (read, write, sync write, ping, etc.) |
| 5 ... N-1 | Parameters (register address, data, etc.) |
| N | Checksum |

The checksum is computed over the ID, length, instruction, and parameters, so any corruption
in transit can be detected.

### Write vs Read

**Write packet:** Controller sends a packet; the servo acts on it (e.g., move to a position).
No response packet is generated for broadcast writes. For single-servo writes, the servo may
send a short status response.

**Read packet:** Controller sends a packet specifying a register address and byte count.
The servo replies with a **response packet** containing the requested register values.

The `FeedBack()` function in the SCServo library performs a read to retrieve position, speed,
load, voltage, and temperature in a single transaction.

### Addressing and Contention

Because all servos share one wire, the bus must be quiet before a read response arrives.
The protocol design relies on:

1. Controller transmits a read request.
2. Controller switches to receive mode.
3. Target servo (and only that servo) transmits its response.
4. All other servos remain silent.

If two devices transmit simultaneously, their signals collide and both become unreadable. This
is **bus contention**. The SCServo library is designed for exactly one bus master (controller)
at a time.

---

## 6. Servo Encoders

### What an Encoder Is

An **encoder** is a sensor that measures the position, speed, or direction of a rotating shaft.

There are two primary types:

**Incremental encoder:** outputs pulses as the shaft rotates. Each pulse represents a small
increment of movement. The controller counts pulses to track relative position. The
fundamental limitation is that power loss or missed pulses lose the count — the absolute
position is unknown until the system re-homes.

**Absolute encoder:** outputs a unique code representing the shaft's current position regardless
of how it got there. No counting required; power can cycle without losing position. The
ST3215 uses an absolute encoder with 4096 steps per full revolution (12-bit resolution).

### Resolution and Conversion

With 4096 steps per 360°, one encoder step represents:

$$\frac{360°}{4096} \approx 0.088° \text{ per step}$$

The firmware converts the raw encoder value to degrees using:

$$\theta = \frac{(\text{raw} - 2047) \times 360}{4095}$$

This maps the midpoint of the encoder range (raw = 2047) to 0°, giving a working range of
approximately $-180°$ to $+180°$.

When `raw = 0` (the zero-initialised default in C++, meaning the read never succeeded):

$$\theta = \frac{(0 - 2047) \times 360}{4095} = -179.956\ldots°$$

This is exactly the stuck value `T=1001.pan = -179.9560394` observed in testing.

### How the ST3215 Encoder Is Read

The ST3215 stores its current encoder reading in internal registers. The SCServo library
reads them with a two-step sequence:

```cpp
int result = st.FeedBack(servo_id);   // sends a read packet; populates library cache
if (result != -1) {
    int16_t raw = st.ReadPos(-1);     // -1 = read from cache, not a new bus transaction
}
```

`FeedBack()` returns -1 if no valid response packet was received. `ReadPos(-1)` only returns
meaningful data if the preceding `FeedBack()` succeeded. If `FeedBack()` always fails,
`ReadPos(-1)` will return stale or zero data regardless of what the servo is physically doing.

---

## 7. The Full Signal Path in This Project

This section traces the complete chain from Raspberry Pi intent to servo encoder value, and
identifies where the chain breaks.

### Hardware Topology

```
Raspberry Pi
    │
    │  /dev/ttyAMA0  (115200 baud, full-duplex, JSON)
    │
ESP32
    │
    │  Serial2, GPIO 18/19  (1 Mbps, half-duplex, SCServo binary)
    │
Single bus wire
    ├── ST3215 pan servo   (ID 2)
    └── ST3215 tilt servo  (ID 1)
```

### The TX Echo Problem on a Two-Pin Half-Duplex Bus

The ESP32 servo bus uses **two separate GPIO pins** wired together onto one physical wire:

```cpp
#define S_RXD 18   // receive
#define S_TXD 19   // transmit
```

Because TX and RX are shorted together externally (both connect to the same servo bus wire),
**every byte the ESP32 transmits arrives back on the RX pin simultaneously.** This is called the
**TX echo**.

Under normal operation, the SCServo library is responsible for draining this echo before
listening for the servo's response. The intended sequence is:

```
1. rFlushSCS()  → clear any stale bytes already in the RX buffer
2. writeBuf()   → place the request packet in the TX FIFO (returns immediately)
3. wFlushSCS()  → wait for TX to finish; drain the echo from RX
4. checkHead()  → wait for 0xFF 0xFF from the servo's response
```

Step 3 is the critical flush. The `wFlushSCS()` function is supposed to:
- Block until the UART hardware has finished transmitting every byte from the TX FIFO.
- Then clear the RX buffer, which now contains the echo of the outgoing packet.

**In the Waveshare firmware, `wFlushSCS()` is an empty function — it does nothing.**

The result:

- `writeBuf()` queues the request packet and returns.
- `wFlushSCS()` does nothing.
- `checkHead()` immediately reads the RX buffer — which contains the echo of the outgoing
  request. The outgoing packet *begins with `0xFF 0xFF`*, so `checkHead()` finds what looks
  like a valid response header and tries to parse the rest of the outgoing request as if it
  were the servo's reply.
- The servo's actual reply arrives later, but nothing is reading at that point.
- `FeedBack()` returns -1 on every call.

### Why Writes Still Work

Write operations (e.g., `SyncWritePosEx()`) call `syncWrite()`, which only transmits. They
never attempt to read a response. The echo is therefore harmless — there is nothing in the
read path waiting for it. This is why commanding the servo to a position via `T=133` works
correctly while position readback never succeeds.

---

## 8. Summary of Terms

| Term | Definition in this project |
|---|---|
| **UART** | Hardware serial module; converts bytes to timed voltage pulses on a wire |
| **TX pin** | Transmit pin; drives data out |
| **RX pin** | Receive pin; listens for incoming data |
| **Baud rate** | Bit rate agreed in advance; Pi↔ESP32 = 115200, ESP32↔servos = 1 Mbps |
| **TX FIFO** | Hardware queue that holds bytes waiting to be shifted out onto the wire |
| **Full-duplex** | Two independent wires; both sides can transmit simultaneously |
| **Half-duplex** | One shared wire; only one side transmits at a time |
| **TX echo** | On a two-pin half-duplex bus, transmitted bytes loop back on the RX pin |
| **Servo bus** | Shared serial wire where servos are addressed by ID; replaces per-servo PWM wires |
| **SCServo protocol** | Feetech's binary packet protocol used by ST3215 servos |
| **Servo ID** | Unique integer address (1–253) assigned to each servo on the bus |
| **Encoder** | Sensor measuring shaft position |
| **Absolute encoder** | Reports current position without needing to home after power loss |
| **Incremental encoder** | Counts relative movement; loses position on power loss |
| **FeedBack()** | SCServo library function that reads position, speed, and diagnostics from a servo |
| **ReadPos(-1)** | SCServo library function that returns the position cached by the last FeedBack() call |
| **wFlushSCS()** | Intended to drain TX echo; is empty in the Waveshare firmware — the root cause of all read failures |
