# Digital Readout (DRO) Firmware Design Document

**Project:** Industrial Digital Readout Controller
**MCU:** STM32C031K6T6
**Language:** Rust (`no_std`)
**Framework:** RTIC
**Communication:** Modbus RTU over RS485
**Version:** 1.2

---

# 1. Introduction

This document describes the firmware architecture for an industrial Digital Readout (DRO) controller based on the STM32C031K6T6.

The design emphasizes:

* Deterministic real-time execution
* Minimal interrupt usage
* Packet-oriented UART communication
* Modular software architecture
* High-speed encoder decoding
* Reliable non-volatile storage
* Simple maintainability

The firmware is designed to resemble a small PLC where almost all processing occurs in periodic RTIC tasks.

---

# 2. System Overview

The controller provides:

* High-speed incremental encoder interface
* Modbus RTU Slave
* RS485 communication
* TM1638 display and keypad
* Parameter storage
* Encoder position recovery after power failure
* Digital inputs and outputs

---

# 3. Hardware

## MCU

STM32C031K6T6

* Cortex-M0+
* 48 MHz
* UART
* DMA
* I²C
* GPIO
* EXTI

---

## Encoder

* Differential incremental encoder
* CHA
* CHB
* Line receiver (26LS32)

Current implementation

* Software quadrature decoding

Future hardware

* Timer Encoder Mode

---

## Communication

* SN75176 RS485 transceiver
* Half duplex
* Modbus RTU
* Default baud rate 9600

---

## Display

TM1638

* LED display
* Keypad

---

## EEPROM

24C02

* I²C interface
* Stores

  * Encoder count
  * Parameters
  * Calibration
  * Firmware data

---

## Power Monitoring

Dedicated Power Sense input.

Purpose

* Detect power failure
* Save encoder count
* Save modified parameters before power loss

---

# 4. Software Architecture

```text
                 RTIC Scheduler
                        │
        ┌───────────────┼────────────────┐
        │               │                │
   Encoder ISR     1 ms Control     20 ms Display
                        │
      ┌─────────────────┼────────────────┐
      │                 │                │
    UART            Modbus         Application
      │
      ▼
    RS485

          Power Fail Interrupt
                   │
                   ▼
            EEPROM Save Task
                   │
                   ▼
                 24C02
```

---

# 5. Design Philosophy

The firmware follows five principles.

1. Keep interrupts short.

2. Use DMA whenever possible.

3. Process complete packets instead of bytes.

4. Minimize shared resources.

5. Give every module one responsibility.

---

# 6. RTIC Task Structure

## Priority 4

Power Fail Interrupt

---

## Priority 3

Encoder EXTI ISR

---

## Priority 2

1 ms Control Task

---

## Priority 1

20 ms Display Task

---

## Priority 0

Heartbeat Task

---

# 7. Interrupt Strategy

Interrupts are reserved only for asynchronous hardware events.

Used

* Encoder EXTI
* Power Fail EXTI

Not used

* USART interrupts
* DMA interrupts

USART interrupt flags are polled periodically.

Advantages

* Deterministic execution
* Easier debugging
* Reduced interrupt nesting
* Predictable timing

---

# 8. Encoder Module

Responsibilities

* Decode quadrature
* Maintain encoder count

ISR sequence

```text
GPIO Snapshot

↓

LUT Decode

↓

Update Count

↓

Clear EXTI
```

Characteristics

* Constant execution time
* Single GPIO read
* LUT decoding
* No protocol processing

Performance target

Approximately 100,000 encoder interrupts/second.

---

# 9. UART Driver

UART is packet oriented.

Public API

```rust
pub fn init();

pub fn rx_packet() -> Option<&[u8]>;

pub fn tx_packet(data: &[u8]);

pub fn poll();
```

Responsibilities

* USART initialization
* DMA configuration
* RS485 Driver Enable control
* Packet reception
* Packet transmission

No Modbus knowledge.

---

# 10. UART Reception

DMA

Normal Mode

Operation

```text
Enable RX DMA

↓

Receive Bytes

↓

Receiver Timeout

↓

Disable RX DMA

↓

Determine Length

↓

Return Packet
```

Packet length

```text
Length =
Buffer Size - DMA Remaining Count
```

No copying.

The received packet remains in the RX buffer until the reply transmission completes.

---

# 11. UART Transmission

Operation

```text
Application

↓

Fill TX Buffer

↓

Start TX DMA

↓

DMA Transfers Data

↓

UART Shift Register

↓

Transmission Complete

↓

Restart RX DMA
```

Only UART TC indicates the final stop bit has left the transmitter.

DMA completion is ignored.

---

# 12. UART Polling

Executed every 1 ms.

Polls

* RTOF
* TC
* ORE
* FE
* NE

Pseudo code

```text
if RTOF

    Packet Complete

if TC

    TX Complete

if ORE

    Clear

if FE

    Clear

if NE

    Clear
```

---

# 13. DMA

RX

* Normal Mode

TX

* Normal Mode

Reasons

* Packet communication
* Simpler software
* No circular buffer
* No byte interrupts

---

# 14. Buffers

Separate DMA buffers.

```text
RX Buffer

256 bytes

TX Buffer

256 bytes
```

Advantages

* Request preserved
* Reply generated independently
* No copying

---

# 15. Modbus Layer

Responsibilities

* CRC
* Address verification
* Function decoding
* Register access
* Exception responses
* Reply generation

Flow

```text
UART Packet

↓

Modbus Parser

↓

Application

↓

Reply

↓

UART Driver
```

The Modbus layer never accesses UART registers.

---

# 16. TM1638 Module

Responsibilities

* Display refresh
* Key scan

Runs every 20 ms.

No interrupts.

---

# 17. EEPROM Driver

Responsibilities

* Read EEPROM
* Write EEPROM

Public API

```rust
pub fn init();

pub fn read(addr, buffer);

pub fn write(addr, data);
```

Contains no application logic.

---

# 18. EEPROM Layout

| Address | Description   |
| ------- | ------------- |
| 0x0000  | Signature     |
| 0x0002  | Version       |
| 0x0004  | Encoder Count |
| 0x0008  | Parameters    |
| 0x0040  | Calibration   |
| 0x0080  | Reserved      |

---

# 19. Parameter Manager

Responsibilities

* Load parameters
* Validate EEPROM
* Restore defaults
* Maintain RAM copy

Architecture

```text
Modbus

↓

Parameter Manager

↓

EEPROM Driver
```

Only the Parameter Manager accesses stored parameters.

---

# 20. Power Failure Handling

A dedicated Power Sense input detects supply failure.

Normal operation

```text
Power Good

↓

No EEPROM Writes
```

Power failure

```text
Power Fail Interrupt

↓

Set Power Fail Flag

↓

Schedule EEPROM Save Task
```

The interrupt remains extremely short.

---

# 21. EEPROM Save Task

Runs immediately after power failure is detected.

Responsibilities

* Save encoder count
* Save modified parameters

Operation

```text
Power Fail

↓

Write Encoder Count

↓

If Parameters Dirty

↓

Write Parameters

↓

Wait For Power Loss
```

No periodic EEPROM writes are performed.

---

# 22. EEPROM Wear Strategy

Encoder Count

Written

* Only during power failure

Parameters

Written

* When modified and power failure occurs

Advantages

* Extremely low EEPROM wear
* Last position restored after power-up
* No unnecessary writes

---

# 23. Shared Resources

Shared

```text
Encoder State

System Parameters
```

Local

```text
UART

Modbus

TM1638

EEPROM Driver

GPIO

Application State
```

This minimizes synchronization.

---

# 24. Application Flow

```text
1 ms Task

↓

Poll UART

↓

Packet Received?

↓

Process Modbus

↓

Generate Reply

↓

Start TX DMA

↓

Check TX Complete

↓

Restart RX DMA

↓

Read Encoder

↓

Update Outputs
```

---

# 25. Startup Sequence

```text
Power On

↓

Clock Initialization

↓

GPIO Initialization

↓

Power Sense Initialization

↓

EEPROM Initialization

↓

Validate EEPROM

↓

Load Parameters

↓

Restore Encoder Count

↓

Initialize UART

↓

Initialize TM1638

↓

Start RTIC Scheduler
```

---

# 26. Timing

UART

9600 baud

Character time

≈1.15 ms

Frame timeout

≈4 ms

Control task period

1 ms

Worst-case UART event latency

<1 ms

---

# 27. Performance Goals

Encoder

* ~100,000 encoder interrupts/second

UART

* DMA driven
* No byte interrupts

Modbus

* Packet oriented
* Deterministic timing

Display

* 20 ms refresh

EEPROM

* Writes only on controlled power-down

---

# 28. Future Improvements

* Hardware Timer Encoder Mode
* Higher baud rates
* Bootloader
* Watchdog supervision
* Event logging
* Firmware updates
* Additional communication interfaces

---

# 29. Design Principles

* Deterministic execution
* Short interrupt handlers
* DMA for data movement
* Packet-oriented communication
* Single ownership of peripherals
* Minimal shared resources
* Deferred non-time-critical processing
* Modular software components
* Hardware abstraction through drivers

---

# 30. Conclusion

The firmware is organized around a deterministic RTIC architecture in which only genuinely asynchronous hardware events generate interrupts. Encoder quadrature decoding and power-fail detection are interrupt driven, while all communication, protocol processing, display updates, and application logic execute in periodic RTIC tasks.

UART communication is fully packet oriented using DMA in Normal mode. Receiver Timeout (RTOF) signals the end of a Modbus RTU request, while Transmission Complete (TC) confirms that the final stop bit has been transmitted before the RS485 receiver is re-enabled.

The power-fail mechanism preserves the encoder position and modified system parameters by writing them to EEPROM only when power loss is detected. This approach minimizes EEPROM wear while ensuring reliable recovery after restart.

The resulting firmware architecture is simple, deterministic, modular, and well suited for industrial embedded applications. It also provides a clear migration path to hardware timer-based encoder decoding in future hardware revisions without requiring major changes to the application software.

If you'd like, I can also convert this into a more formal **software architecture specification (SRS/SDD)** following IEEE-style documentation, with UML component diagrams, state machines, module interfaces, and timing diagrams. That format is more suitable for long-term maintenance and future hardware revisions.
