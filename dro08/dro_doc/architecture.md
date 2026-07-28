# STM32C031 Digital Readout (DRO)
## Firmware Architecture Design

Version: 1.0

---

# 1. Design Goals

The firmware is designed around the following principles:

- Deterministic execution
- Minimal interrupt usage
- No interrupt-driven application logic
- No interrupt-driven UART
- Polling-driven DMA/UART communication
- Clear separation between hardware drivers and application logic
- RTIC used only as a scheduler and resource manager
- Entire application implemented as independent modules

---

# 2. Hardware

MCU

STM32C031K6T6

Clock

48 MHz HSI

Display

TM1638

Communication

RS485 Modbus RTU

Encoder

Incremental Quadrature Encoder

EEPROM

I²C EEPROM

Power Fail

Dedicated Power Sense input

---

# 3. System Overview

```
                 +----------------+
                 |   TM1638 Task  |
                 +-------+--------+
                         |
        display Option<[u8;16]>
                         |
                  key_event Option<KeyEvent>
                         |
                         v
                 +----------------+
                 |    FSM Task    |
                 +-------+--------+
                         |
        -----------------+-------------------
        |        |        |       |          |
    encoder   parameters  relay display state
        |
        |
        +----------------------+
                               |
                               v
                       Modbus Algorithm
                               |
                         UART Driver
                               |
                            DMA/UART

Power Fail Interrupt
        |
        v
 EEPROM Save (Count + Parameters)

Encoder Interrupt
        |
        v
Encoder Count
```

---

# 4. RTIC Tasks

Only four RTIC tasks exist.

## 4.1 Encoder Interrupt

Priority

Highest

Trigger

EXTI0_1

Responsibilities

- Read GPIO
- Decode quadrature
- Update encoder_count

Execution

Very short.

Contains no application logic.

---

## 4.2 TM1638 Task

Periodic

100 ms

Responsibilities

- Read keyboard
- Execute keyboard algorithm
- Detect long presses
- Write display RAM
- Publish key events

Shared Resources

```
display_buffer : Option<[u8;16]>

key_event : Option<KeyEvent>
```

TM1638 owns the hardware.

It does **not** know application states.

It only performs display refresh and keyboard scanning.

---

## 4.3 FSM Task

Periodic

10–20 ms

Responsibilities

Entire user interface.

- Menu navigation
- Edit mode
- Parameter editing
- Factory mode
- Zero
- Presets
- Display generation

Consumes

```
key_event
```

Produces

```
display_buffer
```

Accesses

- encoder_count
- parameters
- EEPROM dirty flag

---

## 4.4 UART Poll Task

Periodic

1 ms

Responsibilities

Poll

- DMA flags
- USART flags
- IDLE
- RTOR
- TC
- TXE

No interrupt used.

Updates UART driver state.

Calls

```
modbus.poll()
```

---

# 5. Algorithms

Algorithms are not RTIC tasks.

---

## Keyboard Algorithm

Called from

TM1638 Task

Produces

```
Option<KeyEvent>
```

Detects

- Press
- Release
- Auto-repeat
- Long press
- 3 second press

---

## Modbus Algorithm

Called from

UART Poll Task

Responsibilities

- Frame detection
- CRC
- Address checking
- Function decoding
- Register access
- Response generation

Independent from FSM.

Reads

- Parameters
- Encoder count

Writes

- Parameters

Requests EEPROM update when parameters change.

---

## Display Algorithm

Implemented by FSM.

Produces

```
Option<[u8;16]>
```

TM1638 simply transfers this buffer to display RAM.

---

# 6. Shared Variables

## Encoder

```
encoder_count : i32
```

Producer

Encoder interrupt

Consumers

- FSM
- Modbus

---

## Display

```
display_buffer : Option<[u8;16]>
```

Producer

FSM

Consumer

TM1638

Using `Option` avoids unnecessary SPI/bit-banged updates when the display has not changed.

---

## Keyboard

```
key_event : Option<KeyEvent>
```

Producer

TM1638

Consumer

FSM

---

## Parameters

Application configuration.

Examples

- Modbus address
- Scale
- Direction
- Decimal position
- Presets

Consumers

FSM

Modbus

Power Fail

---

## EEPROM Dirty Flag

```
parameter_dirty : bool
```

Set by

FSM

Modbus

Cleared by

Power Fail save routine.

---

# 7. Power Fail

Trigger

Power Sense interrupt (or NMI if hardware supports it appropriately).

Responsibilities

Save to EEPROM

- Encoder count
- All modified parameters

EEPROM write occurs exactly once during power loss.

No periodic EEPROM writes.

---

# 8. Interrupt Usage

Only two interrupts exist.

## Encoder

```
EXTI0_1
```

Purpose

Quadrature decoding

---

## Power Fail

```
Power Sense Input
```

Purpose

Store EEPROM

No UART interrupts.

No DMA interrupts.

No timer interrupts other than SysTick for RTIC scheduling.

---

# 9. UART Design

Interrupts

Disabled

DMA

Polled

Every 1 ms

UART driver responsibilities

- DMA synchronization
- Receive buffer
- Transmit buffer
- RTOR polling
- IDLE detection
- Error handling

Application never accesses hardware directly.

---

# 10. Driver Responsibilities

## Encoder Driver

- GPIO sampling
- LUT decode

Nothing else.

---

## TM1638 Driver

- Read keys
- Write display RAM

No UI logic.

---

## UART Driver

- DMA management
- USART polling

No Modbus logic.

---

## EEPROM Driver

- Read parameters
- Write parameters
- Read encoder count
- Write encoder count

No policy.

---

# 11. Layering

```
Application
│
├── FSM
├── Keyboard Algorithm
├── Modbus Algorithm
│
Drivers
│
├── UART
├── TM1638
├── Encoder
├── EEPROM
│
RTIC Scheduler
│
STM32 Hardware
```

---

# 12. Characteristics

- Deterministic execution
- Extremely low interrupt load
- No interrupt-driven communication
- Drivers independent of application
- FSM independent of communication
- Modbus independent of UI
- Power-fail safe state preservation
- Easily testable modules
- Clear separation of responsibilities
- Suitable for long-term maintenance and future feature expansion
