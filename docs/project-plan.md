Smart Environmental Node on STM32F401RE

Goal: Build a polished, interview‑ready embedded systems demo on the Nucleo‑F401RE board that exercises a broad range of MCU peripherals and good engineering practices (architecture, documentation, version control, testing, demo script). Use the BMP280 environmental sensor (temp + pressure) plus a small set of low‑cost components (LEDs, pushbuttons, potentiometer, maybe buzzer or small fan) to create a Smart Environmental Node you can extend with data logging, alerts, and optional Edge/ML hooks.

1. Elevator Pitch (Use in Interviews)

“I designed and implemented a modular embedded system on an STM32F401RE Nucleo board that reads temperature and pressure from a BMP280 over I²C, logs and streams data over UART/USB, shows live status via LEDs/PWM, and demonstrates interrupts, DMA, timers, ADC‑based user input, low‑power modes, and an RTOS option. I produced SysML‑style diagrams, hardware wiring drawings, and a reproducible GitHub repo with build instructions, test logs, and demo videos.”

Keep this ~30 sec. Be ready to expand to 2 min using the topics below.

2. High‑Level Feature Coverage Map

Feature Area

Implemented Element

Why It Matters in Interviews

Project Module

Status

Digital I/O (GPIO)

LEDs, pushbutton

Basic embedded bring‑up, debugging

drivers/gpio

Planned

I²C

BMP280 sensor

Real sensor interface, register read/write, calibration

drivers/bmp280

Planned

UART (Virtual COM)

Data log to PC terminal

Debug + host tools + scripting

drivers/uart_log

Planned

Timers

Periodic sampling, PWM LED dim

Scheduling & signal generation

core/timers

Planned

PWM

LED brightness proportional to temp

Demonstrates timer compare + scaling

app/led_feedback

Planned

ADC

Potentiometer = sample rate or alert threshold

Mixed‑signal integration

drivers/adc_pot

Planned

EXTI Interrupts

Pushbutton triggers forced sample / mode change

Interrupt‑driven design

core/interrupts

Planned

DMA

Burst UART logging; optional I²C+DMA

Performance, CPU offload

core/dma

Optional+

Low‑Power

Sleep between samples; wake on timer/EXTI

Power management awareness

core/power

Stretch

RTOS (FreeRTOS)

Sensor task, UI task, comms task

Real‑world multitasking

rtos/

Stretch

Error Handling

Sensor fault detect, retry, error LED

Robustness story

core/error

Planned

Configuration Persistence

Threshold store in flash/EEPROM emu

Non‑volatile settings

services/config_store

Stretch

Legend: Planned = baseline MVP; Optional+ = nice‑to‑have; Stretch = add if time allows.

3. Incremental Build Roadmap (Sprints)

Each Sprint should end with code in Git, tagged, and a README update. Commit early, commit often.

Sprint 0 – Environment Setup & Repo Skeleton

Install STM32CubeIDE + STM32CubeMX.

Confirm board connects, debug works (LED blink).

Create GitHub repo: /hw, /fw, /docs, /scripts, /tests.

Add README skeleton + build instructions.

Sprint 1 – GPIO Bring‑Up

Blink on‑board LED (LD2) at 1 Hz.

Read user button (on Nucleo) debounced in software.

Add simple CLI over UART to toggle LED.

Sprint 2 – I²C & BMP280 Driver (Polled)

Wire BMP280 to Nucleo I²C pins (see §6 Hardware Wiring).

Read chip ID; implement calibration read; raw → compensated temp/pressure.

Print readings every 1 s.

Sprint 3 – UART Logging & Simple Protocol

Wrap printf retarget or ring‑buffer transmit.

Add CSV output: timestamp_ms,temp_c,press_pa.

Add human‑readable command menu from host terminal.

Sprint 4 – Timers & Periodic Scheduler

Use hardware timer interrupt to set sample flag at configurable rate.

Main loop consumes flag; decouple sensor timing from HAL_Delay.

Sprint 5 – PWM LED Feedback

Map temperature to LED brightness (cool=dim, warm=bright).

Use timer PWM channel; update duty cycle when new sample arrives.

Sprint 6 – ADC Potentiometer Input

Add pot to analog pin; scale 0–3.3 V to 0–100 %.

Use pot to control sampling interval OR temperature alert threshold.

Sprint 7 – Interrupts (EXTI) for Pushbutton

Short press: force immediate sensor sample.

Long press: cycle operating modes (stream, threshold alert, low‑power).

Sprint 8 – DMA Enhancements

UART TX via DMA to reduce CPU load during burst logging.

Optional: I²C+DMA (depends on HAL support & complexity comfort).

Sprint 9 – Low‑Power Mode Demo

Sleep MCU between samples; wake by timer or pushbutton.

Measure current (if you have a USB power meter).

Sprint 10 – RTOS Refactor (Stretch)

FreeRTOS w/ 3 tasks: Sensor, UI, Comms.

Queues for messages; event flags for alerts.

Sprint 11 – Documentation & Diagrams

System context, HW block, wiring, SW architecture, state machine.

Include short design rationale bullets.

Sprint 12 – Interview Packaging

Clean README w/ quick‑start.

Screenshots, short demo video (phone camera OK).

Talking points cheat sheet.

4. Minimal Viable Demo (MVP)

If time is short, target these by default:

BMP280 readout printed over serial.

LED brightness tracks temperature via PWM.

Pushbutton triggers sample.

Pot sets sample rate.

Error blink pattern if sensor missing. This covers GPIO, I²C, UART, PWM, ADC, interrupts – already impressive.

5. Stretch Idea: “Comfort Monitor” Scenario

Add a “Comfort Index” computed from temperature + pressure trend; alert when outside comfort range (LED flashes, buzzer chirp, serial warning). Good story: environment chambers, avionics testing lab (links to your background).

6. Hardware Planning

6.1 Core Parts You Already Have

Nucleo‑F401RE board.

BMP280 breakout (assume 3.3 V‑compatible).

Breadboard + jumpers.

LEDs + resistors (220 Ω–1 k Ω typical).

Pushbuttons (momentary, normally open).

