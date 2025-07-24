# Smart Environmental Node on STM32F401RE

![Block diagram](System Block Diagram.png) 

A showcase project that demonstrates **GPIO, I²C, UART, ADC, PWM, timers, DMA, interrupts and (optionally) FreeRTOS** on an STM32F401RE Nucleo board.  
It continually reads temperature & pressure from a Bosch **BMP280** sensor, streams data to a PC, and provides LED / CLI feedback.  
Designed as an interview‑ready example with clear architecture, diagrams, and reproducible build steps.

---

## Quick Start

```bash
# Clone
git clone https://github.com/nilooeli/stm32f401re-envnode.git
cd stm32f401re-envnode

# Open firmware in STM32CubeIDE v1.15+
#   File ► Open Projects from File System… ► select fw/
# Build (Debug) and flash to Nucleo‑F401RE
# Connect a serial terminal @ 115200‑8‑N‑1
