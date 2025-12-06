#!/usr/bin/env python3
import spidev, time, collections

LEDS = 82

# ── SPI setup ──────────────────────────────────────────────
spi = spidev.SpiDev(0, 0)         # /dev/spidev0.0
spi.max_speed_hz = 2_400_000      # 2.4 MHz  (4‑bit symbols)

SYM0 = 0b1000                     # WS‑bit 0 → 1000
SYM1 = 0b1110                     # WS‑bit 1 → 1110

def expand_byte(b: int) -> bytes:
    out = 0
    for _ in range(8):
        out = (out << 4) | (SYM1 if b & 0x80 else SYM0)
        b <<= 1
    return out.to_bytes(4, "big")

def send(colors):
    buf = bytearray()
    for r, g, b in colors:            # WS sends G‑R‑B
        for ch in (g, r, b):
            buf += expand_byte(ch)
    spi.xfer3(buf)

# ── rainbow palette ────────────────────────────────────────
def wheel(pos):
    if pos <  85: return pos*3, 255-pos*3, 0
    if pos < 170: pos -= 85; return 255-pos*3, 0, pos*3
    pos -= 170;   return 0, pos*3, 255-pos*3

# def wheel(pos):
#     if pos <  85: return 0, 0, 0
#     if pos < 170: pos -= 85; return 255, 0, 0
#     pos -= 170;   return 255, 255, 0

frame = collections.deque(
    wheel(i * 256 // LEDS) for i in range(LEDS)
)

# ── main loop: rotate pattern ──────────────────────────────
ROTATE_RIGHT = True   # set False for left rotation
DELAY = 0.02          # seconds

while True:
    send(frame)
    frame.rotate(1 if ROTATE_RIGHT else -1)
    time.sleep(DELAY)