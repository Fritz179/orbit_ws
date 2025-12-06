import spidev, time

LEDS = 94

spi = spidev.SpiDev(0, 0)       # /dev/spidev0.0
spi.max_speed_hz = 2400000      # 2.4 MHz  -> 1 SPI bit = 417 ns

SYM0 = 0b1000                   # WS ‘0’  → high 1, low 3
SYM1 = 0b1110                   # WS ‘1’  → high 3, low 1

def expand_byte(b):
    out = 0
    for i in range(8):
        out <<= 4
        out |= SYM1 if (b & 0x80) else SYM0
        b <<= 1
    return out.to_bytes(4, 'big')

def send(colors):
    buf = bytearray()
    for r, g, b in colors:             # WS order = G R B
        for ch in (g, r, b):
            buf += expand_byte(ch)
    spi.xfer3(buf)

def wheel(pos):
    if pos <  85: return pos*3, 255-pos*3, 0
    if pos < 170: pos -= 85; return 255-pos*3, 0, pos*3
    pos -= 170;   return 0, pos*3, 255-pos*3

phase = 0
while True:
    frame = [wheel((i*256//LEDS + phase) & 255) for i in range(LEDS)]
    send(frame)
    phase = (phase + 10) & 255
    time.sleep(0.02)