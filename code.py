# SPDX-FileCopyrightText: 2023 John Park for Adafruit Industries
# SPDX-License-Identifier: MIT

#  Grand Central MIDI Knobs
#  for USB MIDI and optional UART MIDI
#  Reads analog inputs, sends out MIDI CC values
#   with Kattni Rembor and Jan Goolsbey for range and hysteresis code

import time
import board
import busio
import rotaryio
from simpleio import map_range
from analogio import AnalogIn
import digitalio
import usb_midi
import adafruit_midi  # MIDI protocol encoder/decoder library
from adafruit_midi.control_change import ControlChange
from adafruit_midi.note_off import NoteOff
from adafruit_midi.note_on import NoteOn
from adafruit_debouncer import Debouncer
from adafruit_ssd1306 import SSD1306_I2C

from rainbowio import colorwheel
import neopixel

pixel_pin = board.GP23
num_pixels = 1
neo = neopixel.NeoPixel(pixel_pin, num_pixels, brightness=0.1, auto_write=False)


RED = (255, 0, 0)
YELLOW = (255, 150, 0)
GREEN = (0, 255, 0)
CYAN = (0, 255, 255)
BLUE = (0, 0, 255)
PURPLE = (180, 0, 255)
ORANGE = (255, 200, 0)
NIL = (0, 0, 0)

neo.fill(RED)
neo.show()

i2c = busio.I2C(scl=board.GP17, sda=board.GP16)
oled = SSD1306_I2C(128, 32, i2c) 
oled.fill(0)
oled.text('MEEDEE', 48, 0, 1)
oled.text('WEEEEE', 48, 20, 1)
oled.show()

# pick your USB MIDI out channel here, 1-16
MIDI_USB_channel = 1

midi_usb = adafruit_midi.MIDI(
    midi_out=usb_midi.ports[1],
    out_channel=MIDI_USB_channel - 1
)

max_channels =4

midi_channels = [];

led = digitalio.DigitalInOut(board.GP25)  # activity indicator
led.direction = digitalio.Direction.OUTPUT

#  range_index converts an analog value (ctl) to an indexed integer
#  Input is masked to 8 bits to reduce noise then a scaled hysteresis offset
#  is applied. The helper returns new index value (idx) and input
#  hysteresis offset (offset) based on the number of control slices (ctrl_max).
def range_index(ctl, ctrl_max, old_idx, offset):
    if (ctl + offset > 65535) or (ctl + offset < 0):
        offset = 0
    idx = int(map_range((ctl + offset) & 0xFF00, 1200, 65500, 0, ctrl_max))
    if idx != old_idx:  # if index changed, adjust hysteresis offset
        # offset is 25% of the control slice (65536/ctrl_max)
        offset = int(
            0.25 * sign(idx - old_idx) * (65535 / ctrl_max)
        )  # edit 0.25 to adjust slices
    return idx, offset


def sign(x):  # determine the sign of x
    if x >= 0:
        return 1
    else:
        return -1

class MyButton:
    pin = None,
    button = None
    note = 0
    def __init__(self, pin, note):
        self.pin = pin
        self.note = note
        btn = digitalio.DigitalInOut(pin)
        btn.direction = digitalio.Direction.INPUT
        btn.pull = digitalio.Pull.UP
        self.button = Debouncer(btn)

MyButtons = [
    MyButton(
        board.GP1,
        61,
    ),
    MyButton(
        board.GP2,
        62,
    ),
    MyButton(
        board.GP3,
        63,
    ),
    MyButton(
        board.GP4,
        64,
    ),
    MyButton(
        board.GP5,
        65,
    ),
    MyButton(
        board.GP6,
        66,
    ),
    MyButton(
        board.GP18,
        69,
    )
]

class MyREncoder:
    pinA = None
    pinB = None
    last_position = 0
    encoder = None
    def __init__(self, pinA, pinB):
        self.pinA = pinA
        self.pinB = pinB
        self.encoder = rotaryio.IncrementalEncoder(self.pinA, self.pinB)

MyREncoders = [
    MyREncoder(
        board.GP19,
        board.GP20
    )
];

class MyKnob:
    adc = None
    cc_number = 0
    cc_range = (0, 127)
    cc_value = (0, 0)
    cc_value_last = (0, 0)
    channel = 0
    def __init__(self, pin, cc_number, cc_range, channel):
        self.pin = pin
        self.cc_number = cc_number
        self.cc_range = cc_range
        self.adc = AnalogIn( pin )
        self.channel = channel

MyKnobs = [
    MyKnob(
        board.GP26,
        7,
        (0, 127),
        0
    ),
    MyKnob(
        board.GP27,
        7,
        (0, 127),
        1
    ),
    MyKnob(
        board.GP28,
        7,
        (0, 127),
        2
    )
]

def knob_neo( value ):
    if not value:
        return (0,0,0)
    red = 0
    green = 255
    if value <= 63:
        red = ( 256 / 64 ) * value
        green = 255
    elif value > 63:
        red = 255
        green = 255 - (( 256 / 64 ) * (value-64))
    print((red, green, 0));
    return (red, green, 0)



while True:
    for btn in MyButtons:
        btn.button.update()
        
        #if button.value:
        #    print("not pressed")
        #else:
        #    print("pressed")
        if btn.button.fell:
            midi_usb.send( NoteOn( btn.note, 127 ) )
            print(' pressed {0}'.format(btn.note))
        if btn.button.rose:
            midi_usb.send( NoteOff( btn.note, 0 ) )
            print('released {0}'.format(btn.note))
    for rencoder in MyREncoders:
        current_position = rencoder.encoder.position
        position_change = current_position - rencoder.last_position
        if position_change > 0:
            for _ in range(position_change):
                print(current_position)
            
        elif position_change < 0:
            for _ in range(-position_change):
                print(current_position)
        rencoder.last_position = current_position
    for knob in MyKnobs:
        knob.cc_value = range_index(
            knob.adc.value,
            ( knob.cc_range[1] - knob.cc_range[0] + 1 ),
            knob.cc_value[0],
            knob.cc_value[1],
        )
        if knob.cc_value != knob.cc_value_last:
            midi_usb.send(
                ControlChange(
                    knob.cc_number,
                    knob.cc_value[0] + knob.cc_range[0]
                ),
                knob.channel
            )
            knob.cc_value_last = knob.cc_value
            print("knob {0}".format(knob.channel))
            neo.fill( knob_neo( knob.cc_value[0] ) )
            neo.show()
            led.value = True
    time.sleep(0.01)
    led.value = False
