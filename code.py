# SPDX-FileCopyrightText: 2023 John Park for Adafruit Industries
# SPDX-License-Identifier: MIT

#  Grand Central MIDI Knobs
#  for USB MIDI and optional UART MIDI
#  Reads analog inputs, sends out MIDI CC values
#   with Kattni Rembor and Jan Goolsbey for range and hysteresis code
VERSION="0.1.0"
print("Starting v" + VERSION)
print("Loading Modules..." , end = " " )
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
import neopixel
print("ok")

print("Loading Program")

print("neopixel init...", end = " " )
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
print("ok")

print("i2c init...", end = " " )
i2c = busio.I2C(scl=board.GP17, sda=board.GP16)
print("ok")
print("oled init...", end = " " )
oled = SSD1306_I2C(128, 32, i2c)
oled.fill(0)
oled.text("MEEDEE", 48, 0, 1)
oled.text("WEEEEE", 48, 10, 1)
oled.text( VERSION, 48, 20, 1)
oled.show()
print("ok")

print("midi_usb init...", end = " " )
# pick your USB MIDI out channel here, 1-16
MIDI_USB_channel = 1
midi_usb = adafruit_midi.MIDI(
    midi_out=usb_midi.ports[1],
    out_channel=MIDI_USB_channel - 1
)
print("ok")

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
    return 1 if x >= 0 else 0
        
def knob_neo( value ):
    if not value:
        return (0,0,0)
    red = 0
    green = 255
    blue = 0
    if value <= 63:
        red = ( 256 / 64 ) * value
        green = 255
    elif value > 63:
        red = 255
        green = 255 - (( 256 / 64 ) * (value-64))
    # print((red, green, blue));
    return (red, green, blue)

def update_neo( newValue ):
    neo.fill( newValue )
    neo.show()

def update_display(rows):
    oled.fill(0)
    row_count = 0
    for row in rows:
        oled.text(row, 0, row_count, 1)
        row_count += 10
    oled.show()

class MyButton:
    name = "Button"
    pin = None,
    button = None
    note = 0
    value = 0
    def __init__(self, name, pin, note, value = 106):
        self.name = name
        self.pin = pin
        self.note = note
        self.value = value
        btn = digitalio.DigitalInOut(pin)
        btn.direction = digitalio.Direction.INPUT
        btn.pull = digitalio.Pull.UP
        self.button = Debouncer(btn)

print("MyButtons init...", end = " " )
MyButtons = [
    MyButton(
        "Button 61",
        board.GP1,
        61,
        None
    ),
    MyButton(
        "Button 62",
        board.GP2,
        62,
        None
    ),
    MyButton(
        "Button 63",
        board.GP3,
        63,
        None
    ),
    MyButton(
        "Button 64",
        board.GP4,
        64,
        None
    ),
    MyButton(
        "Button 65",
        board.GP5,
        65,
        None
    ),
    MyButton(
        "Button 66",
        board.GP6,
        66,
        None
    ),
    MyButton(
        "Button 69",
        board.GP18,
        69,
        None
    )
]
print("ok")

class MyREncoder:
    name = "Encoder"
    pinA = None
    pinB = None
    last_position = 0
    encoder = None
    value = 0
    value_min = 0
    value_max = 127
    step_size = 1
    def __init__(self, name, pinA, pinB, value_min, value_max, value = 106, step_size = 1):
        self.name = name
        self.pinA = pinA
        self.pinB = pinB
        self.value_min = value_min
        self.value_max = value_max
        self.step_size = step_size
        self.value = int( self.value_min + ( ( self.value_max - self.value_min) / 2 ) )
        self.encoder = rotaryio.IncrementalEncoder(self.pinA, self.pinB, 4)
        
    def setValue(self, value):
        if value <= self.value_min:
            self.value = self.value_min
        elif value >= self.value_max:
            self.value = self.value_max
        else:
            self.value = value
        return self.value
        
    def getValue(self):
        return self.value
        
    def plus(self):
        if self.value < self.value_max:
            self.setValue( self.value + 1 * self.step_size )
        return self.value
        
    def minus(self):
        if self.value > self.value_min:
            self.setValue( self.value - 1 * self.step_size )
        return self.value

print("MyREncoders init...", end = " " )
MyREncoders = [
    MyREncoder(
        "Encoder",
        board.GP19,
        board.GP20,
        0,
        127,
        106,
        1
    )
];

class MyKnob:
    name = "Knobby"
    adc = None
    cc_number = 0
    cc_range = (0, 127)
    cc_value = (0, 0)
    cc_value_last = (0, 0)
    channel = 0
    def __init__(self, name, pin, cc_number, cc_range, channel):
        self.name = name
        self.pin = pin
        self.cc_number = cc_number
        self.cc_range = cc_range
        self.adc = AnalogIn( pin )
        self.channel = channel
print("ok")

print("MyKnobs init...", end = " " )
MyKnobs = [
    MyKnob(
        "Knobby ch1cc7",
        board.GP26,
        7,
        (0, 127),
        0
    ),
    MyKnob(
        "Knobby ch2cc7",
        board.GP27,
        7,
        (0, 127),
        1
    ),
    MyKnob(
        "Knobby ch3cc7",
        board.GP28,
        7,
        (0, 127),
        2
    )
]
print("ok")

print("Starting Main Loop")
while True:
    for btn in MyButtons:
        btn.button.update()
        
        #if button.value:
        #    print("not pressed")
        #else:
        #    print("pressed")
        if btn.button.fell:
            midi_usb.send( NoteOn( btn.note, 127 ) )
            # print('pressed {0}'.format(btn.note))
            update_display([
                btn.name,
                "value: pressed"
            ])
        if btn.button.rose:
            midi_usb.send( NoteOff( btn.note, 0 ) )
            # print('released {0}'.format(btn.note))
            update_display([
                btn.name,
                "value: released"
            ])

    for rencoder in MyREncoders:
        current_position = rencoder.encoder.position
        print(rencoder.encoder.position)
        position_change = current_position - rencoder.last_position
        if position_change > 0:
            for _ in range(position_change):
                rencoder.plus()
                #print(rencoder.value)
        elif position_change < 0:
            for _ in range(-position_change):
                rencoder.minus()
                #print(rencoder.value)
        if position_change != 0:
            midi_usb.send(
                ControlChange(
                    7,
                    rencoder.value
                ),
                15
            )
            update_display([
                rencoder.name,
                "value: " + str(rencoder.value)
            ])
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
            #print("knob {0}".format(knob.channel))
            led.value = True
            update_neo( knob_neo( knob.cc_value[0] ) )
            update_display([
                knob.name,
                "value: " + str(knob.cc_value[0])
            ])

    time.sleep(0.01)
    led.value = False
