# SPDX-FileCopyrightText: 2023 John Park for Adafruit Industries
# SPDX-License-Identifier: MIT

#  Grand Central MIDI Knobs
#  for USB MIDI and optional UART MIDI
#  Reads analog inputs, sends out MIDI CC values
#   with Kattni Rembor and Jan Goolsbey for range and hysteresis code

import time
import board
import busio
from simpleio import map_range
from analogio import AnalogIn
import digitalio
import usb_midi
import adafruit_midi  # MIDI protocol encoder/decoder library
from adafruit_midi.control_change import ControlChange
from adafruit_midi.note_off import NoteOff
from adafruit_midi.note_on import NoteOn

from adafruit_debouncer import Debouncer

# pick your USB MIDI out channel here, 1-16
MIDI_USB_channel = 1
# pick your classic MIDI channel for sending over UART serial TX/RX
# CLASSIC_MIDI_channel = 2

midi_usb = adafruit_midi.MIDI(
    midi_out=usb_midi.ports[1],
    out_channel=MIDI_USB_channel - 1
)
#  use DIN-5 or TRS MIDI jack on TX/RX for classic MIDI
# midi_uart = busio.UART(board.TX, board.RX, baudrate=31250, timeout=0.001)  # initialize UART
# classic_midi = adafruit_midi.MIDI(
#     midi_out=midi_uart, midi_in=midi_uart, out_channel=CLASSIC_MIDI_channel - 1, debug=False
# )

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
        board.GP21,
        60,
    )
]

class MyKnob:
    adc = None
    cc_number = 0
    cc_range = (0, 127)
    cc_value = (0, 0)
    cc_value_last = (0, 0)
    def __init__(self, pin, cc_number, cc_range):
        self.pin = pin
        self.cc_number = cc_number
        self.cc_range = cc_range
        self.adc = AnalogIn( pin )

MyKnobs = [
    MyKnob(
        board.GP26,
        7,
        (0, 127)
    )
]

print("   USB MIDI channel: {}".format(MIDI_USB_channel))
# print("   TRS MIDI channel: {}".format(CLASSIC_MIDI_channel))
        
while True:
    for btn in MyButtons:
        btn.button.update()
        
        #if button.value:
        #    print("not pressed")
        #else:
        #    print("pressed")
        if btn.button.fell:
            midi_usb.send( NoteOn( btn.note, 127 ) )
            print('Just pressed')
        if btn.button.rose:
            midi_usb.send( NoteOff( btn.note, 0 ) )
            print('Just released')
    for knob in MyKnobs:
        knob.cc_value = range_index(
            knob.adc.value,
            ( knob.cc_range[1] - knob.cc_range[0] + 1 ),
            knob.cc_value[0],
            knob.cc_value[1],
        )
        print( knob.cc_value )
        if knob.cc_value != knob.cc_value_last:  # only send if it changed
            # Form a MIDI CC message and send it:
            midi_usb.send(ControlChange(knob.cc_number, knob.cc_value[0] + knob.cc_range[0]))
            # classic_midi.send(
            #     ControlChange(knob_cc_number[i], knob_cc_value[i][0] + knob_cc_range[i][0])
            # )
            knob.cc_value_last = knob.cc_value
            led.value = True
    time.sleep(0.1)
    led.value = False
