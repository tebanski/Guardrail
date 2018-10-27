# add path to LARGA-specific Micropython modules
import sys
sys.path.append("/flash/lib")
sys.path.append("/flash/firmware")


# configure SD SPI interface for mounting
import os
os.sdconfig(mode=os.SDMODE_SPI, clk=18, mosi=23, miso=19, cs=4)


# configure and initialize the LCD display
from display import TFT
lcd = TFT()
lcd.init(lcd.M5STACK, width=240, height=320, rst_pin=33, backl_pin=32,
         miso=19, mosi=23, clk=18, cs=14, dc=27, bgr=True, backl_on=1)
lcd.clear()


# reset Neopixel LED bar
from machine import Neopixel as NPX
from machine import Pin
npx = NPX(Pin(15), 24)
npx.clear()
