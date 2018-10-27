# add path to LARGA-specific Micropython modules
import sys
sys.path.append("/flash/lib")
sys.path.append("/flash/firmware")


# configure SD SPI interface for mounting
import os
from time import sleep_ms

os.sdconfig(mode=os.SDMODE_SPI, clk=18, mosi=23, miso=19, cs=4)
try:
  os.mountsd()
except OSError:
  sleep_ms(2000)
  try:
    os.mountsd()
  except OSError:
    raise OSError("FATAL: Cannot mount SD card device.")
print("INFO: SD card device mounted on /sd.")




# start the LARGA IU motion sensor platform
# import driver
# lsp = driver.LargaSensorPlatform()
# lsp.start()
