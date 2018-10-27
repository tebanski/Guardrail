"""This test runs all data collections and log to SD card
with mqtt and wlan enabled.
"""

import sys
sys.path.append("/flash/lib")

import driver



def run():
  driver.init(log_to_sd=True, enable_network=True)
