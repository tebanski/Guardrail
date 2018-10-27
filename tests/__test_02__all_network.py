"""This test runs all data collections and data migration
with mqtt and wlan enabled.
"""

import sys
sys.path.append("/flash/lib")

import driver



def run():
  driver.init(enable_network=True)
