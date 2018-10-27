"""This test runs all data collections and data migration
with mqtt and wlan disabled.
"""
import sys
sys.path.append("/flash/lib")

import driver



def run():
  driver.init()
