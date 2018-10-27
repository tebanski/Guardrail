import machine
import time

# from m5stack import lcd


LOG_DIR = "/flash/__LOG__"


def log_marker(marker_type, mesg=None, on_lcd=False):
  timestamp = int(time.time() * 1000000)
  filename = "{0}/{1}_{2}.log".format(LOG_DIR, timestamp, marker_type)
  with open(filename, "w") as fh_mrkr:
    if mesg:
      fh_mrkr.write("{0}::{1}::{2}".format(timestamp, marker_type, mesg))
      print("{0}: {1}".format(marker_type, mesg))
      if on_lcd:
        # lcd.print("{0}: {1}\n".format(marker_type, mesg))
        pass
    else:
      fh_mrkr.write("{0}::{1}".format(timestamp, marker_type))


def log_START(mesg=None, on_lcd=False):
  log_marker("START", mesg=mesg, on_lcd=on_lcd)


def log_INFO(mesg=None, on_lcd=False):
  log_marker("INFO", mesg=mesg, on_lcd=on_lcd)


def log_WARN(mesg=None, on_lcd=False):
  log_marker("WARN", mesg=mesg, on_lcd=on_lcd)


def log_FATAL(mesg=None, on_lcd=False):
  log_marker("FATAL", mesg=mesg, on_lcd=on_lcd)


def log_DEBUG(mesg=None, on_lcd=False):
  log_marker("DEBUG", mesg=mesg, on_lcd=on_lcd)


def log_STOP(mesg=None, on_lcd=False):
  log_marker("STOP", mesg=mesg, on_lcd=on_lcd)


def printmesg(mesg):
  print(mesg)
  # lcd.print(mesg + "\n")


def set_time_fqdn(nist_fqdn="time.nist.gov"):
  printmesg(
      "INFO: Synchronizing real-time clock with ntp host {0}".format(nist_fqdn))
  rtc = machine.RTC()
  printmesg("INFO: Pre-synchronization time {0}".format(time.time()))
  rtc.ntp_sync(server=nist_fqdn)
  printmesg("INFO: Post-synchronization time {0}".format(time.time()))


def set_time_gps(time_tuple=None):
  if time_tuple:
    printmesg("INFO: Synchronizing real-time clock with GPS timestamps.")
    rtc = machine.RTC()
    printmesg("INFO: Pre-synchronization time {0}".format(time.time()))
    rtc.init(time_tuple)
    printmesg("INFO: Post-synchronization time {0}".format(time.time()))
