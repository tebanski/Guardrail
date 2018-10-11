# Proof-of-Concept for Larga vehicle telemetry
# Copyright (c) October 2018 Steven Yap
#
# Licenced under the GNU LGLP v2 license:
#     https://www.gnu.org/licenses/old-licenses/lgpl-2.1.en.html
#
# Based on examples from Mike Tuupola:
#     https://github.com/tuupola/micropython-examples
#


import micropython
import machine
import uos
import uio

from ucollections import deque, namedtuple
from ustruct import pack
from m5stack import lcd, time
from fusion import Fusion

import filemgmt


# allocate buffer for emergency exceptions
micropython.alloc_emergency_exception_buf(200)

# use M5STACK FIRE as the hardware platform
USE_FIRE = False

if USE_FIRE:
  from mpu6050_fire import MPU6050
else:
  from mpu6050_go import MPU6050

# use timers for the main loop
USE_TIMERS = True

# log the readings in the SD card
LOG_READINGS = True

# font to use for LCD display
DISPLAY_FONT = lcd.FONT_Default

# type of Larga packet generated by this device
PACKET_TYPE = 4

# number of msecs to sleep between each DOF and GPS readings
SAMPLE_INTERVAL_MSEC = 200

# path where to write data files to
DATA_PATH = filemgmt.SOURCE_DIR

# format to use for packing data for storage and transmission
# PACK_FORMAT = "QLLBBdddddddddddd"
PACK_FORMAT = "QLLBBffffffffffff"

# number of seconds for each sample window commited to a log
SAMPLE_WINDOW_SEC = 120

# number of seconds to wait before migrating data from flash to SD card
MIGRATE_INTERVAL_MSEC = 300 * 1000

# maximum number of samples per data file
MAX_ENTRIES_PER_FILE = int(SAMPLE_WINDOW_SEC * (1000 / SAMPLE_INTERVAL_MSEC))

# timers variables to use
TIMER_COLLECT = machine.Timer(0)
TIMER_MIGRATE = machine.Timer(1)

# report status when transmitting data sets
STATUS_NORMAL = 0x00
STATUS_WARN = 0x01
STATUS_DANGER = 0x02
STATUS_IMPACT = 0x04
STATUS_FALL = 0x08
STATUS_ROLL = 0x10

# attach to 9DoF module
if USE_FIRE:
    sensor = MPU6050()
else:
    i2c = machine.I2C(scl=22, sda=21, speed=400000)
    sensor = MPU6050(i2c)

# attach to GPS module
receiver = None
gps = None

# number of entries in the current data file
FILE_ENTRIES = 0

# create DATA_PACKET object for sample readings
DATA_PACKET = None

# global file handle for currently-opened log file
FH_ACTIVE = None

# filename of currently active log file
LOG_NAME = None


def get_unit_tag(file_path="unit.tag"):
  # short-circuit for now
  return 1


def get_org_tag(file_path="org.tag"):
  # short-circuit for now
  return 1


def create_log_file(dir_path=DATA_PATH, retry_limit=3, retry_wait_ms=5):
  global FILE_ENTRIES
  global FH_ACTIVE
  global LOG_NAME

  # there is already an open log file
  if FH_ACTIVE:
    try:
      FH_ACTIVE.close()
    except OSError:
      # file may have been inadvertently moved
      print("WARN: Failed to close log file {0}.".format(LOG_NAME))
    else:
      # rename the file (add extension) so it can be migrated to the SD card
      try:
        os.rename(LOG_NAME, "{0}.dat".format(LOG_NAME))
      except OSError:
        # perhaps the file got moved somehow
        print("WARN: Failed to rename log file {0}.".format(LOG_NAME))
  # create the new name for the log file using the current timestamp
  filename = "{}_{}".format(
      int(time.time() * 1000000), MAX_ENTRIES_PER_FILE)
  LOG_NAME = "{0}/{1}".format(dir_path, filename)
  is_created = False
  for retry_count in range(retry_limit):
    try:
      FH_ACTIVE = open(LOG_NAME, "wb")
    except OSError:
      is_created = False
      time.sleep_ms(retry_wait_ms)
      continue
    else:
      FILE_ENTRIES = 0
      is_created = True
      break
  return is_created


def collect_sensor_readings(timer):
  global FILE_ENTRIES
  
  timestamp = int(time.time() * 1000000)
  sensor_readings = dict(
      a=sensor.acceleration, g=sensor.orientation, m=sensor.direction)
  receiver_readings = _collect_gps(gps, receiver)
  raw_packet = _build_packet(timestamp, sensor_readings, receiver_readings)
  data_packet = pack(PACK_FORMAT, *raw_packet)
  if FILE_ENTRIES >= MAX_ENTRIES_PER_FILE:
    mesg = ("INFO: Current log file {0} already has {1} entries. " + \
            "Closing and creating new log file.")
    print(mesg.format(LOG_NAME, FILE_ENTRIES))
    is_created = create_log_file()
    if not is_created:
      mesg = "FATAL: Failed to create new log file {0}. Restarting device." 
      print(mesg.format(LOG_NAME))
      shutdown()
      machine.reset()
  retry_limit=3
  retry_wait_ms = 5
  is_flushed = False
  for retry_count in range(retry_limit):
    try:
      FH_ACTIVE.write(data_packet)
    except OSError:
      is_flushed = False
      time.sleep_ms(retry_wait_ms)
      continue
    else:
      FH_ACTIVE.flush()
      FILE_ENTRIES += 1
      is_flushed = True
      break
  if not is_flushed:
    print("WARN: Lost write for log file {0} caught.".format(LOG_NAME))
    print("DEBUG: packet <{0}>.".format(data_packet))


def _collect_gps(gps, receiver):
  if receiver:
    _ = [gps.update(chr(char)) for sentence in receiver.sentences()
         for char in sentences]
    return dict(lat=gps.latitude, lng=gps.longtitude, vel=gps.speed)
  return dict(lat=0.0, lng=0.0, vel=0.0)


def _build_packet(timestamp, dof_sensor, gps_receiver):
  status = evaluate_readings()
  org_tag = get_org_tag()
  unit_tag = get_unit_tag()
  packet = (
      timestamp, org_tag, unit_tag, PACKET_TYPE, status,
      dof_sensor["a"][0], dof_sensor["a"][1], dof_sensor["a"][2],
      dof_sensor["g"][0], dof_sensor["g"][1], dof_sensor["g"][2],
      dof_sensor["m"][0], dof_sensor["m"][1], dof_sensor["m"][2],
      gps_receiver["lat"], gps_receiver["lng"], gps_receiver["vel"])
  return packet


def display_readings(timer):
  lcd.text(60, 20, "X-ACCEL: {:+3.4f} m/s/s\r".format(DATA_PACKET[5]))
  lcd.text(60, 40, "Y-ACCEL: {:+3.4f} m/s/s\r".format(DATA_PACKET[6]))
  lcd.text(60, 60, "Z-ACCEL: {:+3.4f} m/s/s\r".format(DATA_PACKET[7]))
  lcd.text(60, 80, "X-ANGV: {:+3.4f} deg/s\r".format(DATA_PACKET[8]))
  lcd.text(60, 100, "Y-ANGV: {:+3.4f} deg/s\r".format(DATA_PACKET[9]))
  lcd.text(60, 120, "Z-ANGV: {:+3.4f} deg/s\r".format(DATA_PACKET[10]))
  lcd.text(102, 140, "LATI: {:+3.4f} deg\r".format(DATA_PACKET[14]))
  lcd.text(102, 160, "LONG: {:+3.4f} deg\r".format(DATA_PACKET[15]))
  lcd.text(88, 180, "SPEED: {:+3.4f} m/s\r".format(DATA_PACKET[16]))


def evaluate_readings():
  return STATUS_NORMAL


def migrate_files(timer):
  status = filemgmt.migrate(
      source_dir=filemgmt.SOURCE_DIR, target_dir=filemgmt.TARGET_DIR)
  if not status:
    print("FATAL: File migration failed. Restarting device.")
    machime.reset()


def init():
  global FH_ACTIVE
    
  if filemgmt.mountsd():
    print("INFO: SD device mounted on /sd.")
    time.sleep(2)
    if  filemgmt.check_dir(filemgmt.SOURCE_DIR) \
        and filemgmt.check_dir(filemgmt.TARGET_DIR):
      # move any file found in /flash/__DATA__ to /sd/__DATA__
      print(
          "INFO: Migrating any existing files in {0} to {1}.".format(
              filemgmt.SOURCE_DIR, filemgmt.TARGET_DIR))
      filemgmt.migrate_all()
      # initialize the display LCD
      print("INFO: Initializing LCD screen.")
      lcd.init(
          lcd.M5STACK, width=240, height=320, rst_pin=33, backl_pin=32,
          miso=19, mosi=23, clk=18, cs=14, dc=27, bgr=True, backl_on=1)
      # create initial log file
      is_created = create_log_file()
      if not is_created:
        mesg = "FATAL: Failed to create new log file {0}. Restarting device." 
        print(mesg.format(LOG_NAME))
        machine.reset()
      print("INFO: Starting main event loop.")
      TIMER_COLLECT.init(
          period=SAMPLE_INTERVAL_MSEC, mode=machine.Timer.PERIODIC,
          callback=collect_sensor_readings)
      
      TIMER_MIGRATE.init(
          period=MIGRATE_INTERVAL_MSEC, mode=machine.Timer.PERIODIC,
          callback=migrate_files)
      print("INFO: Initialization complete.")
      return True
    else:
      print("FATAL: Either {0} or {1} was not found.".format(
        filemgmt.SOURCE_DIR, filemgmt.TARGET_DIR))
  return False


def shutdown():
  print("INFO: Stopping main event loop.")
  TIMER_COLLECT.deinit()
  TIMER_MIGRATE.deinit()
  print("INFO: Migrating any existing files in {0} to {1}.".format(
      filemgmt.SOURCE_DIR, filemgmt.TARGET_DIR))
  filemgmt.migrate_all()
  print("INFO: Shutdown complete.")
  return True
