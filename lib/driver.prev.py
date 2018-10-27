# Proof-of-Concept for Larga vehicle telemetry
# Copyright (c) October 2018 Steven Yap
#
# Licenced under the GNU LGLP v2 license:
#     https://www.gnu.org/licenses/old-licenses/lgpl-2.1.en.html
#
# Based on examples from Mike Tuupola:
#     https://github.com/tuupola/micropython-examples
#
import sys
sys.path.append("/flash/firmware")

import micropython
import machine
import mpu6500
import ak8963

from os import rename as os_rename
from time import time, sleep_ms
from ustruct import pack, unpack
from machine import Neopixel as npx
from machine import unique_id
from mpu9250 import MPU9250, MPU6500, AK8963
from machine import UART
from display import TFT

try:
  from machine import GPS
except:
  import GPS

  
import filemgmt
import tools

from packet import PACKET_FORMAT_9DOF, PACKET_FORMAT_GPS, PACKET_FORMAT_FUSION
from packet import PACKET_TYPE_9DOF, PACKET_TYPE_GPS, PACKET_TYPE_FUSION


# allocate buffer for emergency exceptions
micropython.alloc_emergency_exception_buf(100)

# ------ global constants ------
# enable 9dof data collection
ENABLE_9DOF = True

# enable gps data collection
ENABLE_GPS = True

# fuse 9dof and gps readingss into a single packet
ENABLE_FUSION = True

# enable log migration to SD Card device
ENABLE_MIGRATE = True

# enable network services during data collection
ENABLE_NETWORK = False

# log data directly to log files on the SD Card device
LOG_TO_SD = False

# sampling interval for 9DOF sensor
SAMPLE_INTERVAL_9DOF_MSEC = 125

# sampling interval for GPS receiver
SAMPLE_INTERVAL_GPS_MSEC = 500

# sampling interval for combined 9DOF and GPS readings
SAMPLE_INTERVAL_FUSION_MSEC = 200

# interval beween migration events
MIGRATE_INTERVAL_MSEC = 300000

# path where to write data files to
DATA_DIR = filemgmt.SOURCE_DIR

# path where to migrate data files
MIGRATE_DIR = filemgmt.TARGET_DIR

# device ID tags
ORG_TAG = None
UNIT_TAG = None

# report status when transmitting data sets
STATUS_NORMAL = 0x00
STATUS_WARN = 0x01
STATUS_DANGER = 0x02
STATUS_IMPACT = 0x04
STATUS_FALL = 0x08
STATUS_ROLL = 0x10

# determines if the setup() funtion has executed
IS_CONFIGURED = False


# ------ global variables ------
# sensor properties
_ACCEL_FULLSCALE = mpu6500.ACCEL_FS_SEL_4G
_ACCEL_LPFILTER = None
_ACCEL_OUTPUTSCALER = mpu6500.SF_M_S2
_GYRO_FULLSCALE = mpu6500.GYRO_FS_SEL_500DPS
_GYRO_LPFILTER = None
_GYRO_OUTPUTSCALER = mpu6500.SF_DEG_S
_MAG_MODE = ak8963.MODE_CONTINOUS_MEASURE_1
_MAG_RESOLUTION = ak8963.OUTPUT_16_BIT
_MAG_OFFSET = (0, 0, 0)
_MAG_SCALE = (1, 1, 1)


# ------ global (static) object instances ------
# global 6DoF sensor reference
__9DOF = None
__I2C = None

# global GPS I2C interface reference
__GPS = None
__UART = None

# global real-time clock
__RTC = machine.RTC()

# global timer instances to use
__TIMER_MOTION = machine.Timer(0)
__TIMER_LOCATION = machine.Timer(1)
__TIMER_MIGRATION = machine.Timer(2)

# global Neopixel instance reference
__NPX = None

# global lcd instance referent
__LCD = TFT()


class LogRef(object):

  def __init__(self, name, _type, _fh, max_entries, entry_count):
    self.name = name
    self.type = _type
    self._fh = _fh
    self.max_entries = max_entries
    self.entry_count = entry_count


_LOGREF_9DOF = LogRef(None, "9DOF", None, 0, 0)
_LOGREF_GPS = LogRef(None, "GPS", None, 0, 0)
_LOGREF_FUSION = LogRef(None, "FUSE", None, 0, 0)


def get_unit_tag(file_path="unit.tag"):
  unit_tag_H, unit_tag_L = unpack("LH", unique_id())
  unit_tag = (unit_tag_H << 16) | unit_tag_L
  return unit_tag


def get_org_tag(file_path="org.tag"):
  # short-circuit for now
  return 1


def start_lcd():
  tools.printmesg("INFO: Initializing LCD screen.")
  __LCD.init(
      __LCD.M5STACK, width=240, height=320, rst_pin=33, backl_pin=32,
      miso=19, mosi=23, clk=18, cs=14, dc=27, bgr=True, backl_on=1)
  __LCD.clear()
  # __LCD.setCursor(0, 0)
  # __LCD.setColor(__LCD.ORANGE)


def start_ledbar():
  global __NPX

  # tools.printmesg("INFO: Initializing Neopixel device.")
  __NPX = npx(machine.Pin(15), 24)
  __NPX.clear()


def stop_ledbar():
  if __NPX:
    # tools.printmesg("INFO: Shutting down Neopixel device.")
    __NPX.clear()
    __NPX.deinit()


def _setup_9dof(i2c):

  print("INFO: Configuring 9DOF sensor.")
  _mpu6500 = MPU6500(
      i2c, accel_fs=_ACCEL_FULLSCALE, accel_sf=_ACCEL_OUTPUTSCALER,
      gyro_fs=_GYRO_FULLSCALE, gyro_sf=_GYRO_OUTPUTSCALER)
  _ak8963 = AK8963(
      i2c, mode=_MAG_MODE, output=_MAG_RESOLUTION,
      offset=_MAG_OFFSET, scale=_MAG_SCALE)
  return(_mpu6500, _ak8963)


def start_9dof(**kwargs):
  global __I2C
  global __9DOF
  
  # tools.printmesg("INFO: Initializing 9DOF sensor.")
  __I2C = machine.I2C(scl=machine.Pin(22), sda=machine.Pin(21), speed=400000)
  (_mpu6500, _ak8963) = _setup_9dof(__I2C)
  print("INFO: Initializing 9DOF sensor.")
  __9DOF = MPU9250(__I2C, mpu6500=_mpu6500, ak8963=_ak8963)


def stop_9dof():
  if __9DOF:
    # tools.printmesg("INFO: Shutting down 6DOF sensor.")
    print("INFO: Shutting down 6DOF sensor.")
    __I2C.deinit()
    if _LOGREF_9DOF.name:
      # append file extension to last _9DOF file written
      log_path = "{0}/{1}".format(DATA_DIR, _LOGREF_9DOF.name)
      try:
        os_rename(log_path, "{0}.dat".format(log_path))
      except OSError:
        print("WARN: Could not append file extension to {0}.".format(log_path))


def start_gps(as_service=True, seed_time=True):
  global __GPS
  global __UART
  global __RTC
  
  # tools.printmesg("INFO: Initializing GPS receiver.")
  print("INFO: Initializing GPS receiver.")
  __UART = UART(2, tx=17, rx=16, baudrate=9600, timeout=60000, buffer_size=1024)
  __UART.init()
  __GPS = GPS(__UART)
  __GPS.init(timeout=60000)
  if as_service:
    __GPS.startservice()
    if seed_time:
      # run through gps data to flush bad records
      sleep_ms(2000)
      _ = [__GPS.getdata() for i in range(100)]
      # tools.set_time_gps(__GPS.getdata()[0])
      time_tuple = __GPS.getdata()[0]
      __RTC.init(time_tuple)
      

def stop_gps():
  if __GPS:
    # tools.printmesg("INFO: Shutting down GPS receiver.")
    print("INFO: Shutting down GPS receiver.")
    if __GPS.service():
      __GPS.stopservice()
    try:
      __UART.deinit()
    except ValueError:
      __UART.deinit()
    if _LOGREF_GPS.name:
      # append file extension to last GPS file written
      log_path = "{0}/{1}".format(DATA_DIR, _LOGREF_GPS.name)
      try:
        os_rename(log_path, "{0}.dat".format(log_path))
      except OSError:
        print("WARN: Could not append file extension to {0}.".format(log_path))


def start_network():
    import network
    
    # tools.printmesg("INFO: Initializing WLAN and MQTT services.")
    print("INFO: Initializing WLAN and MQTT services.")
    nic = network.WLAN(network.STA_IF)
    nic.active(True)
    sleep_ms(1000)
    network.mqtt.start()
    sleep_ms(1000)


def stop_network():
    import network
    
    # tools.printmesg("INFO: Shutting down MQTT and WLAN services.")
    print("INFO: Shutting down MQTT and WLAN services.")
    try:
      network.mqtt.stop()
    except TypeError:
      pass
    # m5cloud.mqtt.free()
    sleep_ms(1000)
    nic = network.WLAN(network.STA_IF)
    nic.active(False)
    sleep_ms(1000)


def display_splash(filename="/flash/lib/splash.jpg"):
  sleep_ms(2000)
  __LCD.clear()
  __LCD.image(58, 21, filename)


def activate_ledbar(hue=npx.GREEN, sat=1.0, max_bri=0.5, steps=24,
                    step_delay_ms=60):
  bri_step = max_bri / steps
  bri = 0.0
  for step in range(steps):
    __NPX.clear()
    for led in range(24):
      __NPX.setHSB(led, hue, sat, bri, 1, False)
    bri += bri_step
    __NPX.show()
    sleep_ms(step_delay_ms)


def deactivate_ledbar(hue=npx.GREEN, sat=1.0, max_bri=0.5, steps=24,
                      step_delay_ms=60):
  bri_step = max_bri / steps
  bri = max_bri
  for step in range(steps):
    __NPX.clear()
    for led in range(24):
      __NPX.setHSB(led, hue, sat, bri, 1, False)
    bri -= bri_step
    __NPX.show()
    sleep_ms(step_delay_ms)
  __NPX.clear()


def _create_log(data_dir, log_ref, retry_limit=3, retry_wait_ms=5):
  # there is already an open log file
  if log_ref._fh:
    log_path = "{0}/{1}".format(data_dir, log_ref.name)
    try:
      log_ref._fh.close()
    except OSError:
      # file may Fhave been inadvertently moved
      # tools.log_WARN("Failed to close log file {0}.".format(log_name))
      print("WARN: Failed to close log file {0}.".format(log_path))
    else:
      # rename the file (add extension) so it can be migrated to the SD card
      try:
        os_rename(log_path, "{0}.dat".format(log_path))
      except OSError:
        # perhaps the file got moved somehow
        # tools.log_WARN("Failed to rename log file {0}.".format(log_name))
        print("WARN: Failed to rename log file {0}.".format(log_name))
  # create the new name for the log file using the current timestamp
  filename = "{0}_{1}_{2}".format(
      int(time() * 1000000), log_ref.type, log_ref.max_entries)
  log_path = "{0}/{1}".format(data_dir, filename)
  # print("INFO: Creating log file {0}.".format(new_log_name))
  for retry_count in range(retry_limit):
    try:
      _fh = open(log_path, "wb")
    except OSError:
      log_ref.name = filename
      log_ref._fh = None
      log_ref.entry_count = 0
      sleep_ms(retry_wait_ms)
      continue
    else:
      # print("INFO: Created log file {0}.".format(new_log_name))
      # return (fh, new_log_name)
      log_ref.name = filename
      log_ref._fh = _fh
      log_ref.entry_count = 0
      break
  return log_ref


def _commit_sample(data_packet, data_dir, log_ref, retry_limit=3,
                   retry_wait_ms=5):
  if log_ref.entry_count >= log_ref.max_entries or not log_ref._fh:
    if log_ref._fh:
      mesg = ("INFO: Current log file {0}/{1} already has {2} entries. " + \
              "Closing and creating new log file.")
      print(mesg.format(data_dir, log_ref.name, log_ref.entry_count))
    log_ref = _create_log(
        data_dir, log_ref, retry_limit=retry_limit, retry_wait_ms=retry_wait_ms)
    if not log_ref._fh:
      mesg = "FATAL: Failed to create new log file {0}/{1}. Restarting device."
      # tools.log_FATAL(mesg.format(log_name))
      print(mesg.format(data_dir, log_ref.name))
      # stop()
      sleep_ms(5000)
      machine.reset()
  is_flushed = False
  for retry_count in range(retry_limit):
    try:
      log_ref._fh.write(data_packet)
    except OSError:
      is_flushed = False
      sleep_ms(retry_wait_ms)
      continue
    else:
      log_ref._fh.flush()
      log_ref.entry_count += 1
      is_flushed = True
      break
  if not is_flushed:
    # tools.log_WARN("Lost write for log file {0} caught.".format(log_name))
    mesg = "WARN: Lost write for log file {0}/{1} caught."
    print(mesg.format(data_dir, log_ref.name))
    # tools.log_DEBUG("Droped packet <{0}>.".format(data_packet))
    print("DEBUG: Droped packet <{0}>.".format(data_packet))
  return log_ref


def _evaluate_sample(sample):
  return STATUS_NORMAL


def _simple_checksum(sample):
  return 1


def _get_internal_temp():
  pass


def _build_packet_9dof(timestamp, sample, org_tag, unit_tag):
  status = _evaluate_sample(sample)
  chksm = _simple_checksum(sample)
  _packet = (
      timestamp, org_tag, unit_tag, PACKET_TYPE_9DOF, status, chksm,
      dof_sample["a"][0], dof_sample["a"][1], dof_sample["a"][2],
      dof_sample["g"][0], dof_sample["g"][1], dof_sample["g"][2],
      dof_sample["m"][0], dof_sample["m"][1], dof_sample["m"][2],)
  dof_packet = pack(PACKET_FORMAT_9DOF, *_packet)
  return dof_packet


def _build_packet_gps(timestamp, sample, org_tag, unit_tag):
  status = _evaluate_sample(sample)
  chksm = _simple_checksum(sample)
  _packet = (
      timestamp, org_tag, unit_tag, PACKET_TYPE_GPS, status, chksm,
      gps_sample["lat"], gps_sample["lng"], gps_sample["alt"],
      gps_sample["vel"],)
  gps_packet = pack(PACKET_FORMAT_GPS, *_packet)
  return gps_packet


def _build_packet_fusion(timestamp, dof_sample, gps_sample, org_tag, unit_tag):
  status = _evaluate_sample(dof_sample)
  chksm = _simple_checksum(dof_sample)
  _packet = (
      timestamp, org_tag, unit_tag, PACKET_TYPE_FUSION, status, chksm,
      dof_sample["a"][0], dof_sample["a"][1], dof_sample["a"][2],
      dof_sample["g"][0], dof_sample["g"][1], dof_sample["g"][2],
      dof_sample["m"][0], dof_sample["m"][1], dof_sample["m"][2],
      gps_sample["lat"], gps_sample["lng"], gps_sample["alt"],
      gps_sample["vel"],)
  fuse_packet = pack(PACKET_FORMAT_FUSION, *_packet)
  return fuse_packet


def collect_sample_9dof(timer, retry_limit=3, retry_wait_ms=5):
  global _LOGREF_9DOF

  timestamp = int(time() * 1000000)
  sample = dict(
      a=__9DOF.acceleration, g=__9DOF.gyro, m=__9DOF.magnetic)
  dof_packet = _build_packet_9dof(timestamp, sample, ORG_TAG, UNIT_TAG)
  _LOGREF_9DOF = _commit_sample(
      dof_packet, DATA_DIR, _LOGREF_9DOF, retry_limit=retry_limit,
      retry_wait_ms=retry_wait_ms)


def collect_sample_gps(timer, retry_limit=3, retry_wait_ms=5):
  global _LOGREF_GPS

  timestamp = int(time() * 1000000)
  (_, _lat, _lng, _alt, _, _, _kph, _, _) = __GPS.getdata()
  sample = dict(lat=_lat, lng=_lng, alt=_alt, vel=_kph,)
  gps_packet = _build_packet_gps(timestamp, sample, ORG_TAG, UNIT_TAG)
  _LOGREF_GPS = _commit_sample(
      gps_packet, DATA_DIR, _LOGREF_GPS, retry_limit=retry_limit,
      retry_wait_ms=retry_wait_ms)


def collect_sample_fusion(timer, retry_limit=3, retry_wait_ms=5):
  global _LOGREF_FUSION

  timestamp = int(time() * 1000000)
  (_, _lat, _lng, _alt, _, _, _kph, _, _) = __GPS.getdata()
  dof_sample = dict(
      a=__9DOF.acceleration, g=__9DOF.gyro, m=__9DOF.magnetic)
  gps_sample = dict(lat=_lat, lng=_lng, alt=_alt, vel=_kph)
  fusion_packet = _build_packet_fusion(
      timestamp, dof_sample, gps_sample, ORG_TAG, UNIT_TAG)
  _LOGREF_FUSION = _commit_sample(
      fusion_packet, DATA_DIR, _LOGREF_FUSION, retry_limit=retry_limit,
      retry_wait_ms=retry_wait_ms)


def migrate_logs(timer, retry_limit=3, retry_wait_ms=5):
  status = filemgmt.migrate(
      source_dir=DATA_DIR, target_dir=MIGRATE_DIR,
      retry_limit=retry_limit, retry_wait_ms=retry_wait_ms)
  if not status:
    # tools.log_FATAL("File migration failed. Restarting device.")
    print("FATAL: File migration failed. Restarting device.")
    shutdown()
    sleep_ms(5000)
    machime.reset()


def _start_timers(enable_9dof=True, enable_gps=True, enable_fusion=True,
                  log_to_sd=False, sample_interval_9dof_msec=125,
                  sample_interval_gps_msec=500, sample_interval_fusion_msec=200,
                  enable_migrate=True, migrate_interval_msec=300000):
  print("INFO: Starting data collections.")
  if enable_fusion:
    print("Starting fused 9dof and gps data collection.")
    __TIMER_MOTION.init(
        period=sample_interval_fusion_msec, mode=machine.Timer.PERIODIC,
        callback=collect_sample_fusion)
  else:
    if enable_9dof:
      print("INFO: Starting 9dof data collection.")
      # launch 9dof data collection timer
      __TIMER_MOTION.init(
          period=sample_interval_9dof_msec, mode=machine.Timer.PERIODIC,
          callback=collect_sample_9dof)
    if enable_gps:
      print("INFO: Starting gps data collection.")
      # launch gps data collection timer
      __TIMER_LOCATION.init(
          period=sample_interval_gps_msec, mode=machine.Timer.PERIODIC,
          callback=collect_sample_gps)
  if enable_migrate:
    print("INFO: Starting log migration.")
    # launch data migration timer
    __TIMER_MIGRATION.init(
        period=migrate_interval_msec, mode=machine.Timer.PERIODIC,
        callback=migrate_logs)


def setup(enable_9dof=True, enable_gps=True, enable_fusion=True, 
          enable_network=False, log_to_sd=False, sample_window_sec=120,
          sample_interval_9dof_msec=125, sample_interval_gps_msec=500,
          sample_interval_fusion_msec=200, migrate_interval_msec=300000,
          data_dir=filemgmt.SOURCE_DIR, migrate_dir=filemgmt.TARGET_DIR,
          accelerometer_fullscale=mpu6500.ACCEL_FS_SEL_4G,
          accelerometer_lpfilter=None, accelerometer_scaleout=mpu6500.SF_M_S2,
          gyrometer_fullscale=mpu6500.GYRO_FS_SEL_500DPS,
          gyrometer_lpfilter=None, gyrometer_scaleout=mpu6500.SF_DEG_S,
          magnetometer_mode=ak8963.MODE_CONTINOUS_MEASURE_1,
          magnetometer_resolution=ak8963.OUTPUT_16_BIT, enable_migrate=True,
          magnetometer_offset=(0, 0, 0), magnetometer_scale=(1, 1, 1)):
  global IS_CONFIGURED
  global ENABLE_9DOF
  global ENABLE_GPS
  global ENABLE_FUSION
  global ENABLE_NETWORK
  global ENABLE_MIGRATE
  global LOG_TO_SD
  global SAMPLE_INTERVAL_9DOF_MSEC
  global SAMPLE_INTERVAL_GPS_MSEC
  global SAMPLE_INTERVAL_FUSION_MSEC
  global MIGRATE_INTERVAL_MSEC
  global DATA_DIR
  global MIGRATE_DIR
  global ORG_TAG
  global UNIT_TAG
  global _LOGREF_9DOF
  global _LOGREF_GPS
  global _LOGREF_FUSION
  global _ACCEL_FULLSCALE
  global _ACCEL_LPFILTER
  global _ACCEL_OUTPUTSCALER
  global _GYRO_FULLSCALE
  global _GYRO_LPFILTER
  global _GYRO_OUTPUTSCALER
  global _MAG_MODE
  global _MAG_RESOLUTION
  global _MAG_OFFSET
  global _MAG_SCALE

  ENABLE_9DOF = enable_9dof
  ENABLE_GPS = enable_gps
  ENABLE_FUSION = enable_fusion
  ENABLE_NETWORK = enable_network
  ENABLE_MIGRATE = enable_migrate
  SAMPLE_INTERVAL_9DOF_MSEC = sample_interval_9dof_msec
  SAMPLE_INTERVAL_GPS_MSEC = sample_interval_gps_msec
  SAMPLE_INTERVAL_FUSION_MSEC = sample_interval_fusion_msec
  LOG_TO_SD = log_to_sd
  if enable_fusion:
    ENABLE_9DOF = True
    ENABLE_GPS = True
  if log_to_sd:
    DATA_DIR = migrate_dir
    ENABLE_MIGRATE = False
  else:
    DATA_DIR = data_dir
  ORG_TAG = get_org_tag()
  UNIT_TAG = get_unit_tag()
  MIGRATE_DIR = migrate_dir
  _LOGREF_9DOF.max_entries = int(
      sample_window_sec * (1000 / sample_interval_9dof_msec))
  _LOGREF_GPS.max_entries = int(
      sample_window_sec * (1000 / sample_interval_gps_msec))
  _LOGREF_FUSION.max_entries = int(
      sample_window_sec * (1000 / sample_interval_fusion_msec))
  _ACCEL_FULLSCALE = accelerometer_fullscale
  _ACCEL_LPFILTER = accelerometer_lpfilter
  _ACCEL_OUTPUTSCALER = accelerometer_scaleout
  _GYRO_FULLSCALE = gyrometer_fullscale
  _GYRO_LPFILTER = gyrometer_lpfilter
  _GYRO_OUTPUTSCALER = gyrometer_scaleout
  _MAG_MODE = magnetometer_mode
  _MAG_RESOLUTION = magnetometer_resolution
  _MAG_OFFSET = magnetometer_offset
  _MAG_SCALE = magnetometer_scale 
  IS_CONFIGURED = True


def start():
  global _LOGREF_9DOF
  global _LOGREF_GPS
  global _LOGREF_FUSION

  if not IS_CONFIGURED:
    setup()
  if filemgmt.mountsd():
    sleep_ms(2000)
    # check to see if the source and target data migration folders exist
    if  filemgmt.check_dir(DATA_DIR) \
        and filemgmt.check_dir(MIGRATE_DIR):
      # move any file found in /flash/__DATA__ to /sd/__DATA__
      if ENABLE_MIGRATE:
        mesg = "INFO: Migrating any existing files in {0} to {1}."
        tools.printmesg(mesg.format(DATA_DIR, MIGRATE_DIR))
        filemgmt.migrate_all()
      # initialize lcd display
      start_lcd()
      # initialize led (neopixel) bar
      start_ledbar()
      # display the splash image
      display_splash()

      if ENABLE_GPS:
        # disable MQTT and WLAN services
        if not ENABLE_NETWORK:
          stop_network()
        # initialize gps and seed real-time clock from gps data
        start_gps(as_service=True, seed_time=True)
        # create initial gps log file
        sleep_ms(1200)
      else:
        # seed real-time clock from global hst hosts; requires a WLAN connection
        tools.set_time_fqdn()
      if ENABLE_9DOF:
        # initialize 6dof sensor
        start_9dof()
        sleep_ms(1200)
      # create log entry when starting driver services
      tools.log_START("Starting LARGA driver.")
      _start_timers(
          enable_9dof=ENABLE_9DOF, enable_gps=ENABLE_GPS,
          enable_fusion=ENABLE_FUSION, log_to_sd=LOG_TO_SD,
          sample_interval_9dof_msec=SAMPLE_INTERVAL_9DOF_MSEC,
          sample_interval_gps_msec=SAMPLE_INTERVAL_GPS_MSEC,
          sample_interval_fusion_msec=SAMPLE_INTERVAL_FUSION_MSEC,
          migrate_interval_msec=MIGRATE_INTERVAL_MSEC,
          enable_migrate=ENABLE_MIGRATE)
      activate_ledbar()
      # tools.printmesg("INFO: Initialization complete. LARGA driver started.")
      print("INFO: Initialization complete. LARGA driver started.")
      return True
    else:
      mesg = "Either {0} or {1} directories were not found."
      tools.log_FATAL(mesg.format(filemgmt.SOURCE_DIR, filemgmt.TARGET_DIR))
  return False


def stop():
  __LCD.clear()
  # tools.log_STOP("Stopping LARGA driver.")
  print("INFO: Stopping LARGA driver.")
  # tools.printmesg("INFO: Stopping main event loop.")
  print("INFO: Stopping data collections.")
  if ENABLE_9DOF:
    print("INFO: Stopping 9dof data collection.")
    # stop 9dof data collection timer
    __TIMER_MOTION.deinit()
    sleep_ms(1200)
    # stop the 9DOF sensor and unregister it from the I2C bus
    stop_9dof()
    sleep_ms(600)
  if ENABLE_GPS:
    print("INFO: Stopping gps data collection.")
    # stop gps data collection timer
    __TIMER_LOCATION.deinit()
    sleep_ms(1200)
    # stop the gps receiver
    stop_gps()
    sleep_ms(600)
    if not ENABLE_NETWORK:
      # restart network stack
      start_network()
  # stop data migration timer
  if ENABLE_MIGRATE:
    __TIMER_MIGRATION.deinit()
  sleep_ms(1200)
  if ENABLE_FUSION:
    log_path = "{0}/{1}".format(DATA_DIR, _LOGREF_FUSION.name)
    try:
      os_rename(log_path, "{0}.dat".format(log_path))
    except OSError:
      print("WARN: Could not append file extension to {0}.".format(log_path))
  # move all remaining files in /flash/__DATA__ into /sd/__DATA__
  mesg = "INFO: Migrating any existing files in {0} to {1}."
  # tools.printmesg(mesg.format(filemgmt.SOURCE_DIR, filemgmt.TARGET_DIR))
  print(mesg.format(filemgmt.SOURCE_DIR, filemgmt.TARGET_DIR))
  if ENABLE_MIGRATE:
    filemgmt.migrate_all()
  # power down led bar
  deactivate_ledbar()
  # stop neopixel device and free all used resources
  stop_ledbar()
  tools.log_STOP("Stopped LARGA driver.")
  tools.printmesg("INFO: Shutdown complete. LARGA driver stopped.")
  return True
