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
import network
import mpu6500
import ak8963

from os import rename as os_rename
from time import time, sleep_ms
from ustruct import pack, unpack
from micropython import schedule
from machine import UART, Pin, RTC, unique_id
from machine import Neopixel as npx
from mpu9250 import MPU9250, MPU6500, AK8963
from display import TFT

try:
  from machine import GPS
except:
  import GPS

  
import filemgmt
import tools

from filemgmt import migrate
from packet import PACKET_FORMAT_9DOF, PACKET_FORMAT_GPS, PACKET_FORMAT_FUSION
from packet import PACKET_TYPE_9DOF, PACKET_TYPE_GPS, PACKET_TYPE_FUSION


# allocate buffer for emergency exceptions
micropython.alloc_emergency_exception_buf(100)


# global timer instances to use
__TIMER_9DOF = machine.Timer(0)
__TIMER_GPS = machine.Timer(1)
__TIMER_MIGRATE = machine.Timer(2)


# report status when transmitting data sets
STATUS_NORMAL = 0x00
STATUS_WARN = 0x01
STATUS_DANGER = 0x02
STATUS_IMPACT = 0x04
STATUS_FALL = 0x08
STATUS_ROLL = 0x10


def TASK_CollectSample_9DOF(timer):
  schedule(LargaSensorPlatform.collect_sample_9dof, 3)


def TASK_CollectSample_GPS(timer):
  schedule(LargaSensorPlatform.collect_sample_gps, 3)


def TASK_CollectSample_Fusion(timer):
  schedule(LargaSensorPlatform.collect_sample_fuse, 3)


def TASK_MigrateLogs(timer):
  schedule(LargaSensorPlatform.migrate_logs, 3)


class LogRef(object):
  """ 
  Log Refence class keeps track of the current data log file.
  """

  def __init__(self, name, _type, _fh, max_entries, entry_count):
    """
    Creates a class instance to track the current (active) file used for
    data logging.

    Args:
      name (str): file name of the active log file (excluding directory path)
      _type (str): file type of the active log file; currently supported legal
          values are '9DOF', 'GPS', and 'FUSE'
      _fh (IOBuffer): file handle retuned by the open() function call
      max_entries: maximum nunber of entries for the active log file; can
          change between types
      entry_count (int): current number of entries in the active log file;
          usually updated by LargaSensorPlatform._commit_sample() 
    """
    self.name = name
    self.type = _type
    self._fh = _fh
    self.max_entries = max_entries
    self.entry_count = entry_count


class LargaSensorPlatform(object):
  """
  Encapsulates the logic for managing the Larga Sensor Platform data collection.
  """

  # class variables; persistent over different instances of the class
  __9dof = None   # reference to 9DOF sensor object
  __i2c = None    # reference to I2C object used by the 9DOF sensor object 
  __gps = None    # reference to GPS receiver object
  __uart = None   # reference to UART object used by the GPS receiver object
  __log_9dof = LogRef(None, "9DOF", None, 0, 0)   # keeps track of which files
  __log_gps = LogRef(None, "GPS", None, 0, 0)     # are currently open and are
  __log_fuse = LogRef(None, "FUSE", None, 0, 0)   # being written to
  __rtc = RTC()   # reference to the Real Time Clock object used for timestamps
  __npx = None    # reference to the Neopixel RGB LED device (if available)
  __lcd = TFT()   # reference to the LCD display object (if available)
  __unit_tag = 0  # unique id or device from ESP32 CPU ID
  __org_tag = 0   # unique id assigned to organization where device is deployed
  __data_path = filemgmt.SOURCE_DIR     # where data is written to initially
  __migrate_path = filemgmt.TARGET_DIR  # where data is moved to eventually

  def __init__(
         elf, enable_9dof=True, enable_gps=True, enable_fuse=False,
         enable_migrate=True, log_to_sd=False, enable_network=False,
         accelerometer_fs=mpu6500.ACCEL_FS_SEL_4G,
         accelerometer_so=mpu6500.SF_M_S2, accelerometer_filter=None, 
         gyrometer_fs=mpu6500.GYRO_FS_SEL_500DPS,
         gyrometer_so=mpu6500.SF_DEG_S, gyrometer_filter=None,
         magnetometer_mode=ak8963.MODE_CONTINOUS_MEASURE_1,
         magnetometer_so=ak8963.OUTPUT_16_BIT, magnetometer_offset=(0, 0, 0),
         magnetometer_scale=(1, 1, 1), sample_window_sec=120,
         sample_interval_9dof_msec=200, sample_interval_gps_msec=500,
         sample_interval_fuse_msec=250, migrate_interval_msec=300000,
         data_path=filemgmt.SOURCE_DIR, migrate_path=filemgmt.TARGET_DIR):
    """
    Creates an instance of the Larga Sensor Platform driver.

    Kwargs:
      enable_9dof (bool): enables/disables 9dof sensor operation on startup
      enable_gps (bool): enabled/disables GPS receiver operation on startup
      enable_fuse (bool): enables/disables fused data collection for both
          9dof sensor and GPS receiver
      enable_network (bool): enables/disables network services on startup
      log_to_sd (bool): enabled/disables direct-to-SD device logging
      accelerometer_fs (int): determines the accelerometer's full-scale
          readings; please ckeck the mpu6500 module for valid values
      accelerometer_filter (int): determines the accelerometer's sensitivity;
          please check the mpu6500 module for valid values
      accelerometer_so (int): determines the accelerometer's output scale
          (e.g., m/s/s or G's); please ckeck the mpu6500 module for valid values
      gyrometer_fs (int): determines the gyrometer's full-scale readings;
          please ckeck the mpu6500 module for valid values
      gyrometer_filter (int): determines the gyrometer's sensitivity; please
          ckeck the mpu6500 module for valid values
      gyrometer_so (int): determines the gyrometer's output scale (i.e.,
          deg/sec or rad/sec); please ckeck the mpu6500 module for valid values
      magnetometer_mode (int): 
      magnetometer_so (int): determines the magnetometer's resolutionm; please
          check the ak8963 module for valid values
      magnetometer_offset (tuple, 3): 
      magnetometer_scale (tuple, 3): 
      sample_window_sec (int): amount of time to write data into the active
          log before closing it an opening a new log file; used to determine
          the maximum number of entries in an active log file
      sample_interval_9dof_msec (int): sampling interval for 9dof readings 
      sample_interval_gps_msec (int): sampling interval for GPS readings
      sample_interval_fuse_msec (int): sampling interval for combined 9dof
          and GPS readings
      migrate_interval_msec (int): amount of time to wait before migrating
          data logs; usually from /flash/__DATA__ to /sd/__DATA__
      data_path (str): directory path where data is written to initially
      migrate_path (str): directory path where data is moved to eventually
    """
    self.enable_9dof = enable_9dof
    self.enable_gps = enable_gps
    self.enable_fuse = enable_fuse
    if self.enable_fuse:
      self.enable_9dof = True
      self.enable_gps = True
    self.enable_migrate = enable_migrate
    self.log_to_sd = log_to_sd
    self.enable_network = enable_network
    self.accel_full_scale = accelerometer_fs
    self.accel_filter = accelerometer_filter
    self.accel_out = accelerometer_so
    self.gyro_full_scale = gyrometer_fs
    self.gyro_filter = gyrometer_filter
    self.gyro_out = gyrometer_so
    self.mag_mode = magnetometer_mode
    self.mag_out = magnetometer_so
    self.mag_offset = magnetometer_offset
    self.mag_scale = magnetometer_scale
    self.npx_led_count = 0
    self.sample_window_sec = sample_window_sec
    self.sample_interval_9dof_msec = sample_interval_9dof_msec
    self.sample_interval_gps_msec = sample_interval_gps_msec
    self.sample_interval_fuse_msec = sample_interval_fuse_msec
    self.migrate_interval_msec = migrate_interval_msec
    self.__log_9dof.max_entries = int(
        self.sample_window_sec * (1000 / self.sample_interval_9dof_msec))
    self.__log_gps.max_entries = int(
        self.sample_window_sec * (1000 / self.sample_interval_gps_msec))
    self.__log_fuse.max_entries = int(
        self.sample_window_sec * (1000 / self.sample_interval_fuse_msec))
    self.__data_path = data_path
    self.__migrate_path = migrate_path
    if self.log_to_sd:
      self.__data_path = self.__migrate_path
      self.enable_migrate = False
    self.__unit_tag = self.get_unit_tag()
    self.__org_tag = self.get_org_tag()

  @staticmethod
  def get_unit_tag():
    """Returns the unit tag of this unit from the ESP32's unique ID."""
    unit_tag_H, unit_tag_L = unpack("LH", unique_id())
    unit_tag = (unit_tag_H << 16) | unit_tag_L
    return unit_tag

  @staticmethod
  def get_org_tag(file_path="/flash/org.tag"):
    """Returns the assigned organization tag the unit belongs to.

    Kwargs:
      file_path: local path to the file where the organization tag is stored
    """
    return 1

  def start_lcd(self):
    """Initializes the LCD device, if available."""
    tools.printmesg("INFO: Initializing LCD screen.")
    self.__class__.__lcd.init(
        self.__lcd.M5STACK, width=240, height=320, rst_pin=33, backl_pin=32,
        miso=19, mosi=23, clk=18, cs=14, dc=27, bgr=True, backl_on=1)
    self.__class__.__lcd.clear()

  def start_ledbar(self, led_count=24):
    """Initialized the Neopixel RGB LED bar, if available.
    
    Kwargs:
      led_count (int): number of LED elements in the device
    """
    self.npx_led_count = led_count
    self.__class__.__npx = npx(Pin(15), led_count)
    self.__class__.__npx.clear()

  def stop_ledbar(self):
    """De-initializes and clears the Neopixel RGB LEB bar, if available."""
    if self.__class__.__npx:
      self.__class__.__npx.clear()
      self.__class__.__npx.deinit()

  def _setup_9dof(self):
    """Instantiates and configures the 9DOF sensor prior to initialization."""
    self.__class__.__i2c = machine.I2C(
      scl=machine.Pin(22), sda=machine.Pin(21), speed=400000)
    mpu6500 = MPU6500(
        self.__i2c, accel_fs=self.accel_full_scale, accel_sf=self.accel_out,
        gyro_fs=self.gyro_full_scale, gyro_sf=self.gyro_out)
    ak8963 = AK8963(
        self.__i2c, mode=self.mag_mode, output=self.mag_out,
        offset=self.mag_offset, scale=self.mag_scale)
    return (mpu6500, ak8963)

  def start_9dof(self):
    """Initializes the 9DOF sensor for periodic data collection."""
    # tools.printmesg("INFO: Initializing 9DOF sensor.")
    print("INFO: Initializing 9DOF sensor.")
    (mpu6500, ak8963) = self._setup_9dof()
    self.__class__.__9dof = MPU9250(self.__i2c, mpu6500=mpu6500, ak8963=ak8963)

  def stop_9dof(self):
    """De-initializes the 9DOF sensor and frees used (memory) resources."""
    if self.__class__.__9dof:
      # tools.printmesg("INFO: Shutting down 6DOF sensor.")
      print("INFO: Shutting down 6DOF sensor.")
      self.__class__.__i2c.deinit()
      if self.__class__.__log_9dof.name:
        # append file extension to last _9DOF file written
        log_path = "{0}/{1}".format(
            self.__class__.__data_path, self.__class__.__log_9dof.name)
        try:
          os_rename(log_path, "{0}.dat".format(log_path))
        except OSError:
          print(
              "WARN: Could not append file extension to {0}.".format(log_path))

  def start_gps(self, as_service=True, seed_time=True):
    """Instantiates, configures, and initializes the GPS receiver for periodic
       data collection.

    Kwargs:
      as_service(bool): enables/disables GPS background service on startup
      seed_time (bool): determines if the RTC is seeded from received GPS data
          or by connection to a NIST host
    """
    print("INFO: Initializing GPS receiver.")
    self.__class__.__uart = UART(
        2, tx=17, rx=16, baudrate=9600, timeout=60000, buffer_size=1024)
    self.__class__.__uart.init()
    self.__class__.__gps = GPS(self.__uart)
    self.__class__.__gps.init(timeout=60000)
    if as_service:
      self.__class__.__gps.startservice()
      if seed_time:
        # run through gps data to flush bad records
        sleep_ms(2000)
        _ = [self.__class__.__gps.getdata() for i in range(100)]
        # tools.set_time_gps(__GPS.getdata()[0])
        time_tuple = self.__class__.__gps.getdata()[0]
        self.__class__.__rtc.init(time_tuple)

  def stop_gps(self):
    """De-initializes the GPS receiver and frees used (memory) resources."""
    if self.__class__.__gps:
      # tools.printmesg("INFO: Shutting down GPS receiver.")
      print("INFO: Shutting down GPS receiver.")
      if self.__class__.__gps.service():
        self.__class__.__gps.stopservice()
      try:
        self.__class__.__uart.deinit()
      except ValueError:
        self.__class__.__uart.deinit()
      if self.__class__.__log_gps.name:
        # append file extension to last GPS file written
        log_path = "{0}/{1}".format(
            self.__class__.__data_path, self.__class__.__log_gps.name)
        try:
          os_rename(log_path, "{0}.dat".format(log_path))
        except OSError:
          print(
              "WARN: Could not append file extension to {0}.".format(log_path))

  def start_wlan(self):
    """Initializes the wireless network interface."""
    print("INFO: Initializing WLAN service.")
    nic = network.WLAN(network.STA_IF)
    nic.active(True)

  def stop_wlan(self):
    """De-initializes the wireless network interface."""
    print("INFO: Shutting down WLAN service.")
    nic = network.WLAN(network.STA_IF)
    nic.active(False)

  def start_mqtt(self):
    """Initializes the MQTT service."""
    print("INFO: Initializing MQTT service.")
    try:
      network.mqtt.start()
    except TypeError:
      pass

  def stop_mqtt(self):
    """De-initializes the MQTT service."""
    print("INFO: Shutting down MQTT service.")
    try:
      network.mqtt.stop()
    except TypeError:
      pass

  def start_network(self):
    """Initializes network services."""
    self.start_wlan()
    sleep_ms(1200)
    self.start_mqtt()
    sleep_ms(1200)

  def stop_network(self):
    """De-initializes network services."""
    self.stop_mqtt()
    sleep_ms(1200)
    self.stop_wlan()
    sleep_ms(1200)

  def display_splash(self, file_path="/flash/lib/splash.jpg"):
    """Displays the splash image on the LCD screen, if available.

    Kwargs:
      file_path: path to splash image
    """
    self.__class__.__lcd.clear()
    self.__class__.__lcd.image(58, 21, file_path)

  def activate_ledbar(self, hue=None, sat=1.0, max_bri=0.5, steps=24,
                      step_delay_ms=40):
    """Activates the Neopixed RBD LED device, if available.

    Kwargs:
      hue (float pr None): color hue to display; a value of None defaults to
          the value GREEN which, oddly, shows ad the color RED
      sat (float): color saturation value of current huw
      max_bri (float): value from 0.0 to 1.0 that determines the maximum
          brightness of the LED elements
      steps (int): number of steps to use when increasing brightness from 0.0
          to the value of max_bri
      step_delay_ms: number of milliseconds between each step when increasing
          brightness values
    """
    if hue is None:
      hue = self.__class__.__npx.GREEN
    bri_step = max_bri / steps
    bri = 0.0
    for step in range(steps):
      # self.__class__.__npx.clear()
      for led in range(24):
        self.__class__.__npx.setHSB(led, hue, sat, bri, 1, False)
      bri += bri_step
      self.__class__.__npx.show()
      sleep_ms(step_delay_ms)
    self.__class__.__npx.setHSB(led, hue, max_bri, bri, 1, True)

  def deactivate_ledbar(self, hue=None, sat=1.0, max_bri=0.5, steps=24,
                        step_delay_ms=40):
    """De-activates the Neopixed RBD LED device, if available.

    Kwargs:
      hue (float pr None): color hue to display; a value of None defaults to
          the value GREEN which, oddly, shows ad the color RED
      sat (float): color saturation value of current huw
      max_bri (float): value from 0.0 to 1.0 that determines the maximum
          brightness of the LED elements
      steps (int): number of steps to use when decreasing brightness from the
          value of max_bri to 0.0
      step_delay_ms: number of milliseconds between each step when decreasing
          brightness values
    """
    if hue is None:
      hue = self.__class__.__npx.GREEN
    bri_step = max_bri / steps
    bri = max_bri
    for step in range(steps):
      # self.__class__.__npx.clear()
      for led in range(24):
        self.__class__.__npx.setHSB(led, hue, sat, bri, 1, False)
      bri -= bri_step
      self.__class__.__npx.show()
      sleep_ms(step_delay_ms)
    self.__class__.__npx.setHSB(led, hue, sat, 0.0, 1, True)
    self.__class__.__npx.clear()

  @staticmethod
  def _create_log(dir_path, log_ref, retry_limit=3, retry_wait_ms=5):
    """Creates a new active log file and closes the previous one.

    Kwargs:
      dir_path (str): directory path where the current active log file resides
      log_ref (LogRef): reference to the current active log file
      retry_limit (int): number of times to retry file creation if failures
          are encountered
      retry_wait_ms (int): amount of time to wait between retries
    """
    if log_ref._fh:
      # close current (active) log before creating a new one
      log_path = "{0}/{1}".format(
          LargaSensorPlatform.__data_path, log_ref.name)
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
          print("WARN: Failed to rename log file {0}.".format(log_path))
    # create the new name for the log file using the current timestamp
    filename = "{0}_{1}_{2}".format(
        int(time() * 1000000), log_ref.type, log_ref.max_entries)
    log_path = "{0}/{1}".format(dir_path, filename)
    # print("INFO: Creating log file {0}.".format(log_path))
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
        # print("INFO: Created log file {0}.".format(log_path))
        log_ref.name = filename
        log_ref._fh = _fh
        log_ref.entry_count = 0
        break
    return log_ref

  def _create_initial_log(self, dir_path, log_ref, retry_limit=3,
                          retry_wait_ms=5):
    """Creates the initial actve log file. Only used during startup.

    Kwargs:
      dir_path (str): directory path where the current active log file resides
      log_ref (LogRef): reference to the current active log file
      retry_limit (int): number of times to retry file creation if failures
          are encountered
      retry_wait_ms (int): amount of time to wait between retries
    """
    _log_ref = LargaSensorPlatform._create_log(
        dir_path, log_ref, retry_limit=retry_limit, retry_wait_ms=retry_wait_ms)
    if not _log_ref._fh:
      mesg = "Failed to create new log file {0}. Restarting device."
      # tools.log_FATAL(mesg.format(_LOG_NAME_GPS))
      print("FATAL: " + mesg.format(_log_ref.name))
      # self.stop()
      sleep_ms(5000)
      machine.reset()
    log_ref = _log_ref
    return log_ref

  @staticmethod
  def _commit_sample(data_packet, dir_path, log_ref, retry_limit=3,
                     retry_wait_ms=5):
    """Commits a sample from the 9DOF and/or the GPS to stable storage.

    Kwargs:
      data_packet (bin): binary sequence that represents a sample; fron the
          return value of LargaSensorPlatform._build_packet_*
      dir_path (str): directory path where the current active log file resides
      log_ref (LogRef): reference to the current active log file
      retry_limit (int): number of times to retry file creation if failures
          are encountered
      retry_wait_ms (int): amount of time to wait between retries
    """
    if log_ref.entry_count >= log_ref.max_entries:
      mesg = ("INFO: Current log file {0} already has {1} entries. " + \
              "Closing and creating new log file.")
      print(mesg.format(log_ref.name, log_ref.entry_count))
      _log_ref = LargaSensorPlatform._create_log(
          dir_path, log_ref, retry_limit=retry_limit,
          retry_wait_ms=retry_wait_ms)
      if not _log_ref._fh:
        mesg = "Failed to create new log file {0}. Restarting device."
        # tools.log_FATAL(mesg.format(log_name))
        print("FATAL: " + mesg.format(_log_ref.name))
        # self.stop()
        sleep_ms(5000)
        machine.reset()
      else:
        log_ref = _log_ref
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
      print("WARN: Lost write for log file {0} caught.".format(log_ref.name))
      # tools.log_DEBUG("Droped packet <{0}>.".format(data_packet))
      print("DEBUG: Droped packet <{0}>.".format(data_packet))
    return log_ref

  @staticmethod
  def _build_packet_9dof(timestamp, sample, org_tag, unit_tag):
    """Returns a binary sequence tha represents a 9DOF sample.

    Kwargs:
      timestamp (int): Epoch timestamp of the 9DOF sample
      sample (dict): a dictionary representing a 9DOF sample
      org_tag (int): assigned organization tag the unit belongs to
      unit_tag (int): unit tag of this unit from the ESP32's unique ID
    """
    status = LargaSensorPlatform._evaluate_sample(sample)
    chksm = LargaSensorPlatform._simple_checksum(sample)
    _packet = (
        timestamp, org_tag, unit_tag, PACKET_TYPE_9DOF, status, chksm,
        sample["a"][0], sample["a"][1], sample["a"][2],
        sample["g"][0], sample["g"][1], sample["g"][2],
        sample["m"][0], sample["m"][1], sample["m"][2],)
    packet = pack(PACKET_FORMAT_9DOF, *_packet)
    return packet

  @staticmethod
  def _build_packet_gps(timestamp, sample, org_tag, unit_tag):
    """Returns a binary sequence tha represents a GPS sample.

    Kwargs:
      timestamp (int): Epoch timestamp of the GPS sample
      sample (dict): a dictionary representing a GPS sample
      org_tag (int): assigned organization tag the unit belongs to
      unit_tag (int): unit tag of this unit from the ESP32's unique ID
    """
    status = LargaSensorPlatform._evaluate_sample(sample)
    chksm = LargaSensorPlatform._simple_checksum(sample)
    _packet = (
        timestamp, org_tag, unit_tag, PACKET_TYPE_GPS, status, chksm,
        sample["lat"], sample["lng"], sample["alt"], sample["vel"],)
    packet = pack(PACKET_FORMAT_GPS, *_packet)
    return packet

  @staticmethod
  def _build_packet_fuse(timestamp, dof_sample, gps_sample, org_tag, unit_tag):
    """Returns a binary sequence tha represents a 9DOF/GPS sample.

    Kwargs:
      timestamp (int): Epoch timestamp of the 9DOF/GPS sample
      dof_sample (dict): a dictionary representing a 9DOF sample
      gps_sample (dict): a dictionary representing a GSP sample
      org_tag (int): assigned organization tag the unit belongs to
      unit_tag (int): unit tag of this unit from the ESP32's unique ID
    """
    status = LargaSensorPlatform._evaluate_sample(dof_sample)
    chksm = LargaSensorPlatform._simple_checksum(dof_sample)
    _packet = (
        timestamp, org_tag, unit_tag, PACKET_TYPE_FUSION, status, chksm,
        dof_sample["a"][0], dof_sample["a"][1], dof_sample["a"][2],
        dof_sample["g"][0], dof_sample["g"][1], dof_sample["g"][2],
        dof_sample["m"][0], dof_sample["m"][1], dof_sample["m"][2],
        gps_sample["lat"], gps_sample["lng"], gps_sample["alt"],
        gps_sample["vel"],)
    packet = pack(PACKET_FORMAT_FUSION, *_packet)
    return packet

  @staticmethod
  def _evaluate_sample(payload):
    """Returns the result of simple slope evaluation on a set of samples."""
    return STATUS_NORMAL

  @staticmethod
  def _get_internal_temp():
    """Returns the internal temperature of the ESP32 CPU."""
    pass

  @staticmethod
  def _simple_checksum(payload):
    """Return a value that can be used as a simple checksum for data samples."""
    return 1

  @classmethod
  def collect_sample_9dof(cls, retry_limit=3, retry_wait_ms=5):
    """Used in the timer callback for the 9DOF data collection timer,
       __TIMER_9DOF.

    Kwargs:
      retry_limit (int): number of times to retry file creation if failures
          are encountered
      retry_wait_ms (int): amount of time to wait between retries
    """
    timestamp = int(time() * 1000000)
    sample = dict(
        a=cls.__9dof.acceleration, g=cls.__9dof.gyro, m=cls.__9dof.magnetic)
    packet = LargaSensorPlatform._build_packet_9dof(
        timestamp, sample, cls.__org_tag, cls.__unit_tag)
    LargaSensorPlatform._commit_sample(
        packet, cls.__data_path, cls.__log_9dof, retry_limit=retry_limit,
        retry_wait_ms=retry_wait_ms)

  @classmethod
  def collect_sample_gps(cls, retry_limit=3, retry_wait_ms=5):
    """Used in the timer callback for the GPS data collection timer,
       __TIMER_GPS.

    Kwargs:
      retry_limit (int): number of times to retry file creation if failures
          are encountered
      retry_wait_ms (int): amount of time to wait between retries
    """
    timestamp = int(time() * 1000000)
    (_, _lat, _lng, _alt, _, _, _kph, _, _) = cls.__gps.getdata()
    sample = dict(lat=_lat, lng=_lng, alt=_alt, vel=_kph,)
    packet = LargaSensorPlatform._build_packet_gps(
        timestamp, sample, cls.__org_tag, cls.__unit_tag)
    LargaSensorPlatform._commit_sample(
        packet, cls.__data_path, cls.__log_gps, retry_limit=retry_limit,
        retry_wait_ms=retry_wait_ms)

  @classmethod
  def collect_sample_fuse(cls, retry_limit=3, retry_wait_ms=5):
    """Used in the timer callback for the fused data collection timer.

    Kwargs:
      retry_limit (int): number of times to retry file creation if failures
          are encountered
      retry_wait_ms (int): amount of time to wait between retries
    """
    timestamp = int(time() * 1000000)
    dof_sample = dict(
        a=cls.__9dof.acceleration, g=cls.__9dof.gyro, m=cls.__9dof.magnetic)
    (_, _lat, _lng, _alt, _, _, _kph, _, _) = cls.__gps.getdata()
    gps_sample = dict(lat=_lat, lng=_lng, alt=_alt, vel=_kph,)
    packet = LargaSensorPlatform._build_packet_fuse(
        timestamp, dof_sample, gps_sample, cls.__org_tag, cls.__unit_tag)
    LargaSensorPlatform._commit_sample(
        packet, cls.__data_path, cls.__log_fuse, retry_limit=retry_limit,
        retry_wait_ms=retry_wait_ms)

  @classmethod
  def migrate_logs(cls, retry_limit=3, retry_wait_ms=5):
    """Used in the timer callback for the data migration timer,
       __TIMER_MIGRATE.

    Kwargs:
      retry_limit (int): number of times to retry file creation if failures
          are encountered
      retry_wait_ms (int): amount of time to wait between retries
    """
    status = migrate(
        source_dir=cls.__data_path,
        target_dir=cls.__migrate_path,
        retry_limit=retry_limit, retry_wait_ms=retry_wait_ms)
    if not status:
      # tools.log_FATAL("File migration failed. Restarting device.")
      print("FATAL: File migration failed. Restarting device.")
      # self.stop()
      sleep_ms(5000)
      machine.reset()

  def _start_timers(self):
    """Starts the data collection timers."""
    # tools.printmesg("INFO: Starting main event loop.")
    print("INFO: Starting data collections.")
    if self.enable_fuse:
      print("Starting fused 9dof and gps data collection.")
      __TIMER_9DOF.init(
          period=self.sample_interval_fuse_msec,
          mode=machine.Timer.PERIODIC,
          callback=TASK_CollectSample_Fusion)
    else:
      if self.enable_9dof:
        print("INFO: Starting 9dof data collection.")
        # launch 9dof data collection timer
        __TIMER_9DOF.init(
            period=self.sample_interval_9dof_msec,
            mode=machine.Timer.PERIODIC,
            callback=TASK_CollectSample_9DOF)
      if self.enable_gps:
        print("INFO: Starting gps data collection.")
        # launch gps data collection timer
        __TIMER_GPS.init(
            period=self.sample_interval_gps_msec,
            mode=machine.Timer.PERIODIC,
            callback=TASK_CollectSample_GPS)
    if self.enable_migrate:
      print("INFO: Starting data migration.")
      # launch data migration timer
      __TIMER_MIGRATE.init(
          period=self.migrate_interval_msec,
          mode=machine.Timer.PERIODIC,
          callback=TASK_MigrateLogs)

  def start(self, retry_limit=3, retry_wait_ms=5):
    """Driver entry point."""
    if filemgmt.mountsd():
      sleep_ms(2000)
      # check to see if the source and target data migration folders exist
      if  filemgmt.check_dir(self.__class__.__data_path) \
          and filemgmt.check_dir(self.__class__.__migrate_path):
        # move any file found in /flash/__DATA__ to /sd/__DATA__
        if self.enable_migrate:
          mesg = "INFO: Migrating any existing files in {0} to {1}."
          tools.printmesg(mesg.format(
              self.__class__.__data_path, self.__class__.__migrate_path))
          filemgmt.migrate_all()
        # initialize lcd display
        self.start_lcd()
        # initialize led (neopixel) bar
        self.start_ledbar()
        # display the splash image
        self.display_splash()
        if self.enable_gps:
          # disable MQTT and WLAN services
          if not self.enable_network:
            self.stop_network()
            sleep_ms(1200)
          # initialize gps and seed real-time clock from gps data
          self.start_gps(as_service=True, seed_time=True)
          sleep_ms(1200)
          # create initial gps log file
          if not self.enable_fuse:
            self.__class__.__log_gps = self._create_initial_log(
                self.__class__.__data_path, self.__class__.__log_gps,
                retry_limit=retry_limit, retry_wait_ms=retry_wait_ms)
        else:
          # seed real-time clock from global hst hosts;
          # requires a WLAN connection
          tools.set_time_fqdn()
        if self.enable_9dof:
          # initialize 9dof sensor
          self.start_9dof()
          sleep_ms(1200)
          # create initial 9dof log file
          if not self.enable_fuse:
            self.__class__.__log_9dof = self._create_initial_log(
                self.__class__.__data_path, self.__class__.__log_9dof,
                retry_limit=retry_limit, retry_wait_ms=retry_wait_ms)
        if self.enable_fuse:
          self.__class__.__log_fuse = self._create_initial_log(
              self.__class__.__data_path, self.__class__.__log_fuse,
              retry_limit=retry_limit, retry_wait_ms=retry_wait_ms)
        # launch data collection timers
        # create log entry when starting driver services
        tools.log_START("Starting LARGA driver.")
        self._start_timers()
        self.activate_ledbar()
        # tools.printmesg("INFO: Initialization complete. LARGA driver started.")
        print("INFO: Initialization complete. LARGA driver started.")
        return True
      else:
        mesg = "Either {0} or {1} directories were not found."
        tools.log_FATAL(mesg.format(filemgmt.SOURCE_DIR, filemgmt.TARGET_DIR))
    return False

  def stop(self):
    """Driver exit point."""
    self.__class__.__lcd.clear()
    # tools.log_STOP("Stopping LARGA driver.")
    print("INFO: Stopping LARGA driver.")
    # tools.printmesg("INFO: Stopping main event loop.")
    print("INFO: Stopping data collections.")
    if self.enable_9dof:
      print("INFO: Stopping 9dof data collection.")
      # stop 9dof data collection timer
      __TIMER_9DOF.deinit()
      sleep_ms(1200)
      # stop the 9DOF sensor and unregister it from the I2C bus
      self.stop_9dof()
      sleep_ms(600)
    if self.enable_gps:
      print("INFO: Stopping gps data collection.")
      # stop gps data collection timer
      __TIMER_GPS.deinit()
      sleep_ms(1200)
      # stop the gps receiver
      self.stop_gps()
      sleep_ms(600)
      if not self.enable_network:
        # restart network stack
        self.start_network()
    # stop data migration timer
    if self.enable_migrate:
      __TIMER_MIGRATE.deinit()
      sleep_ms(1200)
    if self.enable_fuse:
      log_path = "{0}/{1}".format(
          self.__class__.__data_path, self.__class__.__log_fuse.name)
      try:
        os_rename(log_path, "{0}.dat".format(log_path))
      except OSError:
        print("WARN: Could not append file extension to {0}.".format(log_path))
    if self.enable_migrate:
      # move all remaining files in /flash/__DATA__ into /sd/__DATA__
      mesg = "INFO: Migrating any existing files in {0} to {1}."
      # tools.printmesg(mesg.format(filemgmt.SOURCE_DIR, filemgmt.TARGET_DIR))
      print(mesg.format(
          self.__class__.__data_path, self.__class__.__migrate_path))
      filemgmt.migrate_all()
    # power down led bar
    self.deactivate_ledbar()
    # stop neopixel device and free all used resources
    self.stop_ledbar()
    self.__class__.__lcd.cler()
    tools.log_STOP("Stopped LARGA driver.")
    tools.printmesg("INFO: Shutdown complete. LARGA driver stopped.")
    return True
