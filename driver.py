# Proof-of-Concept for Larga vehicle telemetry
# Copyright (c) October 2018 Steven Yap
#
# Licenced under the GNU LGLP v2 license:
#     https://www.gnu.org/licenses/old-licenses/lgpl-2.1.en.html
#
# Based on examples from Mike Tuupola:
#     https://github.com/tuupola/micropython-examples
#


import gc
import micropython
import uos

from micropython import const
from machine import Timer
from ucollections import deque, namedtuple
from ustruct import pack
from m5stack import lcd, time
from mpu6050_fire import MPU6050
from fusion import Fusion


# display readings on M5STACK LCD screen
DISPLAY_READINGS = True

# log the readings in the SD card
LOG_READINGS = False

# toggle to use cooked orientation readings from 9DoF sensor:
ORIENT_COOKED = False

# font to use for LCD display
DISPLAY_FONT = lcd.FONT_Default

# type of Larga packet generated by this device
PACKET_TYPE = const(4)

# number of msecs to sleep between each DOF and GPS readings
SAMPLE_INTERVAL = const(200)

# path where to write data files to
DATA_PATH = "/sd/__DATA__"

# format to use for packing data for storage and transmission
PACK_FORMAT = "QLLBBdddddddddddd"

# maximum number of samples per data file
MAX_ENTRIES_PER_FILE = 600 * (1000 / SAMPLE_INTERVAL)

# number of samples to consider when checking conditions
CRITICAL_SAMPLES = const(20)

# dictionary that holds the last CONDITION_WINDOW samples
CRITICAL_WINDOW = dict(
    accel_x=deque((), CRITICAL_SAMPLES),  # x-axis linear acceleration
    accel_y=deque((), CRITICAL_SAMPLES),  # y-axis linear acceleration
    accel_z=deque((), CRITICAL_SAMPLES),  # z-axis linear acceleration
    anglv_x=deque((), CRITICAL_SAMPLES),  # x-axis angular velocity
    anglv_y=deque((), CRITICAL_SAMPLES),  # y-axis angular velocity
    anglv_z=deque((), CRITICAL_SAMPLES),  # z-axis angular velocity
    mag_x=deque((), CRITICAL_SAMPLES),  # x-axis compas reading,
    mag_y=deque((), CRITICAL_SAMPLES),  # y-axis compas reading
    mag_z=deque((), CRITICAL_SAMPLES),  # z-axis compas reading
    deg_long=deque((), CRITICAL_SAMPLES),  # gps longtitude in degrees
    deg_lat=deque((), CRITICAL_SAMPLES), # gps latitude in degrees
    speed=deque((), CRITICAL_SAMPLES)
  )

# field order for critical samples
CRITICAL_FIELDS = (
    "accel_x", "accel_y", "accel_z", "anglv_x", "anglv_y", "anglv_z"
    "mag_x", "mag_y", "mag_z", "deg_lat", "deg_long", "speed")

# field order in in data packet
PACKET_FIELDS = (
    "timestamp", "org_tag", "unit_tag", "packet_type", "status",
    "accel_x", "accel_y", "accel_z", "anglv_x", "anglv_y", "anglv_z"
    "mag_x", "mag_y", "mag_z", "deg_lat", "deg_long", "speed")

GRpacket = namedtuple("GRpacket", PACKET_FIELDS)

# breakpoints for evaluationg readings (WARN, DANGER, IMPACT)
ACCELETRATION_SLOPES = (0.57735, 1.0, 1.73205)

# acceleration burst duration in number of samples that may indicate an impact
ACCELERATION_IMPACT_WINDOW = const(4)

# acceleration peak in m/s^2 during burst duration that may indicate an impact
ACCELERATION_BURST_PEAK = 1.5

# report status when transmitting data sets
STATUS_NORMAL = const(0)
STATUS_WARN = const(1)
STATUS_DANGER = const(2)
STATUS_IMPACT = const(4)
STATUS_FALL = const(8)
STATUS_ROLL = const(16)

# allocate buffer for emergency exceptions
micropython.alloc_emergency_exception_buf(100)

if LOG_READINGS:
  # mount SD card
  uos.mountsd()
  # file handle for current data file
  fh = create_data_file()
  # number of entries in the current data file
  file_entries = 0

# attach to 9DoF module
sensor = MPU6050()

# create data_packet object for sample readings
data_packet = None


def get_unit_tag(file_path="unit.tag"):
  # short-circuit for now
  return 1


def get_org_tag(file_path="org.tag"):
  # short-circuit for now
  return 1


def display_header():
  lcd.clear()
  lcd.font(DISPLAY_FONT)
  lcd.text(60, 20, "X-ACCEL:")
  lcd.text(60, 40, "Y-ACCEL:")
  lcd.text(60, 60, "Z-ACCEL:")
  if ORIENT_COOKED:
    lcd.text(60, 80, "HEADING:")
    lcd.text(88, 100, "PITCH:")
    lcd.text(102, 120, "ROLL:")
  else:
    lcd.text(88, 80, "X-ANGV:")
    lcd.text(88, 100, "Y-ANGV:")
    lcd.text(88, 120, "Z-ANGV:")
  lcd.text(102, 140, "LATI:")
  lcd.text(102, 160, "LONG:")
  lcd.text(88, 180, "SPEED:")


def clear_data_dir(dir_path=DATA_PATH):
  pass


def create_data_file(dir_path=DATA_PATH, fh=None):
  file_path = "{0}/{1}.dat".format(dir_path, utime.time())
  if fh:
    fh.close()
  return uio.open(file_path, "wb")


def collect_readings(timer):
  timestamp = time.time() * 1000000
  sensor_readings = _collect_9dof(sensor)
  # receiver_readings = _collect_gps(gps, receiver)
  data_packet = _build_packet(timestamp, sensor_readings, receiver_readings)
  _push_packet(data_packet)
  gc.mem_free()


def _collect_9dof(sensor):
  # acceleration = sensor.acceleration
  # gyro = sensor.orientation
  # magnetic = sensor.direction
  # if ORIENT_COOKED:
  #   acc.update(acceleration, gyro, magnetic)
  #   gyro = (acc.heading, acc.pitch, acc.roll)
  return dict(a=sensor.acceleration, g=sensor.orientation, m=sensor.magnetic)


def _collect_gps(gps, receiver):
  if receiver:
    _ = [gps.update(chr(char)) for sentence in receiver.sentences()
         for char in sentences]
    return (gps.latitude, gps.longtitude, gps.speed)
  return (None, None, None)


def _build_packet(timestamp, dof_sensor, gps_receiver):
  status = evaluate_readings()
  packet = GRPacket(
        timestamp, get_org_tag(), get_unit_tag(), PACKET_TYPE, status,
        dof_sensor["a"][0], dof_sensor["a"][1], dof_sensor["a"][2],
        dof_sensor["g"][0], dof_sensor["g"][1], dof_sensor["g"][2],
        dof_sensor["m"][0], dof_sensor["m"][1], dof_sensor["m"][2],
        gps_receiver[0], gps_receiver[1], gps_receiver[2])
  return packet


  def _push_packet(packet, field_names=CRITICAL_FIELDS):
    _packet = packet._asdict()
    _ = [CRITICAL_WINDOW[field_name].append(_packet[field_name])
         for field_name in field_names]


def display_readings():
  accelerometer = sensor.acceleration
  lcd.text(180, 20, "{:3.4f}\r".format(data_packet[4]))
  lcd.text(180, 40, "{:3.4f}\r".format(data_packet[5]))
  lcd.text(180, 80, "{:3.4f}\f".format(data_packet[6]))
  lcd.text(180, 80, "{:3.4f}\r".format(data_packet[7]))
  lcd.text(180, 100, "{:3.4f}\r".format(data_packet[8]))
  lcd.text(180, 120, "{:3.4f}\r".format(data_packet[9]))
  lcd.text(180, 140, "{:3.4f}\r".format(data_packet[13]))
  lcd.text(180, 160, "{:3.4f}\r".format(data_packet[14]))
  lcd.text(180, 180, "{:3.4f}\r".format(data_packet[15]))


def store_readings():
  if file_entries >= MAX_ENTRIES_PER_FILE:
    fh = create_data_file(fh=fh)
    file_entries = 0
  fh.write(pack(PACK_FORMAT, data_packet))
  file_entries += 1


def evaluate_readings():
  return STATUS_NORMAL


def transmit_readings():
  pass


clear_data_dir()
display_header()
# initialize the display LCD
lcd.init(
  lcd.M5STACK, width=240, height=320, rst_pin=33, backl_pin=32, miso=19,
  mosi=23, clk=18, cs=14, dc=27, bgr=True, backl_on=1)

timer_0 = Timer(0)
timer_0.init(
  period=SAMPLE_INTERVAL, mode=Timer.PERIODIC, callback=collect_readings)

if LOG_READINGS:
  timer_1 = Timer(1)
  timer_1.init(
    period=SAMPLE_INTERVAL, mode=Timer.PERIODIC, callback=store_readings)

if DISPLAY_READINGS:
  timer_4 = Timer(4)
  timer_4.init(
    period=SAMPLE_INTERVAL, mode=Timer.PERIODIC, callback=display_readings)

# timer_6 = Timer(6)
# timer_6.init(
#   period=600000, mode=Timer.PERIODIC, callback=transmit_readings)
