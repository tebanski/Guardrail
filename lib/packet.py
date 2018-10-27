
import os
import struct
import time

from collections import deque, namedtuple

try:
  import pandas as pd
except ImportError:
  PANDAS_AVAILABLE = False
else:
  PANDAS_AVAILABLE = True


# PACKET_FORMAT = "QLLBBffffffffffff"
# PACKET_SIZE_B = 18 + 46 + 2

HEADER_SIZE_B = 22

TAIL_SIZE_B = 2


PACKET_TYPE_9DOF = 4

PACKET_FORMAT_9DOF = "QLLBBLfffffffff"

PACKET_SIZE_9DOF_B = HEADER_SIZE_B + 36 + TAIL_SIZE_B

FIELD_NAMES_9DOF = (
    "timestamp_us", "org_tag", "unit_tag", "packet_type", "status", "chksum",
    "accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z",
    "mag_x", "mag_y", "mag_x")

Packet9DOF = namedtuple("Packet9DOF", FIELD_NAMES_9DOF)


PACKET_TYPE_GPS = 8

PACKET_FORMAT_GPS = "QLLBBLdddd"

PACKET_SIZE_GPS_B = HEADER_SIZE_B + 32 + TAIL_SIZE_B

FIELD_NAMES_GPS = (
    "timestamp_us", "org_tag", "unit_tag", "packet_type", "status", "chksum",
    "lat_deg", "long_deg", "alt_m", "vel")

PacketGPS = namedtuple("PacketGPS", FIELD_NAMES_GPS)


PACKET_TYPE_FUSION = 16

PACKET_FORMAT_FUSION = "QLLBBLfffffffffdddd"

PACKET_SIZE_FUSION_B = HEADER_SIZE_B + 68 + TAIL_SIZE_B

FIELD_NAMES_FUSION = (
    "timestamp_us", "org_tag", "unit_tag", "packet_type", "status", "chksum",
    "accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z",
    "mag_x", "mag_y", "mag_x", "lat_deg", "long_deg", "alt_m", "vel")

PacketFusion = namedtuple("PacketFusion", FIELD_NAMES_FUSION)


def unpack_log(file_path, packet_format, packet_size,
               as_tuple=False, template=None):
  samples = deque()
  if file_path and os.path.exists(file_path):
    with open(file_path, "rb") as fh:
      while True:
        _buffer = fh.read(packet_size)
        if _buffer:
          try:
            sample = srtuct.unpack_from(packet_format, _buffer)
          except:
            continue
          else:
            if as_tuple and template:
              sample = template(*sample)
            samples.append(sample)
        else:
          break
  return samples


def unpack_9dof_log(file_path, as_tuple=False):
  if file_paths.find("_9DOF") >= 0:
    return unpack_log(
        file_path, PACKET_FORMAT_9DOF, PACKET_SIZE_9DOF_B, as_tuple=as_tuple,
        template=Packet9DOF)
  return None


def unpack_gps_log(file_path, as_tuple=False):
  if file_path.find("_GPS") >= 0:
    return umpack_log(
        file_path, PACKET_FORMAT_GPS, PACKET_SIZE_GPS_B, as_tuple=as_tuple,
        template=PacketGPS)
  return None


def unpack_fusion_log(file_path, as_tuple=False):
  if file_path.find("_FUSE") >= 0:
    return unpack_log(
        file_path, PACKET_FORMAT_FUSION, PACKET_SIZE_FUSION_B,
        as_tuple=as_tuple, template=PacketFusion)
  return None


def unpack_logs(dir_path, file_type=None, as_tuple=False, consolidate=True):
  unpacked = deque()
  if file_type.lower() in ("9dof", "dof",):
    funcs = (unpack_9dof_log,)
    fn_names = ("9dof", )
  elif file_type.lower() in ("gps",):
    funcs = (unpack_gps_log,)
    fn_names = ("gps",)
  elif file_type.lower() in ("fusion", "fuse",):
    funcs = (unpack_fusion_log,)
    fn_names = ("fusion",)
  else:
    if not consolidate:
      unpacked = dict(dof=deque(), gps=deque(), fusion=deque())
    funcs = (unpack_9dof_log, unpack_gps_log, unpack_fusion_log,)
    fn_names = ("dof", "gps", "fusion")
  for file_name in os.listdir(dir_path):
    file_path = os.path.join(dir_path, file_name)
    for (fn_name, func) in zip(fn_names, funcs):
      samples = func(file_path, as_tuple=as_tuple)
      if samples:
        if consolidate:
          unpacked.extend(samples)
        else:
          unpacked[fn_name].extend(samples)
        continue
  return unpacked


def unpack_to_csv(dir_path, csv_path):
  timestamp = int(time.time() * 1000000)
  unpacked = unpack_logs(
      dir_path, file_type=None, as_tuple=False, consolidate=False)
  for (file_type, samples) in unpacked.items():
    if samples:
      if file_type == "dof":
        column_names = Packet9DOF._fields
      elif file_type == "gps":
        columns_names = PacketGPS._fields
      elif file_type == "fusion":
        column_names = PacketFusion._fields
      file_name = "{0}_{1}.csv".format(timestamp, file_type)
      file_path = os.path.join(csv_path, file_name)
      with open(file_path, "w") as fh:
        column_names = ",".join(list(column_names))
        fh.write(column_names + "\n")
        for sample in samples:
          sample = ",".join([str(value) for value in sample])
          fh.write(sample, + "\n")


def unpack_to_dframe(dir_path):
  if PANDAS_AVAILABLE:
    unpacked = unpack_logs(
        dir_path, file_type=None, as_tuple=False, consolidate=False)
    for (file_type, samples) in unpacked.items():
      if samples:
        if file_type == "dof":
          column_names = Packet9DOF._fields
        elif file_type == "gps":
          columns_names = PacketGPS._fields
        elif file_type == "fusion":
          column_names = PacketFusion._fields
        _df = pd.DataFrame(samples, columns=column_names)
        _df.loc[:, "timestamp"] = pd.to_datetime(_df["timestamp_us"])
        _df = _df.set_index("timestamp").sort_index()
        unpacked[file_type] = _pd
    return unpacked
  return None
