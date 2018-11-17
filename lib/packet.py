
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

PACKET_TYPE_9DOF = 4

PACKET_FORMAT_9DOF = "QLLBBLfffffffff"

PACKET_SIZE_9DOF_B = 60

FIELD_NAMES_9DOF = (
    "timestamp_us", "org_tag", "unit_tag", "packet_type", "status", "chksum",
    "accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z",
    "mag_x", "mag_y", "mag_z")

Packet9DOF = namedtuple("Packet9DOF", FIELD_NAMES_9DOF)


PACKET_TYPE_GPS = 8

PACKET_FORMAT_GPS = "QLLBBLdddd"

PACKET_SIZE_GPS_B = 56

FIELD_NAMES_GPS = (
    "timestamp_us", "org_tag", "unit_tag", "packet_type", "status", "chksum",
    "lat_deg", "long_deg", "alt_m", "vel")

PacketGPS = namedtuple("PacketGPS", FIELD_NAMES_GPS)


PACKET_TYPE_FUSION = 16

PACKET_FORMAT_FUSION = "QLLBBLfffffffffdddd"

PACKET_SIZE_FUSION_B = 96

FIELD_NAMES_FUSION = (
    "timestamp_us", "org_tag", "unit_tag", "packet_type", "status", "chksum",
    "accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z",
    "mag_x", "mag_y", "mag_z", "lat_deg", "long_deg", "alt_m", "vel")

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
            sample = struct.unpack_from(packet_format, _buffer)
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


def unpack_logs(log_dir, log_names=None, as_tuple=False):
  unpacked = dict()
  if not log_names:
    log_names = os.listdir(log_dir)
  for log_name in log_names:
    log_path = os.path.join(log_dir, log_name)
    if log_name.find("_FUSE_") >= 0:
      samples = unpack_fusion_log(log_path, as_tuple=as_tuple)
      log_type = "fusion"
    elif log_name.find("_9DOF_") >= 0:
      samples = unpack_9dof_log(log_path, as_tuple=as_tuple)
      log_type = "dof"
    elif log_name.find("_GPS_") >= 0:
      samples = unpack_gps_log(log_path, as_tuple=as_tuple)
      log_type = "gps"
    else:
      continue
    if samples:
      if log_type not in unpacked:
        unpacked[log_type] = deque()
      unpacked[log_type].extend(samples)
  return unpacked


def unpack_to_csv(log_dir, log_names=None, csv_dir=None, csv_file=None):
  timestamp = int(time.time() * 1000000)
  unpacked = unpack_logs(log_dir, log_names=log_names, as_tuple=False)
  csv_paths = list()
  for (log_type, samples) in unpacked.items():
    if samples:
      if log_type == "fusion":
        column_names = FIELD_NAMES_FUSION
      elif log_type == "dof":
        column_names = FIELD_NAMES_9DOF
      elif log_type == "gps":
        columns_names = FIELD_NAMES_GPS
      else:
        continue
      if not csv_dir:
        csv_dir = log_dir
      if not csv_file:
        csv_file = "{0}_{1}.csv".format(timestamp, log_type.upper())
      csv_path = os.path.join(csv_dir, csv_file)
      csv_paths.append(csv_path)
      with open(csv_path, "w") as fh:
        column_names = ",".join(list(column_names))
        fh.write(column_names + "\n")
        for sample in samples:
          sample = ",".join([str(value) for value in sample])
          fh.write(sample + "\n")
  return tuple(csv_paths)


def unpack_to_dframe(log_dir, log_names=None):
  if PANDAS_AVAILABLE:
    unpacked = unpack_logs(log_dir, log_names=log_names, as_tuple=False)
    for (log_type, samples) in unpacked.items():
      if samples:
        if log_type == "fusion":
          _df = pd.DataFrame(list(samples), columns=FIELD_NAMES_FUSION)
        elif log_type == "dof":
          _df = pd.DataFrame(list(samples), columns=FIELD_NAMES_9DOF)
        elif log_type == "gps":
          _df = pd.DataFrame(list(samples), columns=FIELD_NAMES_GPS)
        else:
          continue
        if not _df.empty and "timestamp_us" in _df:
          _df.loc[:, "datetime"] = pd.to_datetime(_df["timestamp_us"],
                                                  unit="us")
          _df = _df.set_index("datetime").sort_index()
        unpacked[log_type] = _df
    return unpacked
  return None
