"""
This module manages files on flash storage and the SD card.
Currently, it allows files to be moved from /flash/__DATA__
to /sd/__DATA__, assuming that a properly-formatted microSD card
is available. This needs to  be rxecuted periodically so avoid
filling up flash storage when accelerometer and GPS data collections
are active.

author: Steven Y. Yap
contact: tebanski@gmail.com
version: 0.01
modified: 2018.10.9
"""

import gc
import os
import machine
from time import sleep_ms
# from upysh import cp as filecp

# import utils

import tools


SOURCE_DIR = "/flash/__DATA__"
LOG_DIR = "/flash/__LOG__"
TARGET_DIR = "/sd/__DATA__"

# number of consecutive times that a file migration has occurred
MIGRATE_RETRY_COUNT = 0
# number of consecutive times that retries can repeat before a failure
MIGRATE_RETRY_LIMIT = 5


def _register_migrate_retry(filename=None):
  global MIGRATE_RETRY_COUNT

  MIGRATE_RETRY_COUNT += 1
  if MIGRATE_RETRY_COUNT >= MIGRATE_RETRY_LIMIT:
    mesg = "Migrate retry limit {0} for file {1} has been reached."
    # tools.log_WARN(mesg.format(MIGRATE_RETRY_COUNT, filename))
    print("WARN: " + mesg.format(MIGRATE_RETRY_COUNT, filename))


def _deregister_migrate_retry():
  global MIGRATE_RETRY_COUNT

  if MIGRATE_RETRY_COUNT > 0:
    MIGRATE_RETRY_COUNT -= 1


def _reset_migrate_retry():
  global MIGRATE_RETRY_COUNT

  MIGRATE_RETRY_COUNT = 0


def mountsd():
  ret_val = False
  try:
    _ = os.ilistdir("/sd")
  except OSError:
    try:
      os.mountsd()
    except OSError:
      # tools.log_FATAL("Cannot mount SD card.")
      print("FATAL: Cannot mount SD card device.")
    else:
      ret_val = True
  else:
    ret_val = True
  if ret_val:
    print("INFO: SD card device mounted on /sd.")
  return ret_val


def check_dir(dir_path):
  ret_val = False
  try:
    _ = os.ilistdir(dir_path)
  except OSError:
    try:
      os.mkdir(dir_path)
    except OSError:
      # tools.log_FATAL("Could not create directory {0}.".format(dir_path))
      print("FATAL: Could not create directory {0}.".format(dir_path))
  else:
    ret_val = True
  return ret_val


def fcopy(source_path, target_path, blocksize=4096, retry_limit=3,
          retry_wait_ms=5):
  is_complete = False
  with open(source_path, 'rb') as sfh:
    with open(target_path, 'wb') as tfh:
      for retry_count in range(retry_limit):
        try:
          while tfh.write(sfh.read(blocksize)):
            pass
        except MemoryError:
          sfh.seek(0)
          tfh.seek(0)
          _register_migrate_retry(target_path)
          gc.collect()
          sleep_ms(retry_wait_ms)
        else:
          _reset_migrate_retry()
          is_complete = True
          break
  return is_complete


def migrate_file(source_dir, target_dir, filename, retry_limit=3,
                 retry_wait_ms=5):
  source_path = "{0}/{1}".format(source_dir, filename)
  target_path = "{0}/{1}".format(target_dir, filename)
  is_migrate_complete = False
  mesg = "INFO: Copying data from {0} to {1}." 
  print(mesg.format(source_path, target_path))
  is_copy_complete = False
  for retry_count in range(retry_limit):
    try:
      is_copy_complete = fcopy(
          source_path, target_path, retry_limit=retry_limit,
          retry_wait_ms=retry_wait_ms)
    except OSError:
      _register_migrate_retry(target_path)
      sleep_ms(retry_wait_ms)
      continue
    else:
      _reset_migrate_retry()
    break
  if is_copy_complete:
    sleep_ms(200)
    print("INFO: Removing source file {0}.".format(source_path))
    try:
      os.remove(source_path)
    except OSError:
      _register_migrate_retry(source_path)
      # tools.log_WARN("Failed to remove source file {0}.".format(source_path))
      print("WARN: Failed to remove source file {0}.".format(source_path))
    else:
      _reset_migrate_retry()
      print("INFO: Removed source file {0}.".format(source_path))
      is_migrate_complete = True
  else:
    mesg = "Failed to complete data copy from {0} to {1}."
    # tools.log_WARN(mesg.format(source_path, target_path))
    print("WARN: " + mesg.format(source_path, target_path))
  return is_migrate_complete


def migrate(source_dir=SOURCE_DIR, target_dir=TARGET_DIR, all_files=False,
            retry_limit=3, retry_wait_ms=5, file_ext=".dat",
            reset_on_limit=False):
  """Called periodically to migrate files from flash to SD. Do not move the
     last file because it may still be open and actively being written to by
     the data collectors.
  """
  is_success = True
  with open("{0}/__LOCK__".format(target_dir), "w") as fh_lock:
    for (filename, _, _) in os.ilistdir(source_dir):
      if not all_files and not filename.endswith(file_ext):
        continue
      for retry_count in range(retry_limit):
        is_success = migrate_file(
            source_dir, target_dir, filename, retry_limit=retry_limit,
            retry_wait_ms=retry_wait_ms)
        if is_success:
          break
        else:
          mesg = ("Number of migration retries ({0}) {1} ({2}).{3}")
          if MIGRATE_RETRY_COUNT < MIGRATE_RETRY_LIMIT:
            print("WARN: " + mesg.format(
                MIGRATION_RETRY_COUNT, "/", MIGRATE_RETRY_LIMIT, ""))
          else:
            if not reset_on_limit:
              tools.log_WARN(mesg.format(
                  MIGRATION_RETRY_COUNT, "has exceeded", MIGRATE_RETRY_LIMIT,
                  ""))
            else:
              tools.log_FATAL(mesg=mesg.format(
                  MIGRATION_RETRY_COUNT, "has exceeded", MIGRATE_RETRY_LIMIT,
                  "Resetting device."))
              sleep_ms(5000)
              machine.reset()
          sleep_ms(retry_wait_ms)
          continue
  try:
    os.remove("{0}/__LOCK__".format(target_dir))
  except OSError:
    pass
  return is_success


def migrate_all(retry_limit=3, retry_wait_ms=5, reset_on_limit=False):
  """Called during driver startup to mount the SD card, check the source and
     target directories, and transfer any files in /flash/__DATA__ before
     starting the main event loop.
  """
  if mountsd():
    if check_dir(SOURCE_DIR) and check_dir(TARGET_DIR):
      migrate(source_dir=SOURCE_DIR, target_dir=TARGET_DIR, all_files=True,
              retry_limit=retry_limit, retry_wait_ms=retry_wait_ms,
              reset_on_limit=reset_on_limit)
      if check_dir(LOG_DIR):
        migrate(source_dir=LOG_DIR, target_dir=TARGET_DIR, all_files=True,
                retry_limit=retry_limit, retry_wait_ms=retry_wait_ms,
                reset_on_limit=reset_on_limit)


def purge(dir_path):
  for (filename, _, _) in os.ilistdir(dir_path):
    try:  
      os.remove("{0}/{1}".format(dir_path, filename))
    except:
      mesg = "Failed to remove {0}/{1}."
      # tools.log_WARN(mesg.format(dir_path, filename))
      print("WARN: " + mesg.format(dir_path, filename))
