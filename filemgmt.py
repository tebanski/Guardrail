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

# import gc
import os
import uio
import utils

from time import sleep_ms

import tools


SOURCE_DIR = "/flash/__DATA__"
LOG_DIR = "/flash/__LOG__"
TARGET_DIR = "/sd/__DATA__"

# number of consecutive times that a file migration has occurred
MIGRATE_RETRY_COUNT = 0
# number of consecutive times that retries can repeat before a failure
MIGRATE_RETRY_LIMIT = 3
# number of times to retry file creation when if fails
CREATE_RETRY_LIMIT = 3



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
      print("FATAL: Cannot mount SD card.")
    else:
      ret_val = True
  else:
    ret_val = True
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


def migrate_file(source_dir, target_dir, filename):
  source_path = "{0}/{1}".format(source_dir, filename)
  target_path = "{0}/{1}".format(target_dir, filename)
  is_migrate_complete = False
  
  is_copy_complete = False
  mesg = "INFO: Copying data from {0} to {1}." 
  print(mesg.format(source_path, target_path))
  try:
    utils.filecp(source_path, target_path)
  except OSError:
    is_copy_complete = False
    _register_migrate_retry(target_path)
    mesg = "Failed to complete data copy from {0} to {1}."
    # tools.log_WARN(mesg.format(source_path, target_path))
    print("WARN: " + mesg.format(source_path, target_path))
  else:
    is_copy_complete = True

  if is_copy_complete:
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
  return is_migrate_complete


def migrate(source_dir=SOURCE_DIR, target_dir=TARGET_DIR, all_files=False,
            retry_wait_ms=5, file_ext=".dat"):
  """Called periodically to migrate files from flash to SD. Do not move the
     last file because it may still be open and actively being written to by
     the data collectors.
  """
  is_success = True
  for (filename, _, _) in os.ilistdir(source_dir):
    if not all_files and not filename.endswith(file_ext):
      continue
    for retry_count in range(MIGRATE_RETRY_LIMIT):
      is_success = migrate_file(source_dir, target_dir, filename)
      if is_success:
        break
      else:
        sleep_ms(retry_wait_ms)
        continue
  return is_success


def migrate_all():
  """Called during driver startup to mount the SD card, check the source and
     target directories, and transfer any files in /flash/__DATA__ before
     starting the main event loop.
  """
  if mountsd():
    if check_dir(SOURCE_DIR) and check_dir(TARGET_DIR):
      migrate(source_dir=SOURCE_DIR, target_dir=TARGET_DIR, all_files=True)
      if check_dir(LOG_DIR):
        migrate(source_dir=LOG_DIR, target_dir=TARGET_DIR, all_files=True)


def purge(dir_path):
  for (filename, _, _) in os.ilistdir(dir_path):
    try:  
      os.remove("{0}/{1}".format(dir_path, filename))
    except:
      mesg = "Failed to remove {0}/{1}."
      # tools.log_WARN(mesg.format(dir_path, filename))
      print("WARN: " + mesg.format(dir_path, filename))
