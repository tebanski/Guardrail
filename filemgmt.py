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

from utime import time


SOURCE_DIR = "/flash/__DATA__"
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
    print(
        "WARN: Migrate retry limit {0} ".format(MIGRATE_RETRY_COUNT) + \
        "for file {0} has been reached.".format(filename))


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
    os.listdir("/sd")
  except OSError:
    try:
      os.mountsd()
    except OSError:
      print("FATAL: Cannot mount SD card.")
    else:
      ret_val = True
  else:
    ret_val = True
  return ret_val


def check_dir(dir_path):
  ret_val = False
  try:
    os.listdir(dir_path)
  except OSError:
    try:
      os.mkdir(dir_path)
    except OSError:
      print(
          "FATAL: Could not create directory {}.".format(dir_path))
  else:
    ret_val = True
  return ret_val


def migrate_file(source_dir, target_dir, filename):
  source_path = "{}/{}".format(source_dir, filename)
  target_path = "{}/{}".format(target_dir, filename)
  is_migrate_complete = False
  is_copy_complete = True
  with open(source_path, "rb") as fh_s:
    with open(target_path, "wb") as fh_t:
      print(
          "INFO: Copying data from {0} to {1}.".format(
            source_path, target_path))
      for line in fh_s:
        try:
          fh_t.write(line)
          fh_t.flush()
        except OSError:
          is_copy_complete = False
          _register_migrate_retry(target_path)
          mesg = "WARN: Failed to complete data copy from {0} to {1}."
          print(mesg.format(source_path, target_path))
          break
  if is_copy_complete:
    try:
      os.remove(source_path)
    except OSError:
      _register_migrate_retry(source_path)
      print(
          "WARN: Failed to remove source file {0}.".format(source_path))
    else:
      _reset_migrate_retry()
      print("INFO: Removed source file {0}.".format(source_path))
      is_migrate_complete = True
  return is_migrate_complete


def migrate(source_dir=SOURCE_DIR, target_dir=TARGET_DIR, all_files=False,
            retry_wait_ms=5, file_ext=".dat"):
  # Called periodically to migrate files from flash to SD.
  # Do not move the last file because it may still be open and
  # actively being written to by the data collectors.
  is_success = False
  if all_files:
    filenames = sorted(os.listdir(source_dir))
  else:
    filenames = sorted([fname for fname in os.listdir(source_dir)
                        if fname.endswith(file_ext)])
  if not len(filenames):
    return True
  for filename in filenames:
    for retry_count in range(MIGRATE_RETRY_LIMIT):
      is_success = migrate_file(source_dir, target_dir, filename)
      if is_success:
        break
      else:
        time.sleep_ms(retry_wait_ms)
        continue
    # if not status:
    #   break
  # do some garbage collection on the side
  # gc.collect()
  return is_success


def migrate_all():
  # Called during driver startup to mount the SD card, check
  # the source and target directories, and transfer any files in
  # /flash/__DATA__ before starting the main event loop.
  if mountsd():
    if check_dir(SOURCE_DIR) and check_dir(TARGET_DIR):
      migrate(source_dir=SOURCE_DIR, target_dir=TARGET_DIR, all_files=True)


def purge(dir_path):
  for filename in os.listdir(dir_path):
    try:  
      os.remove("{}/{}".format(dir_path, filename))
    except:
      print("WARN: Failed to remove {}/{}.".format(dir_path, filename))
