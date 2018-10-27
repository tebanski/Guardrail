# LARGA (Guardrail)
LARGA (Guardrail) Vehicle Sensor Platform

To use, flash the following Micropython build on the M5STACK:

["LoboRis MicroPython for ESP32"] (https://github.com/loboris/MicroPython_ESP32_psRAM_LoBo/tree/master/MicroPython_BUILD/firmware/esp32_all)

NOTE: This build is not configured for OTA updates and the network stack isn't enabled by default.

From REPL, do:

```python
import driver
lsp = driver.LargaSensorPlatform(...)
lsp.start()
```
## Valid arguments to LargaSensorPlatform():
```python
"""
  enable_9dof (bool): enables/disables 9dof sensor operation on startup
  enable_gps (bool): enabled/disables GPS receiver operation on startup
  enable_fuse (bool): enables/disables fused data collection for both
      9dof sensor and GPS receiver
  enable_network (bool): enables/disables network services on startup
  enable_migrate (bool): enables data log migration from flash to SD card
  log_to_sd (bool): enabled/disables direct-to-SD device logging;
      automatically disables data log migration
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
```
