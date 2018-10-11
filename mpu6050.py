# mpu6050.py MicroPython driver for the InvenSense inertial measurement units
# Adapted from Sebastian Plamauer"s MPU9150 driver:
# https://github.com/micropython-IMU/micropython-mpu9150.git
# Authors Peter Hinch, Sebastian Plamauer
# V0.2 17th May 2017 Platform independent: utime and machine replace pyb
# Last modified by Steven Yap on Oct. 5, 2018.

"""
mpu6050 is a micropython module for the InvenSense MPU6050 sensor. This
particular implentation is baed on the M5STACK FIRE.
The MPU6050 measures acceleration and turn rate in three axis.

The MIT License (MIT)
Copyright (c) 2014 Sebastian Plamauer, oeplse@gmail.com, Peter Hinch
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
"""

# User access is now by properties e.g.
# myimu = MPU9250("X")
# magx = myimu.mag.x
# accelxyz = myimu.accel.xyz
# Error handling: on code used for initialisation, abort with message
# At runtime try to continue returning last good data value. We don"t want
# aircraft crashing. However if the I2C has crashed we"re probably stuffed.


from machine import I2C, Pin
from vector import Vector3d


# constant of acceleration via gravity (in m/sec^2)
Gc = 9.80665


class MPUException(OSError):
    """
    Exception for MPU devices
    """
    pass


def bytes_toint(msb, lsb):
    """
    Convert two bytes to signed integer (big endian)
    for little endian reverse msb, lsb arguments
    Can be used in an interrupt handler
    """
    if not msb & 0x80:
      return msb << 8 | lsb  # +ve
    return - (((msb ^ 255) << 8) | (lsb ^ 255) + 1)


class MPU6050(object):
    """
    Module for InvenSense IMUs. Base class implements MPU6050 6DOF sensor, with
    features common to MPU9150 and MPU9250 9DOF sensors.
    """

    _I2Cerror = "I2C failure when communicating with IMU"
    # addresses of MPU9150/MPU6050. There can be two devices
    _mpu_addr = (104, 105)
    # response to WHO_AM_I query
    _chip_id = 0x68

    def __init__(self, side_str="", device_addr=None, transposition=(0, 1, 2),
                 scaling=(1, 1, 1)):
        self._accel = Vector3d(transposition, scaling, self._accel_callback)
        self._gyro = Vector3d(transposition, scaling, self._gyro_callback)
        # Pre-allocated buffers for reads: allows reads to be done in
        # interrupt handlers
        self.buf1 = bytearray(1)
        self.buf2 = bytearray(2) 
        self.buf3 = bytearray(3)
        self.buf6 = bytearray(6)
        # ---------------------------------------------------------------------
        # # Ensure PSU and device have settled
        # sleep_ms(200)
        # # Non-pyb targets may use other than X or Y
        # if isinstance(side_str, str):
        #     self._mpu_i2c = I2C(side_str)
        # # Soft or hard I2C instance. See issue #3097
        # elif hasattr(side_str, "readfrom"):
        #     self._mpu_i2c = side_str
        # else:
        #     raise ValueError("Invalid I2C instance")
        # ---------------------------------------------------------------------
        self._mpu_i2c = I2C(sda=Pin(21), scl=Pin(22), freq=400000)
        if device_addr is None:
            devices = set(self._mpu_i2c.scan())
            mpus = devices.intersection(set(self._mpu_addr))
            number_of_mpus = len(mpus)
            if number_of_mpus == 0:
                raise MPUException(
                    "FATAL: No MPUs detected.")
            elif number_of_mpus == 1:
                self.mpu_addr = mpus.pop()
            else:
                raise ValueError(
                    "FATAL: Two MPUs detected: must specify a device address.")
        else:
            if device_addr not in (0, 1):
                raise ValueError(
                    "FATAL: Device address must be 0 or 1.")
            self.mpu_addr = self._mpu_addr[device_addr]
        # Test communication by reading chip_id: throws exception on error
        self.chip_id                     
        # Now that we can communicate with chip, set it up.
        # Wake it up.
        self.wake()
        # Enable mag access from main I2C bus.
        self.passthrough = True
        # Default to highest sensitivity for accelerometer.
        self.accel_range = 0
        # Default to highest sensitivity for gyroscope.
        self.gyro_range = 0
        # fast filtered response
        self.accel_filter_range = 0
        self.gyro_filter_range = 0

    def _read(self, buf, memaddr, addr):
        """
        Read bytes to pre-allocated buffer Caller traps OSError.
        Args:
            buf: pre-allocated buffer where results are read into
            memaddr: memory location (i.e., register) within the I2C device
            addr: I2C device address
        """
        self._mpu_i2c.readfrom_mem_into(addr, memaddr, buf)

    def _write(self, data, memaddr, addr):
        """
        Perform a memory write. Caller should trap OSError.
        Args:
            buf: pre-allocated buffer where results are read into
            memaddr: memory location (i.e., register) within the I2C device
            addr: I2C device address
        """
        self.buf1[0] = data
        self._mpu_i2c.writeto_mem(addr, memaddr, self.buf1)

    def wake(self):
        """
        Wakes the device.
        """
        try:
            self._write(0x01, 0x6B, self.mpu_addr)  # Use best clock source
        except OSError:
            raise MPUException(self._I2Cerror)
        return "awake"

    def sleep(self):
        """
        Sets the device to sleep mode.
        """
        try:
            self._write(0x40, 0x6B, self.mpu_addr)
        except OSError:
            raise MPUException(self._I2Cerror)
        return "asleep"

    @property
    def chip_id(self):
        """
        Returns Chip ID.
        """
        try:
            self._read(self.buf1, 0x75, self.mpu_addr)
        except OSError:
            raise MPUException(self._I2Cerror)
        chip_id = int(self.buf1[0])
        if chip_id != self._chip_id:
            mesg = ("FATAL: Chip ID ({0})retrieved, expected ({1}): " +
                    "MPU communication failure")
            raise ValueError(
                mesg.format(chip_id, self._chip_id))
        return chip_id

    @property
    def sensors(self):
        """
        Returns sensor objects accel, gyro.
        """
        return (self._accel, self._gyro)

    @property
    def temperature(self):
        """
        Returns the temperature in degree C.
        """
        try:
            self._read(self.buf2, 0x41, self.mpu_addr)
        except OSError:
            raise MPUException(self._I2Cerror)
        return bytes_toint(self.buf2[0], self.buf2[1])/340 + 35  # I think

    @property
    def passthrough(self):
        """
        Returns passthrough mode True or False.
        """
        try:
            self._read(self.buf1, 0x37, self.mpu_addr)
            return self.buf1[0] & 0x02 > 0
        except OSError:
            raise MPUException(self._I2Cerror)

    @passthrough.setter
    def passthrough(self, mode):
        """
        Sets passthrough mode.
        Args:
            mode: enables or disables passthrough mode, either True or False
        """
        if type(mode) is bool:
            val = 2 if mode else 0
            try:
                # I think this is correct.
                self._write(val, 0x37, self.mpu_addr)
                self._write(0x00, 0x6A, self.mpu_addr)
            except OSError:
                raise MPUException(self._I2Cerror)
        else:
            raise ValueError(
                "FATAL: Parameter 'mode' should be True or False.")

    @property
    def sample_rate(self):
        """
        Get sample rate as per Register Map document section 4.4
        SAMPLE_RATE = Internal_Sample_Rate / (1 + rate)
        default rate is zero i.e. sample at internal rate.
        """
        try:
            self._read(self.buf1, 0x19, self.mpu_addr)
            return self.buf1[0]
        except OSError:
            raise MPUException(self._I2Cerror)

    @sample_rate.setter
    def sample_rate(self, rate):
        """
        Set sample rate as per Register Map document section 4.4.
        Arga:
            rate: value from 0 to 255 that determines the sampling rate
        """
        if rate < 0 or rate > 255:
            raise ValueError(
                "FATAL: Rate must be in range 0-255.")
        try:
            self._write(rate, 0x19, self.mpu_addr)
        except OSError:
            raise MPUException(self._I2Cerror)

    def _get_range(self, memaddr):
        """
        Returns the current range of a sensor.
        Args:
            memaddr: register address of sensor to read from;
                0x1B for gyroscope
                0x1C for accelerometer
        """
        try:
            self._read(self.buf1, memaddr, self.mpu_addr)
            sensor_range = self.buf1[0]//8
        except OSError:
            raise MPUException(self._I2Cerror)
        return sensor_range

    def _set_range(self, memaddr, range_value, max_value):
        """
        Sets the range value of a sensor.
        Args:
            memaddr: register address of sensor to write to;
                0x1B for gyroscope
                0x1C for accelerometer
            range_value: value that establishes the range of a sensor
            max_value: maximum value for range_value
        """
        ar_bytes = (0x00, 0x08, 0x10, 0x18)
        if range_value in range(len(ar_bytes)):
            try:
                self._write(ar_bytes[range_value], memaddr, self.mpu_addr)
            except OSError:
                raise MPUException(self._I2Cerror)
        else:
            mesg = "FATAL: Value for 'range_value' must be from 0 to {0}."
            raise ValueError(mesg.format(max_value))

    def _get_filter_range(self, memaddr):
        """
        Returns the sensor low pass filter cutoff frequency. Using the
        filter_range property of the MPU9250 is harmless but gyro_filter_range
        is preferred and offers an extra setting.
        Args:
            memaddr: register of value to read from; valid values are:
                0x1A for gyroscope
                0x1D for accelerometer
        """
        try:
            self._read(self.buf1, memaddr, self.mpu_addr)
            res = self.buf1[0] & 7
        except OSError:
            raise MPUException(self._I2Cerror)
        return res

    def _set_filter_range(self, memaddr, filter_coef, max_value):
        """
        Sets the sensor low-pass filter cutoff frequency.
        Args:
            memaddr: register of value to write to; valid values are:
                0x1A for gyroscope
                0x1D for accelerometer
            filter_coef: value that establishes the low-pass filter coefficient
            max_value: maximum value for filter_range
        """
        # set range
        if filter_coef in range(max_value):
            try:
                self._write(filter_coef, memaddr, self.mpu_addr)
            except OSError:
                raise MPUException(self._I2Cerror)
        else:
            mesg = "FATAL: Value for 'filter_coef' must be from 0 and {0}"
            raise ValueError(mesg.format(max_value))

    @property
    def accel_range(self):
        """
        Returns the range of the accelerometer.

        Return Value:       0   1   2   3
        for range +/-:      2   4   8   16  (g)
        """
        return self._get_range(0x1C)
        
    @accel_range.setter
    def accel_range(self, range_value):
        """
        Set the range of the accelerometer.
        Args:
            filter_range: value from 0 to 3 that establishes the range of the
                accelerometer

        Passed Value:       0   1   2   3
        for range +/-:      2   4   8   16  (g)
        """
        self._set_range(0x1C, range_value, max_value=3)

    @property
    def accel_filter_range(self):
        """
        Returns the accelerator low-pass filter cutoff frequency.

        Return Value:       0   1   2   3   4   5   6   7
        Cutoff (Hz):        460 184 92  41  20  10  5   460
        Sample rate (KHz):  1   1   1   1   1   1   1   1
        """
        return self._get_filter_range(0x1D)

    @accel_filter_range.setter
    def accel_filter_range(self, filter_coef):
        """
        Sets the accelerator low-pass filter cutoff frequency.
        Args:
            filter_coef: Sets the accelerometer low-pass filter cutoff
                frequency.

        Return Value:       0   1   2   3   4   5   6   7
        Cutoff (Hz):        460 184 92  41  20  10  5   460
        Sample rate (KHz):  1   1   1   1   1   1   1   1
        NOTE: filter rage=7 may not work all the time
        """
        self._set_filter_range(0x1D, filter_coef, max_value=7)

    @property
    def gyro_range(self):
        """
        Returns the range of the gyroscope.

        Return Value:       0   1   2    3
        for range +/-:      250 500 1000 2000  degrees/second
        """
        return self._get_range(0x1B)

    @gyro_range.setter
    def gyro_range(self, range_value):
        """
        Set the range of the gyroscope.
        Args:
            range_value: value from 0 to 3 that determines the range of the
                gyroscope

        Passed Value:       0   1   2    3
        for range +/-:      250 500 1000 2000  degrees/second
        """
        self._set_range(0x1B, range_value, max_value=3)

    @property
    def gyro_filter_range(self):
        """
        Returns the gyroscope low-pass filter cutoff frequency.

        Return Value:       0   1   2   3   4   5   6   7
        Cutoff (Hz):        250 184 92  41  20  10  5   3600
        Sample rate (KHz):  8   1   1   1   1   1   1   8
        """
        return self._get_filter_range(0x1A)

    @gyro_filter_range.setter
    def gyro_filter_range(self, filter_coef):
        """
        Sets the accelerator low-pass filter cutoff frequency.
        Args:
            filter_coef: Sets the gyroscope low-pass filter cutoff frequency.

        Passed Value:       0   1   2   3   4   5   6   7
        Cutoff (Hz):        250 184 92  41  20  10  5   3600
        Sample rate (KHz):  8   1   1   1   1   1   1   8
        """
        self._set_filter_range(0x1A, filter_coef, max_value=7)

    @property
    def accel(self):
        """
        Return the acceleremoter object for a class instance.
        """
        return self._accel

    def _accel_callback(self):
        """
        Update accelerometer Vector3d object.
        """
        try:
            self._read(self.buf6, 0x3B, self.mpu_addr)
        except OSError:
            raise MPUException(self._I2Cerror)
        self._accel._ivector[0] = bytes_toint(self.buf6[0], self.buf6[1])
        self._accel._ivector[1] = bytes_toint(self.buf6[2], self.buf6[3])
        self._accel._ivector[2] = bytes_toint(self.buf6[4], self.buf6[5])
        scale = (16384, 8192, 4096, 2048)
        self._accel._vector[0] = self._accel._ivector[0]/scale[self.accel_range]
        self._accel._vector[1] = self._accel._ivector[1]/scale[self.accel_range]
        self._accel._vector[2] = self._accel._ivector[2]/scale[self.accel_range]

    def get_accel_irq(self):
        """
        For use in interrupt handlers. Sets self._accel._ivector[] to signed
        unscaled integer accelerometer values.
        """
        self._read(self.buf6, 0x3B, self.mpu_addr)
        self._accel._ivector[0] = bytes_toint(self.buf6[0], self.buf6[1])
        self._accel._ivector[1] = bytes_toint(self.buf6[2], self.buf6[3])
        self._accel._ivector[2] = bytes_toint(self.buf6[4], self.buf6[5])

    @property
    def gyro(self):
        """
        Returns the gyroscope object for a class instance.
        """
        return self._gyro

    def _gyro_callback(self):
        """
        Update gyroscope Vector3d object.
        """
        try:
            self._read(self.buf6, 0x43, self.mpu_addr)
        except OSError:
            raise MPUException(self._I2Cerror)
        self._gyro._ivector[0] = bytes_toint(self.buf6[0], self.buf6[1])
        self._gyro._ivector[1] = bytes_toint(self.buf6[2], self.buf6[3])
        self._gyro._ivector[2] = bytes_toint(self.buf6[4], self.buf6[5])
        scale = (131, 65.5, 32.8, 16.4)
        self._gyro._vector[0] = self._gyro._ivector[0]/scale[self.gyro_range]
        self._gyro._vector[1] = self._gyro._ivector[1]/scale[self.gyro_range]
        self._gyro._vector[2] = self._gyro._ivector[2]/scale[self.gyro_range]

    def get_gyro_irq(self):
        """
        For use in interrupt handlers. Sets self._gyro._ivector[] to signed
        unscaled integer gyro values. Error trapping disallowed.
        """
        self._read(self.buf6, 0x43, self.mpu_addr)
        self._gyro._ivector[0] = bytes_toint(self.buf6[0], self.buf6[1])
        self._gyro._ivector[1] = bytes_toint(self.buf6[2], self.buf6[3])
        self._gyro._ivector[2] = bytes_toint(self.buf6[4], self.buf6[5])

    @property
    def acceleration(self):
        # scale_factor = self.accel_range
        # return [(value / scale_factor) * Gc for value in self.accel.xyz]
        return self.accel.xyz

    @property
    def orientation(self):
      return self.gyro.xyz

    @property
    def direction(self):
        return(0.00, 0.00, 0.00)
