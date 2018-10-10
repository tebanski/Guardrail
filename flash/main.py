import urequests
import os, time, machine

from m5stack import *
from machine import Pin
from mpu9250 import MPU9250
from time import sleep_ms
from machine import I2C
from fusion import Fusion

#------------------------
def set_time():
    rtc = machine.RTC()
    print("Synchronize time from NTP server ...")
    lcd.println("Synchronize time from NTP server ...")
    rtc.ntp_sync(server="sg.pool.ntp.org")

def send_data():
    r = urequests.get("http://www.baidu.com")
    print(r.text)

def disp_splash():
    lcd.clear()
    lcd.setCursor(0, 0)
    lcd.setColor(lcd.WHITE)
    lcd.image(50, 10, '/flash/splash.jpg')
    sleep_ms(1000)
    lcd.clear()
    lcd.setCursor(0, 0)
    lcd.setColor(lcd.BLACK)
    
def get_readings():
    i2c = I2C(sda = 21, scl = 22)
    imu = MPU9250(i2c)
    fuse = Fusion()

    os.mountsd()
    file_name = '/sd/' + time.strftime("%Y%m%d_%H%M%S", time.localtime()) + '.dat'
    log = open(file_name, 'w')
    file_entries = 0
    
    lcd.clear()
    lcd.font(lcd.FONT_Small)
    lcd.setTextColor(lcd.WHITE, lcd.BLACK)
    
    while not buttonA.isPressed():
        accel = imu.acceleration
        gyro = imu.gyro
        mag = imu.magnetic
        fuse.update(accel, gyro, mag) 
        
        file_entries += 1
        if file_entries > 1000:
            log.close() 
            file_name = '/sd/' + time.strftime("%Y%m%d_%H%M%S", time.localtime()) + '.dat'
            log = open(file_name, 'w')
            file_entries = 0

        # write data
        log.write("ACC,{:8.3f},{:8.3f},{:8.3f},{}\n".format(accel[0], accel[1], accel[2], time.strftime("%Y%m%d_%H%M%S", time.localtime())))
        log.write("GYR,{:8.3f},{:8.3f},{:8.3f},{}\n".format(gyro[0], gyro[1], gyro[2], time.strftime("%Y%m%d_%H%M%S", time.localtime())))
        log.write("MAG,{:8.3f},{:8.3f},{:8.3f},{}\n".format(mag[0], mag[1], mag[2], time.strftime("%Y%m%d_%H%M%S", time.localtime())))
        log.write("FUS,{:8.3f},{:8.3f},{:8.3f},{}\n".format(fuse.heading, fuse.pitch,fuse.roll, time.strftime("%Y%m%d_%H%M%S", time.localtime())))
        # ----
        
        lcd.setColor(lcd.RED)
        lcd.print("Accel(X): {:8.3f}".format(accel[0]), lcd.CENTER, 10)
        lcd.print("Accel(Y): {:8.3f}".format(accel[1]), lcd.CENTER, 20)
        lcd.print("Accel(Z): {:8.3f}".format(accel[2]), lcd.CENTER, 30)

        lcd.setColor(lcd.ORANGE)
        lcd.print("Gyro(X) : {:8.3f}".format(gyro[0]), lcd.CENTER, 50)
        lcd.print("Gyro(Y) : {:8.3f}".format(gyro[1]), lcd.CENTER, 60)
        lcd.print("Gyro(Z) : {:8.3f}".format(gyro[2]), lcd.CENTER, 70)

        lcd.setColor(lcd.YELLOW)
        lcd.print("Mag(X): {:8.3f}".format(mag[0]), lcd.CENTER, 90)
        lcd.print("Mag(Y): {:8.3f}".format(mag[1]), lcd.CENTER, 100)
        lcd.print("Mag(Z): {:8.3f}".format(mag[2]), lcd.CENTER, 110)     
        
        lcd.setColor(lcd.CYAN)
        lcd.print("Heading: {:8.3f}".format(fuse.heading), lcd.CENTER, 130)     
        lcd.print("Pitch  : {:8.3f}".format(fuse.pitch), lcd.CENTER, 140)            
        lcd.print("Roll   : {:8.3f}".format(fuse.roll), lcd.CENTER, 150)            
        
        sleep_ms(200)
    
    log.close()   
    lcd.setTextColor(lcd.WHITE, lcd.BLACK)
    lcd.print(file_name, lcd.CENTER, 180)
    lcd.print('Larga Sensor Out.', lcd.CENTER, 200)

# ----------------------
disp_splash()
set_time()
get_readings()
# ----------------------

