import time

from m5stack import lcd

import driver


lcd.clear()
lcd.setCursor(0, 0)
lcd.setColor(lcd.ORANGE)
lcd.print("Welcome to Dagitab Systems\n")
lcd.print("LARGA IU Vehicle Sensor Platform Prototype\n")
lcd.print("Loading drivers. Please wait....")
time.sleep_ms(2000)


# Larga IU sensor platform driver entry point 
driver.init()
