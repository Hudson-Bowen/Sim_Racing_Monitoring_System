#This is a library to use the BMP280 and VL53L0X easily in the main code
from machine import Pin, I2C 
from bmp280 import BMP280
from vl53l0x import VL53L0X
import time

class sensorsi2c:
    def __init__(self, temp_cal = 0, pres_cal = 0, dist_cal = 0, ultra_dist_cal = 0, sda_pin = 4, scl_pin = 5, freq_set = 100000):
        #I2C addys for sensors
        self.bmpaddy = 0x77
        self.ultraaddy = 0x2F
        self.ultra_trigger_register = 0x01

        #intiilizing I2C connection and sensor objects
        self.i2c = I2C(0, sda=Pin(sda_pin), scl=Pin(scl_pin), freq=freq_set)
        self.bmp = BMP280(self.i2c, addr=self.bmpaddy)
        self.vl = VL53L0X(self.i2c)

        #calibration variables
        self.temp_cal = temp_cal
        self.pres_cal = pres_cal
        self.dist_cal = dist_cal
        self.ultra_dist_cal = ultra_dist_cal


    def get_temp(self):
        #get reading
        temp_read = self.bmp.temperature
        #convert to F and apply calibration offset
        temp_f = temp_read*(9/5) + 32 + self.temp_cal
        return temp_f

    def get_pres(self):
        #get reading
        pres_read = self.bmp.pressure
        #convert to atm and apply calibration offset
        pres_atm = (pres_read / 101325.0) + self.pres_cal
        return pres_atm

    def get_dist(self):
        #get reading
        dist_read = self.vl.ping()
        #apply calibration offset and returning
        dist_final = dist_read + self.dist_cal
        return int(dist_final)
    
    def get_dist_ultra(self):
        #get reading
        self.i2c.writeto(self.ultraaddy, bytes([self.ultra_trigger_register]))
        #time.sleep_ms(5)
        data = self.i2c.readfrom_mem(self.ultraaddy, self.ultra_trigger_register, 2)
        #converting data to mm
        ultra_dist_read = (data[0] << 8) | data[1]
        #applying calibration offset and returning
        ultra_dist_final = ultra_dist_read + self.ultra_dist_cal
        return ultra_dist_final

    