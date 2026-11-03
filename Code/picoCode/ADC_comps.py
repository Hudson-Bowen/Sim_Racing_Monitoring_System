from machine import ADC
from machine import Pin
import time

class sensorsADC:
    def __init__(self, adc_pin = 26, GPIO_pin = 16):
        #creating ADC objects for thermistor and ldr
        self.adc_obj = ADC(adc_pin)
        self.gpio_obj = Pin(GPIO_pin, Pin.OUT)

        #setting output to low by default
        self.gpio_obj.value(0)

    def get_ldr_V(self):
        #getting ldr output voltage from multiplexer
        ldr_adc_out = self.adc_obj.read_u16()
        #ldr voltage
        ldr_V = ldr_adc_out * 3.3 / 65535

        return ldr_V

    def get_therm_V(self):
        #switching GPIO pin to read thermister voltage
        self.gpio_obj.value(1)
        #allowing switching time for MOSFETs
        time.sleep_ms(20)
        #get thermister voltage from multiplexer
        therm_adc_out = self.adc_obj.read_u16()
        #switch GPIO back to low for next  LDR reading
        self.gpio_obj.value(0)
        #calculate therm voltage
        therm_V = therm_adc_out * 3.3 / 65535

        return therm_V