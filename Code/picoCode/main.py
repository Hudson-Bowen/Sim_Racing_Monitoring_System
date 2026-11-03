from I2C_comps import sensorsi2c
from ADC_comps import sensorsADC
import aioble
import bluetooth
import asyncio
import struct
from machine import Pin
import time

#sleeping to ensure proper boot up
time.sleep(5)

#Turing on LED to indicate pico on/program running
pin = Pin("LED", Pin.OUT)
pin.toggle()

#variables for capacitive touch sensor
cap_pin = 15 #GPIO pin 555 timer output is tied to
press_freq = 2000 #threshold for cap sesnor to be deemed pressed
release_freq = 3000 #threshold for cap sensor to be deemed released
cap_pin_obj = Pin(cap_pin, Pin.IN) #pin object for cap sensor
last_edge = time.ticks_us() #time last edge was detected
period_sum = 0 #sum of total period time for measurment window
edge_count = 0 #number of edges in measurment window
f = 0 #frequency measurment
pressed = False #tracks state of sensor
transmit = False #tracks if transmission should occur
suc_pressed_count = 0 #counts the number of sucessive times the frequency of the input passes the pressed threshold for deboucing purposes
suc_released_count = 0 #counts the number of sucessive times the frequency of the input passes the released threshold for deboucing purposes

#callback function for when rising edge is detected
def edge_detect(pin):
    global press_freq, release_freq, last_edge, period_sum, edge_count, f, pressed, transmit, suc_pressed_count, suc_released_count

    #getting current time (time of edge), the period between last edge, then reseting last edge
    now = time.ticks_us()
    period = time.ticks_diff(now, last_edge)
    last_edge = now

    #making sure period measurment is valid
    if period > 0:
        #updating period sum and edge count
        period_sum += period
        edge_count += 1

        #when number of edges is 15, we calculate frequency
        if edge_count >= 15:
            #finding average period of input, then calculating frequency
            avg_period = period_sum/edge_count
            f = 1000000/avg_period
            #print(f)

            #resetting edge/period tracking variables
            period_sum = 0
            edge_count = 0



            #should sucessive inputs be updated based on thresholds
            if not pressed and f < press_freq:
                suc_pressed_count += 1
            elif not pressed and f > press_freq:
                suc_pressed_count = 0
            elif pressed and f > release_freq:
                suc_released_count += 1
            elif pressed and f < release_freq:
                suc_released_count = 0
 
            #if sucessive inputs is 5 or more, change the pressed state and transmit state accordingly
            if suc_pressed_count >= 5:
                pressed = True
                transmit = not transmit
                suc_pressed_count = 0
            elif suc_released_count >= 5:
                pressed = False
                suc_released_count = 0

            
            
#setting up interupt with above callback funciton
cap_pin_obj.irq(trigger=Pin.IRQ_RISING, handler=edge_detect)


#setting calibration constants for sensors
temp_cal_const = 0
pres_cal_const = 0
dist_cal_const = 0
ultra_dist_cal_const = 0

#initilizing i2c and ADC sensor objects
comps_obj = sensorsi2c(temp_cal= temp_cal_const, pres_cal= pres_cal_const, dist_cal= dist_cal_const, ultra_dist_cal= ultra_dist_cal_const)
adc_comps_obj = sensorsADC(adc_pin = 26, GPIO_pin = 16)

#setting up bluetooth service
SERVICE_UUID = bluetooth.UUID("e01663d5-fced-44bd-8fd2-7704b4f0d7f4")
SLOW_DATA_UUID = bluetooth.UUID("fa781b19-571e-4e44-a78e-262615811f3d")
FAST_DATA_UUID = bluetooth.UUID("069aa201-8f68-4f75-8e33-0599a44feba4")
aioble.stop()
service = aioble.Service(SERVICE_UUID)
slow_data_characteristic = aioble.Characteristic(service, SLOW_DATA_UUID, read=True, notify=True)
fast_data_characteristic = aioble.Characteristic(service, FAST_DATA_UUID, read=True, notify=True)
aioble.register_services(service)

#async event to determine if bluetooth connection is made
ble_connected = asyncio.Event()

#time keeping variables for previous time a sensor measurment was taken
prev_fast_meas_time = time.ticks_ms()
prev_slow_meas_time = time.ticks_ms()


#async function to handle bluetooth connection
async def ble_connection_manager():
    while True:
        connection = await aioble.advertise(100_000, name="simRacingSensors", services=[SERVICE_UUID],)

        #connection made and updating connection event
        print("Connected")
        ble_connected.set()

        #checking connection every 100 ms
        while connection.is_connected():
            await asyncio.sleep(1)

        #if disconnected, update connection event
        print("Disconnected")
        ble_connected.clear()

async def ble_fast_data():
    global prev_fast_meas_time
    while True:
        #wait until laptop is connected
        await ble_connected.wait()

        if transmit:
            #get distance and time reading and pack it into struct
            fast_meas_time = time.ticks_ms()
            dist = comps_obj.get_dist()
            ultra_dist = comps_obj.get_dist_ultra()
            time_diff_fast = time.ticks_diff(fast_meas_time, prev_fast_meas_time)
            fast_data = struct.pack("<iiH", dist, ultra_dist, time_diff_fast)
            prev_fast_meas_time = fast_meas_time

            #sending data
            fast_data_characteristic.write(fast_data, send_update=True)

        await asyncio.sleep(0.01)

async def ble_slow_data():
    global prev_slow_meas_time
    while True:
            #wait until laptop is connected
            await ble_connected.wait()

            if transmit:
            #get distance reading and pack it into struct
                slow_meas_time = time.ticks_ms()
                temp = comps_obj.get_temp()
                pres = comps_obj.get_pres()
                ldr_V_out = adc_comps_obj.get_ldr_V()
                therm_V_out = adc_comps_obj.get_therm_V()
                #therm_V_out, ldr_V_out = adc_comps_obj.get_sensor_Vs()
                time_diff_slow = time.ticks_diff(slow_meas_time, prev_slow_meas_time)
                slow_data = struct.pack("<ffffH", temp, pres, therm_V_out, ldr_V_out, time_diff_slow)
                prev_slow_meas_time = slow_meas_time

                #sending data
                slow_data_characteristic.write(slow_data, send_update=True)
    
            await asyncio.sleep(1)

async def main():
    asyncio.create_task(ble_connection_manager())
    asyncio.create_task(ble_fast_data())
    asyncio.create_task(ble_slow_data())
    
    while True:
        await asyncio.sleep(1)


asyncio.run(main())

