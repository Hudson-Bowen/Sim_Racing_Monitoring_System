import asyncio
from bleak import BleakScanner, BleakClient
import struct
import csv
import math
import statistics


#Global variables for data
temp = 0
pres = 0
dist = 0
therm_temp = 0
ldr_brightness = 0
ultra_dist = 0
ToF_percentage = 0
ultra_percentage = 0

#kalman filter initial variables
therm_uncert = 0.5 #degrees F
estimate_uncert = 10 #degrees F
estimate = 77 #degrees F
estimate_sens = 0.1 #estimate uncertinty sensitivity
therm_cal_const = -7.374 #calibration constant for thermister (F)

#ldr brightness calibration
ldr_brightness_cal_const = 0

#stress calculation variables
ideal_temp = 68 #F, ideal temp to be driving in for no temp caused stress
max_temp_diff = 15 #F, maximum difference between temp and ideal temp for full thermal stress

#tracking previous data values for GUI purposes
prev_temp = 0
prev_pres = 0
prev_dist = 0
prev_therm_temp = 0
prev_ldr_brightness = 0
prev_ultra_dist = 0

#Bluetooth parameters
DEVICE_NAME = "simRacingSensors"
SLOW_DATA_UUID = "fa781b19-571e-4e44-a78e-262615811f3d"
FAST_DATA_UUID = "069aa201-8f68-4f75-8e33-0599a44feba4"

#creating csv files to write data to
fast_data_file = open("fastdata.csv", "w", newline="")
slow_data_file = open("slowdata.csv", "w", newline="")

#writers to csv files
fast_data_writer = csv.writer(fast_data_file)
slow_data_writer = csv.writer(slow_data_file)

#giving csv files headers
fast_data_writer.writerow(["Time Between Readings (ms)", "Distance (mm)", "Distance (mm Ultrasonic)", "Throttle % ToF", "Throttle % Ultra"])
slow_data_writer.writerow(["Time Between Readings (ms)", "BMP Temperature (F)", "Pressure (atm)", "Therm Temp (F)", "Therm Kalman (F)", "Brightness %"])

#function takes thermister circuit output voltage and converts it to degrees F
def therm_V_to_F(therm_mp_v):
    #Calculating inamp output voltage from multiplexer
    therm_V = 3.3 - therm_mp_v
    #calculating differential voltage from inamp gain equation
    V_diff = (therm_V - 1.807)/(1+ ((2*42000)/3000))
    #calculating voltage across thermister from differential voltage
    V_therm = V_diff + 1.65
    #calculating the resistance of the thermister from voltage divider of it, three points R, and sereis R
    R_thr = (400*590*V_therm)/((590)*(3.3-V_therm)-(400*V_therm))
    #calculating 1/temperature (in K) from R_thr
    temp_inv = (1/298.15) + (1/3636)* math.log(R_thr/1000)
    #inverting temp_inv
    temp_k = 1/temp_inv
    #converting to F
    temp_f = (temp_k - 273.15)*(9/5)+32 + therm_cal_const
    return temp_f

#function handles the kalman filter of the thermistor
def therm_kalman(therm_reading):
     global therm_uncert, estimate_uncert, estimate, estimate_sens
     #adding estimate sensitivity
     estimate_uncert = estimate_uncert + estimate_sens
     #calculating Kalman Gain
     KG = estimate_uncert/(estimate_uncert+therm_uncert)
     #Calculating current (new) estimate
     estimate = estimate + KG*(therm_reading-estimate)
     #calculating new estimate uncertinty
     estimate_uncert = (1-KG)*estimate_uncert
     return estimate

#function takes LDR output voltage and turns it to % brightness based on charecterization data
def ldr_V_to_per(ldr_mp_v):
    #calculating brightness % and doing edge cases
    ldr_per = (-0.0037 + math.sqrt(0.00263329 - 0.0008*ldr_mp_v)) / 0.0004
    if ldr_per > 100:
       ldr_per = 100
    elif ldr_per < 0:
       ldr_per = 0

    return ldr_per

#function gets pedal percentage from the TCT40 utlrasonic distance sensor MAKE THIS WHEN YOU HAVE TRANSFER FUNCTION DATA
def pedal_percent_ultra(ultra_dist_in):
    #calculation
    ultra_percentage_calc = -1.667 * ultra_dist_in + 145

    #bounding between 0 and 100
    if ultra_percentage_calc > 99.9:
        ultra_percentage_calc = 100
    elif ultra_percentage_calc < 0:
        ultra_percentage_calc = 0

    #rounding to make it look nice
    ultra_percentage_calc = round(ultra_percentage_calc, 2)

    return ultra_percentage_calc

#function gets pedal percentage from the VL53L0X ToF distance sensor MAKE THIS WHEN YOU HAVE TRANSFER FUNCTION DATA
def pedal_percent_ToF(ToF_dist_in):
    #calculation
    ToF_percentage_calc = -2.174 * ToF_dist_in + 230.434
    
    #bounding between 0 and 100
    if ToF_percentage_calc >= 100:
        ToF_percentage_calc = 100
    elif ToF_percentage_calc < 0:
        ToF_percentage_calc = 0

    #rounding to make it look nice
    ToF_percentage_calc = round(ToF_percentage_calc, 2)
    
    return ToF_percentage_calc

#function that extracts BMP temp, Therm temp, and brightness % from slowdata.csv and returns their averages
def slow_data_avg(file_reader):
    BMP_temps, Therm_temps, Brightness_readings = [], [], []

    for row in file_reader:
        BMP_temps.append(float(row["BMP Temperature (F)"]))
        Therm_temps.append(float(row["Therm Kalman (F)"]))
        Brightness_readings.append(float(row["Brightness %"]))

    avg_bmp = statistics.mean(BMP_temps)
    avg_therm = statistics.mean(Therm_temps)
    avg_bright = statistics.mean(Brightness_readings)

    return avg_bmp, avg_therm, avg_bright

#function that extracts both throttle % from fastdata.csv and returns their averages
def fast_data_avg(file_reader):
    ToF_readings, Ultra_readings = [], []

    for row in file_reader:
        ToF_readings.append(float(row["Throttle % ToF"]))
        Ultra_readings.append(float(row["Throttle % Ultra"]))

    avg_ToF = statistics.mean(ToF_readings)
    avg_Ultra = statistics.mean(Ultra_readings)

    return avg_ToF, avg_Ultra

#function that calculates the stress percentage of a window of driving from temperature, throttle percentage, and time of day (brightness) during it
def find_stress_per(temp, throt_per, brightness):
    global ideal_temp, max_temp_diff

    #calculating Temperature Stress Index
    TSindex = min(1,abs(temp-ideal_temp)/max_temp_diff)

    #calculating Throttle Percentage Stress Index
    TPSindex = throt_per/100

    #calcualting Visibility Stress Index
    VSindex = 1-(brightness/50)
    VSindex = max (0, VSindex)


    #print(TSindex, TPSindex, VSindex)
    #stress percentage calcualtion, capped at 100%
    stress = min(100, 80 * TPSindex + 20 * TSindex + 30 * VSindex)
    return stress

    

def slow_notification_handler(sender, data):
    global temp, pres, therm_temp, ldr_brightness, slow_data_writer
    temp, pres, therm_V_out, ldr_V_out, diff= struct.unpack("<ffffH", data)
    therm_temp_reading = therm_V_to_F(therm_V_out)
    therm_temp = therm_kalman(therm_temp_reading)
    ldr_brightness = ldr_V_to_per(ldr_V_out)
    temp, pres, therm_temp_reading, therm_temp, ldr_brightness = round(temp, 3), round(pres, 3), round(therm_temp_reading, 3), round(therm_temp, 3), round(ldr_brightness, 3)
    #print(f"temp: {temp}, pres: {pres}, diff: {diff}, thermistor V: {therm_V_to_F(therm_V_out)}, ldr V: {ldr_V_out} acutal diff: {time.perf_counter()}")
    #print(f"Thermistor Reading: {therm_temp}, Kalman Filter Model: {therm_kalman(therm_temp)}")
    #print(ldr_brightness)
    slow_data_writer.writerow([diff, temp, pres, therm_temp_reading, therm_temp, ldr_brightness])

def fast_notification_handler(sender, data):
    global dist, fast_data_writer, ultra_dist, ToF_percentage, ultra_percentage
    #unpack data
    dist, ultra_dist, diff = struct.unpack("<iiH", data)
    #print(f"dist: {dist}, ultra dist: {ultra_dist}, diff: {diff}")

    #calculating throttle position from distances
    ToF_percentage = pedal_percent_ToF(dist)
    ultra_percentage = pedal_percent_ultra(ultra_dist)

    #write data to csv file
    fast_data_writer.writerow([diff, dist, ultra_dist, ToF_percentage, ultra_percentage])

async def ble_connection_manager():
    print("Scanning...")

    device = await BleakScanner.find_device_by_name(DEVICE_NAME)

    if device is None:
        print("Pico not found")
        return

    print(f"Connecting to {device.address}")

    async with BleakClient(device) as client:
        print("Connected")

        await client.start_notify(SLOW_DATA_UUID, slow_notification_handler)
        await client.start_notify(FAST_DATA_UUID, fast_notification_handler)

        print("Waiting for sensor data")

        while client.is_connected:
            await asyncio.sleep(1)


async def gui():
     global temp, pres, dist, therm_temp, ldr_brightness, ultra_dist, ToF_percentage, ultra_percentage, prev_temp, prev_pres, prev_dist, prev_therm_temp, prev_ldr_brightness, prev_ultra_dist
     while True:
        if (temp != prev_temp) or (dist != prev_dist) or (pres != prev_pres) or (therm_temp != prev_therm_temp) or (ldr_brightness != prev_ldr_brightness) or (ultra_dist != prev_ultra_dist):
            print(f"BMPT (F): {temp:.3f}, Therm (F): {therm_temp:.3f} (+/- 1), P (atm): {pres:.3f}, Bright (%): {ldr_brightness:.2f}, ToF (mm): {dist}, Ultra (mm): {ultra_dist}, %ToF: {ToF_percentage:.2f}, %Ultra: {ultra_percentage:.2f}")
            prev_temp, prev_pres, prev_dist, prev_therm_temp, prev_ldr_brightness, prev_ultra_dist = temp, pres, dist, therm_temp, ldr_brightness, ultra_dist
        await asyncio.sleep(0.1)


async def main():
    global fast_data_file, slow_data_file
    #try finally statemnt to handle ctrl + c without raising error
    try:
        #using taskgroup to manage ansync functions
        async with asyncio.TaskGroup() as tg:
            tg.create_task(ble_connection_manager())
            tg.create_task(gui())

            #waiting for ctrl + c
            await asyncio.Event().wait()

    finally:
        #closing main connection to perform analysis on data gathered
        print("Closed")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nProcessing Results")

        #closing write file handlers so we can read from the csvs to calculate stress level
        fast_data_file.close()
        slow_data_file.close()

        #creating new file handlers to read the data
        fast_data_file = open("fastdata.csv", "r", newline="")
        slow_data_file = open("slowdata.csv", "r", newline="")
        fast_data_reader = csv.DictReader(fast_data_file)
        slow_data_reader = csv.DictReader(slow_data_file)

        #calculating on throttle percentages, average temps, and average brightness
        on_throt_per_ToF, on_throt_per_Ultra = fast_data_avg(fast_data_reader)
        avg_bmp_temp, avg_therm_temp, avg_bright = slow_data_avg(slow_data_reader)
        #averaging temp and throttle percentage readings
        avg_throt_per =  (on_throt_per_ToF + on_throt_per_Ultra) / 2
        avg_temp = (avg_bmp_temp + avg_therm_temp) /2

        #Calculating stress % from go beyond function
        final_stress = find_stress_per(avg_temp, avg_throt_per, avg_bright)

        print(f"Final Statistics: Throttle %: {round(avg_throt_per,2)}, Average Temperature: {round(avg_temp,3)}, Average Brightness: {round(avg_bright,2)}")
        print(f"Stage Stress Value: {round(final_stress,2)}%")

        #closing file readers
        fast_data_file.close()
        slow_data_file.close()


