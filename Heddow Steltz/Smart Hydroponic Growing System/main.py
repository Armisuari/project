#!/usr/bin/python
#!/bin/sh -e

import RPi.GPIO as GPIO
import board
import busio
import time
import datetime
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from w1thermsensor import W1ThermSensor
import paho.mqtt.client as mqtt
import json
import logging

import os
from wifi import Cell, Scheme
import subprocess

wpa_supplicant_conf = "/etc/wpa_supplicant/wpa_supplicant.conf"
sudo_mode = "sudo "

def wifi_connect(ssid, psk):
    # write wifi config to file
    #cmd = 'wpa_passphrase {ssid} {psk} | sudo tee -a {conf} > /dev/null'.format(
    #        ssid=str(ssid).replace('!', '\!'),
    #        psk=str(psk).replace('!', '\!'),
    #        conf=wpa_supplicant_conf
    #    )
    cmd = 'wpa_passphrase {ssid} {psk} | sudo tee -a {conf} > /dev/null'.format(
            ssid=str(ssid).replace('!', '\!'),
            psk=str(psk).replace('!', '\!'),
            conf=wpa_supplicant_conf
        )
    cmd_result = ""
    cmd_result = os.system(cmd)
    print (cmd + " - " + str(cmd_result))
    # reconfigure wifi
    cmd = sudo_mode + 'wpa_cli -i wlan0 reconfigure'
    cmd_result = os.system(cmd)
    print (cmd + " - " + str(cmd_result))
    time.sleep(10)
    cmd = 'iwconfig wlan0'
    cmd_result = os.system(cmd)
    print (cmd + " - " + str(cmd_result))
    cmd = 'ifconfig wlan0'
    cmd_result = os.system(cmd)
    print (cmd + " - " + str(cmd_result))
    p = subprocess.Popen(['hostname', '-I'], stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE)
    out, err = p.communicate()
    if out:
        ip_address = out
    else:
        ip_address = "<Not Set>"
    return ip_address
def ssid_discovered():
    Cells = Cell.all('wlan0')
    wifi_info = 'Found ssid : \n'
    for current in range(len(Cells)):
        wifi_info +=  Cells[current].ssid + "\n"
    wifi_info+="!"
    print (wifi_info)
    return wifi_info

i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c)
chan0 = AnalogIn(ads, ADS.P0)
chan1 = AnalogIn(ads, ADS.P1)
WL = 17
pH_up = 13
pH_down = 19
Nutrient_A = 18
Nutrient_B = 12
Water_P = 16
RLed = 23
GLed = 24
BLed = 8
waterss = False

THINGSBOARD_HOST = 'thingsboard.cloud'
ACCES_TOKEN = 'qZD7WS9BXBLzWCFKtakJ'

gpio_state = {4: False, BLed: False, Nutrient_A: False, 27: False, 22: False, RLed: False, GLed: False, 25: False, 5: False,
              6: False, Nutrient_B: False, pH_up: False, pH_down: False, Water_P: False, 26: False, 20: False, 21: False}

#gpio_state = {7: False, 12: False, 13: False, 15: False, 16: False, 18: False, 22: False, 29: False,
#              31: False, 32: False, 33: False, 35: False, 36: False, 37: False, 38: False, 40: False}

# Using board GPIO layout
#GPIO.setmode(GPIO.BCM)

#GPIO.setwarnigs(False)

errorPH = 0.0
errorTDS = 0.0

def read_file(File):
    PPM = open('TDS_value.txt', 'r')
    PH = open('PH_value.txt', 'r')
    if File == 'PPM':    
        f = float(PPM.read())
    else:
        f = float(PH.read())
    return f

Desired_PPM = float(read_file('PPM'))
Desired_PH = float(read_file('PH'))
EC_PPM = False

sensor_data = {'pH Value': 0, 'Temp Value': 0, 'Tds Value': 0, 'Ec Value': 0, 'waterss': False}
variabel = {'time LED': '', 'Desired PH': Desired_PH, 'Desired PPM': Desired_PPM, 'PH Sim': '', 'TDS Sim': 0.0, 'EC Sim': 0.0, 'Water Sim': False }
display = {'error TDS': errorTDS, 'error PH': errorPH}
wifi_input = {'ssid': '', 'psk': ''}
tombol = {'EC_PPM': EC_PPM}

ssid = ''
psk = ''
PH_Sim = 0.0

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, rc, *extra_params):
   # global variabel
    global EC_PPM
    print('Connected with result code ' + str(rc))
    # Subscribing to receive RPC requests
    client.subscribe('v1/devices/me/rpc/request/+')
    client.subscribe('v1/devices/me/attributes')
    # Sending current GPIO status
    client.publish('v1/devices/me/attributes', get_gpio_status(), 1)
    client.publish('v1/devices/me/attributes', json.dumps(variabel), 1)
    client.publish('v1/devices/me/attributes', json.dumps(wifi_input), 1)
    client.publish('v1/devices/me/attributes', json.dumps(tombol), 1)

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print ('Topic: ' + msg.topic + '\nMessage: ' + str(msg.payload))
    # Decode JSON request
    global data
    data = json.loads(msg.payload)
    print('isi data:', data)

    if msg.topic == 'v1/devices/me/attributes':
        global data_time, start_hour, start_min, stop_hour, stop_min, PH_Sim, TDS_Sim, EC_Sim, water_pump_state
        global Desired_PH, Desired_PPM, Water_Sim, ssid, psk
        data_split = str(data).split(':')
        print('hasil split:', data_split)
        if data_split[0] == "{'ssid'":
            ssid = str(data['ssid'])
        elif data_split[0] == "{'psk'":
            psk = str(data['psk'])
            print('SSID: ', ssid, '\nPSK: ', psk)
            ip_address = wifi_connect(ssid, psk)
        elif data_split[0] == "{'time LED'":
            data_time = str(data['time LED']).split('-')
            print(data_time)
            start_hour = int(str(data_time[0]).split(':')[0])
            start_min = int(str(data_time[0]).split(':')[1])
            stop_hour = int(str(data_time[1]).split(':')[0])
            stop_min = int(str(data_time[1]).split(':')[1])
            print(start_hour, start_min, stop_hour, stop_min)
        elif data_split[0] == "{'Desired PPM'":
            Desired_PPM = float(data['Desired PPM'])
            print('Desired PPM:', Desired_PPM)
            file = open('TDS_value.txt', 'w')
            file.write(str(Desired_PPM))
            file.close()
        elif data_split[0] == "{'Desired PH'":
            Desired_PH = float(data['Desired PH'])
            print('Desired PH:', Desired_PH, 'okok')
            file = open('PH_value.txt', 'w')
            file.write(str(Desired_PH))
            file.close()
        elif data_split[0] == "{'PH Sim'":
            PH_Sim = float(data['PH Sim'])
            print(PH_Sim)
        elif data_split[0] == "{'TDS Sim'":
            TDS_Sim = float(data['TDS Sim'])
            print(TDS_Sim)
        elif data_split[0] == "{'EC Sim'":
            EC_Sim = float(data['EC Sim'])
            print(EC_Sim)
        elif data_split[0] == "{'Water Sim'":
            Water_Sim = float(data['Water Sim'])
            print(Water_Sim)
    elif data['method'] == 'getValue':
        client.publish(msg.topic.replace('request', 'response'), json.dumps(tombol), 1)
    elif data['method'] == 'setValue':
        global EC_PPM
        if data['params'] == True:
            EC_PPM = True
        else:
            EC_PPM = False
        client.publish(msg.topic.replace('request', 'response'), json.dumps(tombol), 1)
    else:
        manual_push()
    client.publish('v1/devices/me/attributes', json.dumps({'EC_PPM': EC_PPM}), 1)
        
def manual_push():
    global data, LED_state, variabel, tombol
    if data['method'] == 'set_ph_up_pump_state':
       print(data['params'])
       if data['params'] == True:
            GPIO.output(pH_up, GPIO.HIGH)
       else:
            GPIO.output(pH_up, GPIO.LOW)
    elif data['method'] == 'set_ph_down_pump_state':
       print(data['params'])
       if data['params'] == True:
            GPIO.output(pH_down, GPIO.HIGH)
       else:
            GPIO.output(pH_down, GPIO.LOW)
    elif data['method'] == 'set_Nut_A_pump_state':
       print(data['params'])
       if data['params'] == True:
            GPIO.output(Nutrient_A, GPIO.HIGH)
       else:
            GPIO.output(Nutrient_A, GPIO.LOW)
    elif data['method'] == 'set_Nut_B_pump_state':
       print(data['params'])
       if data['params'] == True:
            GPIO.output(Nutrient_B, GPIO.HIGH)
       else:
            GPIO.output(Nutrient_B, GPIO.LOW)
    elif data['method'] == 'set_WL_pump_state':
       global water_pump_state
       print(data['params'])
       if data['params'] == True:
            water_pump_state = True
       else:
            water_pump_state = False
            #GPIO.output(Water_P, GPIO.LOW)
    elif data['method'] == 'setLED':
       print(data['params'])
       if data['params'] == True:
            GPIO.output(23, 1)
            GPIO.output(24, 1)
            GPIO.output(8, 1)
            print('LED ON')
            LED_state = True
       else:
            GPIO.output(23, 0)
            GPIO.output(24, 0)
            GPIO.output(8, 0)
            print('LED OFF')
            LED_state = False

data = 0
data_time = ''
start_hour = 0
start_min = 0
stop_hour = 0
stop_min = 0
LED_state = False
TDS_Sim = 0.0
EC_Sim = 0.0
Water_Sim = False
water_pump_state = False

GPIO.setmode(GPIO.BCM) 
GPIO.setwarnings(False)


for pin in gpio_state:
   # Set output mode for all GPIO pins
   GPIO.setup(pin, GPIO.OUT)    
    

GPIO.setup(WL, GPIO.IN)

td = datetime.datetime.today()

def set_time_LED():
    global td
    td = datetime.datetime.today()
    global data_time, start_hour, start_min, stop_hour, stop_min
    if start_hour != td.hour and start_min != td.minute and LED_state != True:
        GPIO.output(23, 0)
        GPIO.output(24, 0)
        GPIO.output(8, 0)
        print('LED OFF')
    elif start_hour == td.hour and start_min == td.minute:
        GPIO.output(23, 1)
        GPIO.output(24, 1)
        GPIO.output(8, 1)
        print('LED ON')
    if stop_hour == td.hour and stop_min == td.minute:
        GPIO.output(23, 0)
        GPIO.output(24, 0)
        GPIO.output(8, 0)
        print('LED OFF')
    print('...............................')
    print('Schedule LED Time:',start_hour,':',start_min,'-', stop_hour,':', stop_min)

def get_gpio_status():
    # Encode GPIOs state to json
    return json.dumps(gpio_state)


def set_gpio_status(pin, status):
    # Output GPIOs state
    #if status:
    GPIO.output(pin, GPIO.HIGH if status else GPIO.LOW)
    #    GPIO.output(pin, GPIO.HIGH)
    #else:
    #    GPIO.output(pin, GPIO.LOW)
    # Update GPIOs state
    gpio_state[pin] = status

client = mqtt.Client()
#Register pconnect callback
client.on_connect = on_connect
#Registed publish message callback
client.on_message = on_message
#set acces token
client.username_pw_set(ACCES_TOKEN)
#Connect to Thingsboard using default MQTT port
client.connect(THINGSBOARD_HOST, 1883, 60)
client.loop_start()

#ip_address = wifi_connect(ssid, psk)

#returns ms since the epoch
def millis():
    return time.time() * 1000

def read_ph():
    Po = round((-6.557 * chan0.voltage) + 24.65, 2)
    return Po

def read_temp():
    sensor = W1ThermSensor()
    temperature = round(sensor.get_temperature(), 1)
    return temperature

def read_tds():
    ppm = round((132.6 * chan1.voltage) + 85.67, 2)
    return ppm

def read_ec():
    ec = round((2.000 * read_tds()) - 0.0, 3)
    return ec
    
def read_WL():
    global WL
    WL1 = GPIO.input(WL)
    return WL1

def upload():
    global waterss, EC_PPM
    if EC_PPM == False:
        select = read_ec()
    else:
        select = read_tds()
    sensor_data['pH Value'] = read_ph()
    sensor_data['Temp Value'] = float(read_temp())
    #sensor_data['Tds Value'] = read_tds()
    sensor_data['Ec Value'] = select
    sensor_data['Water Level'] = waterss
    display['error TDS'] = errorTDS
    client.publish('v1/devices/me/telemetry', json.dumps(sensor_data), 1)
    client.publish('v1/devices/me/telemetry', json.dumps(display), 1)
    

def main_loop(howlong):
    global PH_Sim, TDS_Sim, Water_Sim, errorPH, errorTDS, WPstate, waterss
    if bool(read_WL()) == True:
        waterss = 'Top Up Water !'
    else:
        waterss = 'Level OK'
        
    print('Reading Value...')
    #time.sleep(1)
    #set The loop
    #print('pH Value:', read_ph(), '\nTemp Value:', read_temp(), '\nTDS in ppm:', read_tds(ppm),'\nTDS in ec:', read_tds(ec))
    #print('pH Value :', read_ph(), round(chan0.voltage, 2), '\nTDS Value :', read_tds(), 'ppm\nEC Value :', read_ec(), '\nTemp Value :', read_temp(), 'Celcius\nWater Pump State :', read_WL())
    print('pH Value :', PH_Sim, '\nTDS Value:', read_tds(), 'ppm\nEC Value:', read_ec(), '\nWater Level:', waterss, '\nTemp Value:', read_temp(), 'Celcius\nWater Pump State:', WPstate)
    print('...............................')
    errorTDS = round(read_tds() - Desired_PPM, 1)
    errorPH = round(read_ph() - Desired_PH, 1)
    print('Desired PPM :', Desired_PPM, 'error:', errorTDS, '\nDesired PH :', Desired_PH, 'error:', errorPH)
    upload()
    start = millis()
    while (start + howlong > millis()):
        pass

WPstate = 0
lastMinute = 0

def water_pump(interval):
    global WPstate, td, lastMinute
    
    if td.minute - lastMinute >= interval:
        lastMinute = td.minute
    
        if WPstate == 0:
            WPstate = 1
        elif WPstate == 1:
            WPstate = 0
    GPIO.output(Water_P, WPstate)

def water_pump_switch():
    #print('Water Pump Manual:', water_pump_state)
    if water_pump_state == True:
        #water_pump(2000)
        GPIO.output(Water_P, 1)

a = 0
b = 0
lastTDS = 0
TDS = False

range_stableTDS = 50
pump_activationTDS = 1
Interval_WTDS = 15
waterpump_off = 1


def auto_TDS():
    global errorTDS, td, lastTDS, a, b, TDS
    if errorTDS < -range_stableTDS:
        print('Water Pump On and Calibrating TDS Value...\nNutrient A and B pumps Active in 1 Second...')
        TDS = False
        a = 0
        b = 0
        GPIO.output(Water_P, 1)
        GPIO.output(Nutrient_A, 1)
        GPIO.output(Nutrient_B, 1)
        time.sleep(pump_activationTDS)
        print('Wait 1 minute')#, 'TDS=', TDS, 'a =',a)
        GPIO.output(Nutrient_A, 0)
        GPIO.output(Nutrient_B, 0)
        time.sleep(Interval_WTDS)
        
    if errorTDS >= -range_stableTDS:
        #Both Nutrient Pumps are Off
        GPIO.output(Nutrient_A, 0)
        GPIO.output(Nutrient_B, 0)
        TDS = True
        print('TDS Satble...')#, 'TDS =', TDS, 'a =',a)
        if a == 1 and b == 0:
            GPIO.output(Water_P, 1)
            print('Water Pump Off after 5 minutes')
            time.sleep(1)
            if td.minute - lastTDS >= waterpump_off:
                print('selisih =', td.minute - lastTDS)
                if td.minute - lastTDS == waterpump_off:
                    b = 1
                    print('TDS value calibrating done')
                    WPstate = 1
                    time.sleep(3)
                lastTDS = td.minute
            else:
                GPIO.output(Water_P, 1)
                lastTDS = td.minute
                #print('lastTDS =', lastTDS)
                
    if errorTDS >range_stableTDS:
        GPIO.output(Nutrient_A, 0)
        GPIO.output(Nutrient_B, 0)
        TDS = True
        print('TDS Over...')#, 'TDS =', TDS, 'a =',a)
        time.sleep(1)
        
    if TDS == True and a == 0 and b == 0:
        print('TDS Value achieved... at', td.hour, ':', td.minute)
        a = 1
        GPIO.output(Water_P, 1)
        time.sleep(2)
        
c = 0
d = 0
lastPH = 0
PH = False

range_stablePH = 0.5
pump_activationPH = 1
Interval_WPH = 15


def auto_PH():
    global errorPH, td, lastPH, c, d, PH
    if errorPH < -range_stablePH:
        print('Water Pump On and Calibrating PH Value...\nAlkaline pumps Active in 1 Second...')
        PH = False
        c = 0
        d = 0
        GPIO.output(Water_P, 1)
        GPIO.output(pH_up, 1)
        GPIO.output(pH_down, 0)
        time.sleep(pump_activationPH)
        print('Wait 1 minute')#, 'PH =', PH, 'c =', c)
        GPIO.output(pH_up, 0)
        GPIO.output(pH_down, 0)
        time.sleep(Interval_WPH)
        
    if errorPH >= -range_stablePH:
        GPIO.output(pH_up, 0)
        GPIO.output(pH_down, 0)
        PH = True
        print('PH Satble...')#, 'PH =', PH, 'c =', c)
        if c == 1 and d == 0:
            GPIO.output(Water_P, 1)
            print('Water Pump Off after 5 minutes')
            time.sleep(1)
            if td.minute - lastPH >= waterpump_off:
                #print('selisih =', td.minute - lastPH)
                if td.minute - lastPH == waterpump_off:
                    d = 1
                    print('PH value calibrating done')
                    WPstate = 1
                    time.sleep(3)
                lastPH = td.minute
            else:
                GPIO.output(Water_P, 1)
                lastPH = td.minute
                #print('lastPH =', lastPH)
                
    if errorPH >range_stablePH:
        print('Water Pump On and Calibrating PH Value...\nAcid pumps Active in 1 Second...')
        PH = False
        c = 0
        d = 0
        GPIO.output(Water_P, 1)
        GPIO.output(pH_up, 0)
        GPIO.output(pH_down, 1)
        time.sleep(pump_activationPH)
        print('Wait 1 minute')#, 'PH =', PH, 'c =', c)
        GPIO.output(pH_up, 0)
        GPIO.output(pH_down, 0)
        time.sleep(Interval_WPH)
        
    if PH == True and c == 0 and d == 0:
        print('PH Value achieved... at', td.hour, ':', td.minute)
        c = 1
        GPIO.output(Water_P, 1)
        time.sleep(2)

try:
    while True:
        print('===============================')
        water_pump_switch()
        print('-------------------------------')
        main_loop(1000)
        print('-------------------------------')
        set_time_LED()
        print('-------------------------------')
        water_pump(1)
        print('>>>>>>>>>>>>>Status<<<<<<<<<<<<')
        #auto_TDS()
        #auto_PH()
        print('===============================\n.\n.\n.')

except KeyboardInterrupt:
    pass

client.loop_stop()
client.disconnect()
GPIO.cleanup()
