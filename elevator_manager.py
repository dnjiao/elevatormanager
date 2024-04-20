import numpy as np
import pandas as pd
from rdp import rdp
import math
import configparser
from datetime import date, datetime
import logging
from logging.handlers import RotatingFileHandler
import sys
import os
import signal
import bme280
#import smbus2
from time import sleep
from argparse import ArgumentParser
import random
from queue import Queue
import time
import threading

CONFIG_FILE = './config.ini'
LOG_FILE = './logs/elevator_manager_log.txt'
DATA_FILE = '/data/pressures.csv'
PORT = 1
ADDRESS = 0x76
MAX_QUEUE_SIZE = 20

# Set up logging
rfh = RotatingFileHandler(
    filename=LOG_FILE, 
    mode='a',
    maxBytes=5*1024*1024,
    backupCount=2,
    encoding=None,
    delay=0
)
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(message)s',
    handlers=[
        rfh
    ]
)
logger = logging.getLogger('elevator_manager')

class SensorQueue(Queue):
    def __init__(self, maxsize):
        super().__init__(maxsize)

    def put(self, item):
        if self.full():
            self.get()
        super().put(item)

class ElevatorManager:

    def __init__(self):
        self.bp_default = None
        self.deltas = None
        self.std = None

    def load_calibration(self):
        config = configparser.ConfigParser()
        config.read(CONFIG_FILE)
        bp_default = float(config['DEFAULT']['base_pres'])
        deltas_str = config['DEFAULT']['deltas']
        deltas = [float(x) for x in deltas_str[1:-1].split(',')]
        std = float(config['DEFAULT']['std_dev'])

        self.bp_default = bp_default
        self.deltas = deltas
        self.std = std
    
    def smooth(self, arr, span): 
        return np.convolve(arr, np.ones(span * 2 + 1) / (span * 2 + 1), mode='same')

    def conv_smooth_with_average(self, arr, span):
        new_arr = self.smooth(arr, span)

        # The "average" part: shrinks the averaging window on the side that 
        # reaches beyond the data, keeps the other side the same size as given by "span"
        new_arr[0] = np.average(arr[:span])
        for i in range(1, span + 1):
            new_arr[i] = np.average(arr[:i + span])
            new_arr[-i] = np.average(arr[-i - span:])
        return new_arr

    def simplify(self, arr, span):
        y = self.conv_smooth_with_average(arr, span)
        x = range(len(y))

        coords = []
        for i in range(len(y)):
            coords.append((i, y[i]))

        coords = np.array(coords)
        simplified = rdp(coords, 0.1)
        return simplified
    
    def comp_stats(self, arr, i, j):
        len = j - i + 1
        shift = math.floor(len * 0.1)
        start = int(i + shift)
        end = int(j - shift)
        sub_arr = arr[start:end]
        return np.mean(sub_arr), np.median(sub_arr), np.std(sub_arr), np.min(sub_arr), np.max(sub_arr)

    def find_plateaus(self, pressures, simplified, thresh, labels=None):
        starts = []
        ends = []
        slopes = []
        means = []
        meds = []
        stds = []
        mins = []
        maxs = []
        floors = []
        sx, sy = simplified.T
        for i in range(len(sx) - 1):
            slope = (sy[i+1] - sy[i]) / (sx[i+1] - sx[i])
            if abs(slope) < thresh:
                mean, med, std, mn, mx = self.comp_stats(pressures, sx[i], sx[i+1])
                starts.append(sx[i])
                ends.append(sx[i+1])
                slopes.append(slope)
                means.append(mean)
                meds.append(med)
                stds.append(std)
                mins.append(mn)
                maxs.append(mx)
                if labels is not None:
                    floors.append(labels[int(sx[i])])
        
        rs_df = pd.DataFrame(
            {
                'Start': starts,
                'End': ends,
                'Slope': slopes,
                'Mean': means,
                'Median': meds,
                'STD': stds
            }
        )

        if labels is not None:         
            rs_df = pd.DataFrame(
                {
                'Start': starts,
                'End': ends,
                'Slope': slopes,
                'Mean': means,
                'Median': meds,
                'STD': stds,
                'Min': mins,
                'Max': maxs,
                'Floor': floors
                }
            )

        return rs_df
    
    def calibrate(self, pressures, span=5, slope_thresh=0.005, labels=None):
        x = range(len(pressures))
        y = self.conv_smooth_with_average(pressures, span)
        trajectory = []
        for i in range(len(x)):
            trajectory.append((x[i], y[i]))

        trajectory = np.array(trajectory)
        simplified = rdp(trajectory, 0.1)
        plat_df = self.find_plateaus(pressures, simplified, slope_thresh, labels)
        top_floor_idx = plat_df['Mean'].idxmin()
        plat_df = plat_df.iloc[:top_floor_idx]
        max_std = np.max(plat_df['STD'])
        pressures = plat_df['Mean'].values
        deltas = []
        base_pres = pressures[0]
        prev_pres = base_pres
        for i in range(1, len(pressures)):
            curr_pres = pressures[i]
            if curr_pres > prev_pres:
                break
            delta = base_pres - curr_pres
            deltas.append(delta)
            prev_pres = curr_pres
        
        today = date.today()
        dt_str = today.strftime('%m/%d/%y')
        config = configparser.ConfigParser()
        config.set('DEFAULT', 'base_pres', base_pres)
        config.set('DEFAULT', 'std_dev', max_std)
        config.set('DEFAULT', 'deltas', deltas)
        config.set('DEFAULT', 'date', dt_str)
        with open(CONFIG_FILE, 'w') as configfile:
            config.write(configfile)
    
    def get_elevator_status(self, bp_current, readings):
        std_2 = self.std * 2
        simplified = self.simplify(readings, 5)
        sx, sy = simplified.T
        cnt = len(sx)
        slope = (sy[cnt-1] - sy[cnt-2]) / (sx[cnt-1] - sx[cnt-2])
        if slope > 0.005:
            print('Down')
        elif slope < -0.005:
            print('Up')
        else:
            avg = (sy[cnt-1] + sy[cnt-2]) / 2
            print(avg)
            if abs(avg - bp_current)  < std_2:
                return 0
            for i, d in enumerate(self.deltas):
                pres = self.bp_default - float(d)
                if abs(avg - pres) < std_2:
                    return i + 1
                
    def get_current_base_pressure(self, duration=1):
        pressures = self.read_data_batch(duration=duration)
        x = range(len(pressures))
        y = self.conv_smooth_with_average(pressures)
        trajectory = []
        for i in range(len(x)):
            trajectory.append((x[i], y[i]))

        trajectory = np.array(trajectory)
        simplified = rdp(trajectory, 0.1)
        plat_df = self.find_plateaus(pressures, simplified, 0.005)
        bp = plat_df.iloc[0]['Mean']
        return bp
    
    def read_data_batch(self, freq=10, duration=-1):
        bus = smbus2.SMBus(PORT)
        bme280.load_calibration_params(bus, ADDRESS)
        with open(DATA_FILE, 'w') as f:
            f.write('index, pressure, timestamp\n')
        if duration == -1:
            stop_int = sys.maxsize
        else:
            stop_int = duration * freq
        for i in range(stop_int):
            bme280_data = bme280.sample(bus, ADDRESS)
            pressure  = bme280_data.pressure
            ts = datetime.now().strftime('%Y-%m-%dT%H:%M:%S.%f')[:-2]
            with open(DATA_FILE, 'a') as f:
                f.write(f'{i}, {pressure}, {ts}\n')
                print(pressure)
            sleep(1/freq)
    
    def read_data_batch_virtual(self, freq=10, duration=-1):
        with open(DATA_FILE, 'w') as f:
            f.write('index, pressure, timestamp\n')
        if duration == -1:
            stop_int = sys.maxsize
        else:
            stop_int = duration * freq
        for i in range(stop_int):
            pres = random.uniform(900.0, 1000.0)
            ts = datetime.now().strftime('%Y-%m-%dT%H:%M:%S.%f')[:-2]
            with open(DATA_FILE, 'a') as f:
                f.write(f'{i}, {pres}, {ts}\n')
            sleep(1/freq)
    
    def kill_process(self, app_name):

        for line in os.popen("ps ax | grep " + app_name + " | grep -v grep"): 
            fields = line.split()
            pid = fields[0] 
            os.kill(int(pid), signal.SIGKILL) 
        logger.info(f'{app_name} Successfully terminated')

squeue = SensorQueue(MAX_QUEUE_SIZE)

def collect_sensor_data(freq=10):
    while True:
        bus = smbus2.SMBus(PORT)
        bme280.load_calibration_params(bus, ADDRESS)
        bme280_data = bme280.sample(bus, ADDRESS)
        squeue.put(bme280_data.pressure)
        time.sleep(1/freq)

def process_sensor_data(em, bp):
    while True:
        lst = list(squeue.queue)
        queue_len = len(lst)
        if queue_len == 20:
            status = em.get_elevator_status(bp, lst)
            logger.info(f'Elevator status: {status}')
            print(status)
        time.sleep(1)


if __name__ == '__main__':
    
    em = ElevatorManager()
    if not os.path.exists(CONFIG_FILE):        
        logger.warning('config.ini does not exist. Must calibration first')
        if not os.path.exists(DATA_FILE):
            em.read_data_batch()
        data = pd.read_csv(DATA_FILE)
        em.calibrate(data['pressure'].values)
    em.load_calibration()

    bp = em.get_current_base_pressure()

    collect_thread = threading.Thread(target=collect_sensor_data)
    process_thread = threading.Thread(target=process_sensor_data, args=(em, bp))

    collect_thread.daemon = True
    process_thread.daemon = True

    collect_thread.start()
    process_thread.start()

    while True:
        time.sleep(0.5)





        


            
                


