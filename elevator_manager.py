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


CONFIG_FILE = './config.ini'
LOG_FILE = './logs/elevator_manager_log.txt'
DATA_FILE = '/data/pressures.csv'
PORT = 1
ADDRESS = 0x76

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

class ElevatorManager:

    def __init__(self):
        bp_default, deltas, std, floors = self.load_calibration()
        self.bp_default = bp_default
        self.deltas = deltas
        self.std = std
        self.floors = floors


    def load_calibration(self):
        config = configparser.ConfigParser()
        try:
            config.read(CONFIG_FILE)
        except:
            logger.error('No config.ini found. Must run calibration first')
            sys.exit()
        bp_default = float(config['DEFAULT']['base_pres'])
        deltas_str = config['DEFAULT']['deltas']
        deltas = [float(x) for x in deltas_str[1:-1].split(',')]
        std = float(config['DEFAULT']['std_dev'])
        floor_str = config['DEFAULT']['floors']
        floors = [int(x) for x in floor_str[1:-1].split(',')]

        return bp_default, deltas, std, floors
    
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
                return self.floors[0]
            for i, d in enumerate(self.deltas):
                pres = self.bp_default - float(d)
                if abs(avg - pres) < std_2:
                    return self.floors[i + 1]
                
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
            ambient_temperature = bme280_data.temperature
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

if __name__ == '__main__':
    p = ArgumentParser()
    p.add_argument('-r', '--record', action='store_true')
    p.add_argument('-c', '--calibrate', action='store_true')
    p.add_argument('-d', '--detect', action='store_true')
    p.add_argument('-k', '--kill', action='store_true')
    args = p.parse_args()

    em = ElevatorManager()
    if args.record:
        em.read_data_batch()
    if args.kill:
        em.kill_process('elevator_manager')
    if args.calibrate:
        try:
            df = pd.read_csv(DATA_FILE)
        except:
            print(f'{DATA_FILE} does not exist.')
            sys.exit()
        pressures = df['pressure'].values
        em.calibrate(pressures)
    if args.detect:




        


            
                


