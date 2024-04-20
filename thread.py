import threading
import time
from queue import Queue
import random

class SensorQueue(Queue):
    def __init__(self, maxsize):
        super().__init__(maxsize)

    def put(self, item):
        if self.full():
            self.get()
        super().put(item)

sq = SensorQueue(10)

# Dummy function to simulate sensor data collection
def collect_sensor_data():
    while True:
        # Replace this with actual sensor reading code
        data = random.randint(1, 100)  # Dummy data for demonstration
        sq.put(data)
        print("Collected data:", data)
        time.sleep(1/2)  # Simulate sensor reading every second

# Dummy function to simulate data processing
def process_data():
    while True:
        # Replace this with actual data processing code
        # For demonstration, just print a message
        print("Processing data...")
        lst = list(sq.queue)
        q_len = len(lst)
        print('queue size', q_len)
        if q_len == 10:
            print('Current', lst)
        time.sleep(2)  # Simulate processing taking 2 seconds

def multi_thread():

    # Start threads for data collection and processing
    collect_thread = threading.Thread(target=collect_sensor_data)
    process_thread = threading.Thread(target=process_data)

    # Set threads as daemon so they will exit when the main thread exits
    collect_thread.daemon = True
    process_thread.daemon = True

    # Start threads
    collect_thread.start()
    process_thread.start()

    # Keep the main thread alive
    while True:
        time.sleep(1)

multi_thread()