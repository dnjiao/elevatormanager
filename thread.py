import threading
import time

dataset = []

# Dummy function to simulate sensor data collection
def collect_sensor_data():
    while True:
        # Replace this with actual sensor reading code
        data = 10  # Dummy data for demonstration
        dataset.append(data)
        print("Collected data:", data)
        time.sleep(1/2)  # Simulate sensor reading every second

# Dummy function to simulate data processing
def process_data():
    while True:
        # Replace this with actual data processing code
        # For demonstration, just print a message
        print("Processing data...")
        print('Current', dataset)
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