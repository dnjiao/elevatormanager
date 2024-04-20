from queue import Queue

class SensorQueue(Queue):
    def __init__(self, maxsize):
        super().__init__(maxsize)

    def put(self, item):
        if self.full():
            self.get()
        super().put(item)

# Create a queue with a size limit of 3
queue = SensorQueue(3)

# Add some items to the queue
queue.put(1)
queue.put(2)
queue.put(3)

# Try to add another item to the queue
queue.put(4)

# Print the contents of the queue
print(list(queue.queue))