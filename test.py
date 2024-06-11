import socket
import json
import numpy as np

class Receiver:
    def __init__(self):
        self.HOST = ''  # Empty string means to listen on all available interfaces
        self.PORT = 11111
        self.receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.receiver.bind((self.HOST, self.PORT))
        self.BUFFER_SIZE = 65536  # Increase buffer size to handle larger data
    def receive_solution(self):
        while True:

            try:
                data, _ = self.receiver.recvfrom(self.BUFFER_SIZE)  # Increased buffer size
                solution_json = data.decode('utf-8')
                solution_list = json.loads(solution_json)
                solution_array = np.array(solution_list)
                print("Received solution:", np.shape(solution_array))
            except json.JSONDecodeError as e:
                print(f"JSON decode error: {e}")
            except Exception as e:
                print(f"An error occurred: {e}")

# Example usage
if __name__ == "__main__":
    receiver = Receiver()
    receiver.receive_solution()
