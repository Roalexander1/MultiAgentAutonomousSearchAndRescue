import socket
import time
import threading
from run import semaphore, location

# Tello drone's correct IP and command port
TELLO_IP = "192.168.10.1"
TELLO_PORT = 8889
LOCAL_PORT = 9000  # Port to receive responses

class Tello(object):

    def __init__(self):
        # Create a UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("", LOCAL_PORT))  # Bind to receive data

        self.speed = 0
        self.flag_open = True

        self.location = [0,0] # in m, m
        self.height = 4
        self.width = 4
        self.increment = 1

        self.x = 0
        self.y = 0

    def __del__(self):
        self.sock.close()

    def send_command(self, command, wait=3):
        """Send a command to Tello and wait for a response."""
        print(f"Sending: {command}")
        self.sock.sendto(command.encode('utf-8'), (TELLO_IP, TELLO_PORT))

        self.sock.settimeout(wait)  # Wait for a response
        try:
            response, _ = self.sock.recvfrom(1024)
            print(f"Response: {response.decode()}")

        except socket.timeout:
            print("No response received")
    
    def move(self, direction, distance):
        distance = int(round(distance * 100)) # 100 for 1 meter
        
        self.send_command(f'{direction} {distance}')

    # Threaded function of execution for the scanning
    def drone_scan(self):
        semaphore.acquire()
        try:
            print("Scanning")
            # Sets drone to SDK mode
            self.send_command("command")
            self.send_command("takeoff")
            time.sleep(5)

            # Traverse x
            for self.x in range(self.width):

                # Traverse y
                for self.y in range(self.height):
                    if (self.flag_open == False):
                        self.location = [self.x, self.y]
                        self.send_location()
                        break

                    # Note goes forward and then back
                    if (self.x % 2 == 0):
                        self.move('forward', self.increment)
                        time.sleep(3)
                    else:
                        self.move('back', self.increment)
                        time.sleep(3)
                
                if (self.flag_open == False):
                    self.location = [self.x, self.y]
                    self.send_location()
                    break

                self.move('right', self.increment)
                time.sleep(3)

            # Go back to starting location
            self.move('left', self.increment * self.x)
            time.sleep(5)
            self.move('back', self.increment * self.y)
            time.sleep(5)

            self.send_command("land")
            time.sleep(2)
        finally:
            semaphore.release()

            
    def send_location(self):
        # Change value of location in run to hold final location
        print("x :", self.location[0], "y :",  self.location[1])
        location[0] = self.location[0]
        location[1] = self.location[1]

    # Threaded function of execution for the Interrupt
    def drone_interrupt(self):
        semaphore.acquire()
        try:
            print("Interrupt")

            # interrupt 
            while(True):
                # input() == "i" or 
                if (self.x == 2 and self.y == 3):
                    self.flag_open = False
        finally:
            semaphore.release()

    def drone_start(self):
        scanning_thread = threading.Thread(target=self.drone_scan, args=())
        interrupt_thread = threading.Thread(target=self.drone_interrupt, args=())

        scanning_thread.start()
        interrupt_thread.start()
            
def init_drone():
    drone = Tello()
    drone.drone_start()