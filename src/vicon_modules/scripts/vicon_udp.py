import socket
import xml.etree.ElementTree as ET
import time
HOST = '192.168.0.255'  # IP address of the broadcaster
PORT = 12345  # The same port number used by the broadcaster


def parse_xml_data(xml_string):
    # Parse the XML string
    root = ET.fromstring(xml_string)
    
    # Initialize a list to store the objects' data
    objects = []
    
    # Iterate over each 'object' element in the XML
    for obj in root.findall('object'):
        # Extract the attributes of the object
        obj_id = obj.get('id')
        x = float(obj.get('x'))
        y = float(obj.get('y'))
        yaw = float(obj.get('yaw'))
        time= float(obj.get('time'))
        # Create a dictionary for the object's data
        object_data = {
            'id': obj_id,
            'x': x,
            'y': y,
            'yaw': yaw,
            'time':time 
        }
        
        # Append the object data to the list
        objects.append(object_data)
    
    return objects

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as subscriber_socket:
    subscriber_socket.bind((HOST, PORT))
    print(f"Subscriber listening on {HOST}:{PORT}")

    while True:
        data, addr = subscriber_socket.recvfrom(1024)  # Buffer size is 1024 bytes
        #print(f"Received message from {addr}: {data.decode()}")
        parsed=parse_xml_data(data.decode())
        print(parsed[0].get('x'),parsed[0].get('time')-time.time())
