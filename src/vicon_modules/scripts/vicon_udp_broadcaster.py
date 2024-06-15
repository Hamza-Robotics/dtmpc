from __future__ import print_function
import socket
from vicon_dssdk import ViconDataStream
import time
import xml.etree.ElementTree as ET
import numpy as np

client = ViconDataStream.RetimingClient()

# List of broadcast IP addresses of your network
HOSTS = ["192.168.1.68", "192.168.1.69", "192.168.1.70"]  # Example IP addresses
PORT = 1234  # The port you want to broadcast on

# Create a UDP socket
with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as client_socket:
    # Enable broadcasting mode
    client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    print("UDP socket created with broadcasting enabled")

    client.Connect("localhost:801")  # Connect to the Vicon system

    # Check the version
    print('Version', client.GetVersion())

    client.SetAxisMapping(ViconDataStream.Client.AxisMapping.EForward,
                          ViconDataStream.Client.AxisMapping.ELeft,
                          ViconDataStream.Client.AxisMapping.EUp)
    xAxis, yAxis, zAxis = client.GetAxisMapping()
    print('X Axis', xAxis, 'Y Axis', yAxis, 'Z Axis', zAxis)

    while True:
        try:
            loop_starttime = time.time()
            client.UpdateFrame()
            root = ET.Element('data')
            subjectNames = client.GetSubjectNames()
            for subjectName in subjectNames:
                segmentNames = client.GetSegmentNames(subjectName)
                for segmentName in segmentNames:
                    dataXYZ = client.GetSegmentGlobalTranslation(subjectName, segmentName)
                    dataEuler = client.GetSegmentGlobalRotationEulerXYZ(subjectName, segmentName)

                    pos, _ = dataXYZ
                    rot, _ = dataEuler

                    x, y, z = pos
                    roll, pitch, yaw = rot

                    theta = 0
                    x = (x * np.cos(theta) - y * np.sin(theta))
                    y = (y * np.sin(theta) + x * np.cos(theta))
                    yaw = (yaw + theta)

                    object_element = ET.SubElement(root, 'object', id=str(segmentName))
                    object_element.set('time', str(time.time()))
                    object_element.set('x', str(x))
                    object_element.set('y', str(y))
                    object_element.set('yaw', str(yaw))

            xml_data = ET.tostring(root)

            # Maintain the rate of 110 Hz
            start_time = time.perf_counter()
            while time.perf_counter() - start_time < 1 / 110:
                pass

        except ViconDataStream.DataStreamException as e:
            print('Handled data stream error', e)
            continue

        loop_endtime = time.time()

        # Broadcast the data using UDP to each host in HOSTS
        for host in HOSTS:
            client_socket.sendto(xml_data, (host, PORT))

print("Socket closed")
