

from __future__ import print_function
import socket
from vicon_dssdk import ViconDataStream
import time
import xml.etree.ElementTree as ET

client = ViconDataStream.RetimingClient()

HOST = "192.168.0.255"  # Replace with the IP address of your broadcast
PORT = 12345           # The port you want to broadcast on
# Sample data
segmentName = "segment1"
x = 10
y = 20
z = 30
roll = 45
pitch = 30
yaw = 60
message = "Hello World"

# Construct XML data
root = ET.Element('data')
object_element = ET.SubElement(root, 'object', id=str(segmentName))
object_element.set('x', str(x))
object_element.set('y', str(y))
object_element.set('yaw', str(yaw))

xml_data = ET.tostring(root)
# Create a UDP socket
with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client_socket:
    print("UDP socket created")
    client.Connect("localhost:801")  # Connect to the Vicon system

    # Check the version
    print('Version', client.GetVersion())

    client.SetAxisMapping(ViconDataStream.Client.AxisMapping.EForward, ViconDataStream.Client.AxisMapping.ELeft, ViconDataStream.Client.AxisMapping.EUp)
    xAxis, yAxis, zAxis = client.GetAxisMapping()
    print('X Axis', xAxis, 'Y Axis', yAxis, 'Z Axis', zAxis)

    # client.SetMaximumPrediction(10)
    print('Maximum Prediction', client.MaximumPrediction())
    
    while True:
        dataSend = ""
        try:
            client.UpdateFrame()
            root = ET.Element('data')
            subjectNames = client.GetSubjectNames()
            for subjectName in subjectNames:
                segmentNames = client.GetSegmentNames(subjectName)
                for segmentName in segmentNames:
                    dataXYZ = (client.GetSegmentGlobalTranslation(subjectName, segmentName))
                    dataEuler = (client.GetSegmentGlobalRotationEulerXYZ(subjectName, segmentName))
                    # Extract data to np.array, separated by commas
                    print
                    pos,_= dataXYZ
                    rot,_= dataEuler
                    
                    x,y,z=pos
                    roll,pitch,yaw=rot
                    yaw=yaw-2.823818432449826
                    #dataSend = " id=:" + str(segmentName) + " " + x + "," + y + "," + z + "," + roll + "," + pitch + "," + yaw + dataSend
                    #print(dataSend)
                    
                    object_element = ET.SubElement(root, 'object', id=str(segmentName))
                    object_element.set('time',str(time.time()))
                    object_element.set('x', str(x))
                    object_element.set('y', str(y))
                    object_element.set('yaw', str(yaw-2.823818432449826))
                    print(yaw)
                    xml_data = ET.tostring(root)
                    # Maintain the rate of 110 Hz
                    start_time = time.perf_counter()
                    while time.perf_counter() - start_time < 1/110:
                        pass
                        
        except ViconDataStream.DataStreamException as e:
            #print('Handled data stream error', e)
            pass
        
        # Broadcast the data using UDP
        #print(xml_data)
        client_socket.sendto(xml_data, (HOST, PORT))

print("Socket closed")
