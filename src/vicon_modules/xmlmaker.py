import xml.etree.ElementTree as ET





names=["robot1","robot2"]

root = ET.Element("robots")

for name in names:
    x = 0
    y = 0

    robot = ET.SubElement(root, "robot")
    ET.SubElement(robot, "name").text = name
    ET.SubElement(robot, "x").text = str(x)
    ET.SubElement(robot, "y").text = str(y)


    



print(ET['robots'])
print("XML files created successfully")
