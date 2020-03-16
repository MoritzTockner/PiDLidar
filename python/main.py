import PiDLidar
import serial.tools.list_ports

laser = PiDLidar.CYdLidar()
port = '/dev/ttyUSB0'
baudrate = 128000 # defined in datasheet
fixedResolution = False
reversion = False
inverted = False
autoReconnect = True
singleChannel = False
lidarType = 1
maxAngle = 180
minAngle = -180
minRange = 0.01
maxRange = 64.0
ignoreArray = []

# lidar serial port
laser.setSerialPort(port)

# lidar baudrate
laser.setSerialBaudrate(baudrate)

# fixed angle resolution
laser.setFixedResolution(fixedResolution)

# rotate 180
laser.setReversion(reversion) #rotate 180

# Counterclockwise
laser.setInverted(inverted) #ccw
laser.setAutoReconnect(autoReconnect) # hot plug

# two-way communication
laser.setSingleChannel(singleChannel)

# triangulation lidar
laser.setLidarType(lidarType)
# unit: °
laser.setMaxAngle(maxAngle)
laser.setMinAngle(minAngle)

# unit: m
laser.setMinRange(minRange)
laser.setMaxRange(maxRange)

laser.setIgnoreArray(ignoreArray)

ret = laser.initialize()

if ret:
    ret = laser.turnOn()

while ret:
    hardError = False
    scan = PiDLidar.LaserScan()
    if laser.doProcessSimple(scan, hardError):
        print("Scan received {}: {} ranges is [{}]Hz".format(scan.stamp, len(scan.points), 1.0 / scan.config.scan_time))
        for i in range(10):
            angle = scan.points[i].angle
            point_range = scan.points[i].range
            intensity = scan.points[i].intensity
            print("Angle: {}, Range: {}, Intensity: {}".format(angle, point_range, intensity))
    else:
        print("Failed to get Lidar Data")
