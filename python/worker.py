from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import (QThread, QObject, pyqtSlot, pyqtSignal)
import numpy as np
import PiDLidar
import time


class Worker(QObject):

    start = pyqtSignal()        # Starts the run method
    dataSignal = pyqtSignal(object)     # Notifies GUI thread to update plot with object as data

    def __init__(self, laser, minRange, maxRange, halfCarWidth, driveAngle, autoScale):
        QThread.__init__(self)
        self.laser = laser      # CYdLidar class object
        self.finish = False     # Is set to true when stop button is pressed to close worker thread
        self.maxRange = maxRange
        self.minRange = minRange
        self.autoScale = autoScale
        self.driveAngle = driveAngle     # Angle in degree for which collision range will be calculated
        self.halfCarWidth = halfCarWidth       # Car width in metres
        self.start.connect(self.run)    # Run method is executed if start signal is emitted

    @pyqtSlot()
    def run(self):
        import cProfile
        cProfile.runctx('self.run_()', globals(), locals(), 'profileWorkerThread')


    #@pyqtSlot()
    def run_(self):
        """ Gets lidar data from sensor in a loop and emits a signal when new data arrived.
            Sleeps for a constant time before fetching new data from the sensor. """
        scan = PiDLidar.LaserScan()
        hardError = False
        fetchAndSignalTime = 0
        maxRange = None

        # Fetch data until finish is set to False by StopButton
        while not self.finish:
            # If new data arrived, store it in scan
            if self.laser.doProcessSimple(scan, hardError):
                # Measure time of data processing for calculation of the thread timeout afterwards
                fetchAndSignalTime = time.time()
                # When autoscale is enabled
                if self.autoScale:
                    # Convert all samples to cartesian coordinate system,
                    # except the ones under 0.1 meters. They are mapped to 0 by the sensor
                    # because that is below the minimal range.
                    data = np.array(
                        [(point.range * np.cos(point.angle + np.pi/2), \
                          point.range * np.sin(point.angle + np.pi/2)) \
                         for point in scan.points \
                         if point.range >= self.minRange], \
                        dtype=[('x', float), ('y', float)])
                    # Find sample with the highest range, to adjust autoscaling accordingly
                    maxRange = max([point.range for point in scan.points])
                else:
                    # Convert all samples to cartesian coordinate system
                    # that are under the configured maximum range and above 0.1 meters.
                    data = np.array(
                        [(point.range * np.cos(point.angle + np.pi/2),
                          point.range * np.sin(point.angle + np.pi/2)) \
                         for point in scan.points \
                         if point.range <= self.maxRange
                         and point.range >= self.minRange], \
                        dtype=[('x', float), ('y', float)])
                    # The configured max range
                    maxRange = self.maxRange

                # Find the collision range
                collisionPoint = self.findCollisionPoint(data)

                # Notify GUI process to plot the sampled data
                # and update the scaling of the plot if necessary
                self.dataSignal.emit({
                    'points' : data,
                    'maxRange' : maxRange,
                    'collisionPoint' : collisionPoint
                })
                fetchAndSignalTime = time.time() - fetchAndSignalTime
            if hardError:
                self.finish = True
                print('Hard error in CYdLidar.doProcessSimple().')

            # Thread sleeps for approximately one rotation of the sensor until it tries to fetch new data.
            if scan.config.scan_time > fetchAndSignalTime:
                time.sleep(scan.config.scan_time - fetchAndSignalTime)
            else:
                print('Skipped a rotation')


    def findCollisionPoint(self, points):
        if self.driveAngle == 0:
            # No normal vector is needed. Just check x values of the samples.
            collisionPoints = np.array(
                [(point['x'], point['y']) \
                 for point in points \
                 if point['x'] <= self.halfCarWidth \
                 and point['x'] >= -self.halfCarWidth \
                 and point['y'] >= self.minRange], \
                dtype=[('pathDistance', float), ('carDistance', float)])
            firstCollisionPoint = self.getMinCarDistance(collisionPoints)
        else:
            # Create a unit vector pointing in the direction of the current
            # drive angle.
            pathVec = (np.cos(np.deg2rad(self.driveAngle + 90)),
                       np.sin(np.deg2rad(self.driveAngle + 90)))
            # Calculate a normal vector of the current path angle
            normalVec = (pathVec[1], -pathVec[0])
            # Calculate the distance from the points to the driving path line
            # via the scalar product of the normal vector and the point vectors
            # and the distance from these points to the car (= coordinate origin)
            pointDistances = np.array(
                [(point['x']*normalVec[0] + point['y']*normalVec[1], \
                  point['x']*pathVec[0] + point['y']*pathVec[1]) \
                 for point in points], \
                 dtype=[('pathDistance', float), ('carDistance', float)])
            # Select the points with which the car would collide on its path
            collisionPoints = np.array(
                [(pointDistance['pathDistance'], \
                  pointDistance['carDistance']) \
                 for pointDistance in pointDistances \
                 if abs(pointDistance['pathDistance']) <= self.halfCarWidth \
                 and pointDistance['carDistance'] >= self.minRange], \
                 dtype=[('pathDistance', float), ('carDistance', float)])
            # Find minimum distance from a collision point to the car
            firstCollisionPoint = self.getMinCarDistance(collisionPoints)

        return firstCollisionPoint


    def getMinCarDistance(self, collisionPoints):
        if collisionPoints.size != 0:
            minPoint = collisionPoints[0]
        else:
            minPoint = np.array(
                [(0, self.maxRange)],
                dtype=[('pathDistance', float),
                       ('carDistance', float)])

        for point in collisionPoints:
            if point['carDistance'] < minPoint['carDistance']:
                minPoint = point

        return minPoint

