from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import (QThread, QObject, pyqtSlot, pyqtSignal)
import numpy as np
import PiDLidar
import time


class Worker(QObject):

    start = pyqtSignal()        # Starts the run method
    dataSignal = pyqtSignal(object)     # Notifies GUI thread to update plot with object as data

    def __init__(self, laser, minRange, maxRange, halfCarWidth, turnRadius, autoScale):
        QThread.__init__(self)
        self.laser = laser      # CYdLidar class object
        self.finish = False     # Is set to true when stop button is pressed to close worker thread
        self.maxRange = maxRange
        self.minRange = minRange
        self.autoScale = autoScale
        self.turnRadius = turnRadius     # Curve radius of the path for which the collision range will be calculated
        self.halfCarWidth = halfCarWidth       # Car width in metres
        self.start.connect(self.run)    # Run method is executed if start signal is emitted

    #@pyqtSlot()
    #def run(self):
    #    import cProfile
    #    cProfile.runctx('self.run_()', globals(), locals(), 'profileWorkerThread')


    @pyqtSlot()
    def run(self):
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
                forwardCollisionPoint, backwardCollisionPoint = self.__findCollisionPoint(data)

                # Notify GUI process to plot the sampled data
                # and update the scaling of the plot if necessary
                self.dataSignal.emit({
                    'points' : data,
                    'maxRange' : maxRange,
                    'collisionPoint' : {
                        'forward' : forwardCollisionPoint,
                        'backward' : backwardCollisionPoint
                    }
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


    def __findCollisionPoint(self, points):
        if self.turnRadius == 0:
            # No normal vector is needed. Just check x values of the samples.
            collisionPoints = np.array(
                [(point['x'], point['y']) \
                 for point in points \
                 if point['x'] <= self.halfCarWidth \
                 and point['x'] >= -self.halfCarWidth \
                 and abs(point['y']) >= self.minRange], \
                dtype=[('pathDistance', float), ('carDistance', float)])

        else:
            # Translate all samples so that the middle of the circle is
            # the new coordinate origin and convert to polar coordinates.
            translatedPoints = np.array(
                [(np.sqrt((point['x'] - self.turnRadius)**2 + point['y']**2), \
                  np.arctan(point['y']/(point['x'] - self.turnRadius))) \
                 for point in points], \
                dtype=[('radius', float), ('angle', float)])

            # Select all points that are in the car path and store their
            # path distance to the car (circular segment) and to the path
            collisionPoints = np.array(
                [(abs(self.turnRadius) - point['radius'], \
                  point['angle'] * abs(self.turnRadius))
                 for point in translatedPoints \
                 if point['radius'] <= abs(self.turnRadius) + self.halfCarWidth \
                 and point['radius'] >= abs(self.turnRadius) - self.halfCarWidth], \
                dtype=[('pathDistance', float), ('carDistance', float)])

        firstForwardCollisionPoint, firstBackwardCollisionPoint \
            = self.__getMinCarDistance(collisionPoints)

        return firstForwardCollisionPoint, firstBackwardCollisionPoint


    def __getMinCarDistance(self, collisionPoints):
        """ Finds the points with the minimum positive carDistance
            and the maximum negative carDistance. """
        minForwardPoint  =  np.Inf
        minBackwardPoint = -np.Inf

        for point in collisionPoints:
            distance = point['carDistance']
            if distance > 0:
                if distance < minForwardPoint:
                    minForwardPoint = distance
            else:
                if distance > minBackwardPoint:
                    minBackwardPoint = distance

        # If no point was found set to max range
        if self.turnRadius == 0:
            if minForwardPoint == np.Inf:
                minForwardPoint = self.maxRange
            if minBackwardPoint == -np.Inf:
                minBackwardPoint = -self.maxRange
        else:
            if minForwardPoint == np.Inf:
                minForwardPoint = abs(self.turnRadius) * np.pi
            if minBackwardPoint == -np.Inf:
                minBackwardPoint = -abs(self.turnRadius) * np.pi

        return minForwardPoint, minBackwardPoint

