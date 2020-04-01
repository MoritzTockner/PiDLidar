import time

import PiDLidar
import numpy as np
from PyQt5.QtCore import (QThread, QObject, pyqtSlot, pyqtSignal)


class Worker(QObject):
    start = pyqtSignal()  # Starts the run method
    dataSignal = pyqtSignal(object)  # Notifies GUI thread to update plot with object as data

    def __init__(self, laser, minRange, maxRange, halfCarWidth, turnRadius, autoScale):
        QObject.__init__(self)
        self.laser = laser  # CYdLidar class object
        self.finish = False  # Is set to true when stop button is pressed to close worker thread
        self.maxRange = maxRange
        self.minRange = minRange
        self.autoScale = autoScale
        self.turnRadius = turnRadius  # Curve radius of the path for which the collision range will be calculated
        self.halfCarWidth = halfCarWidth  # Car width in metres
        self.start.connect(self.run)  # Run method is executed if start signal is emitted

    # @pyqtSlot()
    # def run(self):
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
                        [(point.range * np.cos(point.angle + np.pi / 2),
                          point.range * np.sin(point.angle + np.pi / 2))
                         for point in scan.points
                         if point.range >= self.minRange],
                        dtype=[('x', float), ('y', float)])
                    # Find sample with the highest range, to adjust autoscaling accordingly
                    maxRange = max([point.range for point in scan.points])
                else:
                    # Convert all samples to cartesian coordinate system
                    # that are under the configured maximum range and above 0.1 meters.
                    data = np.array(
                        [(point.range * np.cos(point.angle + np.pi / 2),
                          point.range * np.sin(point.angle + np.pi / 2))
                         for point in scan.points
                         if self.maxRange >= point.range >= self.minRange],
                        dtype=[('x', float), ('y', float)])
                    # The configured max range
                    maxRange = self.maxRange

                # Find the collision range
                forwardCollisionPoint, backwardCollisionPoint = self.__findCollisionPoint(data)

                # Notify GUI process to plot the sampled data
                # and update the scaling of the plot if necessary
                self.dataSignal.emit({
                    'points': data,
                    'maxRange': maxRange,
                    'collisionPoint': {
                        'forward': forwardCollisionPoint,
                        'backward': backwardCollisionPoint
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
        """ Finds nearest points that are in the path of the car (defined by halfCarWidth and turnRadius)
            in forward and backward direction. """
        if self.turnRadius == 0:
            # No normal vector is needed. Just check x values of the samples.
            collisionPoints = np.array(
                [(point['x'], point['y'])
                 for point in points
                 if abs(point['x']) <= self.halfCarWidth
                 and abs(point['y']) >= self.minRange],
                dtype=[('pathDistance', float), ('carDistance', float)])

        else:
            # 1.) translate all samples so that the middle of the circle is
            #     the new coordinate origin (shift them to the left).
            # 2.) Only for a right turn:
            #     Mirror them over the x axis so that the 0 degree angle
            #     is in the original coordinate systems origin.
            # 3.) Convert to polar coordinates.

            # Create factor to mirror only for a right turn
            if self.turnRadius > 0:
                mirror = -1
            else:
                mirror = 1
            # Translate and convert
            translatedPoints = np.array(
                [(np.abs(mirror * (point['x'] - self.turnRadius) + 1j * point['y']),
                  np.angle(mirror * (point['x'] - self.turnRadius) + 1j * point['y']))
                 for point in points],
                dtype=[('radius', float), ('angle', float)]
            )

            # 4.) Select all points that are in the car path and store their
            #     path distance to the car (circular segment) and to the path center.
            collisionPoints = np.array(
                [(abs(self.turnRadius) - point['radius'],
                  point['angle'] * abs(self.turnRadius))
                 for point in translatedPoints
                 if self.halfCarWidth >= abs(self.turnRadius) - point['radius'] >= -self.halfCarWidth],
                dtype=[('pathDistance', float), ('carDistance', float)])

        # 5.) Find the path distance to the first forward and backwards collision points.
        firstForwardCollisionPoint, firstBackwardCollisionPoint \
            = self.__getMinCarDistance(collisionPoints)

        return firstForwardCollisionPoint, firstBackwardCollisionPoint

    def __getMinCarDistance(self, collisionPoints):
        """ Finds the points with the minimum positive carDistance
            and the maximum negative carDistance. """
        minForwardPoint = np.Inf
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
            if (abs(self.turnRadius) + self.halfCarWidth) * 2 <= self.maxRange:
                # Full 180 degree turn is possible
                if minForwardPoint == np.Inf:
                    minForwardPoint = self.turnRadius * np.pi
                if minBackwardPoint == -np.Inf:
                    minBackwardPoint = -self.turnRadius * np.pi
            else:
                # Find the path distance until the outer edge of the car would touch the vision range circle
                # by intersections of the two circles
                if minForwardPoint == np.Inf or minBackwardPoint == -np.Inf:
                    outerTurnRadius = abs(self.turnRadius) + self.halfCarWidth
                    outerTurnRadiusSqu = outerTurnRadius * outerTurnRadius
                    turnRadiusSqu = self.turnRadius * self.turnRadius
                    maxRangeSqu = self.maxRange * self.maxRange
                    alpha = np.arccos((turnRadiusSqu + outerTurnRadiusSqu - maxRangeSqu)
                                      / (2*outerTurnRadius*abs(self.turnRadius)))

                    if minForwardPoint == np.Inf:
                        minForwardPoint = alpha * abs(self.turnRadius)
                    if minBackwardPoint == -np.Inf:
                        minBackwardPoint = -alpha * abs(self.turnRadius)

        return minForwardPoint, minBackwardPoint
