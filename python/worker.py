import time

import PiDLidar
import numpy as np
from PyQt5.QtCore import (QObject, pyqtSlot, pyqtSignal)


class Worker(QObject):
    """
    LiDAR GUI worker class which fetches the samples from the LiDAR sensor (defined by the 'laser' object) and sends
    the GUI object with 'dataSignal' the fetched points in cartesian coordinates, the current 'maxRange'
    and the distance that the car can drive until it could collide with an obstacle.
    """
    start = pyqtSignal()  # Starts the run method
    dataSignal = pyqtSignal(object)  # Notifies GUI thread to update plot with object as data

    def __init__(self, laser, minRange, maxRange, halfCarWidth, turnRadius, autoScale):
        """
        Constructor of the LiDAR gui worker class.
        :param laser: laser object of the LiDAR sensor.
        :param minRange: Minimum recognizable distance from the sensor to the scanned object.
        :param maxRange: Initial maximum recognizable distance from the sensor to the scanned object.
        :param halfCarWidth: Half of the cars width in meters.
        :param turnRadius: Initial curve radius of the path for which the collision range will be calculated in meters.
                           Zero means driving in a straight line.
        :param autoScale: If True --> maxRange is changed every scan depending on the furthest distance of the scanned
                          samples.
                          If False --> maxRange won't be changed from inside this class.
        """
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
        """
        Gets LiDAR data from sensor in a loop and emits the 'dataSignal' signal when new data arrived necessary
        all information about the new data. Sleeps until the sensor completes the current rotation.
        """
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
                        dtype=[('x', float), ('y', float)]
                    )
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
                        dtype=[('x', float), ('y', float)]
                    )
                    # The configured max range
                    maxRange = self.maxRange

                # Find the collision range
                forwardCollisionPoint, backwardCollisionPoint = self.__findCollisionPoint(
                    data, maxRange, self.turnRadius)

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

    def __findCollisionPoint(self, points, maxRange, turnRadius):
        """
        Finds nearest points that are in the path of the car (defined by 'halfCarWidth' and 'turnRadius') in forward
        and backward direction.
        :param points: numpy array with 'x' and 'y' dimension containing the coordinates of the points.
        :param maxRange: configured maximum vision range of the sensor.
        :param turnRadius: The radius of the circle formed by the cars driving path.
        :return A 2-tuple containing the range the car can drive with the given 'turnRadius' until it could collide with
        and obstacle in forward and backward driving direction.
        """
        if turnRadius == 0:
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
            if turnRadius > 0:
                mirror = -1
            else:
                mirror = 1
            # Translate and convert
            translatedPoints = np.array(
                [(np.abs(mirror * (point['x'] - turnRadius) + 1j * point['y']),
                  np.angle(mirror * (point['x'] - turnRadius) + 1j * point['y']))
                 for point in points],
                dtype=[('radius', float), ('angle', float)]
            )

            # 4.) Select all points that are in the car path and store their
            #     path distance to the car (circular segment) and to the path center.
            collisionPoints = np.array(
                [(abs(turnRadius) - point['radius'],
                  point['angle'] * abs(turnRadius))
                 for point in translatedPoints
                 if self.halfCarWidth >= abs(turnRadius) - point['radius'] >= -self.halfCarWidth],
                dtype=[('pathDistance', float), ('carDistance', float)])

            # 5.) Find points that limit the free path by blocking vision behind them
            intersectionPoints = self.__intersect(translatedPoints, turnRadius)

            # 6.) Add them to the collision points.
            collisionPoints = np.append(collisionPoints, intersectionPoints)

        # 7.) Return the path distance to the first forward and backwards collision points.
        return self.__getMinCarDistance(collisionPoints, turnRadius, maxRange)

    def __intersect(self, points, turnRadius):
        """
        Calculates guaranteed free driving distance possible considering the points that are in the turn circle
        and block the vision of the driving path behind them.
        The calculation is done via intersection of the turn circle (defined by 'turnRadius') with a line from the
        original coordinate origin (the location of the sensor) through the given points.
        :param points: two-dimensional numpy array with 'radius' and 'angle' dimension, which are the polar coordinates
                       of the individual points. Their coordinate origin is in the middle of the turn circle.
        :param turnRadius: Radius of the turn circle. Negative means left turn, positive means right turn.
        :return: Numpy array with 'pathDistance' and 'carDistance' information about the free driving range
                 that is possible due to obstacles inside the turn circle blocking the vision behind them.
        """
        # Return array types
        ret_dt = np.dtype([('pathDistance', float), ('carDistance', float)])

        # 1.) Select all points (in polar coordinates with the middle of the circle as coordinate origin)
        #     that are in the turn path circle and convert them back to cartesian coordinates.

        # Factor to undo the previous x-axis mirroring.
        if turnRadius > 0:
            mirror = -1
        else:
            mirror = 1

        # Select and convert the points
        visionBlockingPoints = np.array(
            [(mirror * np.real(point['radius'] * np.exp(1j * point['angle'])),
              np.imag(point['radius'] * np.exp(1j * point['angle'])))
             for point in points
             if point['radius'] < abs(turnRadius) - self.halfCarWidth],
            dtype=[('x', float), ('y', float)]
        )

        # Return empty array if no points are in the turn path circle
        if visionBlockingPoints.size == 0:
            return np.array([], dtype=ret_dt)

        # 2.) Calculate the slope k for a line through the sample and the original
        #     coordinate origin (in current coordinate system (-turnRadius, 0)).
        kArr = np.array(
            [point['y'] / (point['x'] + turnRadius)
             for point in visionBlockingPoints]
        )

        # 3.) Take the lines with the steepest positive and negative slope.
        k = [kArr.argmin(), kArr.argmax()]

        # 4.) Calculate the second parameter d for the lines with the minimum and maximum slope.
        d = visionBlockingPoints['y'][k] - kArr[k]*visionBlockingPoints['x'][k]

        # 5.) Find the intersections of these lines with the turn circle.
        a = 1 + kArr[k] * kArr[k]
        b = 2 * kArr[k] * d
        c = d * d - turnRadius * turnRadius

        if turnRadius > 0:
            # Right turn --> take intersection with greater x value
            x = (-b + np.sqrt(b * b - 4 * a * c)) / (2 * a)
        else:
            # Left turn --> take intersection with smaller x value
            x = (-b - np.sqrt(b * b - 4 * a * c)) / (2 * a)

        y = kArr[k] * x + d

        # 6.) Convert the intersection points to polar coordinates
        z = mirror * x + 1j * y
        intersectionPoints = np.array(
            [(abs(turnRadius) - np.abs(z),
              np.angle(z) * abs(turnRadius))
             for z in z],
            dtype=ret_dt
        )

        return intersectionPoints

    def __getMinCarDistance(self, collisionPoints, turnRadius, maxRange):
        """
        Finds the guaranteed free driving ranges forwards and backwards and returns them as 2-tuple.
        They are limited either by points (collisionPoints) that are in the current driving path
        (described by turnRadius and self.halfCarWidth) or if there are no forward/backward 'collisionPoints',
        calculates the range from the given 'turnRadius' and 'maxRange'.
        :param collisionPoints: numpy array of points that would block the driving path. Must contain
               field 'carDistance' which represents the possible driving distance until the car would collide
               with the point.
        :param turnRadius: Radius of the turn circle. Negative means left circle, positive means right circle.
        :param maxRange: 2-tuple in the form of (guaranteed free driving range forwards, backwards)
        """
        minForwardRange = np.Inf
        minBackwardRange = -np.Inf

        # Find minimum forward and backward distance
        for point in collisionPoints:
            distance = point['carDistance']
            if distance > 0:
                if distance < minForwardRange:
                    minForwardRange = distance
            else:
                if distance > minBackwardRange:
                    minBackwardRange = distance

        if turnRadius == 0:
            # If driving path is a straight line
            if minForwardRange == np.Inf:
                # and no point was found in forward direction set to maxRange
                minForwardRange = maxRange
            if minBackwardRange == -np.Inf:
                # and no point was found in backward direction set to -maxRange
                minBackwardRange = -maxRange
        else:
            # If driving path is a curve
            if (abs(turnRadius) + self.halfCarWidth) * 2 > maxRange:
                # and driving full circle is not possible due to maxRange
                # Find the path distance until the outer edge of the car would touch the vision range circle
                # by intersections of the two circles
                outerTurnRadius = abs(turnRadius) + self.halfCarWidth
                outerTurnRadiusSqu = outerTurnRadius * outerTurnRadius
                turnRadiusSqu = turnRadius * turnRadius
                maxRangeSqu = maxRange * maxRange
                alpha = np.arccos((turnRadiusSqu + outerTurnRadiusSqu - maxRangeSqu)
                                  / (2*outerTurnRadius*abs(turnRadius)))
                turnArc = alpha * abs(turnRadius)

                # Use that path distance if no nearer collision point was found
                if turnArc < minForwardRange:
                    minForwardRange = turnArc
                if -turnArc > minBackwardRange:
                    minBackwardRange = -turnArc
            else:
                # and driving full circle could be possible due to maxRange
                if collisionPoints.size == 0:
                    # and there are no collision points
                    # Set the range to full circle
                    minForwardRange = abs(turnRadius) * 2 * np.pi
                    minBackwardRange = -abs(turnRadius) * 2 * np.pi
                else:
                    # but driving full circle is not possible due to collision points
                    if minForwardRange == np.Inf and not minBackwardRange == -np.Inf:
                        # and there is at least one collision point in the lower half of the turn circle
                        # but not in the upper half. Extend the minForwardRange to more than half the
                        # turn circle arc length.
                        minForwardRange = 2*np.pi*abs(turnRadius) + collisionPoints['carDistance'].min()
                    elif not minForwardRange == np.Inf and minBackwardRange == -np.Inf:
                        # and there is at least one collision point in the upper half of the turn circle
                        # but not in the lower half. Extend the minBackwardRange to more than half the
                        # turn circle arc length.
                        minBackwardRange = -2*np.pi*abs(turnRadius) + collisionPoints['carDistance'].max()

        return minForwardRange, minBackwardRange
