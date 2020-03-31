import sys
from PyQt5.QtWidgets import (QTextEdit, QSizePolicy, QSlider, QCheckBox, QWidget, QPushButton, QHBoxLayout, QVBoxLayout,
                             QApplication)
from PyQt5.QtCore import (Qt, QThread, QObject, pyqtSlot, pyqtSignal, QPointF)
import pyqtgraph as pg

import numpy as np
import PiDLidar
import worker
import time


class LidarGUI(QWidget):

    def __init__(self, parent=None):
        QWidget.__init__(self, parent)
        self.started = False
        self.laser = None
        self.scalingCheck = None
        self.runtime = None
        self.scalings = range(1, 11)
        self.scalingSliderMax = len(self.scalings) - 1
        self.scalingSliderMin = 0
        self.gridCircles = []
        self.nrOfGridCircles = 10
        self.initialMinRange = 0.1
        self.initialMaxRange = self.scalings[0]
        self.halfCarWidth = 0.1  # Car width in meters
        self.turnRadiusMin = 0.5
        self.turnRadiusMax = 100
        self.nrOfTurnRadii = 50
        self.turnRadii = self.createTurnRadiiLookupTable()
        self.initialTurnRadius = 0  # Angle in degree for which collision range will be calculated
        self.turnRadius = self.initialTurnRadius
        self.initLidar()
        self.initUI()

    def __del__(self):
        """ Stop Lidar and disconnect serial connection. """
        self.laser.turnOff()
        self.laser.disconnecting()

    def createTurnRadiiLookupTable(self):
        # nrOfTurnRadii different radii for every direction,
        # +1 for the case turnRadius = Inf --> straight line
        turnRadii = np.zeros(self.nrOfTurnRadii * 2 + 1)
        turnRadii[:self.nrOfTurnRadii] = -1 * np.logspace(
            np.log10(self.turnRadiusMin),
            np.log10(self.turnRadiusMax),
            self.nrOfTurnRadii)
        turnRadii[self.nrOfTurnRadii] = 0
        turnRadii[self.nrOfTurnRadii + 1:] = np.logspace(
            np.log10(self.turnRadiusMax),
            np.log10(self.turnRadiusMin),
            self.nrOfTurnRadii)
        return turnRadii

    def initWorkerThread(self):
        """ Creates a new thread and assigns an instance of the Worker class to it. """
        print('Initialize worker thread ...')
        # Create a new thread
        self.thread = QThread()
        # Start thread
        self.thread.start()
        # Create worker object that updates lidar data plot
        self.worker = worker.Worker(
            self.laser,
            self.initialMinRange,
            self.initialMaxRange,
            self.halfCarWidth,
            self.turnRadius,
            self.autoScalingCheck.isChecked())
        # Connect dataSignal of Worker to plotData method
        self.worker.dataSignal.connect(self.plotData)
        # Move worker object to another thread
        self.worker.moveToThread(self.thread)
        # Start run method of worker thread
        self.worker.start.emit()
        print('Worker thread initialized')

    def stopWorkerThread(self):
        """ Shuts down the workerthread safely. """
        self.worker.finish = True  # Let worker thread exit main loop
        self.thread.quit()  # Quit the worker thread
        self.thread.wait()  # Wait until worker thread has quit

    def initUI(self):
        """ Initialization of all UI Widgets. """
        print('Initialize GUI ...')
        # Create buttons
        startBtn = QPushButton('Start Sensor')
        stopBtn = QPushButton('Stop Sensor')
        exitBtn = QPushButton('Exit Application')
        startBtn.clicked.connect(self.startBtnClicked)
        stopBtn.clicked.connect(self.stopBtnClicked)
        exitBtn.clicked.connect(self.exitBtnClicked)

        # Create combo box to toggle automatic scaling of the plot
        self.autoScalingCheck = QCheckBox('Enable auto-scaling')
        self.autoScalingCheck.stateChanged.connect(self.autoScalingToggled)

        # Create slider for manual scaling of the plot
        self.scalingSlider = QSlider(orientation=Qt.Horizontal)
        self.scalingSlider.setMinimum(self.scalingSliderMin)
        self.scalingSlider.setMaximum(self.scalingSliderMax)
        self.scalingSlider.setTickPosition(QSlider.TicksAbove)
        self.scalingSlider.setTickInterval(1)
        self.scalingSlider.setValue(self.scalings.index(self.initialMaxRange))
        self.scalingSlider.valueChanged.connect(self.scalingSliderChanged)

        # Create a text field for the collision range
        self.collisionRangeText = QTextEdit()
        self.collisionRangeText.setReadOnly(True)

        # Create slider for adjusting the car turn radius
        self.turnRadiusSlider = QSlider(orientation=Qt.Horizontal)
        self.turnRadiusSlider.setMinimum(0)
        self.turnRadiusSlider.setMaximum(self.nrOfTurnRadii * 2)
        self.turnRadiusSlider.setTickPosition(QSlider.TicksAbove)
        self.turnRadiusSlider.setTickInterval(1)
        self.turnRadiusSlider.setValue(self.nrOfTurnRadii)
        self.turnRadiusSlider.valueChanged.connect(self.turnRadiusSliderChanged)

        # Arrange buttons in a row
        vbox = QVBoxLayout()
        vbox.addWidget(self.autoScalingCheck, 1)
        vbox.addWidget(self.scalingSlider, 1)
        vbox.addWidget(self.collisionRangeText, 1)
        vbox.addWidget(self.turnRadiusSlider, 1)
        vbox.addWidget(startBtn, 1)
        vbox.addWidget(stopBtn, 1)
        vbox.addWidget(exitBtn, 1)

        # Create an hbox for the input elements and the graph
        hbox = QHBoxLayout()

        # Create a pyqtgraph plot widget instance
        self.plotWidget = pg.PlotWidget(enableMenu=False, enableMouse=False)
        self.plotWidget.disableAutoRange()
        self.plotWidget.setMouseEnabled(False, False)
        self.plotWidget.setYRange(-self.initialMaxRange, self.initialMaxRange)
        self.plotWidget.setXRange(-self.initialMaxRange, self.initialMaxRange)

        # Draw the graph and add a plot item for displaying future data points.
        self.drawGraph()
        # Add plotDataItem so that plt.setData can be used
        self.plt = self.plotWidget.plot(
            pen=None,
            symbol='o',
            symbolSize=2,
            symbolPen='w',
            symbolBrush='w')

        hbox.addWidget(self.plotWidget, 3)
        hbox.addLayout(vbox, 1)

        self.setLayout(hbox)
        self.setWindowTitle('Lidar')

        # Show the gui window
        # self.showFullScreen()
        self.show()
        print('GUI Initialized')

    def drawGraph(self):
        """ Draws the grid lines for current scaling and the current car path. """
        # Clear all items from plot widget
        self.drawGrid()
        # Draw car path lines
        self.drawCarPath()

    def redrawGrid(self):
        """ Changes the polar grid lines according to the caling value. """
        for i in np.arange(1, self.nrOfGridCircles + 1, 1):
            r = i / 10 * self.scalings[self.scalingSlider.value()]
            self.gridCircles[i - 1].setRect(-r, -r, r * 2, r * 2)

    def drawGrid(self):
        """ Draws the horizontal and vertical axis and polar grid circles. """
        # Add horizontal and vertical axis
        self.plotWidget.addLine(x=0, pen=0.2)
        self.plotWidget.addLine(y=0, pen=0.2)
        # Add polar grid circles
        for i in np.arange(1, self.nrOfGridCircles + 1, 1):
            r = i / 10 * self.scalings[self.scalingSlider.value()]
            self.gridCircles.append(pg.QtGui.QGraphicsEllipseItem(-r, -r, r * 2, r * 2))
            self.gridCircles[i - 1].setPen(pg.mkPen(0.2))
            self.plotWidget.addItem(self.gridCircles[i - 1])

    def removeCarPath(self):
        self.plotWidget.removeItem(self.rightPathCurve)
        self.plotWidget.removeItem(self.leftPathCurve)
        self.plotWidget.removeItem(self.rightPathCurveGreen)
        self.plotWidget.removeItem(self.leftPathCurveGreen)
        self.plotWidget.removeItem(self.rightPathLine)
        self.plotWidget.removeItem(self.leftPathLine)
        self.plotWidget.removeItem(self.rightPathLineGreen)
        self.plotWidget.removeItem(self.leftPathLineGreen)

    def redrawCarPath(self, collisionPoints=None):
        """ Updates the angle and position of the car path lines
            corresponding to the current drive angle. """
        self.removeCarPath()
        if self.turnRadius == 0:
            self.plotWidget.addItem(self.rightPathLine)
            self.plotWidget.addItem(self.leftPathLine)
            if collisionPoints is not None:
                # Change length of the green lines
                self.leftPathLineGreen.setLine(
                    -self.halfCarWidth, collisionPoints['backward'],
                    -self.halfCarWidth, collisionPoints['forward'])
                self.plotWidget.addItem(self.leftPathLineGreen)

                self.rightPathLineGreen.setLine(
                    self.halfCarWidth, collisionPoints['backward'],
                    self.halfCarWidth, collisionPoints['forward'])
                self.plotWidget.addItem(self.rightPathLineGreen)
        else:
            # Change radius of the right curves
            self.rightPathCurve.setRect(
                self.halfCarWidth,
                -self.turnRadius + self.halfCarWidth,
                2 * self.turnRadius - self.halfCarWidth * 2,
                2 * self.turnRadius - self.halfCarWidth * 2)
            self.rightPathCurveGreen.setRect(
                self.halfCarWidth,
                -self.turnRadius + self.halfCarWidth,
                2 * self.turnRadius - self.halfCarWidth * 2,
                2 * self.turnRadius - self.halfCarWidth * 2)

            # Change radius of the left curves
            self.leftPathCurve.setRect(
                -self.halfCarWidth,
                -self.turnRadius - self.halfCarWidth,
                2 * self.turnRadius + self.halfCarWidth * 2,
                2 * self.turnRadius + self.halfCarWidth * 2)
            self.leftPathCurveGreen.setRect(
                -self.halfCarWidth,
                -self.turnRadius - self.halfCarWidth,
                2 * self.turnRadius + self.halfCarWidth * 2,
                2 * self.turnRadius + self.halfCarWidth * 2)

            self.plotWidget.addItem(self.rightPathCurve)
            self.plotWidget.addItem(self.leftPathCurve)

            if collisionPoints is not None:
                # Calculate the new span angle in 1/16 degrees for the setSpanAngle() function
                newSpanAngle = np.rad2deg((collisionPoints['forward']
                                           - collisionPoints['backward']) / self.turnRadius) * 16

                # Change span angle of the right green curve
                self.rightPathCurveGreen.setSpanAngle(newSpanAngle)

                # Change span angle of the left green curve
                self.leftPathCurveGreen.setSpanAngle(newSpanAngle)

                if self.turnRadius > 0:
                    # Right turn
                    newStartAngle = (np.rad2deg(collisionPoints['backward'] / self.turnRadius) + 180) * 16
                else:
                    # Left turn
                    newStartAngle = np.rad2deg(collisionPoints['backward'] / self.turnRadius) * 16

                self.rightPathCurveGreen.setStartAngle(newStartAngle)
                self.leftPathCurveGreen.setStartAngle(newStartAngle)

                self.plotWidget.addItem(self.rightPathCurveGreen)
                self.plotWidget.addItem(self.leftPathCurveGreen)

    def drawCarPath(self):
        """ Draws the current path of the car in red dashed lines. """
        self.redPen = pg.mkPen('r', width=0.5, style=Qt.DashLine)
        self.greenPen = pg.mkPen('g', width=0.5, style=Qt.DashLine)

        # Right line
        self.rightPathLine = pg.InfiniteLine(
            pos=QPointF(self.halfCarWidth, 0),
            angle=90,
            pen=self.redPen)
        self.rightPathLineGreen = pg.QtGui.QGraphicsLineItem(
            self.halfCarWidth, -self.scalings[self.scalingSlider.value()],
            self.halfCarWidth, -self.scalings[self.scalingSlider.value()])
        self.rightPathLineGreen.setPen(self.greenPen)

        # Left line
        self.leftPathLine = pg.InfiniteLine(
            pos=QPointF(-self.halfCarWidth, 0),
            angle=90,
            pen=self.redPen)
        self.leftPathLineGreen = pg.QtGui.QGraphicsLineItem(
            -self.halfCarWidth, -self.scalings[self.scalingSlider.value()],
            -self.halfCarWidth, -self.scalings[self.scalingSlider.value()])
        self.leftPathLineGreen.setPen(self.greenPen)

        # Right curve
        self.rightPathCurve = pg.QtGui.QGraphicsEllipseItem(
            0, -self.turnRadius / 2,
            self.turnRadius, self.turnRadius)
        self.rightPathCurve.setPen(self.redPen)


        self.rightPathCurveGreen = pg.QtGui.QGraphicsEllipseItem(
            0, -self.turnRadius / 2,
            self.turnRadius, self.turnRadius)
        self.rightPathCurveGreen.setPen(self.greenPen)

        # Left curve
        self.leftPathCurve = pg.QtGui.QGraphicsEllipseItem(
            0, self.turnRadius / 2,
            self.turnRadius, self.turnRadius)
        self.leftPathCurve.setPen(self.redPen)
        self.leftPathCurveGreen = pg.QtGui.QGraphicsEllipseItem(
            0, self.turnRadius / 2,
            self.turnRadius, self.turnRadius)
        self.leftPathCurveGreen.setPen(self.greenPen)

        if self.turnRadius == 0:
            self.plotWidget.addItem(self.rightPathLine)
            self.plotWidget.addItem(self.rightPathLineGreen)
            self.plotWidget.addItem(self.leftPathLine)
            self.plotWidget.addItem(self.leftPathLineGreen)
        else:
            self.plotWidget.addItem(self.rightPathCurve)
            self.plotWidget.addItem(self.leftPathCurve)
            self.plotWidget.addItem(self.rightPathCurveGreen)
            self.plotWidget.addItem(self.leftPathCurveGreen)

    @pyqtSlot(object)
    def plotData(self, data):
        """ Plots given numpy array with 'x' and 'y' elements. """
        # Adjust grid and scaling of plot to current max range
        if self.worker.autoScale:
            self.scalingSlider.setValue(self.scalings.index(next(x for x in self.scalings if x > data['maxRange'])))
            self.scalingSliderChanged()
        # Plot data points
        self.plt.setData(data['points']['x'], data['points']['y'])
        # Show free path distance until collision, in text field
        self.redrawCarPath(data['collisionPoint'])
        self.collisionRangeText.setPlainText('Free forward path:')
        self.collisionRangeText.append('{:.2f}'.format(data['collisionPoint']['forward']) + 'm')
        self.collisionRangeText.append('\nFree backward path:')
        self.collisionRangeText.append('{:.2f}'.format(data['collisionPoint']['backward']) + 'm')

    def autoScalingToggled(self):
        """ Notify worker that autoscaling is enabled/disabled """
        status = self.autoScalingCheck.isChecked()
        self.scalingSlider.setEnabled(not status)
        if hasattr(self, 'worker'):
            self.worker.autoScale = self.autoScalingCheck.isChecked()

    def startBtnClicked(self):
        """ Start the laser. """
        if not self.started:
            self.started = True
            print('Laser started')
            self.initWorkerThread()
            self.runtime = time.time()
            return self.laser.turnOn()

    def stopBtnClicked(self):
        """ Stop the laser. """
        if self.started:
            self.started = False
            self.runtime = time.time() - self.runtime
            self.stopWorkerThread()
            print('Laser stopped')
            print('Laser scanned for {} seconds'.format(self.runtime))
            return self.laser.turnOff()  # Stop Lidar

    def exitBtnClicked(self):
        """ Stop the laser and exit the application. """
        if self.started:
            self.stopBtnClicked()

        print('Exiting application')
        self.close()

    def scalingSliderChanged(self):
        """ Changes the scaling of the plot. """
        val = self.scalings[self.scalingSlider.value()]
        self.plotWidget.setYRange(-val, val)
        self.plotWidget.setXRange(-val, val)
        self.redrawGrid()
        if hasattr(self, 'worker'):
            self.worker.maxRange = val

    def turnRadiusSliderChanged(self):
        """ Changes the drive angle of the car and therefore
            also the path that has to be checked for collisions. """
        self.turnRadius = self.turnRadii[self.turnRadiusSlider.value()]
        self.redrawCarPath()
        if hasattr(self, 'worker'):
            self.worker.turnRadius = self.turnRadius

    def initLidar(self):
        """ Initializes Lidar sensor with fixed parameters. """
        self.laser = PiDLidar.CYdLidar()
        #        port = '/dev/ttyUSB0'
        port = 'COM3'
        baudrate = 128000
        fixedResolution = False
        reversion = False
        inverted = True
        autoReconnect = True
        singleChannel = False
        lidarType = 1
        maxAngle = 180
        minAngle = -180
        minRange = 0.1
        maxRange = 10
        ignoreArray = []
        scanFrequency = 8

        self.laser.setSerialPort(port)
        self.laser.setSerialBaudrate(baudrate)
        self.laser.setFixedResolution(fixedResolution)

        # Flips the angle measurements by 180 degree
        self.laser.setReversion(reversion)

        # Defines the angle measurement direction
        # False --> clockwise
        # True --> counter clockwise
        self.laser.setInverted(inverted)
        self.laser.setAutoReconnect(autoReconnect)
        self.laser.setSingleChannel(singleChannel)

        # Lidar types:
        # 1 --> Triangulation lidar
        # 0 --> Time of flight lidar
        self.laser.setLidarType(lidarType)

        # Changes the range of the measured angles
        self.laser.setMaxAngle(maxAngle)
        self.laser.setMinAngle(minAngle)

        self.laser.setMinRange(minRange)
        self.laser.setMaxRange(maxRange)

        # The function is provided by the SDK but it doesn't do anything (for the X4 model)
        # The scanning frequency can only be configured by an external PWM signal
        # to the sensor (https://github.com/YDLIDAR/ydlidar_ros/issues/1)
        # It is set here nevertheless to use it for the sleep duration of the worker thread
        self.laser.setScanFrequency(scanFrequency)

        self.laser.setIgnoreArray(ignoreArray)

        return self.laser.initialize()


def __main__():
    app = QApplication(sys.argv)
    lidargui = LidarGUI()
    return app.exec_()


if __name__ == '__main__':
    sys.exit(__main__())
