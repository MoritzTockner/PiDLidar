import sys
from PyQt5.QtWidgets import (QSizePolicy, QSlider, QCheckBox, QWidget, QPushButton, QHBoxLayout, QVBoxLayout, QApplication)
from PyQt5.QtCore import (Qt, QThread, QObject, pyqtSlot, pyqtSignal)
import pyqtgraph as pg
import numpy as np
import PiDLidar
import time


class Worker(QObject):

    start = pyqtSignal()        # Starts the run method
    dataSignal = pyqtSignal(object)     # Notifies GUI thread to update plot with object as data

    def __init__(self, laser, minRange, maxRange, autoScale):
        QThread.__init__(self)
        self.laser = laser      # CYdLidar class object
        self.finish = False     # Is set to true when stop button is pressed to close worker thread
        self.maxRange = maxRange
        self.minRange = minRange
        self.autoScale = autoScale
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
        scanTime = 1/self.laser.getScanFrequency()

        # Fetch data until finish is set to False by StopButton
        while not self.finish:
            # If new data arrived, store it in scan
            if self.laser.doProcessSimple(scan, hardError):
                # Measure time of data processing for calculation of the thread timeout afterwards
                fetchAndSignalTime = time.time()
                # When autoscale is enabled
                if self.autoScale:
                    # Take all samples, except the ones under 0.1 meters. They are mapped to 0 by the sensor
                    # because that is below the minimal range.
                    data = np.array([(point.range * np.cos(point.angle + np.pi/2), point.range * np.sin(point.angle + np.pi/2)) \
                        for point in scan.points \
                        if point.range >= self.minRange], \
                        dtype=[('x', float), ('y', float)])
                    # Find sample with the highest range, to adjust autoscaling accordingly
                    maxRange = max([point.range for point in scan.points])
                else:
                    # Take all samples that are under the configured maximum range and above 0.1 meters.
                    data = np.array([(point.range * np.cos(point.angle + np.pi/2), point.range * np.sin(point.angle + np.pi/2)) \
                        for point in scan.points \
                        if point.range <= self.maxRange and point.range >= self.minRange], \
                        dtype=[('x', float), ('y', float)])
                    # The configured max range
                    maxRange = self.maxRange

                # Notify GUI process to plot the sampled data
                # and update the scaling of the plot if necessary
                self.dataSignal.emit({'points' : data, 'maxRange' : maxRange})
                fetchAndSignalTime = time.time() - fetchAndSignalTime
            if hardError:
                self.finish = True
                print('Hard error in CYdLidar.doProcessSimple().')

            # Thread sleeps for approximately one rotation of the sensor until it tries to fetch new data.
            if scanTime > fetchAndSignalTime:
                time.sleep(scanTime - fetchAndSignalTime)



class LidarGUI(QWidget):

    def __init__(self, parent=None):
        QWidget.__init__(self, parent)
        self.started = False
        self.laser = None
        self.scalingCheck = None
        self.runtime = None
        self.scalings = range(1, 11)
        #self.scalings = [1, 5, 10, 20, 30, 40, 50, 60] # Lookup table for the scaling slider
        self.scalingSliderMax = len(self.scalings) - 1
        self.scalingSliderMin = 0
        self.initialMinRange = 0.1
        self.initialMaxRange = self.scalings[0]
        self.initLidar()
        self.initUI()


    def __del__(self):
        """ Stop Lidar and disconnect serial connection. """
        self.laser.turnOff()
        self.laser.disconnecting()

    def initWorkerThread(self):
        """ Creates a new thread and assigns an instance of the Worker class to it. """
        print('Initialize worker thread ...')
        # Create a new thread
        self.thread = QThread()
        # Start thread
        self.thread.start()
        # Create worker object that updates lidar data plot
        self.worker = Worker(self.laser, \
                             self.initialMinRange, \
                             self.initialMaxRange, \
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
        self.worker.finish = True # Let worker thread exit main loop
        self.thread.quit() # Quit the worker thread
        self.thread.wait() # Wait until worker thread has quit


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

        # Arrange buttons in a row
        vbox = QVBoxLayout()
        vbox.addWidget(self.autoScalingCheck, 1)
        vbox.addWidget(self.scalingSlider, 1)
        vbox.addWidget(startBtn, 1)
        vbox.addWidget(stopBtn, 1)
        vbox.addWidget(exitBtn, 1)

        # Create an empty box above the button row
        hbox = QHBoxLayout()

        # Create a pyqtgraph plot widget instance
        self.plotWidget = pg.PlotWidget(enableMenu=False, enableMouse=False)
        self.plotWidget.disableAutoRange()
        self.plotWidget.setMouseEnabled(False, False)
        self.plotWidget.setYRange(-self.initialMaxRange, self.initialMaxRange)
        self.plotWidget.setXRange(-self.initialMaxRange, self.initialMaxRange)

        # Draw grid
        self.drawGridLines()

        hbox.addWidget(self.plotWidget, 3)
        hbox.addLayout(vbox, 1)

        self.setLayout(hbox)
        self.setWindowTitle('Lidar')

        # Show the gui window
        self.showFullScreen()
        print('GUI Initialized')

    def drawGridLines(self):
        """ Clears the plot widget, redraws the grid lines for current scaling
            and adds a plotDataItem to the plot widget. """
        # Clear all items from plot widget
        self.plotWidget.clear()
        # Add polar grid lines
        self.plotWidget.addLine(x=0, pen=0.2)
        self.plotWidget.addLine(y=0, pen=0.2)
        for r in np.linspace(self.initialMinRange, self.scalings[self.scalingSlider.value()], 10):
            circle = pg.QtGui.QGraphicsEllipseItem(-r, -r, r*2, r*2)
            circle.setPen(pg.mkPen(0.2))
            self.plotWidget.addItem(circle)
        # Add plotDataItem so that plt.setData can be used
        self.plt = self.plotWidget.plot(pen=None, symbol='x')

    @pyqtSlot(object)
    def plotData(self, data):
        """ Plots given numpy array with 'x' and 'y' elements. """
        # Adjust grid and scaling of plot to current max range
        if self.worker.autoScale:
            self.scalingSlider.setValue(self.scalings.index(next(x for x in self.scalings if x > data['maxRange'])))
            self.scalingSliderChanged()

        self.plt.setData(data['points']['x'], data['points']['y'])


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
            return self.laser.turnOff() # Stop Lidar

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
        self.drawGridLines()
        if hasattr(self, 'worker'):
            self.worker.maxRange = val

    def initLidar(self):
        """ Initializes Lidar sensor with fixed parameters. """
        self.laser = PiDLidar.CYdLidar()
        port = '/dev/ttyUSB0'
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

