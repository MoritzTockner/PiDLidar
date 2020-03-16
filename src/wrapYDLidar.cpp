#include <../pybind11/include/pybind11/pybind11.h>
#include <../pybind11/include/pybind11/stl.h>
#include "CYdLidar.h"

namespace py = pybind11;

PYBIND11_MODULE(PiDLidar, m) {
    py::class_<CYdLidar>(m, "CYdLidar")
        .def(py::init())
        .def("setMaxRange", &CYdLidar::setMaxRange)
        .def("getMaxRange", &CYdLidar::getMaxRange)
        .def("setMinRange", &CYdLidar::setMinRange)
        .def("getMinRange", &CYdLidar::getMinRange)
        .def("getMaxAngle", &CYdLidar::getMaxAngle)
        .def("setMaxAngle", &CYdLidar::setMaxAngle)
        .def("getMinAngle", &CYdLidar::getMinAngle)
        .def("setMinAngle", &CYdLidar::setMinAngle)
        .def("getSampleRate", &CYdLidar::getSampleRate)
        .def("setSampleRate", &CYdLidar::setSampleRate)
        .def("getScanFrequency", &CYdLidar::getScanFrequency)
        .def("setScanFrequency", &CYdLidar::setScanFrequency)
        .def("getFixedResolution", &CYdLidar::getFixedResolution)
        .def("setFixedResolution", &CYdLidar::setFixedResolution)
        .def("getReversion", &CYdLidar::getReversion)
        .def("setReversion", &CYdLidar::setReversion)
        .def("getInverted", &CYdLidar::getInverted)
        .def("setInverted", &CYdLidar::setInverted)
        .def("getAutoReconnect", &CYdLidar::getAutoReconnect)
        .def("setAutoReconnect", &CYdLidar::setAutoReconnect)
        .def("getSerialBaudrate", &CYdLidar::getSerialBaudrate)
        .def("setSerialBaudrate", &CYdLidar::setSerialBaudrate)
        .def("getAbnormalCheckCount", &CYdLidar::getAbnormalCheckCount)
        .def("setAbnormalCheckCount", &CYdLidar::setAbnormalCheckCount)
        .def("setSerialPort", &CYdLidar::setSerialPort)
        .def("getSerialPort", &CYdLidar::getSerialPort)
        .def("setIgnoreArray", &CYdLidar::setIgnoreArray)
        .def("getIgnoreArray", &CYdLidar::getIgnoreArray)
        .def("setOffsetTime", &CYdLidar::setOffsetTime)
        .def("getOffsetTime", &CYdLidar::getOffsetTime)
        .def("setSingleChannel", &CYdLidar::setSingleChannel)
        .def("getSingleChannel", &CYdLidar::getSingleChannel)
        .def("setLidarType", &CYdLidar::setLidarType)
        .def("getLidarType", &CYdLidar::getLidarType)
        .def("initialize", &CYdLidar::initialize)
        .def("doProcessSimple", &CYdLidar::doProcessSimple)
        .def("turnOn", &CYdLidar::turnOn)
        .def("turnOff", &CYdLidar::turnOff)
        .def("disconnecting", &CYdLidar::disconnecting)
        .def("getAngleOffset", &CYdLidar::getAngleOffset)
        .def("isAngleOffsetCorrected", &CYdLidar::isAngleOffetCorrected)
        .def("getSoftVersion", &CYdLidar::getSoftVersion)
        .def("getHardwareVersion", &CYdLidar::getHardwareVersion)
        .def("getSerialNumber", &CYdLidar::getSerialNumber);

    py::class_<LaserPoint>(m, "LaserPoint")
        .def(py::init())
        .def_readwrite("angle", &LaserPoint::angle)
        .def_readwrite("range", &LaserPoint::range)
        .def_readwrite("intensity", &LaserPoint::intensity);

    py::class_<LaserConfig>(m, "LaserConfig")
        .def(py::init())
        .def_readwrite("min_angle", &LaserConfig::min_angle)
        .def_readwrite("max_angle", &LaserConfig::max_angle)
        .def_readwrite("angle_increment", &LaserConfig::angle_increment)
        .def_readwrite("time_increment", &LaserConfig::time_increment)
        .def_readwrite("scan_time", &LaserConfig::scan_time)
        .def_readwrite("min_range", &LaserConfig::min_range)
        .def_readwrite("max_range", &LaserConfig::max_range);

    py::class_<LaserScan>(m, "LaserScan")
        .def(py::init())
        .def_readwrite("stamp", &LaserScan::stamp)
        .def_readwrite("points", &LaserScan::points)
        .def_readwrite("config", &LaserScan::config);
}
