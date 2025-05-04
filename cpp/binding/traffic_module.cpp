#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include "../include/models/Vehicle.h"
#include "../include/models/Road.h"
#include "../include/models/Intersection.h"
#include "../include/models/TrafficLight.h"
#include "../include/controllers/SimulationController.h"
#include "../include/utils/Vector2D.h"
#include "../include/utils/UUID.h"

namespace py = pybind11;

PYBIND11_MODULE(traffic_system, m) {
    m.doc() = "Traffic Simulation System Python Bindings";
    
    // UUID binding
    py::class_<TrafficSim::UUID>(m, "UUID")
        .def(py::init<>())
        .def(py::init<const std::string&>())
        .def("toString", &TrafficSim::UUID::toString)
        .def("__eq__", [](const TrafficSim::UUID& a, const TrafficSim::UUID& b) {
            return a == b;
        })
        .def("__hash__", [](const TrafficSim::UUID& uuid) {
            return TrafficSim::UUIDHasher()(uuid);
        })
        .def("__repr__", [](const TrafficSim::UUID& uuid) {
            return "UUID('" + uuid.toString() + "')";
        });
    
    // Vector2D binding
    py::class_<TrafficSim::Vector2D>(m, "Vector2D")
        .def(py::init<>())
        .def(py::init<double, double>())
        .def_readwrite("x", &TrafficSim::Vector2D::x)
        .def_readwrite("y", &TrafficSim::Vector2D::y)
        .def("length", &TrafficSim::Vector2D::length)
        .def("lengthSquared", &TrafficSim::Vector2D::lengthSquared)
        .def("normalize", &TrafficSim::Vector2D::normalize)
        .def("dot", &TrafficSim::Vector2D::dot)
        .def("cross", &TrafficSim::Vector2D::cross)
        .def_static("distance", &TrafficSim::Vector2D::distance)
        .def_static("angle", &TrafficSim::Vector2D::angle)
        .def_static("lerp", &TrafficSim::Vector2D::lerp)
        .def("__repr__", [](const TrafficSim::Vector2D& v) {
            return "Vector2D(" + std::to_string(v.x) + ", " + std::to_string(v.y) + ")";
        })
        .def(py::self + py::self)
        .def(py::self - py::self)
        .def(py::self * double())
        .def(py::self / double())
        .def(py::self += py::self)
        .def(py::self -= py::self)
        .def(py::self *= double())
        .def(py::self /= double());
    
    // Enum bindings
    py::enum_<TrafficSim::VehicleType>(m, "VehicleType")
        .value("CAR", TrafficSim::VehicleType::CAR)
        .value("BUS", TrafficSim::VehicleType::BUS)
        .value("TRUCK", TrafficSim::VehicleType::TRUCK)
        .value("MOTORCYCLE", TrafficSim::VehicleType::MOTORCYCLE)
        .value("EMERGENCY", TrafficSim::VehicleType::EMERGENCY)
        .value("BICYCLE", TrafficSim::VehicleType::BICYCLE)
        .export_values();
    
    py::enum_<TrafficSim::RoadType>(m, "RoadType")
        .value("RESIDENTIAL", TrafficSim::RoadType::RESIDENTIAL)
        .value("URBAN", TrafficSim::RoadType::URBAN)
        .value("ARTERIAL", TrafficSim::RoadType::ARTERIAL)
        .value("HIGHWAY", TrafficSim::RoadType::HIGHWAY)
        .value("FREEWAY", TrafficSim::RoadType::FREEWAY)
        .export_values();
    
    py::enum_<TrafficSim::LightState>(m, "LightState")
        .value("RED", TrafficSim::LightState::RED)
        .value("YELLOW", TrafficSim::LightState::YELLOW)
        .value("GREEN", TrafficSim::LightState::GREEN)
        .export_values();
    
    py::enum_<TrafficSim::SimulationStatus>(m, "SimulationStatus")
        .value("INITIALIZING", TrafficSim::SimulationStatus::INITIALIZING)
        .value("READY", TrafficSim::SimulationStatus::READY)
        .value("RUNNING", TrafficSim::SimulationStatus::RUNNING)
        .value("PAUSED", TrafficSim::SimulationStatus::PAUSED)
        .value("STOPPED", TrafficSim::SimulationStatus::STOPPED)
        .value("ERROR", TrafficSim::SimulationStatus::ERROR)
        .export_values();
    
    // SimulationStatistics binding
    py::class_<TrafficSim::SimulationStatistics>(m, "SimulationStatistics")
        .def(py::init<>())
        .def_readwrite("averageSpeed", &TrafficSim::SimulationStatistics::averageSpeed)
        .def_readwrite("averageDensity", &TrafficSim::SimulationStatistics::averageDensity)
        .def_readwrite("averageFlow", &TrafficSim::SimulationStatistics::averageFlow)
        .def_readwrite("totalVehicles", &TrafficSim::SimulationStatistics::totalVehicles)
        .def_readwrite("activeVehicles", &TrafficSim::SimulationStatistics::activeVehicles)
        .def_readwrite("completedTrips", &TrafficSim::SimulationStatistics::completedTrips)
        .def_readwrite("totalCongestionTime", &TrafficSim::SimulationStatistics::totalCongestionTime)
        .def_readwrite("averageTripTime", &TrafficSim::SimulationStatistics::averageTripTime)
        .def_readwrite("simulationTime", &TrafficSim::SimulationStatistics::simulationTime)
        .def_readwrite("simulationSpeed", &TrafficSim::SimulationStatistics::simulationSpeed);
    
    // Vehicle binding
    py::class_<TrafficSim::Vehicle>(m, "Vehicle")
        .def(py::init<TrafficSim::VehicleType, double, double, double>(),
             py::arg("type"), py::arg("length") = 4.5, py::arg("width") = 2.0, py::arg("maxSpeed") = 33.3)
        .def("getId", &TrafficSim::Vehicle::getId)
        .def("getType", &TrafficSim::Vehicle::getType)
        .def("getPosition", &TrafficSim::Vehicle::getPosition)
        .def("getHeading", &TrafficSim::Vehicle::getHeading)
        .def("getLength", &TrafficSim::Vehicle::getLength)
        .def("getWidth", &TrafficSim::Vehicle::getWidth)
        .def("getCurrentSpeed", &TrafficSim::Vehicle::getCurrentSpeed)
        .def("getMaxSpeed", &TrafficSim::Vehicle::getMaxSpeed)
        .def("getCurrentRoad", &TrafficSim::Vehicle::getCurrentRoad)
        .def("getCurrentLane", &TrafficSim::Vehicle::getCurrentLane)
        .def("setPosition", &TrafficSim::Vehicle::setPosition)
        .def("setHeading", &TrafficSim::Vehicle::setHeading)
        .def("setCurrentSpeed", &TrafficSim::Vehicle::setCurrentSpeed)
        .def("setCurrentLane", &TrafficSim::Vehicle::setCurrentLane)
        .def("accelerate", &TrafficSim::Vehicle::accelerate)
        .def("brake", &TrafficSim::Vehicle::brake)
        .def("stop", &TrafficSim::Vehicle::stop)
        .def("toString", &TrafficSim::Vehicle::toString);
    
    // Car binding
    py::class_<TrafficSim::Car, TrafficSim::Vehicle>(m, "Car")
        .def(py::init<double>(), py::arg("maxSpeed") = 33.3);
    
    // Bus binding
    py::class_<TrafficSim::Bus, TrafficSim::Vehicle>(m, "Bus")
        .def(py::init<double>(), py::arg("maxSpeed") = 25.0)
        .def("loadPassengers", &TrafficSim::Bus::loadPassengers)
        .def("unloadPassengers", &TrafficSim::Bus::unloadPassengers)
        .def("getPassengerCount", &TrafficSim::Bus::getPassengerCount)
        .def("getMaxPassengers", &TrafficSim::Bus::getMaxPassengers);
    
    // Truck binding
    py::class_<TrafficSim::Truck, TrafficSim::Vehicle>(m, "Truck")
        .def(py::init<double, double>(), 
             py::arg("maxSpeed") = 22.2, py::arg("cargoWeight") = 5000.0)
        .def("getCargoWeight", &TrafficSim::Truck::getCargoWeight)
        .def("setCargoWeight", &TrafficSim::Truck::setCargoWeight);
    
    // EmergencyVehicle binding
    py::class_<TrafficSim::EmergencyVehicle, TrafficSim::Vehicle>(m, "EmergencyVehicle")
        .def(py::init<double>(), py::arg("maxSpeed") = 44.4)
        .def("activateSiren", &TrafficSim::EmergencyVehicle::activateSiren)
        .def("isSirenActive", &TrafficSim::EmergencyVehicle::isSirenActive);
    
    // Lane binding
    py::class_<TrafficSim::Lane>(m, "Lane")
        .def("getId", &TrafficSim::Lane::getId)
        .def("getIndex", &TrafficSim::Lane::getIndex)
        .def("getWidth", &TrafficSim::Lane::getWidth)
        .def("getSpeedLimit", &TrafficSim::Lane::getSpeedLimit)
        .def("getRoad", &TrafficSim::Lane::getRoad)
        .def("getVehicles", &TrafficSim::Lane::getVehicles)
        .def("getLeftLane", &TrafficSim::Lane::getLeftLane)
        .def("getRightLane", &TrafficSim::Lane::getRightLane)
        .def("getDensity", &TrafficSim::Lane::getDensity)
        .def("getFlow", &TrafficSim::Lane::getFlow)
        .def("getAverageSpeed", &TrafficSim::Lane::getAverageSpeed);
    
    // Road binding
    py::class_<TrafficSim::Road>(m, "Road")
        .def("getId", &TrafficSim::Road::getId)
        .def("getName", &TrafficSim::Road::getName)
        .def("getType", &TrafficSim::Road::getType)
        .def("getLength", &TrafficSim::Road::getLength)
        .def("getSpeedLimit", &TrafficSim::Road::getSpeedLimit)
        .def("getLaneCount", &TrafficSim::Road::getLaneCount)
        .def("getStartPoint", &TrafficSim::Road::getStartPoint)
        .def("getEndPoint", &TrafficSim::Road::getEndPoint)
        .def("getHeading", &TrafficSim::Road::getHeading)
        .def("getLane", &TrafficSim::Road::getLane)
        .def("getLanes", &TrafficSim::Road::getLanes)
        .def("getStartIntersection", &TrafficSim::Road::getStartIntersection)
        .def("getEndIntersection", &TrafficSim::Road::getEndIntersection)
        .def("getTrafficLight", &TrafficSim::Road::getTrafficLight)
        .def("getAverageSpeed", &TrafficSim::Road::getAverageSpeed)
        .def("getAverageDensity", &TrafficSim::Road::getAverageDensity)
        .def("getAverageFlow", &TrafficSim::Road::getAverageFlow)
        .def("worldToRoadCoordinates", &TrafficSim::Road::worldToRoadCoordinates)
        .def("roadToWorldCoordinates", &TrafficSim::Road::roadToWorldCoordinates);
    
    // Intersection binding
    py::class_<TrafficSim::Intersection>(m, "Intersection")
        .def("getId", &TrafficSim::Intersection::getId)
        .def("getName", &TrafficSim::Intersection::getName)
        .def("getPosition", &TrafficSim::Intersection::getPosition)
        .def("getConnectedRoads", &TrafficSim::Intersection::getConnectedRoads)
        .def("getTrafficLight", &TrafficSim::Intersection::getTrafficLight)
        .def("getPossibleExits", &TrafficSim::Intersection::getPossibleExits)
        .def("hasRightOfWay", &TrafficSim::Intersection::hasRightOfWay);
    
    // TrafficLight binding
    py::class_<TrafficSim::TrafficLight>(m, "TrafficLight")
        .def("getState", [](TrafficSim::TrafficLight& self) {
            return self.getState(nullptr);
        })
        .def("getState", [](TrafficSim::TrafficLight& self, TrafficSim::Road* road) {
            return self.getState(road);
        })
        .def("getCurrentPhase", &TrafficSim::TrafficLight::getCurrentPhase)
        .def("getPhaseCount", &TrafficSim::TrafficLight::getPhaseCount)
        .def("getPhaseDuration", &TrafficSim::TrafficLight::getPhaseDuration);
    
    // SimulationController binding
    py::class_<TrafficSim::SimulationController>(m, "SimulationController")
        .def_static("getInstance", &TrafficSim::SimulationController::getInstance, 
                   py::return_value_policy::reference)
        .def("initialize", &TrafficSim::SimulationController::initialize)
        .def("loadNetwork", &TrafficSim::SimulationController::loadNetwork)
        .def("saveNetwork", &TrafficSim::SimulationController::saveNetwork)
        .def("loadScenario", &TrafficSim::SimulationController::loadScenario)
        .def("start", &TrafficSim::SimulationController::start)
        .def("pause", &TrafficSim::SimulationController::pause)
        .def("resume", &TrafficSim::SimulationController::resume)
        .def("stop", &TrafficSim::SimulationController::stop)
        .def("reset", &TrafficSim::SimulationController::reset)
        .def("run", &TrafficSim::SimulationController::run,
             py::arg("duration"), py::arg("realTimeFactor") = 1.0)
        .def("step", &TrafficSim::SimulationController::step)
        .def("getCurrentTime", &TrafficSim::SimulationController::getCurrentTime)
        .def("getStatus", &TrafficSim::SimulationController::getStatus)
        .def("getTimeStep", &TrafficSim::SimulationController::getTimeStep)
        .def("setTimeStep", &TrafficSim::SimulationController::setTimeStep)
        .def("getSimulationSpeedFactor", &TrafficSim::SimulationController::getSimulationSpeedFactor)
        .def("setSimulationSpeedFactor", &TrafficSim::SimulationController::setSimulationSpeedFactor)
        .def("generateRandomTraffic", &TrafficSim::SimulationController::generateRandomTraffic,
             py::arg("vehicleCount"), py::arg("seed") = 0)
        .def("createVehicle", &TrafficSim::SimulationController::createVehicle,
             py::arg("type"), py::arg("initialPosition"), 
             py::arg("initialRoad") = nullptr, py::arg("initialLane") = 0,
             py::return_value_policy::reference)
        .def("removeVehicle", &TrafficSim::SimulationController::removeVehicle)
        .def("createRoad", &TrafficSim::SimulationController::createRoad,
             py::arg("name"), py::arg("type"), py::arg("startPoint"), py::arg("endPoint"),
             py::arg("laneCount") = 1, py::arg("speedLimit") = 0.0,
             py::return_value_policy::reference)
        .def("createIntersection", &TrafficSim::SimulationController::createIntersection,
             py::arg("position"), py::arg("name") = "",
             py::return_value_policy::reference)
        .def("getVehicle", &TrafficSim::SimulationController::getVehicle,
             py::arg("id"), py::return_value_policy::reference)
        .def("getRoad", &TrafficSim::SimulationController::getRoad,
             py::arg("id"), py::return_value_policy::reference)
        .def("getIntersection", &TrafficSim::SimulationController::getIntersection,
             py::arg("id"), py::return_value_policy::reference)
        .def("getAllVehicles", &TrafficSim::SimulationController::getAllVehicles,
             py::return_value_policy::reference)
        .def("getAllRoads", &TrafficSim::SimulationController::getAllRoads,
             py::return_value_policy::reference)
        .def("getAllIntersections", &TrafficSim::SimulationController::getAllIntersections,
             py::return_value_policy::reference)
        .def("getStatistics", &TrafficSim::SimulationController::getStatistics)
        .def("registerUpdateCallback", [](TrafficSim::SimulationController& self, 
                                         py::function callback) {
            return self.registerUpdateCallback(
                [callback](double deltaTime) {
                    py::gil_scoped_acquire acquire;
                    try {
                        callback(deltaTime);
                    } catch (py::error_already_set& e) {
                        PyErr_Print();
                    }
                }
            );
        })
        .def("unregisterUpdateCallback", &TrafficSim::SimulationController::unregisterUpdateCallback);
}