#include "../../include/controllers/SimulationController.h"
#include "../../include/utils/Logger.h"
#include "../../include/utils/Timer.h"
#include "../../include/utils/ConfigParser.h"
#include "../../include/models/Driver.h"
#include "../../include/models/RouteManager.h"
#include <algorithm>
#include <random>
#include <chrono>
#include <thread>
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace TrafficSim {

// Initialize static instance pointer
SimulationController* SimulationController::m_instance = nullptr;

SimulationController& SimulationController::getInstance() {
    if (!m_instance) {
        m_instance = new SimulationController();
    }
    return *m_instance;
}

SimulationController::SimulationController() {
    Logger::getInstance().info("Initializing SimulationController");
}

SimulationController::~SimulationController() {
    // Clean up all entities
    stop();
    
    // Clean up vehicles
    for (auto vehicle : m_vehicles) {
        delete vehicle;
    }
    m_vehicles.clear();
    m_vehicleMap.clear();
    
    // Clean up roads
    for (auto road : m_roads) {
        delete road;
    }
    m_roads.clear();
    m_roadMap.clear();
    
    // Clean up intersections
    for (auto intersection : m_intersections) {
        delete intersection;
    }
    m_intersections.clear();
    m_intersectionMap.clear();
    
    Logger::getInstance().info("SimulationController destroyed");
}

bool SimulationController::initialize(const std::string& configFile) {
    std::lock_guard<std::mutex> lock(m_simulationMutex);
    
    Logger::getInstance().info("Initializing simulation");
    
    // Reset state
    m_currentTime = 0.0;
    m_status = SimulationStatus::INITIALIZING;
    
    // If config file provided, load it
    if (!configFile.empty()) {
        Logger::getInstance().info("Loading configuration from " + configFile);
        ConfigParser config;
        if (!config.load(configFile)) {
            Logger::getInstance().error("Failed to load configuration file: " + configFile);
            m_status = SimulationStatus::ERROR;
            return false;
        }
        
        // Apply configuration
        m_timeStep = config.getDouble("simulation.timeStep", m_timeStep);
        
        // Other config parameters could be applied here
    }
    
    // Build default network if no roads exist
    if (m_roads.empty()) {
        Logger::getInstance().info("Building sample network");
        buildSampleNetwork();
    }
    
    // Initialize statistics
    updateStatistics();
    
    m_status = SimulationStatus::READY;
    Logger::getInstance().info("Simulation initialized successfully");
    return true;
}

bool SimulationController::loadNetwork(const std::string& filename) {
    std::lock_guard<std::mutex> lock(m_simulationMutex);
    
    if (m_status == SimulationStatus::RUNNING) {
        Logger::getInstance().warning("Cannot load network while simulation is running");
        return false;
    }
    
    Logger::getInstance().info("Loading network from " + filename);
    
    // Clean up existing network
    for (auto vehicle : m_vehicles) {
        delete vehicle;
    }
    m_vehicles.clear();
    m_vehicleMap.clear();
    
    for (auto road : m_roads) {
        delete road;
    }
    m_roads.clear();
    m_roadMap.clear();
    
    for (auto intersection : m_intersections) {
        delete intersection;
    }
    m_intersections.clear();
    m_intersectionMap.clear();
    
    // Load network from file
    try {
        std::ifstream file(filename);
        if (!file.is_open()) {
            Logger::getInstance().error("Failed to open network file: " + filename);
            return false;
        }
        
        json networkJson;
        file >> networkJson;
        
        // Parse intersections
        if (networkJson.contains("intersections")) {
            for (const auto& intersectionJson : networkJson["intersections"]) {
                Vector2D position(
                    intersectionJson["position"]["x"].get<double>(),
                    intersectionJson["position"]["y"].get<double>()
                );
                
                std::string name = intersectionJson["name"].get<std::string>();
                createIntersection(position, name);
            }
        }
        
        // Parse roads
        if (networkJson.contains("roads")) {
            for (const auto& roadJson : networkJson["roads"]) {
                Vector2D startPoint(
                    roadJson["startPoint"]["x"].get<double>(),
                    roadJson["startPoint"]["y"].get<double>()
                );
                
                Vector2D endPoint(
                    roadJson["endPoint"]["x"].get<double>(),
                    roadJson["endPoint"]["y"].get<double>()
                );
                
                std::string name = roadJson["name"].get<std::string>();
                
                RoadType type;
                std::string typeStr = roadJson["type"].get<std::string>();
                if (typeStr == "RESIDENTIAL") {
                    type = RoadType::RESIDENTIAL;
                } else if (typeStr == "URBAN") {
                    type = RoadType::URBAN;
                } else if (typeStr == "ARTERIAL") {
                    type = RoadType::ARTERIAL;
                } else if (typeStr == "HIGHWAY") {
                    type = RoadType::HIGHWAY;
                } else if (typeStr == "FREEWAY") {
                    type = RoadType::FREEWAY;
                } else {
                    type = RoadType::URBAN; // Default
                }
                
                int laneCount = roadJson["laneCount"].get<int>();
                double speedLimit = roadJson["speedLimit"].get<double>();
                
                createRoad(name, type, startPoint, endPoint, laneCount, speedLimit);
            }
        }
        
        // Parse road-intersection connections
        if (networkJson.contains("connections")) {
            for (const auto& connectionJson : networkJson["connections"]) {
                std::string roadId = connectionJson["roadId"].get<std::string>();
                std::string startIntersectionId = connectionJson["startIntersectionId"].get<std::string>();
                std::string endIntersectionId = connectionJson["endIntersectionId"].get<std::string>();
                
                Road* road = getRoad(UUID(roadId));
                Intersection* startIntersection = getIntersection(UUID(startIntersectionId));
                Intersection* endIntersection = getIntersection(UUID(endIntersectionId));
                
                if (road && startIntersection) {
                    road->setStartIntersection(startIntersection);
                }
                
                if (road && endIntersection) {
                    road->setEndIntersection(endIntersection);
                }
            }
        }
        
        Logger::getInstance().info("Network loaded successfully");
        return true;
        
    } catch (const std::exception& e) {
        Logger::getInstance().error("Error loading network: " + std::string(e.what()));
        return false;
    }
}

bool SimulationController::saveNetwork(const std::string& filename) const {
    std::lock_guard<std::mutex> lock(m_simulationMutex);
    
    Logger::getInstance().info("Saving network to " + filename);
    
    try {
        // Create JSON structure
        json networkJson;
        
        // Save intersections
        json intersectionsJson = json::array();
        for (const auto& intersection : m_intersections) {
            json intersectionJson;
            intersectionJson["id"] = intersection->getId().toString();
            intersectionJson["name"] = intersection->getName();
            intersectionJson["position"]["x"] = intersection->getPosition().x;
            intersectionJson["position"]["y"] = intersection->getPosition().y;
            
            intersectionsJson.push_back(intersectionJson);
        }
        networkJson["intersections"] = intersectionsJson;
        
        // Save roads
        json roadsJson = json::array();
        for (const auto& road : m_roads) {
            json roadJson;
            roadJson["id"] = road->getId().toString();
            roadJson["name"] = road->getName();
            
            // Serialize road type
            switch (road->getType()) {
                case RoadType::RESIDENTIAL: roadJson["type"] = "RESIDENTIAL"; break;
                case RoadType::URBAN: roadJson["type"] = "URBAN"; break;
                case RoadType::ARTERIAL: roadJson["type"] = "ARTERIAL"; break;
                case RoadType::HIGHWAY: roadJson["type"] = "HIGHWAY"; break;
                case RoadType::FREEWAY: roadJson["type"] = "FREEWAY"; break;
                default: roadJson["type"] = "URBAN"; break;
            }
            
            roadJson["startPoint"]["x"] = road->getStartPoint().x;
            roadJson["startPoint"]["y"] = road->getStartPoint().y;
            roadJson["endPoint"]["x"] = road->getEndPoint().x;
            roadJson["endPoint"]["y"] = road->getEndPoint().y;
            roadJson["laneCount"] = road->getLaneCount();
            roadJson["speedLimit"] = road->getSpeedLimit();
            
            roadsJson.push_back(roadJson);
        }
        networkJson["roads"] = roadsJson;
        
        // Save connections between roads and intersections
        json connectionsJson = json::array();
        for (const auto& road : m_roads) {
            if (road->getStartIntersection() || road->getEndIntersection()) {
                json connectionJson;
                connectionJson["roadId"] = road->getId().toString();
                
                if (road->getStartIntersection()) {
                    connectionJson["startIntersectionId"] = road->getStartIntersection()->getId().toString();
                }
                
                if (road->getEndIntersection()) {
                    connectionJson["endIntersectionId"] = road->getEndIntersection()->getId().toString();
                }
                
                connectionsJson.push_back(connectionJson);
            }
        }
        networkJson["connections"] = connectionsJson;
        
        // Write to file
        std::ofstream file(filename);
        if (!file.is_open()) {
            Logger::getInstance().error("Failed to open file for writing: " + filename);
            return false;
        }
        
        file << networkJson.dump(4);  // Pretty-print with 4-space indentation
        file.close();
        
        Logger::getInstance().info("Network saved successfully");
        return true;
        
    } catch (const std::exception& e) {
        Logger::getInstance().error("Error saving network: " + std::string(e.what()));
        return false;
    }
}

bool SimulationController::loadScenario(const std::string& filename) {
    std::lock_guard<std::mutex> lock(m_simulationMutex);
    
    if (m_status == SimulationStatus::RUNNING) {
        Logger::getInstance().warning("Cannot load scenario while simulation is running");
        return false;
    }
    
    Logger::getInstance().info("Loading scenario from " + filename);
    
    // Clean up existing vehicles
    for (auto vehicle : m_vehicles) {
        delete vehicle;
    }
    m_vehicles.clear();
    m_vehicleMap.clear();
    
    // Load scenario from file
    try {
        std::ifstream file(filename);
        if (!file.is_open()) {
            Logger::getInstance().error("Failed to open scenario file: " + filename);
            return false;
        }
        
        json scenarioJson;
        file >> scenarioJson;
        
        // Parse vehicles
        if (scenarioJson.contains("vehicles")) {
            for (const auto& vehicleJson : scenarioJson["vehicles"]) {
                VehicleType type;
                std::string typeStr = vehicleJson["type"].get<std::string>();
                if (typeStr == "CAR") {
                    type = VehicleType::CAR;
                } else if (typeStr == "BUS") {
                    type = VehicleType::BUS;
                } else if (typeStr == "TRUCK") {
                    type = VehicleType::TRUCK;
                } else if (typeStr == "MOTORCYCLE") {
                    type = VehicleType::MOTORCYCLE;
                } else if (typeStr == "EMERGENCY") {
                    type = VehicleType::EMERGENCY;
                } else if (typeStr == "BICYCLE") {
                    type = VehicleType::BICYCLE;
                } else {
                    type = VehicleType::CAR; // Default
                }
                
                Vector2D position(
                    vehicleJson["position"]["x"].get<double>(),
                    vehicleJson["position"]["y"].get<double>()
                );
                
                // Find road and lane if specified
                Road* road = nullptr;
                int laneIndex = 0;
                
                if (vehicleJson.contains("roadId")) {
                    std::string roadId = vehicleJson["roadId"].get<std::string>();
                    road = getRoad(UUID(roadId));
                    
                    if (vehicleJson.contains("laneIndex")) {
                        laneIndex = vehicleJson["laneIndex"].get<int>();
                    }
                }
                
                // Create vehicle
                createVehicle(type, position, road, laneIndex);
            }
        }
        
        // Parse traffic signals and other scenario elements
        // ...
        
        Logger::getInstance().info("Scenario loaded successfully with " + 
                                  std::to_string(m_vehicles.size()) + " vehicles");
        return true;
        
    } catch (const std::exception& e) {
        Logger::getInstance().error("Error loading scenario: " + std::string(e.what()));
        return false;
    }
}

void SimulationController::start() {
    std::lock_guard<std::mutex> lock(m_simulationMutex);
    
    if (m_status != SimulationStatus::READY && m_status != SimulationStatus::PAUSED) {
        Logger::getInstance().warning("Cannot start simulation: not in READY or PAUSED state");
        return;
    }
    
    m_running = true;
    m_status = SimulationStatus::RUNNING;
    Logger::getInstance().info("Simulation started");
}

void SimulationController::pause() {
    std::lock_guard<std::mutex> lock(m_simulationMutex);
    
    if (m_status != SimulationStatus::RUNNING) {
        Logger::getInstance().warning("Cannot pause simulation: not RUNNING");
        return;
    }
    
    m_running = false;
    m_status = SimulationStatus::PAUSED;
    Logger::getInstance().info("Simulation paused at time: " + std::to_string(m_currentTime));
}

void SimulationController::resume() {
    std::lock_guard<std::mutex> lock(m_simulationMutex);
    
    if (m_status != SimulationStatus::PAUSED) {
        Logger::getInstance().warning("Cannot resume simulation: not PAUSED");
        return;
    }
    
    m_running = true;
    m_status = SimulationStatus::RUNNING;
    Logger::getInstance().info("Simulation resumed at time: " + std::to_string(m_currentTime));
}

void SimulationController::stop() {
    std::lock_guard<std::mutex> lock(m_simulationMutex);
    
    m_running = false;
    m_status = SimulationStatus::STOPPED;
    Logger::getInstance().info("Simulation stopped at time: " + std::to_string(m_currentTime));
}

void SimulationController::reset() {
    std::lock_guard<std::mutex> lock(m_simulationMutex);
    
    // Stop simulation first
    stop();
    
    // Reset time
    m_currentTime = 0.0;
    
    // Clean up vehicles but keep roads and intersections
    for (auto vehicle : m_vehicles) {
        delete vehicle;
    }
    m_vehicles.clear();
    m_vehicleMap.clear();
    
    // Reset statistics
    updateStatistics();
    
    m_status = SimulationStatus::READY;
    Logger::getInstance().info("Simulation reset");
}

void SimulationController::run(double duration, double realTimeFactor) {
    // Start simulation if not running
    if (m_status != SimulationStatus::RUNNING) {
        start();
    }
    
    // Calculate end time
    double endTime = m_currentTime + duration;
    
    // Set simulation speed factor
    m_speedFactor = realTimeFactor;
    
    // Timer for real-time control
    Timer timer;
    
    // Run simulation loop until end time or stopped
    while (m_running && m_currentTime < endTime) {
        timer.start();
        
        // Run a single simulation step
        step(m_timeStep);
        
        // Calculate wait time for real-time factor
        double realTimeStep = m_timeStep / m_speedFactor;
        double elapsedMs = timer.getElapsedMilliseconds();
        int waitMs = static_cast<int>(realTimeStep * 1000 - elapsedMs);
        
        // Sleep if needed to maintain real-time factor
        if (waitMs > 0 && m_speedFactor <= 10.0) {  // Only sleep for reasonable speeds
            std::this_thread::sleep_for(std::chrono::milliseconds(waitMs));
        }
    }
    
    // Ensure statistics are up to date
    updateStatistics();
}

void SimulationController::step(double deltaTime) {
    std::lock_guard<std::mutex> lock(m_simulationMutex);
    
    if (m_status != SimulationStatus::RUNNING) {
        return;
    }
    
    // Update simulation time
    m_currentTime += deltaTime;
    
    // Update all entities
    updateEntities(deltaTime);
    
    // Update statistics periodically (every second of simulation time)
    static double lastStatsUpdate = 0.0;
    if (m_currentTime - lastStatsUpdate >= 1.0) {
        updateStatistics();
        lastStatsUpdate = m_currentTime;
    }
    
    // Clean up inactive entities periodically
    static double lastCleanup = 0.0;
    if (m_currentTime - lastCleanup >= 5.0) {
        cleanupInactiveEntities();
        lastCleanup = m_currentTime;
    }
    
    // Notify callbacks
    notifyUpdateCallbacks(deltaTime);
}

void SimulationController::updateEntities(double deltaTime) {
    // Update vehicles first
    for (auto vehicle : m_vehicles) {
        vehicle->update(deltaTime);
    }
    
    // Update roads
    for (auto road : m_roads) {
        road->update(deltaTime);
    }
    
    // Update intersections
    for (auto intersection : m_intersections) {
        intersection->update(deltaTime);
    }
}

void SimulationController::updateStatistics() {
    // Reset statistics
    m_statistics = SimulationStatistics();
    m_statistics.simulationTime = m_currentTime;
    m_statistics.simulationSpeed = m_speedFactor;
    m_statistics.totalVehicles = static_cast<int>(m_vehicles.size());
    
    // Count active vehicles and calculate averages
    double totalSpeed = 0.0;
    int activeCount = 0;
    
    for (auto vehicle : m_vehicles) {
        if (vehicle->getCurrentSpeed() > 0.001) {  // Vehicle is moving
            activeCount++;
            totalSpeed += vehicle->getCurrentSpeed();
        }
    }
    
    m_statistics.activeVehicles = activeCount;
    
    if (activeCount > 0) {
        m_statistics.averageSpeed = totalSpeed / activeCount;
    }
    
    // Calculate road statistics
    double totalDensity = 0.0;
    double totalFlow = 0.0;
    int roadCount = 0;
    
    for (auto road : m_roads) {
        totalDensity += road->getAverageDensity();
        totalFlow += road->getAverageFlow();
        roadCount++;
    }
    
    if (roadCount > 0) {
        m_statistics.averageDensity = totalDensity / roadCount;
        m_statistics.averageFlow = totalFlow / roadCount;
    }
    
    // Other statistics would be calculated here...
}

void SimulationController::cleanupInactiveEntities() {
    // Remove vehicles that have reached their destinations or are inactive
    std::vector<Vehicle*> vehiclesToRemove;
    
    for (auto vehicle : m_vehicles) {
        // For this simplified example, we'll just remove vehicles that are not moving
        // and have no route planned - in a real simulation, there would be more criteria
        if (vehicle->getCurrentSpeed() < 0.001 && !vehicle->getRouteManager()) {
            vehiclesToRemove.push_back(vehicle);
        }
    }
    
    for (auto vehicle : vehiclesToRemove) {
        removeVehicle(vehicle->getId());
    }
    
    if (!vehiclesToRemove.empty()) {
        Logger::getInstance().debug("Cleaned up " + std::to_string(vehiclesToRemove.size()) + 
                                   " inactive vehicles");
    }
}

void SimulationController::generateRandomTraffic(int vehicleCount, unsigned int seed) {
    std::lock_guard<std::mutex> lock(m_simulationMutex);
    
    if (m_roads.empty()) {
        Logger::getInstance().warning("Cannot generate traffic: no roads in network");
        return;
    }
    
    Logger::getInstance().info("Generating " + std::to_string(vehicleCount) + " random vehicles");
    
    // Set up random number generator
    std::mt19937 rng;
    if (seed == 0) {
        // Use time-based seed
        rng.seed(std::chrono::system_clock::now().time_since_epoch().count());
    } else {
        rng.seed(seed);
    }
    
    // Distribution for selecting roads
    std::uniform_int_distribution<size_t> roadDist(0, m_roads.size() - 1);
    
    // Distribution for vehicle types
    std::discrete_distribution<int> typeDist({75, 5, 10, 5, 2, 3});  // Probabilities for each type
    
    // Distribution for position along road
    std::uniform_real_distribution<double> posDist(0.0, 1.0);
    
    // Create vehicles
    for (int i = 0; i < vehicleCount; i++) {
        // Select random road
        Road* road = m_roads[roadDist(rng)];
        
        // Select random lane
        int laneCount = road->getLaneCount();
        std::uniform_int_distribution<int> laneDist(0, laneCount - 1);
        int laneIndex = laneDist(rng);
        
        // Calculate position along road
        double roadPos = posDist(rng) * road->getLength();
        Vector2D position = road->roadToWorldCoordinates(roadPos, laneIndex);
        
        // Select vehicle type
        int typeIndex = typeDist(rng);
        VehicleType type = static_cast<VehicleType>(typeIndex);
        
        // Create vehicle
        Vehicle* vehicle = createVehicle(type, position, road, laneIndex);
        
        // Set initial speed (50-90% of speed limit)
        if (vehicle) {
            std::uniform_real_distribution<double> speedDist(0.5, 0.9);
            double speedFactor = speedDist(rng);
            double initialSpeed = road->getSpeedLimit() * speedFactor;
            vehicle->setCurrentSpeed(initialSpeed);
        }
    }
    
    Logger::getInstance().info("Generated " + std::to_string(vehicleCount) + " vehicles");
}

Vehicle* SimulationController::createVehicle(VehicleType type, 
                                            const Vector2D& initialPosition,
                                            Road* initialRoad,
                                            int initialLane) {
    std::lock_guard<std::mutex> lock(m_simulationMutex);
    
    Vehicle* vehicle = nullptr;
    
    // Create vehicle based on type
    switch (type) {
        case VehicleType::CAR:
            vehicle = new Car();
            break;
            
        case VehicleType::BUS:
            vehicle = new Bus();
            break;
            
        case VehicleType::TRUCK:
            vehicle = new Truck();
            break;
            
        case VehicleType::MOTORCYCLE:
            vehicle = new EmergencyVehicle();
            break;
            
        case VehicleType::EMERGENCY:
            vehicle = new EmergencyVehicle();
            break;
            
        case VehicleType::BICYCLE:
            // Bicycle would be implemented here
            vehicle = new Vehicle(VehicleType::BICYCLE, 1.7, 0.5, 8.3);  // 30 km/h max
            break;
            
        default:
            vehicle = new Car();
            break;
    }
    
    if (!vehicle) {
        Logger::getInstance().error("Failed to create vehicle");
        return nullptr;
    }
    
    // Set initial position
    vehicle->setPosition(initialPosition);
    
    // Set initial lane if road is provided
    if (initialRoad) {
        if (initialLane >= 0 && initialLane < initialRoad->getLaneCount()) {
            Lane* lane = initialRoad->getLane(initialLane);
            if (lane) {
                vehicle->setCurrentLane(lane);
                
                // Set heading based on road direction
                vehicle->setHeading(initialRoad->getHeading());
            }
        }
    }
    
    // Create a default driver model
    std::shared_ptr<Driver> driver = std::make_shared<Driver>();
    vehicle->setDriver(driver);
    
    // Add to collections
    m_vehicles.push_back(vehicle);
    m_vehicleMap[vehicle->getId()] = vehicle;
    
    Logger::getInstance().debug("Created " + vehicle->toString());
    return vehicle;
}

bool SimulationController::removeVehicle(const UUID& vehicleId) {
    std::lock_guard<std::mutex> lock(m_simulationMutex);
    
    auto it = m_vehicleMap.find(vehicleId);
    if (it == m_vehicleMap.end()) {
        return false;
    }
    
    Vehicle* vehicle = it->second;
    
    // Remove from vector
    auto vecIt = std::find(m_vehicles.begin(), m_vehicles.end(), vehicle);
    if (vecIt != m_vehicles.end()) {
        m_vehicles.erase(vecIt);
    }
    
    // Remove from map
    m_vehicleMap.erase(it);
    
    // Delete vehicle
    Logger::getInstance().debug("Removing " + vehicle->toString());
    delete vehicle;
    
    return true;
}

Road* SimulationController::createRoad(const std::string& name, 
                                      RoadType type,
                                      const Vector2D& startPoint,
                                      const Vector2D& endPoint,
                                      int laneCount,
                                      double speedLimit) {
    std::lock_guard<std::mutex> lock(m_simulationMutex);
    
    // Create road
    Road* road = new Road(name, type, startPoint, endPoint, laneCount, speedLimit);
    
    if (!road) {
        Logger::getInstance().error("Failed to create road");
        return nullptr;
    }
    
    // Add to collections
    m_roads.push_back(road);
    m_roadMap[road->getId()] = road;
    
    Logger::getInstance().debug("Created road: " + name);
    return road;
}

bool SimulationController::removeRoad(const UUID& roadId) {
    std::lock_guard<std::mutex> lock(m_simulationMutex);
    
    auto it = m_roadMap.find(roadId);
    if (it == m_roadMap.end()) {
        return false;
    }
    
    Road* road = it->second;
    
    // First remove all vehicles on this road
    std::vector<Vehicle*> vehiclesToRemove;
    for (auto vehicle : m_vehicles) {
        if (vehicle->getCurrentRoad() == road) {
            vehiclesToRemove.push_back(vehicle);
        }
    }
    
    for (auto vehicle : vehiclesToRemove) {
        removeVehicle(vehicle->getId());
    }
    
    // Remove from vector
    auto vecIt = std::find(m_roads.begin(), m_roads.end(), road);
    if (vecIt != m_roads.end()) {
        m_roads.erase(vecIt);
    }
    
    // Remove from map
    m_roadMap.erase(it);
    
    // Delete road
    Logger::getInstance().debug("Removing road: " + road->getName());
    delete road;
    
    return true;
}

Intersection* SimulationController::createIntersection(const Vector2D& position, 
                                                     const std::string& name) {
    std::lock_guard<std::mutex> lock(m_simulationMutex);
    
    // Generate a name if not provided
    std::string intersectionName = name;
    if (intersectionName.empty()) {
        intersectionName = "Intersection " + std::to_string(m_nextIntersectionId++);
    }
    
    // Create intersection
    Intersection* intersection = new Intersection(intersectionName, position);
    
    if (!intersection) {
        Logger::getInstance().error("Failed to create intersection");
        return nullptr;
    }
    
    // Add to collections
    m_intersections.push_back(intersection);
    m_intersectionMap[intersection->getId()] = intersection;
    
    Logger::getInstance().debug("Created intersection: " + intersectionName);
    return intersection;
}

bool SimulationController::removeIntersection(const UUID& intersectionId) {
    std::lock_guard<std::mutex> lock(m_simulationMutex);
    
    auto it = m_intersectionMap.find(intersectionId);
    if (it == m_intersectionMap.end()) {
        return false;
    }
    
    Intersection* intersection = it->second;
    
    // First disconnect all roads from this intersection
    for (auto road : m_roads) {
        if (road->getStartIntersection() == intersection) {
            road->setStartIntersection(nullptr);
        }
        
        if (road->getEndIntersection() == intersection) {
            road->setEndIntersection(nullptr);
        }
    }
    
    // Remove from vector
    auto vecIt = std::find(m_intersections.begin(), m_intersections.end(), intersection);
    if (vecIt != m_intersections.end()) {
        m_intersections.erase(vecIt);
    }
    
    // Remove from map
    m_intersectionMap.erase(it);
    
    // Delete intersection
    Logger::getInstance().debug("Removing intersection: " + intersection->getName());
    delete intersection;
    
    return true;
}

Road* SimulationController::getRoad(const UUID& id) const {
    auto it = m_roadMap.find(id);
    if (it != m_roadMap.end()) {
        return it->second;
    }
    return nullptr;
}

Vehicle* SimulationController::getVehicle(const UUID& id) const {
    auto it = m_vehicleMap.find(id);
    if (it != m_vehicleMap.end()) {
        return it->second;
    }
    return nullptr;
}

Intersection* SimulationController::getIntersection(const UUID& id) const {
    auto it = m_intersectionMap.find(id);
    if (it != m_intersectionMap.end()) {
        return it->second;
    }
    return nullptr;
}

SimulationStatistics SimulationController::getStatistics() const {
    std::lock_guard<std::mutex> lock(m_simulationMutex);
    return m_statistics;
}

int SimulationController::registerUpdateCallback(const SimulationCallback& callback) {
    std::lock_guard<std::mutex> lock(m_simulationMutex);
    
    int callbackId = m_nextCallbackId++;
    m_updateCallbacks[callbackId] = callback;
    return callbackId;
}

void SimulationController::unregisterUpdateCallback(int callbackId) {
    std::lock_guard<std::mutex> lock(m_simulationMutex);
    
    auto it = m_updateCallbacks.find(callbackId);
    if (it != m_updateCallbacks.end()) {
        m_updateCallbacks.erase(it);
    }
}

void SimulationController::notifyUpdateCallbacks(double deltaTime) {
    // Copy callbacks to avoid issues if callbacks register/unregister others
    std::unordered_map<int, SimulationCallback> callbacks;
    {
        std::lock_guard<std::mutex> lock(m_simulationMutex);
        callbacks = m_updateCallbacks;
    }
    
    // Notify all callbacks
    for (const auto& pair : callbacks) {
        const SimulationCallback& callback = pair.second;
        try {
            callback(deltaTime);
        } catch (const std::exception& e) {
            Logger::getInstance().error("Exception in update callback: " + std::string(e.what()));
        }
    }
}

void SimulationController::buildSampleNetwork() {
    // Create a simple grid network of roads
    
    // Create grid of intersections
    const int gridSize = 5;
    const double spacing = 500.0;  // 500 meters between intersections
    
    // Store intersections in a grid for easy access
    Intersection* intersectionGrid[gridSize][gridSize];
    
    // Create intersections
    for (int i = 0; i < gridSize; i++) {
        for (int j = 0; j < gridSize; j++) {
            Vector2D position(i * spacing, j * spacing);
            std::string name = "Intersection " + std::to_string(i) + "-" + std::to_string(j);
            
            intersectionGrid[i][j] = createIntersection(position, name);
        }
    }
    
    // Create horizontal roads
    for (int i = 0; i < gridSize; i++) {
        for (int j = 0; j < gridSize - 1; j++) {
            std::string name = "Road H" + std::to_string(i) + "-" + std::to_string(j);
            
            RoadType type;
            int lanes;
            
            // Make central roads larger
            if (i == gridSize / 2) {
                type = RoadType::ARTERIAL;
                lanes = 3;
            } else {
                type = RoadType::URBAN;
                lanes = 2;
            }
            
            Road* road = createRoad(
                name,
                type,
                intersectionGrid[j][i]->getPosition(),
                intersectionGrid[j+1][i]->getPosition(),
                lanes
            );
            
            // Connect to intersections
            road->setStartIntersection(intersectionGrid[j][i]);
            road->setEndIntersection(intersectionGrid[j+1][i]);
        }
    }
    
    // Create vertical roads
    for (int i = 0; i < gridSize; i++) {
        for (int j = 0; j < gridSize - 1; j++) {
            std::string name = "Road V" + std::to_string(i) + "-" + std::to_string(j);
            
            RoadType type;
            int lanes;
            
            // Make central roads larger
            if (i == gridSize / 2) {
                type = RoadType::ARTERIAL;
                lanes = 3;
            } else {
                type = RoadType::URBAN;
                lanes = 2;
            }
            
            Road* road = createRoad(
                name,
                type,
                intersectionGrid[i][j]->getPosition(),
                intersectionGrid[i][j+1]->getPosition(),
                lanes
            );
            
            // Connect to intersections
            road->setStartIntersection(intersectionGrid[i][j]);
            road->setEndIntersection(intersectionGrid[i][j+1]);
        }
    }
    
    Logger::getInstance().info("Built sample network with " + 
                              std::to_string(m_intersections.size()) + " intersections and " +
                              std::to_string(m_roads.size()) + " roads");
}

} // namespace TrafficSim