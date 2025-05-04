#ifndef TRAFFIC_SIMULATION_CONTROLLER_H
#define TRAFFIC_SIMULATION_CONTROLLER_H

#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include <mutex>
#include <atomic>
#include <functional>
#include "../models/Vehicle.h"
#include "../models/Road.h"
#include "../models/Intersection.h"
#include "../utils/UUID.h"

namespace TrafficSim {

struct SimulationStatistics {
    double averageSpeed = 0.0;         // Average speed in m/s
    double averageDensity = 0.0;       // Average vehicle density (vehicles/km)
    double averageFlow = 0.0;          // Average flow rate (vehicles/hour)
    int totalVehicles = 0;             // Total vehicles in simulation
    int activeVehicles = 0;            // Currently active vehicles
    int completedTrips = 0;            // Vehicles that reached destination
    double totalCongestionTime = 0.0;  // Time spent in congestion (vehicle-hours)
    double averageTripTime = 0.0;      // Average trip time (seconds)
    double simulationTime = 0.0;       // Current simulation time
    double simulationSpeed = 0.0;      // Simulation speed factor
};

enum class SimulationStatus {
    INITIALIZING,
    READY,
    RUNNING,
    PAUSED,
    STOPPED,
    ERROR
};

// Callback type for simulation events
using SimulationCallback = std::function<void(double)>;

/**
 * @brief Main controller for the traffic simulation
 * 
 * Manages the simulation components, updates, and provides control interfaces
 */
class SimulationController {
public:
    /**
     * @brief Get the singleton instance
     * 
     * @return SimulationController& Reference to the singleton instance
     */
    static SimulationController& getInstance();
    
    /**
     * @brief Initialize the simulation
     * 
     * @param configFile Path to configuration file (optional)
     * @return true if initialization successful
     * @return false if initialization failed
     */
    bool initialize(const std::string& configFile = "");
    
    /**
     * @brief Load road network from file
     * 
     * @param filename Path to network file
     * @return true if loading successful
     * @return false if loading failed
     */
    bool loadNetwork(const std::string& filename);
    
    /**
     * @brief Save current network to file
     * 
     * @param filename Output file path
     * @return true if save successful
     * @return false if save failed
     */
    bool saveNetwork(const std::string& filename) const;
    
    /**
     * @brief Load traffic scenario from file
     * 
     * @param filename Path to scenario file
     * @return true if loading successful
     * @return false if loading failed
     */
    bool loadScenario(const std::string& filename);
    
    /**
     * @brief Start the simulation
     */
    void start();
    
    /**
     * @brief Pause the simulation
     */
    void pause();
    
    /**
     * @brief Resume a paused simulation
     */
    void resume();
    
    /**
     * @brief Stop the simulation
     */
    void stop();
    
    /**
     * @brief Reset the simulation to initial state
     */
    void reset();
    
    /**
     * @brief Run simulation for specified duration
     * 
     * @param duration Duration in simulation seconds
     * @param realTimeFactor Speed multiplier (1.0 = realtime)
     */
    void run(double duration, double realTimeFactor = 1.0);
    
    /**
     * @brief Advance simulation by single step
     * 
     * @param deltaTime Time step in seconds
     */
    void step(double deltaTime);
    
    /**
     * @brief Get current simulation time
     * 
     * @return double Current time in seconds
     */
    double getCurrentTime() const { return m_currentTime; }
    
    /**
     * @brief Get current simulation status
     * 
     * @return SimulationStatus Current status
     */
    SimulationStatus getStatus() const { return m_status; }
    
    /**
     * @brief Get simulation time step
     * 
     * @return double Time step in seconds
     */
    double getTimeStep() const { return m_timeStep; }
    
    /**
     * @brief Set simulation time step
     * 
     * @param timeStep Time step in seconds
     */
    void setTimeStep(double timeStep) { m_timeStep = timeStep; }
    
    /**
     * @brief Get simulation speed factor
     * 
     * @return double Speed factor (1.0 = realtime)
     */
    double getSimulationSpeedFactor() const { return m_speedFactor; }
    
    /**
     * @brief Set simulation speed factor
     * 
     * @param factor Speed factor (1.0 = realtime)
     */
    void setSimulationSpeedFactor(double factor) { m_speedFactor = factor; }
    
    /**
     * @brief Generate random traffic
     * 
     * @param vehicleCount Number of vehicles to generate
     * @param seed Random seed (0 for time-based)
     */
    void generateRandomTraffic(int vehicleCount, unsigned int seed = 0);
    
    /**
     * @brief Create vehicle of specified type
     * 
     * @param type Vehicle type
     * @param initialPosition Initial position (world coordinates)
     * @param initialRoad Initial road (optional)
     * @param initialLane Initial lane index (optional)
     * @return Vehicle* Pointer to created vehicle (nullptr if failed)
     */
    Vehicle* createVehicle(VehicleType type, 
                          const Vector2D& initialPosition,
                          Road* initialRoad = nullptr,
                          int initialLane = 0);
    
    /**
     * @brief Remove vehicle from simulation
     * 
     * @param vehicleId Vehicle ID to remove
     * @return true if successfully removed
     * @return false if vehicle not found
     */
    bool removeVehicle(const UUID& vehicleId);
    
    /**
     * @brief Create road
     * 
     * @param name Road name
     * @param type Road type
     * @param startPoint Start coordinates
     * @param endPoint End coordinates
     * @param laneCount Number of lanes
     * @param speedLimit Speed limit (0 = use default for type)
     * @return Road* Pointer to created road (nullptr if failed)
     */
    Road* createRoad(const std::string& name, 
                    RoadType type,
                    const Vector2D& startPoint,
                    const Vector2D& endPoint,
                    int laneCount = 1,
                    double speedLimit = 0.0);
    
    /**
     * @brief Remove road from simulation
     * 
     * @param roadId Road ID to remove
     * @return true if successfully removed
     * @return false if road not found
     */
    bool removeRoad(const UUID& roadId);
    
    /**
     * @brief Create intersection
     * 
     * @param position Position in world coordinates
     * @param name Intersection name
     * @return Intersection* Pointer to created intersection (nullptr if failed)
     */
    Intersection* createIntersection(const Vector2D& position, const std::string& name = "");
    
    /**
     * @brief Remove intersection from simulation
     * 
     * @param intersectionId Intersection ID to remove
     * @return true if successfully removed
     * @return false if intersection not found
     */
    bool removeIntersection(const UUID& intersectionId);
    
    /**
     * @brief Get road by ID
     * 
     * @param id Road ID
     * @return Road* Pointer to road (nullptr if not found)
     */
    Road* getRoad(const UUID& id) const;
    
    /**
     * @brief Get vehicle by ID
     * 
     * @param id Vehicle ID
     * @return Vehicle* Pointer to vehicle (nullptr if not found)
     */
    Vehicle* getVehicle(const UUID& id) const;
    
    /**
     * @brief Get intersection by ID
     * 
     * @param id Intersection ID
     * @return Intersection* Pointer to intersection (nullptr if not found)
     */
    Intersection* getIntersection(const UUID& id) const;
    
    /**
     * @brief Get all roads
     * 
     * @return const std::vector<Road*>& Vector of road pointers
     */
    const std::vector<Road*>& getAllRoads() const { return m_roads; }
    
    /**
     * @brief Get all vehicles
     * 
     * @return const std::vector<Vehicle*>& Vector of vehicle pointers
     */
    const std::vector<Vehicle*>& getAllVehicles() const { return m_vehicles; }
    
    /**
     * @brief Get all intersections
     * 
     * @return const std::vector<Intersection*>& Vector of intersection pointers
     */
    const std::vector<Intersection*>& getAllIntersections() const { return m_intersections; }
    
    /**
     * @brief Get current simulation statistics
     * 
     * @return SimulationStatistics Current statistics
     */
    SimulationStatistics getStatistics() const;
    
    /**
     * @brief Register callback for update events
     * 
     * @param callback Function to call after each update
     * @return int Callback ID for later removal
     */
    int registerUpdateCallback(const SimulationCallback& callback);
    
    /**
     * @brief Unregister update callback
     * 
     * @param callbackId Callback ID to remove
     */
    void unregisterUpdateCallback(int callbackId);
    
    // Singleton enforcement
    SimulationController(const SimulationController&) = delete;
    SimulationController& operator=(const SimulationController&) = delete;
    
private:
    // Private constructor for singleton pattern
    SimulationController();
    ~SimulationController();
    
    // Singleton instance
    static SimulationController* m_instance;
    
    // Current simulation state
    SimulationStatus m_status = SimulationStatus::INITIALIZING;
    double m_currentTime = 0.0;
    double m_timeStep = 0.1;  // Default time step (100ms)
    double m_speedFactor = 1.0;
    std::atomic<bool> m_running{false};
    
    // Entity storage
    std::vector<Vehicle*> m_vehicles;
    std::vector<Road*> m_roads;
    std::vector<Intersection*> m_intersections;
    
    // Entity lookup maps
    std::unordered_map<UUID, Vehicle*, UUIDHasher> m_vehicleMap;
    std::unordered_map<UUID, Road*, UUIDHasher> m_roadMap;
    std::unordered_map<UUID, Intersection*, UUIDHasher> m_intersectionMap;
    
    // Statistics
    SimulationStatistics m_statistics;
    
    // ID counters
    int m_nextVehicleId = 0;
    int m_nextRoadId = 0;
    int m_nextIntersectionId = 0;
    
    // Update callbacks
    std::unordered_map<int, SimulationCallback> m_updateCallbacks;
    int m_nextCallbackId = 0;
    
    // Thread safety
    mutable std::mutex m_simulationMutex;
    
    // Internal helpers
    void updateEntities(double deltaTime);
    void updateStatistics();
    void buildSampleNetwork();
    void notifyUpdateCallbacks(double deltaTime);
    void cleanupInactiveEntities();
};

} // namespace TrafficSim

#endif // TRAFFIC_SIMULATION_CONTROLLER_H