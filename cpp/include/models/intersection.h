#ifndef TRAFFIC_SIMULATION_INTERSECTION_H
#define TRAFFIC_SIMULATION_INTERSECTION_H

#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include "../utils/Vector2D.h"
#include "../utils/UUID.h"

namespace TrafficSim {

// Forward declarations
class Road;
class Lane;
class TrafficLight;
class Vehicle;

/**
 * @brief Represents an intersection between roads
 */
class Intersection {
public:
    /**
     * @brief Construct a new Intersection
     * 
     * @param name Human-readable name
     * @param position World coordinates
     */
    Intersection(const std::string& name, const Vector2D& position);
    
    /**
     * @brief Destructor
     */
    ~Intersection();
    
    // Disable copying
    Intersection(const Intersection&) = delete;
    Intersection& operator=(const Intersection&) = delete;
    
    // Basic properties
    const UUID& getId() const { return m_id; }
    const std::string& getName() const { return m_name; }
    const Vector2D& getPosition() const { return m_position; }
    
    // Road connections
    void addRoad(Road* road);
    void removeRoad(Road* road);
    const std::vector<Road*>& getConnectedRoads() const { return m_connectedRoads; }
    
    // Traffic control
    void setTrafficLight(TrafficLight* trafficLight);
    TrafficLight* getTrafficLight() const { return m_trafficLight; }
    
    /**
     * @brief Calculate connection lane from one road to another
     * 
     * @param fromRoad Source road
     * @param fromLane Source lane
     * @param toRoad Target road
     * @return Lane* Target lane, or nullptr if no connection
     */
    Lane* getConnectionLane(Road* fromRoad, Lane* fromLane, Road* toRoad) const;
    
    /**
     * @brief Get all possible exit roads from a given entry road
     * 
     * @param entryRoad Entry road
     * @return std::vector<Road*> Vector of possible exit roads
     */
    std::vector<Road*> getPossibleExits(Road* entryRoad) const;
    
    /**
     * @brief Check if vehicle has right-of-way through intersection
     * 
     * @param vehicle Vehicle to check
     * @param fromRoad Road the vehicle is coming from
     * @param toRoad Road the vehicle wants to go to
     * @return true if vehicle has right-of-way
     */
    bool hasRightOfWay(Vehicle* vehicle, Road* fromRoad, Road* toRoad) const;
    
    /**
     * @brief Update the intersection
     * 
     * @param deltaTime Time step in seconds
     */
    void update(double deltaTime);
    
private:
    UUID m_id;
    std::string m_name;
    Vector2D m_position;
    
    // Connected roads
    std::vector<Road*> m_connectedRoads;
    
    // Traffic control
    TrafficLight* m_trafficLight;
    
    // Road connections (fromRoad ID -> toRoad ID -> lane mapping)
    std::unordered_map<UUID, 
                       std::unordered_map<UUID, 
                                         std::vector<std::pair<int, int>>>, 
                       UUIDHasher> m_connections;
    
    // Vehicle tracking for traffic control
    std::vector<Vehicle*> m_approachingVehicles;
    
    // Calculate traffic light phases
    void updateTrafficLightPhases();
    
    // Calculate optimal lane connections
    void calculateLaneConnections();
    
    // Check vehicle precedence (right-hand rule)
    bool checkRightHandPrecedence(Road* fromRoad, Road* toRoad) const;
};

} // namespace TrafficSim

#endif // TRAFFIC_SIMULATION_INTERSECTION_H