#ifndef TRAFFIC_SIMULATION_ROAD_H
#define TRAFFIC_SIMULATION_ROAD_H

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
#include "../utils/Vector2D.h"
#include "../utils/UUID.h"

namespace TrafficSim {

// Forward declarations
class Vehicle;
class Intersection;
class TrafficLight;

/**
 * @brief Road speed limits in m/s
 */
enum class SpeedLimit {
    RESIDENTIAL = 8.33,   // 30 km/h
    URBAN = 13.89,        // 50 km/h
    ARTERIAL = 16.67,     // 60 km/h
    HIGHWAY = 27.78,      // 100 km/h
    FREEWAY = 33.33       // 120 km/h
};

/**
 * @brief Types of roads in the network
 */
enum class RoadType {
    RESIDENTIAL,
    URBAN,
    ARTERIAL,
    HIGHWAY,
    FREEWAY
};

/**
 * @brief Lane represents a single lane on a road
 */
class Lane {
public:
    /**
     * @brief Construct a new Lane object
     * 
     * @param id Unique identifier
     * @param index Index of lane in the road (0 = rightmost lane)
     * @param width Width of lane in meters
     * @param speedLimit Speed limit for this specific lane
     */
    Lane(const UUID& id, int index, double width = 3.5, double speedLimit = 0.0);
    ~Lane();
    
    // Disable copy and assignment
    Lane(const Lane&) = delete;
    Lane& operator=(const Lane&) = delete;
    
    // Lane properties
    const UUID& getId() const { return m_id; }
    int getIndex() const { return m_index; }
    double getWidth() const { return m_width; }
    double getSpeedLimit() const { return m_speedLimit; }
    Road* getRoad() const { return m_road; }
    
    // Vehicle management
    void addVehicle(Vehicle* vehicle);
    void removeVehicle(Vehicle* vehicle);
    const std::vector<Vehicle*>& getVehicles() const { return m_vehicles; }
    
    // Lane neighbors
    Lane* getLeftLane() const { return m_leftLane; }
    Lane* getRightLane() const { return m_rightLane; }
    void setLeftLane(Lane* lane) { m_leftLane = lane; }
    void setRightLane(Lane* lane) { m_rightLane = lane; }
    
    // Traffic flow
    Vehicle* getVehicleAhead(Vehicle* vehicle, double& distance) const;
    Vehicle* getVehicleBehind(Vehicle* vehicle, double& distance) const;
    double getDensity() const;  // Vehicles per km
    double getFlow() const;     // Vehicles per hour
    double getAverageSpeed() const;
    
    // Update
    void update(double deltaTime);
    
private:
    UUID m_id;
    int m_index;         // Lane index (0 = rightmost)
    double m_width;      // Lane width in meters
    double m_speedLimit; // Lane-specific speed limit (if 0, uses road's)
    Road* m_road;        // Parent road
    
    // Connected lanes
    Lane* m_leftLane = nullptr;
    Lane* m_rightLane = nullptr;
    
    // Vehicles currently in lane
    std::vector<Vehicle*> m_vehicles;
    
    // Traffic statistics
    double m_trafficDensity = 0.0;
    double m_trafficFlow = 0.0;
    double m_averageSpeed = 0.0;
    
    friend class Road;  // Road can access private members
};

/**
 * @brief Road represents a road segment with multiple lanes
 */
class Road {
public:
    /**
     * @brief Construct a new Road object
     * 
     * @param name Human-readable name
     * @param type Road type
     * @param startPoint Start coordinate
     * @param endPoint End coordinate
     * @param laneCount Number of lanes in each direction
     * @param speedLimit Speed limit in m/s
     */
    Road(const std::string& name, RoadType type, 
         const Vector2D& startPoint, const Vector2D& endPoint,
         int laneCount = 1, double speedLimit = 0.0);
    ~Road();
    
    // Disable copy and assignment
    Road(const Road&) = delete;
    Road& operator=(const Road&) = delete;
    
    // Basic properties
    const UUID& getId() const { return m_id; }
    const std::string& getName() const { return m_name; }
    RoadType getType() const { return m_type; }
    double getLength() const { return m_length; }
    double getSpeedLimit() const { return m_speedLimit; }
    
    // Geometry
    const Vector2D& getStartPoint() const { return m_startPoint; }
    const Vector2D& getEndPoint() const { return m_endPoint; }
    double getHeading() const { return m_heading; }
    
    // Lane management
    int getLaneCount() const { return static_cast<int>(m_lanes.size()); }
    Lane* getLane(int index);
    const std::vector<Lane*>& getLanes() const { return m_lanes; }
    
    void addLane(double width = 3.5, double speedLimit = 0.0);
    void removeLane(int index);
    
    // Intersection connections
    void setStartIntersection(Intersection* intersection);
    void setEndIntersection(Intersection* intersection);
    Intersection* getStartIntersection() const { return m_startIntersection; }
    Intersection* getEndIntersection() const { return m_endIntersection; }
    
    // Traffic signals
    void setTrafficLight(TrafficLight* light);
    TrafficLight* getTrafficLight() const { return m_trafficLight; }
    
    // Coordinate conversion
    Vector2D worldToRoadCoordinates(const Vector2D& worldPos) const;
    Vector2D roadToWorldCoordinates(double distance, int lane, double lateralOffset = 0.0) const;
    
    // Traffic statistics
    double getAverageSpeed() const;
    double getAverageDensity() const;
    double getAverageFlow() const;
    
    // Update
    void update(double deltaTime);
    
private:
    UUID m_id;
    std::string m_name;
    RoadType m_type;
    
    // Geometry
    Vector2D m_startPoint;
    Vector2D m_endPoint;
    double m_length;     // Cached length in meters
    double m_heading;    // Cached heading in radians
    double m_speedLimit; // Speed limit in m/s
    
    // Lanes
    std::vector<Lane*> m_lanes;
    
    // Connected intersections
    Intersection* m_startIntersection = nullptr;
    Intersection* m_endIntersection = nullptr;
    
    // Traffic control
    TrafficLight* m_trafficLight = nullptr;
    
    // Update internal geometry calculations
    void updateGeometry();
    
    // Connect lanes after changes
    void connectLanes();
};

} // namespace TrafficSim

#endif // TRAFFIC_SIMULATION_ROAD_H