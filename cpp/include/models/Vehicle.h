#ifndef TRAFFIC_SIMULATION_VEHICLE_H
#define TRAFFIC_SIMULATION_VEHICLE_H

#include <string>
#include <memory>
#include <vector>
#include "../utils/Vector2D.h"
#include "../utils/UUID.h"

namespace TrafficSim {

// Forward declarations
class Road;
class Lane;
class Driver;
class RouteManager;

/**
 * @brief Enumeration of different vehicle types
 */
enum class VehicleType {
    CAR,
    BUS,
    TRUCK,
    MOTORCYCLE,
    EMERGENCY,
    BICYCLE
};

/**
 * @brief Base class for all vehicles in the simulation
 * 
 * Provides common functionality and properties for different vehicle types
 */
class Vehicle {
public:
    /**
     * @brief Construct a new Vehicle object
     * 
     * @param type Type of vehicle
     * @param length Length of vehicle in meters
     * @param width Width of vehicle in meters
     * @param maxSpeed Maximum speed of vehicle in m/s
     */
    Vehicle(VehicleType type, double length = 4.5, double width = 2.0, double maxSpeed = 33.3);
    
    /**
     * @brief Virtual destructor
     */
    virtual ~Vehicle();
    
    // Disable copy and assignment
    Vehicle(const Vehicle&) = delete;
    Vehicle& operator=(const Vehicle&) = delete;
    
    // Basic properties
    const UUID& getId() const { return m_id; }
    VehicleType getType() const { return m_type; }
    double getLength() const { return m_length; }
    double getWidth() const { return m_width; }
    double getMaxSpeed() const { return m_maxSpeed; }
    double getCurrentSpeed() const { return m_currentSpeed; }
    
    // Positioning
    const Vector2D& getPosition() const { return m_position; }
    double getHeading() const { return m_heading; }
    double getLateralOffset() const { return m_lateralOffset; }
    Lane* getCurrentLane() const { return m_currentLane; }
    Road* getCurrentRoad() const;
    
    // Acceleration and movement
    double getAcceleration() const { return m_acceleration; }
    double getBraking() const { return m_braking; }
    
    // State modifiers
    void setPosition(const Vector2D& position) { m_position = position; }
    void setHeading(double heading) { m_heading = heading; }
    void setCurrentSpeed(double speed);
    void setCurrentLane(Lane* lane);
    void setLateralOffset(double offset) { m_lateralOffset = offset; }
    
    // Driver behavior
    void setDriver(std::shared_ptr<Driver> driver);
    std::shared_ptr<Driver> getDriver() const { return m_driver; }
    
    // Route management
    void setRouteManager(std::shared_ptr<RouteManager> routeManager);
    std::shared_ptr<RouteManager> getRouteManager() const { return m_routeManager; }
    
    // Update method called every simulation step
    virtual void update(double deltaTime);
    
    // Movement control
    virtual void accelerate(double amount);
    virtual void brake(double amount);
    virtual void stop();
    
    // Collision detection
    virtual bool detectCollision(const Vehicle& other) const;
    
    // Debug string
    virtual std::string toString() const;
    
protected:
    // Core properties
    UUID m_id;
    VehicleType m_type;
    double m_length;     // Vehicle length in meters
    double m_width;      // Vehicle width in meters
    double m_maxSpeed;   // Maximum speed in m/s
    
    // Current state
    Vector2D m_position;      // Position in world coordinates
    double m_heading;         // Heading in radians
    double m_currentSpeed;    // Current speed in m/s
    double m_acceleration;    // Current acceleration in m/s²
    double m_braking;         // Current braking force in m/s²
    double m_lateralOffset;   // Lateral offset from lane center
    
    // Performance characteristics
    double m_maxAcceleration; // Maximum acceleration capability
    double m_maxBraking;      // Maximum braking capability
    
    // Current lane
    Lane* m_currentLane;
    
    // Driver behavior model
    std::shared_ptr<Driver> m_driver;
    
    // Route management
    std::shared_ptr<RouteManager> m_routeManager;
    
    // Vehicle dynamics model - overridable by subclasses
    virtual void updatePosition(double deltaTime);
    virtual void updateDynamics(double deltaTime);
    
    // Helper methods
    double calculateSafeDistance() const;
    double calculateFollowingSpeed() const;
};

/**
 * @brief Car implementation
 */
class Car : public Vehicle {
public:
    Car(double maxSpeed = 33.3);  // Default 120 km/h
    virtual ~Car() = default;
    
    virtual void update(double deltaTime) override;
};

/**
 * @brief Bus implementation
 */
class Bus : public Vehicle {
public:
    Bus(double maxSpeed = 25.0);  // Default 90 km/h
    virtual ~Bus() = default;
    
    virtual void update(double deltaTime) override;
    
    // Bus-specific methods
    void loadPassengers(int count);
    void unloadPassengers(int count);
    int getPassengerCount() const { return m_passengerCount; }
    int getMaxPassengers() const { return m_maxPassengers; }
    
private:
    int m_passengerCount = 0;
    int m_maxPassengers = 80;
    std::vector<Vector2D> m_stops;
    double m_doorOpenTime = 0.0;
};

/**
 * @brief Truck implementation
 */
class Truck : public Vehicle {
public:
    Truck(double maxSpeed = 22.2, double cargoWeight = 5000.0);  // Default 80 km/h
    virtual ~Truck() = default;
    
    virtual void update(double deltaTime) override;
    
    // Truck-specific methods
    double getCargoWeight() const { return m_cargoWeight; }
    void setCargoWeight(double weight);
    
private:
    double m_cargoWeight;  // Weight in kg
};

/**
 * @brief Emergency vehicle implementation
 */
class EmergencyVehicle : public Vehicle {
public:
    EmergencyVehicle(double maxSpeed = 44.4);  // Default 160 km/h
    virtual ~EmergencyVehicle() = default;
    
    virtual void update(double deltaTime) override;
    
    // Emergency vehicle specific methods
    void activateSiren(bool state);
    bool isSirenActive() const { return m_sirenActive; }
    
private:
    bool m_sirenActive = false;
    double m_sirenEffectRadius = 50.0;  // Radius in meters
};

} // namespace TrafficSim

#endif // TRAFFIC_SIMULATION_VEHICLE_H