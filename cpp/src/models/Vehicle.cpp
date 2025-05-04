#include "../../include/models/Vehicle.h"
#include "../../include/models/Road.h"
#include "../../include/models/Driver.h"
#include "../../include/models/RouteManager.h"
#include "../../include/utils/Logger.h"
#include <algorithm>
#include <cmath>
#include <sstream>

namespace TrafficSim {

Vehicle::Vehicle(VehicleType type, double length, double width, double maxSpeed)
    : m_id(UUID::generate())
    , m_type(type)
    , m_length(length)
    , m_width(width)
    , m_maxSpeed(maxSpeed)
    , m_currentSpeed(0.0)
    , m_heading(0.0)
    , m_acceleration(0.0)
    , m_braking(0.0)
    , m_lateralOffset(0.0)
    , m_currentLane(nullptr)
    , m_driver(nullptr)
    , m_routeManager(nullptr)
{
    // Set performance characteristics based on vehicle type
    switch (type) {
        case VehicleType::CAR:
            m_maxAcceleration = 3.0;  // 0-100 km/h in about 9 seconds
            m_maxBraking = 6.0;       // Emergency braking at ~0.6G
            break;
            
        case VehicleType::BUS:
            m_maxAcceleration = 1.5;  // Slower acceleration
            m_maxBraking = 4.0;       // Less aggressive braking
            break;
            
        case VehicleType::TRUCK:
            m_maxAcceleration = 1.2;  // Slowest acceleration
            m_maxBraking = 3.5;       // Limited by weight
            break;
            
        case VehicleType::MOTORCYCLE:
            m_maxAcceleration = 4.0;  // Rapid acceleration
            m_maxBraking = 7.0;       // Strong braking
            break;
            
        case VehicleType::EMERGENCY:
            m_maxAcceleration = 3.5;  // Better than standard car
            m_maxBraking = 6.5;       // Better than standard car
            break;
            
        case VehicleType::BICYCLE:
            m_maxAcceleration = 1.0;  // Limited by human power
            m_maxBraking = 3.0;       // Limited by friction
            break;
            
        default:
            m_maxAcceleration = 2.5;
            m_maxBraking = 5.0;
    }
    
    // Log creation
    Logger::getInstance().debug("Created vehicle: " + toString());
}

Vehicle::~Vehicle() {
    // Clean up and remove from current lane if set
    if (m_currentLane) {
        m_currentLane->removeVehicle(this);
    }
    
    // Log destruction
    Logger::getInstance().debug("Destroyed vehicle: " + toString());
}

Road* Vehicle::getCurrentRoad() const {
    if (m_currentLane) {
        return m_currentLane->getRoad();
    }
    return nullptr;
}

void Vehicle::setCurrentSpeed(double speed) {
    // Clamp speed between 0 and max speed
    m_currentSpeed = std::max(0.0, std::min(speed, m_maxSpeed));
}

void Vehicle::setCurrentLane(Lane* lane) {
    // Remove from current lane if set
    if (m_currentLane) {
        m_currentLane->removeVehicle(this);
    }
    
    // Set new lane and add vehicle to it
    m_currentLane = lane;
    
    if (lane) {
        lane->addVehicle(this);
        
        // Set heading based on road direction
        if (auto road = lane->getRoad()) {
            m_heading = road->getHeading();
        }
    }
}

void Vehicle::setDriver(std::shared_ptr<Driver> driver) {
    m_driver = driver;
}

void Vehicle::setRouteManager(std::shared_ptr<RouteManager> routeManager) {
    m_routeManager = routeManager;
}

void Vehicle::update(double deltaTime) {
    // First update the driver behavior if present
    if (m_driver) {
        m_driver->update(this, deltaTime);
    }
    
    // Update vehicle dynamics
    updateDynamics(deltaTime);
    
    // Update position based on current speed
    updatePosition(deltaTime);
    
    // Check for lane changes or route decisions
    if (m_routeManager) {
        m_routeManager->update(this, deltaTime);
    }
}

void Vehicle::updateDynamics(double deltaTime) {
    // Apply acceleration/braking to current speed
    double speedChange = (m_acceleration - m_braking) * deltaTime;
    setCurrentSpeed(m_currentSpeed + speedChange);
    
    // Apply drag (air resistance) as a simple quadratic function
    double dragCoefficient = 0.0005;  // Simplified drag constant
    double drag = dragCoefficient * m_currentSpeed * m_currentSpeed;
    
    // Don't let drag increase speed, only reduce it
    if (m_currentSpeed > 0) {
        setCurrentSpeed(m_currentSpeed - drag * deltaTime);
    }
}

void Vehicle::updatePosition(double deltaTime) {
    // Calculate movement direction vector
    Vector2D direction(cos(m_heading), sin(m_heading));
    
    // Move vehicle based on current speed
    m_position += direction * (m_currentSpeed * deltaTime);
    
    // If we're on a road, snap to road coordinates
    if (auto road = getCurrentRoad()) {
        // First get road-relative position
        Vector2D roadCoord = road->worldToRoadCoordinates(m_position);
        
        // Then snap back to world, incorporating lateral offset
        if (m_currentLane) {
            m_position = road->roadToWorldCoordinates(
                roadCoord.x,  // Distance along road
                m_currentLane->getIndex(),
                m_lateralOffset  // Lateral offset from lane center
            );
        }
    }
}

void Vehicle::accelerate(double amount) {
    // Clamp to max acceleration
    m_acceleration = std::min(amount, m_maxAcceleration);
    m_braking = 0.0;  // Can't accelerate and brake simultaneously
}

void Vehicle::brake(double amount) {
    // Clamp to max braking
    m_braking = std::min(amount, m_maxBraking);
    m_acceleration = 0.0;  // Can't accelerate and brake simultaneously
}

void Vehicle::stop() {
    m_acceleration = 0.0;
    m_braking = m_maxBraking;  // Apply full braking
}

bool Vehicle::detectCollision(const Vehicle& other) const {
    // Simple bounding box collision detection
    // More sophisticated models would use vehicle shape and orientation
    
    // If not on the same road, no collision
    if (getCurrentRoad() != other.getCurrentRoad()) {
        return false;
    }
    
    // Calculate safe longitudinal distance (based on length and a safety margin)
    double longitudinalSafeDistance = (m_length + other.getLength()) / 2.0 + 0.5;
    
    // Calculate safe lateral distance (based on width and a safety margin)
    double lateralSafeDistance = (m_width + other.getWidth()) / 2.0 + 0.2;
    
    // Get road-relative positions
    Road* road = getCurrentRoad();
    Vector2D thisRoadPos = road->worldToRoadCoordinates(m_position);
    Vector2D otherRoadPos = road->worldToRoadCoordinates(other.getPosition());
    
    // Calculate distances
    double longitudinalDistance = std::abs(thisRoadPos.x - otherRoadPos.x);
    double lateralDistance = std::abs(thisRoadPos.y - otherRoadPos.y);
    
    // Collision if both distances are less than safe distances
    return (longitudinalDistance < longitudinalSafeDistance) && 
           (lateralDistance < lateralSafeDistance);
}

double Vehicle::calculateSafeDistance() const {
    // Safe following distance based on current speed
    // Using a simplified version of the 2-second rule
    double reactionTime = 2.0;  // seconds
    
    return m_currentSpeed * reactionTime + m_length;
}

double Vehicle::calculateFollowingSpeed() const {
    if (!m_currentLane) {
        return m_maxSpeed;
    }
    
    // Get vehicle ahead and distance
    double distance = 0.0;
    Vehicle* ahead = m_currentLane->getVehicleAhead(const_cast<Vehicle*>(this), distance);
    
    if (!ahead || distance > calculateSafeDistance()) {
        // No vehicle ahead or far enough, use max speed
        return m_maxSpeed;
    }
    
    // Calculate safe speed based on distance
    // This is a simplified car-following model
    double safeSpeed = ahead->getCurrentSpeed() * (distance / calculateSafeDistance());
    
    return std::min(safeSpeed, m_maxSpeed);
}

std::string Vehicle::toString() const {
    std::stringstream ss;
    ss << "Vehicle[ID=" << m_id.toString().substr(0, 8) << ", ";
    
    // Type to string
    switch (m_type) {
        case VehicleType::CAR: ss << "Car"; break;
        case VehicleType::BUS: ss << "Bus"; break;
        case VehicleType::TRUCK: ss << "Truck"; break;
        case VehicleType::MOTORCYCLE: ss << "Motorcycle"; break;
        case VehicleType::EMERGENCY: ss << "Emergency"; break;
        case VehicleType::BICYCLE: ss << "Bicycle"; break;
        default: ss << "Unknown"; break;
    }
    
    ss << ", Pos=(" << m_position.x << "," << m_position.y << ")";
    ss << ", Speed=" << m_currentSpeed << "m/s";
    
    if (getCurrentRoad()) {
        ss << ", Road=" << getCurrentRoad()->getName();
    }
    
    ss << "]";
    return ss.str();
}

// Car implementation
Car::Car(double maxSpeed)
    : Vehicle(VehicleType::CAR, 4.5, 1.8, maxSpeed)
{
    // Car-specific initialization
}

void Car::update(double deltaTime) {
    // Base vehicle update
    Vehicle::update(deltaTime);
    
    // Car-specific behavior
    // Cars tend to maintain speed close to speed limit when possible
    if (m_currentLane && m_acceleration < 0.1 && m_braking < 0.1) {
        double speedLimit = m_currentLane->getSpeedLimit();
        if (speedLimit > 0 && m_currentSpeed < speedLimit * 0.9) {
            // Gradually accelerate towards speed limit
            accelerate(m_maxAcceleration * 0.5);
        }
    }
}

// Bus implementation
Bus::Bus(double maxSpeed)
    : Vehicle(VehicleType::BUS, 12.0, 2.5, maxSpeed)
{
    // Bus-specific initialization
}

void Bus::update(double deltaTime) {
    // Check if we're at a bus stop and need to wait
    if (m_doorOpenTime > 0) {
        m_doorOpenTime -= deltaTime;
        // Bus is stopped while loading/unloading
        stop();
        
        if (m_doorOpenTime <= 0) {
            // Doors closed, ready to move again
            Logger::getInstance().debug("Bus " + m_id.toString().substr(0, 8) + 
                                       " finished stop, moving again");
        }
        
        return;
    }
    
    // Regular update
    Vehicle::update(deltaTime);
    
    // Bus-specific behavior
    // Buses drive more carefully and maintain larger following distances
    // They also need to check for bus stops
    
    // Check for nearby bus stops (would be implemented with route information)
    // This is a placeholder for actual bus stop detection
    for (const auto& stopPos : m_stops) {
        if (Vector2D::distance(m_position, stopPos) < 5.0) {
            // We're at a bus stop
            m_doorOpenTime = 10.0;  // Wait 10 seconds at stop
            Logger::getInstance().debug("Bus " + m_id.toString().substr(0, 8) + 
                                       " stopping at bus stop");
            stop();
            break;
        }
    }
}

void Bus::loadPassengers(int count) {
    m_passengerCount += count;
    if (m_passengerCount > m_maxPassengers) {
        m_passengerCount = m_maxPassengers;
    }
    
    Logger::getInstance().debug("Bus " + m_id.toString().substr(0, 8) + 
                               " loaded " + std::to_string(count) + 
                               " passengers, now at " + std::to_string(m_passengerCount));
}

void Bus::unloadPassengers(int count) {
    m_passengerCount -= count;
    if (m_passengerCount < 0) {
        m_passengerCount = 0;
    }
    
    Logger::getInstance().debug("Bus " + m_id.toString().substr(0, 8) + 
                               " unloaded " + std::to_string(count) + 
                               " passengers, now at " + std::to_string(m_passengerCount));
}

// Truck implementation
Truck::Truck(double maxSpeed, double cargoWeight)
    : Vehicle(VehicleType::TRUCK, 18.0, 2.5, maxSpeed)
    , m_cargoWeight(cargoWeight)
{
    // Adjust performance based on cargo weight
    setCargoWeight(cargoWeight);
}

void Truck::update(double deltaTime) {
    // Base vehicle update
    Vehicle::update(deltaTime);
    
    // Truck-specific behavior
    // Trucks maintain steadier speeds and accelerate more gradually
    if (m_currentSpeed > 0 && m_acceleration > 0) {
        // Limit acceleration based on current speed
        double speedFactor = 1.0 - (m_currentSpeed / m_maxSpeed);
        m_acceleration = m_acceleration * speedFactor;
    }
}

void Truck::setCargoWeight(double weight) {
    m_cargoWeight = std::max(0.0, weight);
    
    // Adjust performance based on weight
    // Maximum effect at 40 tons (40,000 kg)
    double weightFactor = 1.0 - std::min(0.7, m_cargoWeight / 40000.0);
    
    // Adjust max acceleration & braking based on weight
    m_maxAcceleration = 1.2 * weightFactor;
    m_maxBraking = 3.5 * weightFactor;
}

// Emergency vehicle implementation
EmergencyVehicle::EmergencyVehicle(double maxSpeed)
    : Vehicle(VehicleType::EMERGENCY, 5.5, 2.0, maxSpeed)
{
    // Emergency vehicle specific initialization
}

void EmergencyVehicle::update(double deltaTime) {
    // Base vehicle update
    Vehicle::update(deltaTime);
    
    // Emergency vehicle specific behavior
    if (m_sirenActive) {
        // When siren is active, try to maintain higher speed
        // and other vehicles should yield to this one
        
        if (m_currentSpeed < m_maxSpeed * 0.8 && m_acceleration < m_maxAcceleration * 0.7) {
            // Accelerate more aggressively when siren is on
            accelerate(m_maxAcceleration * 0.8);
        }
        
        // In a full implementation, we would notify nearby vehicles
        // to move out of the way. This would be handled by a notification
        // system or by vehicles checking for nearby emergency vehicles.
    }
}

void EmergencyVehicle::activateSiren(bool state) {
    m_sirenActive = state;
    Logger::getInstance().debug("Emergency vehicle " + m_id.toString().substr(0, 8) + 
                              (state ? " activated siren" : " deactivated siren"));
}

} // namespace TrafficSim