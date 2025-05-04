#include "../../include/models/Intersection.h"
#include "../../include/models/Road.h"
#include "../../include/models/TrafficLight.h"
#include "../../include/models/Vehicle.h"
#include "../../include/utils/Logger.h"
#include <algorithm>
#include <cmath>
#include <sstream>

namespace TrafficSim {

Intersection::Intersection(const std::string& name, const Vector2D& position)
    : m_id(UUID::generate())
    , m_name(name)
    , m_position(position)
    , m_trafficLight(nullptr)
{
    Logger::getInstance().debug("Created intersection: " + m_name);
}

Intersection::~Intersection() {
    // Clean up traffic light if owned
    if (m_trafficLight) {
        delete m_trafficLight;
        m_trafficLight = nullptr;
    }
    
    Logger::getInstance().debug("Destroyed intersection: " + m_name);
}

void Intersection::addRoad(Road* road) {
    if (!road) {
        return;
    }
    
    // Check if road already connected
    auto it = std::find(m_connectedRoads.begin(), m_connectedRoads.end(), road);
    if (it != m_connectedRoads.end()) {
        return;
    }
    
    // Add road to connected roads
    m_connectedRoads.push_back(road);
    
    // Set intersection as start or end based on proximity
    double startDist = Vector2D::distance(m_position, road->getStartPoint());
    double endDist = Vector2D::distance(m_position, road->getEndPoint());
    
    if (startDist < endDist) {
        road->setStartIntersection(this);
    } else {
        road->setEndIntersection(this);
    }
    
    // Recalculate lane connections
    calculateLaneConnections();
    
    Logger::getInstance().debug("Added road " + road->getName() + 
                              " to intersection " + m_name);
}

void Intersection::removeRoad(Road* road) {
    if (!road) {
        return;
    }
    
    // Find and remove road
    auto it = std::find(m_connectedRoads.begin(), m_connectedRoads.end(), road);
    if (it == m_connectedRoads.end()) {
        return;
    }
    
    m_connectedRoads.erase(it);
    
    // Disconnect from road
    if (road->getStartIntersection() == this) {
        road->setStartIntersection(nullptr);
    }
    
    if (road->getEndIntersection() == this) {
        road->setEndIntersection(nullptr);
    }
    
    // Remove from connection map
    m_connections.erase(road->getId());
    
    for (auto& pair : m_connections) {
        pair.second.erase(road->getId());
    }
    
    // Recalculate lane connections
    calculateLaneConnections();
    
    Logger::getInstance().debug("Removed road " + road->getName() + 
                              " from intersection " + m_name);
}

void Intersection::setTrafficLight(TrafficLight* trafficLight) {
    // Clean up existing traffic light if owned
    if (m_trafficLight) {
        delete m_trafficLight;
    }
    
    m_trafficLight = trafficLight;
    
    // Initialize traffic light phases if needed
    if (m_trafficLight) {
        updateTrafficLightPhases();
    }
}

Lane* Intersection::getConnectionLane(Road* fromRoad, Lane* fromLane, Road* toRoad) const {
    if (!fromRoad || !fromLane || !toRoad) {
        return nullptr;
    }
    
    // Look up connection in map
    auto fromIt = m_connections.find(fromRoad->getId());
    if (fromIt == m_connections.end()) {
        return nullptr;
    }
    
    auto toIt = fromIt->second.find(toRoad->getId());
    if (toIt == fromIt->second.end()) {
        return nullptr;
    }
    
    // Find matching lane connection
    int fromLaneIndex = fromLane->getIndex();
    for (const auto& connection : toIt->second) {
        if (connection.first == fromLaneIndex) {
            // Found matching from lane, return corresponding to lane
            return toRoad->getLane(connection.second);
        }
    }
    
    return nullptr;
}

std::vector<Road*> Intersection::getPossibleExits(Road* entryRoad) const {
    std::vector<Road*> exits;
    
    if (!entryRoad) {
        return exits;
    }
    
    // Look up connections for this entry road
    auto fromIt = m_connections.find(entryRoad->getId());
    if (fromIt == m_connections.end()) {
        return exits;
    }
    
    // Add all connected exit roads
    for (const auto& pair : fromIt->second) {
        UUID toRoadId = pair.first;
        
        // Find road with this ID
        for (Road* road : m_connectedRoads) {
            if (road->getId() == toRoadId) {
                exits.push_back(road);
                break;
            }
        }
    }
    
    return exits;
}

bool Intersection::hasRightOfWay(Vehicle* vehicle, Road* fromRoad, Road* toRoad) const {
    if (!vehicle || !fromRoad || !toRoad) {
        return false;
    }
    
    // If we have a traffic light, use its state
    if (m_trafficLight) {
        LightState state = m_trafficLight->getState(fromRoad);
        
        // Check if light is green for this direction
        if (state == LightState::GREEN) {
            return true;
        } else if (state == LightState::RED) {
            // Emergency vehicles can pass on red
            if (vehicle->getType() == VehicleType::EMERGENCY) {
                EmergencyVehicle* emergency = dynamic_cast<EmergencyVehicle*>(vehicle);
                if (emergency && emergency->isSirenActive()) {
                    return true;
                }
            }
            return false;
        } else if (state == LightState::YELLOW) {
            // Allow passage if vehicle is close to intersection
            // (already committed to crossing)
            double distToIntersection = Vector2D::distance(
                vehicle->getPosition(), m_position);
            
            // Allow if within 10m or emergency vehicle with siren
            if (distToIntersection < 10.0) {
                return true;
            }
            
            if (vehicle->getType() == VehicleType::EMERGENCY) {
                EmergencyVehicle* emergency = dynamic_cast<EmergencyVehicle*>(vehicle);
                if (emergency && emergency->isSirenActive()) {
                    return true;
                }
            }
            
            return false;
        }
    }
    
    // No traffic light, use right-hand rule
    return checkRightHandPrecedence(fromRoad, toRoad);
}

void Intersection::update(double deltaTime) {
    // Update traffic light
    if (m_trafficLight) {
        m_trafficLight->update(deltaTime);
    }
    
    // Update vehicle tracking
    m_approachingVehicles.clear();
    
    // Find vehicles approaching intersection
    for (Road* road : m_connectedRoads) {
        // Determine if intersection is at start or end of road
        bool isAtStart = (road->getStartIntersection() == this);
        bool isAtEnd = (road->getEndIntersection() == this);
        
        if (!isAtStart && !isAtEnd) {
            continue;
        }
        
        // Check vehicles on this road
        const std::vector<Lane*>& lanes = road->getLanes();
        for (Lane* lane : lanes) {
            const std::vector<Vehicle*>& vehicles = lane->getVehicles();
            
            for (Vehicle* vehicle : vehicles) {
                // Calculate distance to intersection
                double distToIntersection = Vector2D::distance(
                    vehicle->getPosition(), m_position);
                
                // Consider vehicles within 50m of intersection
                if (distToIntersection < 50.0) {
                    m_approachingVehicles.push_back(vehicle);
                }
            }
        }
    }
    
    // In a more complex implementation, we would analyze approaching vehicles
    // and potentially adjust traffic light timings adaptively
}

void Intersection::updateTrafficLightPhases() {
    if (!m_trafficLight) {
        return;
    }
    
    // Simple traffic light phase generation based on connected roads
    // In a real implementation this would be more sophisticated,
    // handling turn lanes, multiple phases, etc.
    
    int roadCount = static_cast<int>(m_connectedRoads.size());
    
    // If only 2 roads, simple 2-phase signal
    if (roadCount <= 2) {
        m_trafficLight->setPhaseCount(2);
        
        // Equal time for each phase
        double phaseDuration = 30.0;  // 30 seconds per phase
        m_trafficLight->setPhaseDuration(0, phaseDuration);
        m_trafficLight->setPhaseDuration(1, phaseDuration);
        
        // Set initial phase
        m_trafficLight->setCurrentPhase(0);
        
        return;
    }
    
    // For more than 2 roads, create a phase for each road
    m_trafficLight->setPhaseCount(roadCount);
    
    // Duration proportional to number of lanes
    for (int i = 0; i < roadCount; i++) {
        Road* road = m_connectedRoads[i];
        int laneCount = road->getLaneCount();
        
        // Base duration of 20 seconds, plus 5 seconds per lane
        double phaseDuration = 20.0 + laneCount * 5.0;
        m_trafficLight->setPhaseDuration(i, phaseDuration);
    }
    
    // Set initial phase
    m_trafficLight->setCurrentPhase(0);
}

void Intersection::calculateLaneConnections() {
    // Clear existing connections
    m_connections.clear();
    
    // Simple lane connections based on angles between roads
    for (Road* fromRoad : m_connectedRoads) {
        // Get road direction vector (normalized)
        Vector2D fromDir;
        
        if (fromRoad->getStartIntersection() == this) {
            // Road starts at this intersection, direction is away from intersection
            fromDir = (fromRoad->getEndPoint() - m_position).normalize();
        } else if (fromRoad->getEndIntersection() == this) {
            // Road ends at this intersection, direction is towards intersection
            fromDir = (m_position - fromRoad->getStartPoint()).normalize();
        } else {
            // Road not properly connected
            continue;
        }
        
        for (Road* toRoad : m_connectedRoads) {
            // Skip self connections
            if (fromRoad == toRoad) {
                continue;
            }
            
            // Get direction of to-road
            Vector2D toDir;
            
            if (toRoad->getStartIntersection() == this) {
                // Road starts at this intersection, direction is away from intersection
                toDir = (toRoad->getEndPoint() - m_position).normalize();
            } else if (toRoad->getEndIntersection() == this) {
                // Road ends at this intersection, direction is towards intersection
                toDir = (m_position - toRoad->getStartPoint()).normalize();
            } else {
                // Road not properly connected
                continue;
            }
            
            // Calculate angle between roads
            double angle = Vector2D::angle(fromDir, toDir);
            
            // Normalize angle to [0, 2π)
            while (angle < 0) angle += 2 * M_PI;
            while (angle >= 2 * M_PI) angle -= 2 * M_PI;
            
            // Skip U-turns (angle close to π)
            if (std::abs(angle - M_PI) < 0.1) {
                continue;
            }
            
            // Create lane connections between these roads
            int fromLaneCount = fromRoad->getLaneCount();
            int toLaneCount = toRoad->getLaneCount();
            
            // Simple lane mapping: just connect one-to-one where possible
            int maxLanes = std::min(fromLaneCount, toLaneCount);
            
            for (int i = 0; i < maxLanes; i++) {
                // Map from lane i to lane i
                m_connections[fromRoad->getId()][toRoad->getId()].push_back(
                    std::make_pair(i, i));
            }
        }
    }
}

bool Intersection::checkRightHandPrecedence(Road* fromRoad, Road* toRoad) const {
    if (!fromRoad || !toRoad) {
        return false;
    }
    
    // Get directions of all roads connected to this intersection
    std::vector<std::pair<Road*, Vector2D>> roadDirections;
    
    for (Road* road : m_connectedRoads) {
        Vector2D direction;
        
        if (road->getStartIntersection() == this) {
            direction = (road->getEndPoint() - m_position).normalize();
        } else if (road->getEndIntersection() == this) {
            direction = (m_position - road->getStartPoint()).normalize();
        } else {
            continue;
        }
        
        roadDirections.push_back(std::make_pair(road, direction));
    }
    
    // Find from road direction
    Vector2D fromDir;
    for (const auto& pair : roadDirections) {
        if (pair.first == fromRoad) {
            fromDir = pair.second;
            break;
        }
    }
    
    // For each other road, check if it's to the right of fromRoad
    for (const auto& pair : roadDirections) {
        Road* road = pair.first;
        Vector2D roadDir = pair.second;
        
        // Skip fromRoad and toRoad
        if (road == fromRoad || road == toRoad) {
            continue;
        }
        
        // Check if this road is to the right of fromRoad
        // This is a cross product check: if fromDir × roadDir < 0, road is to the right
        double crossProduct = fromDir.x * roadDir.y - fromDir.y * roadDir.x;
        
        if (crossProduct < 0) {
            // This road is to the right of fromRoad
            // Check if there's a vehicle on this road approaching the intersection
            double maxDistToCheck = 50.0;  // meters
            
            for (Vehicle* vehicle : m_approachingVehicles) {
                Road* vehicleRoad = vehicle->getCurrentRoad();
                if (vehicleRoad == road) {
                    double distToIntersection = Vector2D::distance(
                        vehicle->getPosition(), m_position);
                    
                    if (distToIntersection < maxDistToCheck) {
                        // There's a vehicle on the right road approaching the intersection
                        // It has precedence
                        return false;
                    }
                }
            }
        }
    }
    
    // No conflicting vehicles with higher precedence found
    return true;
}

} // namespace TrafficSim