#include "../../include/models/TrafficLight.h"
#include "../../include/models/Intersection.h"
#include "../../include/models/Road.h"
#include "../../include/utils/Logger.h"
#include <algorithm>
#include <cmath>
#include <sstream>

namespace TrafficSim {

TrafficLight::TrafficLight(Intersection* intersection)
    : m_intersection(intersection)
    , m_currentPhase(0)
    , m_phaseTimer(0.0)
    , m_yellowDuration(3.0)  // Default 3 seconds yellow
{
    // Default to 2 phases of 30 seconds each
    m_phaseDurations.resize(2, 30.0);
    
    // Initialize phase states
    updatePhaseStates();
    
    Logger::getInstance().debug("Created TrafficLight at intersection " + 
                               (intersection ? intersection->getName() : "unknown"));
}

TrafficLight::~TrafficLight() {
    Logger::getInstance().debug("Destroyed TrafficLight");
}

LightState TrafficLight::getState(Road* road) const {
    if (!road || m_phaseStates.empty()) {
        return LightState::RED;  // Default to red if no road or phases
    }
    
    // Get current phase states
    auto phaseIt = m_phaseStates.find(m_currentPhase);
    if (phaseIt == m_phaseStates.end()) {
        return LightState::RED;  // Phase not found
    }
    
    // Find road in current phase
    const auto& roadStates = phaseIt->second;
    auto roadIt = roadStates.find(road->getId());
    
    if (roadIt == roadStates.end()) {
        return LightState::RED;  // Road not found in phase
    }
    
    // Check if we're in yellow transition
    if (m_phaseTimer <= m_yellowDuration) {
        // In yellow transition to this phase
        if (roadIt->second == LightState::GREEN) {
            // Road will be green in this phase, but we're still in transition
            return LightState::YELLOW;
        } else {
            // Road is red in this phase
            return LightState::RED;
        }
    }
    
    // Return road's state for current phase
    return roadIt->second;
}

double TrafficLight::getPhaseDuration(int phase) const {
    if (phase < 0 || phase >= static_cast<int>(m_phaseDurations.size())) {
        return 0.0;
    }
    
    return m_phaseDurations[phase];
}

void TrafficLight::setPhaseCount(int count) {
    if (count <= 0) {
        Logger::getInstance().warning("Invalid phase count: " + std::to_string(count));
        return;
    }
    
    m_phaseDurations.resize(count, 30.0);  // Default 30 seconds
    
    // Reset current phase if needed
    if (m_currentPhase >= count) {
        m_currentPhase = 0;
        m_phaseTimer = 0.0;
    }
    
    // Update phase states
    updatePhaseStates();
}

void TrafficLight::setPhaseDuration(int phase, double duration) {
    if (phase < 0 || phase >= static_cast<int>(m_phaseDurations.size())) {
        Logger::getInstance().warning("Invalid phase index: " + std::to_string(phase));
        return;
    }
    
    if (duration <= m_yellowDuration) {
        Logger::getInstance().warning("Phase duration must be greater than yellow duration");
        duration = m_yellowDuration + 1.0;
    }
    
    m_phaseDurations[phase] = duration;
}

void TrafficLight::setCurrentPhase(int phase) {
    if (phase < 0 || phase >= static_cast<int>(m_phaseDurations.size())) {
        Logger::getInstance().warning("Invalid phase index: " + std::to_string(phase));
        return;
    }
    
    m_currentPhase = phase;
    m_phaseTimer = 0.0;  // Reset timer to start with yellow transition
}

void TrafficLight::update(double deltaTime) {
    // No phases or durations
    if (m_phaseDurations.empty()) {
        return;
    }
    
    // Update timer
    m_phaseTimer += deltaTime;
    
    // Check if current phase is done
    double currentDuration = m_phaseDurations[m_currentPhase];
    if (m_phaseTimer >= currentDuration) {
        // Move to next phase
        m_phaseTimer = 0.0;
        m_currentPhase = (m_currentPhase + 1) % m_phaseDurations.size();
        
        Logger::getInstance().debug("TrafficLight changed to phase " + 
                                 std::to_string(m_currentPhase));
    }
}

void TrafficLight::updatePhaseStates() {
    if (!m_intersection) {
        return;
    }
    
    // Clear current phase states
    m_phaseStates.clear();
    
    // Get connected roads
    const auto& roads = m_intersection->getConnectedRoads();
    if (roads.empty()) {
        return;
    }
    
    // For each phase, assign states to roads
    for (int phase = 0; phase < static_cast<int>(m_phaseDurations.size()); phase++) {
        // Simple alternating pattern: for each phase, some roads are green, others red
        int roadsPerPhase = std::max(1, static_cast<int>(roads.size() / m_phaseDurations.size()));
        
        for (size_t i = 0; i < roads.size(); i++) {
            Road* road = roads[i];
            
            // Determine if this road is green in this phase
            bool isGreen = (i / roadsPerPhase) == phase;
            
            // Set state
            m_phaseStates[phase][road->getId()] = isGreen ? LightState::GREEN : LightState::RED;
        }
    }
    
    // In a real implementation, we would use a more sophisticated algorithm
    // to determine which roads get green lights in which phases, considering
    // traffic flow, road connections, turning conflicts, etc.
}

// AdaptiveTrafficLight implementation
AdaptiveTrafficLight::AdaptiveTrafficLight(Intersection* intersection)
    : TrafficLight(intersection)
    , m_adaptiveMode(true)
{
    Logger::getInstance().debug("Created AdaptiveTrafficLight");
}

void AdaptiveTrafficLight::update(double deltaTime) {
    // Regular update
    TrafficLight::update(deltaTime);
    
    // If adaptive mode is enabled, periodically adjust phase durations
    if (m_adaptiveMode) {
        // In a real implementation, we would analyze traffic more frequently
        // For this example, we'll adapt once per full cycle
        if (getCurrentPhase() == 0 && m_phaseTimer < deltaTime) {
            adaptPhaseDurations();
        }
    }
}

void AdaptiveTrafficLight::adaptPhaseDurations() {
    if (!m_intersection) {
        return;
    }
    
    // Get connected roads
    const auto& roads = m_intersection->getConnectedRoads();
    if (roads.empty()) {
        return;
    }
    
    // Simple adaptive algorithm: adjust phase durations based on traffic density
    for (int phase = 0; phase < getPhaseCount(); phase++) {
        double totalDensity = 0.0;
        int roadCount = 0;
        
        // Calculate average density for roads that are green in this phase
        for (Road* road : roads) {
            auto phaseIt = m_phaseStates.find(phase);
            if (phaseIt == m_phaseStates.end()) {
                continue;
            }
            
            auto roadIt = phaseIt->second.find(road->getId());
            if (roadIt == phaseIt->second.end()) {
                continue;
            }
            
            if (roadIt->second == LightState::GREEN) {
                totalDensity += road->getAverageDensity();
                roadCount++;
            }
        }
        
        if (roadCount > 0) {
            double avgDensity = totalDensity / roadCount;
            
            // Set phase duration based on density
            // Higher density = longer green time, within limits
            double baseDuration = 20.0;  // Minimum 20 seconds
            double maxDuration = 60.0;   // Maximum 60 seconds
            double densityFactor = std::min(3.0, avgDensity / 10.0);  // Cap at 3x
            
            double newDuration = baseDuration * (1.0 + densityFactor);
            newDuration = std::min(maxDuration, newDuration);
            
            setPhaseDuration(phase, newDuration);
            
            std::stringstream ss;
            ss << "AdaptiveTrafficLight adjusted phase " << phase 
               << " to " << newDuration << "s (density: " << avgDensity << ")";
            Logger::getInstance().debug(ss.str());
        }
    }
}

} // namespace TrafficSim