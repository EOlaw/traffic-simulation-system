#ifndef TRAFFIC_SIMULATION_TRAFFIC_LIGHT_H
#define TRAFFIC_SIMULATION_TRAFFIC_LIGHT_H

#include <vector>
#include <string>
#include <unordered_map>
#include "../utils/UUID.h"

namespace TrafficSim {

// Forward declarations
class Road;
class Intersection;

/**
 * @brief Traffic light states
 */
enum class LightState {
    RED,
    YELLOW,
    GREEN
};

/**
 * @brief Traffic light controller for intersections
 */
class TrafficLight {
public:
    /**
     * @brief Create a new traffic light
     * 
     * @param intersection Associated intersection
     */
    explicit TrafficLight(Intersection* intersection);
    
    /**
     * @brief Destructor
     */
    virtual ~TrafficLight();
    
    // Disable copying
    TrafficLight(const TrafficLight&) = delete;
    TrafficLight& operator=(const TrafficLight&) = delete;
    
    /**
     * @brief Get the current state for a road
     * 
     * @param road Road to check (optional)
     * @return LightState Current state for the road
     */
    LightState getState(Road* road = nullptr) const;
    
    /**
     * @brief Get the current phase
     * 
     * @return int Current phase index
     */
    int getCurrentPhase() const { return m_currentPhase; }
    
    /**
     * @brief Get the number of phases
     * 
     * @return int Number of phases
     */
    int getPhaseCount() const { return static_cast<int>(m_phaseDurations.size()); }
    
    /**
     * @brief Get the duration of a phase
     * 
     * @param phase Phase index
     * @return double Phase duration in seconds
     */
    double getPhaseDuration(int phase) const;
    
    /**
     * @brief Set the number of phases
     * 
     * @param count Number of phases
     */
    void setPhaseCount(int count);
    
    /**
     * @brief Set the duration of a phase
     * 
     * @param phase Phase index
     * @param duration Duration in seconds
     */
    void setPhaseDuration(int phase, double duration);
    
    /**
     * @brief Set the current phase
     * 
     * @param phase Phase index
     */
    void setCurrentPhase(int phase);
    
    /**
     * @brief Update the traffic light state
     * 
     * @param deltaTime Time step in seconds
     */
    void update(double deltaTime);
    
private:
    Intersection* m_intersection;
    int m_currentPhase;
    double m_phaseTimer;
    double m_yellowDuration;  // Duration of yellow light
    
    // Configuration
    std::vector<double> m_phaseDurations;  // Duration of each phase
    
    // Road state mapping (phase -> road ID -> state)
    std::unordered_map<int, std::unordered_map<UUID, LightState, UUIDHasher>> m_phaseStates;
    
    // Update phase states
    void updatePhaseStates();
};

/**
 * @brief Advanced traffic light with adaptive timing
 */
class AdaptiveTrafficLight : public TrafficLight {
public:
    /**
     * @brief Create a new adaptive traffic light
     * 
     * @param intersection Associated intersection
     */
    explicit AdaptiveTrafficLight(Intersection* intersection);
    
    /**
     * @brief Update the traffic light state
     * 
     * @param deltaTime Time step in seconds
     */
    void update(double deltaTime) override;
    
    /**
     * @brief Enable or disable adaptive mode
     * 
     * @param enabled Whether adaptive mode is enabled
     */
    void setAdaptiveMode(bool enabled) { m_adaptiveMode = enabled; }
    
    /**
     * @brief Check if adaptive mode is enabled
     * 
     * @return bool Whether adaptive mode is enabled
     */
    bool isAdaptiveMode() const { return m_adaptiveMode; }
    
private:
    bool m_adaptiveMode;
    
    // Analyze traffic and adjust phases
    void adaptPhaseDurations();
};

} // namespace TrafficSim

#endif // TRAFFIC_SIMULATION_TRAFFIC_LIGHT_H