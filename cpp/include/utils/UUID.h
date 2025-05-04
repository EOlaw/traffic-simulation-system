#ifndef TRAFFIC_SIMULATION_UUID_H
#define TRAFFIC_SIMULATION_UUID_H

#include <string>
#include <random>
#include <iostream>
#include <sstream>
#include <iomanip>

namespace TrafficSim {

/**
 * @brief Simple UUID implementation for unique identifiers
 */
class UUID {
public:
    /**
     * @brief Create a new random UUID
     */
    UUID() {
        generateRandom();
    }
    
    /**
     * @brief Create a UUID from string representation
     * 
     * @param str UUID string
     */
    explicit UUID(const std::string& str) {
        // Parse UUID from string
        fromString(str);
    }
    
    /**
     * @brief Copy constructor
     */
    UUID(const UUID& other) = default;
    
    /**
     * @brief Assignment operator
     */
    UUID& operator=(const UUID& other) = default;
    
    /**
     * @brief Equality comparison
     */
    bool operator==(const UUID& other) const {
        return m_data1 == other.m_data1 &&
               m_data2 == other.m_data2 &&
               m_data3 == other.m_data3 &&
               m_data4 == other.m_data4;
    }
    
    /**
     * @brief Inequality comparison
     */
    bool operator!=(const UUID& other) const {
        return !(*this == other);
    }
    
    /**
     * @brief Generate a new random UUID
     * 
     * @return UUID New UUID
     */
    static UUID generate() {
        UUID uuid;
        uuid.generateRandom();
        return uuid;
    }
    
    /**
     * @brief Convert to string representation
     * 
     * @return std::string UUID string (format: xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx)
     */
    std::string toString() const {
        std::stringstream ss;
        ss << std::hex << std::setfill('0');
        
        // Format: 8-4-4-4-12
        ss << std::setw(8) << m_data1 << '-';
        ss << std::setw(4) << ((m_data2 >> 16) & 0xFFFF) << '-';
        ss << std::setw(4) << (m_data2 & 0xFFFF) << '-';
        ss << std::setw(4) << ((m_data3 >> 16) & 0xFFFF) << '-';
        ss << std::setw(4) << (m_data3 & 0xFFFF);
        ss << std::setw(8) << m_data4;
        
        return ss.str();
    }
    
    /**
     * @brief Parse UUID from string
     * 
     * @param str UUID string
     * @return bool True if parsing was successful
     */
    bool fromString(const std::string& str) {
        // Clear any existing data
        m_data1 = m_data2 = m_data3 = m_data4 = 0;
        
        // Check string format
        if (str.length() != 36 || str[8] != '-' || str[13] != '-' || 
            str[18] != '-' || str[23] != '-') {
            return false;
        }
        
        // Parse using streams
        try {
            std::stringstream ss;
            ss << std::hex;
            
            // Parse data1 (8 chars)
            ss << str.substr(0, 8);
            ss >> m_data1;
            ss.clear();
            
            // Parse data2 (4-4 chars)
            uint32_t high, low;
            ss << str.substr(9, 4);
            ss >> high;
            ss.clear();
            
            ss << str.substr(14, 4);
            ss >> low;
            ss.clear();
            
            m_data2 = (high << 16) | low;
            
            // Parse data3 (4-4 chars)
            ss << str.substr(19, 4);
            ss >> high;
            ss.clear();
            
            ss << str.substr(24, 4);
            ss >> low;
            ss.clear();
            
            m_data3 = (high << 16) | low;
            
            // Parse data4 (12 chars)
            ss << str.substr(28, 8);
            ss >> m_data4;
            
            return true;
        } catch (...) {
            // Reset on any error
            m_data1 = m_data2 = m_data3 = m_data4 = 0;
            return false;
        }
    }
    
private:
    uint32_t m_data1 = 0;
    uint32_t m_data2 = 0;
    uint32_t m_data3 = 0;
    uint32_t m_data4 = 0;
    
    /**
     * @brief Generate a random UUID
     */
    void generateRandom() {
        // Initialize random generator
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::uniform_int_distribution<uint32_t> dis(0, 0xFFFFFFFF);
        
        // Generate random data
        m_data1 = dis(gen);
        m_data2 = dis(gen);
        m_data3 = dis(gen);
        m_data4 = dis(gen);
        
        // Set version (v4 - random)
        m_data2 = (m_data2 & 0xFFFF0FFF) | 0x4000;  // Version 4
        m_data3 = (m_data3 & 0x3FFFFFFF) | 0x80000000;  // Variant
    }
};

/**
 * @brief Hash function for UUID
 */
struct UUIDHasher {
    std::size_t operator()(const UUID& uuid) const {
        return std::hash<std::string>()(uuid.toString());
    }
};

} // namespace TrafficSim

#endif // TRAFFIC_SIMULATION_UUID_H