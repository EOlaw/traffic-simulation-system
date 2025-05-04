#ifndef TRAFFIC_SIMULATION_CONFIG_PARSER_H
#define TRAFFIC_SIMULATION_CONFIG_PARSER_H

#include <string>
#include <unordered_map>
#include <vector>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <iostream>
#include <optional>
#include <nlohmann/json.hpp>

namespace TrafficSim {

/**
 * @brief Configuration parser
 * 
 * Handles loading and parsing configuration from JSON files
 */
class ConfigParser {
public:
    /**
     * @brief Default constructor
     */
    ConfigParser() = default;
    
    /**
     * @brief Load configuration from file
     * 
     * @param filename Path to configuration file
     * @return bool True if loading was successful
     */
    bool load(const std::string& filename) {
        try {
            // Open file
            std::ifstream file(filename);
            if (!file.is_open()) {
                std::cerr << "Failed to open configuration file: " << filename << std::endl;
                return false;
            }
            
            // Parse JSON
            file >> m_data;
            return true;
            
        } catch (const std::exception& e) {
            std::cerr << "Error parsing configuration file: " << e.what() << std::endl;
            return false;
        }
    }
    
    /**
     * @brief Save configuration to file
     * 
     * @param filename Output path
     * @param pretty Whether to format JSON with indentation
     * @return bool True if saving was successful
     */
    bool save(const std::string& filename, bool pretty = true) const {
        try {
            // Open file
            std::ofstream file(filename);
            if (!file.is_open()) {
                std::cerr << "Failed to open file for writing: " << filename << std::endl;
                return false;
            }
            
            // Write JSON
            if (pretty) {
                file << m_data.dump(4);
            } else {
                file << m_data.dump();
            }
            
            return true;
            
        } catch (const std::exception& e) {
            std::cerr << "Error saving configuration file: " << e.what() << std::endl;
            return false;
        }
    }
    
    /**
     * @brief Check if a key exists
     * 
     * @param key Configuration key
     * @return bool True if key exists
     */
    bool hasKey(const std::string& key) const {
        try {
            return getNestedValue(key).has_value();
        } catch (...) {
            return false;
        }
    }
    
    /**
     * @brief Set a configuration value
     * 
     * @param key Configuration key (can be nested with dots)
     * @param value Value to set
     */
    template<typename T>
    void setValue(const std::string& key, const T& value) {
        // Split key into path components
        std::vector<std::string> path = splitPath(key);
        
        // Set value in JSON
        nlohmann::json* current = &m_data;
        for (size_t i = 0; i < path.size() - 1; i++) {
            // Create nested objects if needed
            if (!current->contains(path[i]) || !(*current)[path[i]].is_object()) {
                (*current)[path[i]] = nlohmann::json::object();
            }
            current = &(*current)[path[i]];
        }
        
        // Set final value
        (*current)[path.back()] = value;
    }
    
    /**
     * @brief Get a configuration value
     * 
     * @param key Configuration key (can be nested with dots)
     * @param defaultValue Default value if key not found
     * @return T Value
     */
    template<typename T>
    T getValue(const std::string& key, const T& defaultValue) const {
        try {
            auto value = getNestedValue(key);
            if (value.has_value()) {
                return value->get<T>();
            }
        } catch (...) {
            // Ignore errors
        }
        
        return defaultValue;
    }
    
    /**
     * @brief Get string value
     * 
     * @param key Configuration key
     * @param defaultValue Default value if key not found
     * @return std::string String value
     */
    std::string getString(const std::string& key, const std::string& defaultValue = "") const {
        return getValue<std::string>(key, defaultValue);
    }
    
    /**
     * @brief Get integer value
     * 
     * @param key Configuration key
     * @param defaultValue Default value if key not found
     * @return int Integer value
     */
    int getInt(const std::string& key, int defaultValue = 0) const {
        return getValue<int>(key, defaultValue);
    }
    
    /**
     * @brief Get double value
     * 
     * @param key Configuration key
     * @param defaultValue Default value if key not found
     * @return double Double value
     */
    double getDouble(const std::string& key, double defaultValue = 0.0) const {
        return getValue<double>(key, defaultValue);
    }
    
    /**
     * @brief Get boolean value
     * 
     * @param key Configuration key
     * @param defaultValue Default value if key not found
     * @return bool Boolean value
     */
    bool getBool(const std::string& key, bool defaultValue = false) const {
        return getValue<bool>(key, defaultValue);
    }
    
    /**
     * @brief Get array value
     * 
     * @param key Configuration key
     * @return std::vector<T> Array value
     */
    template<typename T>
    std::vector<T> getArray(const std::string& key) const {
        try {
            auto value = getNestedValue(key);
            if (value.has_value() && value->is_array()) {
                return value->get<std::vector<T>>();
            }
        } catch (...) {
            // Ignore errors
        }
        
        return std::vector<T>();
    }
    
    /**
     * @brief Get JSON object for a key
     * 
     * @param key Configuration key
     * @return nlohmann::json JSON object
     */
    nlohmann::json getObject(const std::string& key) const {
        try {
            auto value = getNestedValue(key);
            if (value.has_value()) {
                return *value;
            }
        } catch (...) {
            // Ignore errors
        }
        
        return nlohmann::json::object();
    }
    
    /**
     * @brief Get the entire configuration as JSON
     * 
     * @return const nlohmann::json& JSON configuration
     */
    const nlohmann::json& getJson() const {
        return m_data;
    }
    
private:
    nlohmann::json m_data;
    
    /**
     * @brief Split a key path into components
     * 
     * @param key Path with dot separators
     * @return std::vector<std::string> Path components
     */
    std::vector<std::string> splitPath(const std::string& key) const {
        std::vector<std::string> result;
        std::stringstream ss(key);
        std::string item;
        
        while (std::getline(ss, item, '.')) {
            result.push_back(item);
        }
        
        return result;
    }
    
    /**
     * @brief Get a nested JSON value
     * 
     * @param key Configuration key (can be nested with dots)
     * @return std::optional<nlohmann::json> JSON value if found
     */
    std::optional<nlohmann::json> getNestedValue(const std::string& key) const {
        // Split key into path components
        std::vector<std::string> path = splitPath(key);
        
        // Navigate through JSON
        const nlohmann::json* current = &m_data;
        for (const auto& part : path) {
            if (!current->is_object() || !current->contains(part)) {
                return std::nullopt;
            }
            current = &(*current)[part];
        }
        
        return *current;
    }
};

} // namespace TrafficSim

#endif // TRAFFIC_SIMULATION_CONFIG_PARSER_H