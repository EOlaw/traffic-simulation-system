#ifndef TRAFFIC_SIMULATION_LOGGER_H
#define TRAFFIC_SIMULATION_LOGGER_H

#include <string>
#include <fstream>
#include <mutex>
#include <vector>
#include <memory>
#include <iostream>

namespace TrafficSim {

/**
 * @brief Log severity levels
 */
enum class LogLevel {
    DEBUG,
    INFO,
    WARNING,
    ERROR,
    CRITICAL
};

/**
 * @brief Interface for log sinks
 */
class LogSink {
public:
    /**
     * @brief Destructor
     */
    virtual ~LogSink() = default;
    
    /**
     * @brief Write a log message
     * 
     * @param level Log level
     * @param timestamp Timestamp string
     * @param message Log message
     */
    virtual void write(LogLevel level, const std::string& timestamp, const std::string& message) = 0;
};

/**
 * @brief Console log sink
 */
class ConsoleSink : public LogSink {
public:
    void write(LogLevel level, const std::string& timestamp, const std::string& message) override {
        // Format: [TIMESTAMP] [LEVEL] MESSAGE
        std::string formattedMessage = "[" + timestamp + "] [" + logLevelToString(level) + "] " + message;
        
        // Write to appropriate stream
        if (level == LogLevel::ERROR || level == LogLevel::CRITICAL) {
            std::cerr << formattedMessage << std::endl;
        } else {
            std::cout << formattedMessage << std::endl;
        }
    }
    
private:
    std::string logLevelToString(LogLevel level) const {
        switch (level) {
            case LogLevel::DEBUG: return "DEBUG";
            case LogLevel::INFO: return "INFO";
            case LogLevel::WARNING: return "WARNING";
            case LogLevel::ERROR: return "ERROR";
            case LogLevel::CRITICAL: return "CRITICAL";
            default: return "UNKNOWN";
        }
    }
};

/**
 * @brief File log sink
 */
class FileSink : public LogSink {
public:
    /**
     * @brief Create a file sink
     * 
     * @param filename Log file path
     */
    explicit FileSink(const std::string& filename)
        : m_filename(filename)
    {
        m_file.open(filename, std::ios::out | std::ios::app);
        if (!m_file.is_open()) {
            std::cerr << "Failed to open log file: " << filename << std::endl;
        }
    }
    
    /**
     * @brief Destructor
     */
    ~FileSink() {
        if (m_file.is_open()) {
            m_file.close();
        }
    }
    
    void write(LogLevel level, const std::string& timestamp, const std::string& message) override {
        if (!m_file.is_open()) {
            return;
        }
        
        // Format: [TIMESTAMP] [LEVEL] MESSAGE
        m_file << "[" << timestamp << "] [" << logLevelToString(level) << "] " << message << std::endl;
        m_file.flush();
    }
    
private:
    std::string m_filename;
    std::ofstream m_file;
    
    std::string logLevelToString(LogLevel level) const {
        switch (level) {
            case LogLevel::DEBUG: return "DEBUG";
            case LogLevel::INFO: return "INFO";
            case LogLevel::WARNING: return "WARNING";
            case LogLevel::ERROR: return "ERROR";
            case LogLevel::CRITICAL: return "CRITICAL";
            default: return "UNKNOWN";
        }
    }
};

/**
 * @brief Main logger class
 * 
 * Singleton logger that writes to multiple sinks
 */
class Logger {
public:
    /**
     * @brief Get the singleton instance
     * 
     * @return Logger& Reference to the singleton
     */
    static Logger& getInstance() {
        static Logger instance;
        return instance;
    }
    
    /**
     * @brief Add a log sink
     * 
     * @param sink Shared pointer to a log sink
     */
    void addSink(std::shared_ptr<LogSink> sink) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_sinks.push_back(sink);
    }
    
    /**
     * @brief Set minimum log level
     * 
     * @param level Minimum level to log
     */
    void setLevel(LogLevel level) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_level = level;
    }
    
    /**
     * @brief Set logging to file
     * 
     * @param filename Log file path
     */
    void setLogFile(const std::string& filename) {
        std::lock_guard<std::mutex> lock(m_mutex);
        
        // Remove any existing file sinks
        auto it = m_sinks.begin();
        while (it != m_sinks.end()) {
            if (dynamic_cast<FileSink*>(it->get())) {
                it = m_sinks.erase(it);
            } else {
                ++it;
            }
        }
        
        // Add new file sink
        m_sinks.push_back(std::make_shared<FileSink>(filename));
    }
    
    /**
     * @brief Log a debug message
     * 
     * @param message Message to log
     */
    void debug(const std::string& message) {
        log(LogLevel::DEBUG, message);
    }
    
    /**
     * @brief Log an info message
     * 
     * @param message Message to log
     */
    void info(const std::string& message) {
        log(LogLevel::INFO, message);
    }
    
    /**
     * @brief Log a warning message
     * 
     * @param message Message to log
     */
    void warning(const std::string& message) {
        log(LogLevel::WARNING, message);
    }
    
    /**
     * @brief Log an error message
     * 
     * @param message Message to log
     */
    void error(const std::string& message) {
        log(LogLevel::ERROR, message);
    }
    
    /**
     * @brief Log a critical message
     * 
     * @param message Message to log
     */
    void critical(const std::string& message) {
        log(LogLevel::CRITICAL, message);
    }
    
    /**
     * @brief Log a message with specified level
     * 
     * @param level Log level
     * @param message Message to log
     */
    void log(LogLevel level, const std::string& message) {
        std::lock_guard<std::mutex> lock(m_mutex);
        
        // Check if level is enabled
        if (level < m_level) {
            return;
        }
        
        // Get current timestamp
        std::string timestamp = getCurrentTimestamp();
        
        // Write to all sinks
        for (auto& sink : m_sinks) {
            sink->write(level, timestamp, message);
        }
    }
    
private:
    // Private constructor for singleton
    Logger() 
        : m_level(LogLevel::INFO)
    {
        // Add default console sink
        m_sinks.push_back(std::make_shared<ConsoleSink>());
    }
    
    // Prevent copying
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;
    
    LogLevel m_level;
    std::vector<std::shared_ptr<LogSink>> m_sinks;
    std::mutex m_mutex;
    
    /**
     * @brief Get current timestamp as string
     * 
     * @return std::string Formatted timestamp
     */
    std::string getCurrentTimestamp() const {
        // Get current time
        auto now = std::chrono::system_clock::now();
        auto time_t_now = std::chrono::system_clock::to_time_t(now);
        
        // Format time
        char buffer[128];
        std::tm tm_now;
        
        #ifdef _WIN32
        localtime_s(&tm_now, &time_t_now);
        #else
        localtime_r(&time_t_now, &tm_now);
        #endif
        
        // Format: YYYY-MM-DD HH:MM:SS
        strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &tm_now);
        
        // Add milliseconds
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;
        
        std::string result(buffer);
        result += "." + std::to_string(ms.count());
        
        return result;
    }
};

} // namespace TrafficSim

#endif // TRAFFIC_SIMULATION_LOGGER_H