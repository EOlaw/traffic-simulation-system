# Traffic Simulation System

A comprehensive traffic simulation platform that combines C++ performance with Python flexibility to model complex vehicle traffic scenarios, intersections, and traffic control systems.

## Overview

This hybrid C++/Python system simulates vehicle movement, traffic flow, and road networks with real-time visualization and analysis capabilities. The architecture leverages C++ for performance-critical components while using Python for high-level control, visualization, and data analysis.

## Key Features

- **High-performance simulation core**: Written in C++ for efficient computation
- **Multiple vehicle types**: Cars, buses, trucks, motorcycles, and emergency vehicles
- **Advanced traffic modeling**: Lane-based movement, vehicle interactions, and collision detection
- **Intelligent traffic control**: Traffic lights and intersections with customizable behavior
- **Real-time visualization**: Python-based visualization with interactive controls
- **Data analysis tools**: Traffic flow analysis, congestion detection, and statistics
- **Extensible architecture**: Easily add new vehicle types, road elements, or behavior models

## Architecture

The system follows a hybrid architecture:

- **C++ Core**: Handles performance-critical simulation components
  - Vehicle dynamics and movement
  - Road network representation
  - Collision detection
  - Core simulation logic
  
- **Python Layer**: Provides visualization and high-level control
  - Simulation configuration and scenario setup
  - Real-time visualization and UI
  - Data analysis and reporting
  - Extension and customization interfaces

## Requirements

### C++ Components
- C++17 compatible compiler
- CMake 3.15+
- pybind11
- Eigen (for math operations)

### Python Components
- Python 3.8+
- NumPy
- Matplotlib
- PyQt5 (for visualization)
- pandas (for data analysis)

## Project Structure

