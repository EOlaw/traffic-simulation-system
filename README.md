# Traffic Simulation System

A comprehensive traffic simulation platform that combines C++ performance with Python flexibility. This system models complex vehicle traffic scenarios, intersections, and traffic control mechanisms with real-time visualization and analysis capabilities.

[![Build Status](https://img.shields.io/badge/build-passing-brightgreen.svg)](https://github.com/yourusername/traffic-simulation-system)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](https://opensource.org/licenses/MIT)

## Overview

This hybrid C++/Python system provides a high-performance platform for simulating urban traffic scenarios. The architecture leverages C++ for computationally intensive simulation while providing intuitive Python interfaces for visualization, analysis, and control.

![Simulation Screenshot](docs/images/simulation_screenshot.png)

## Key Features

- **High-performance simulation core**: C++ engine for efficient computation
- **Multiple vehicle types**: Cars, buses, trucks, motorcycles, and emergency vehicles
- **Advanced traffic modeling**: Lane-based movement, vehicle interactions, collision detection
- **Intelligent traffic control**: Customizable traffic signals and intersection behavior
- **Real-time visualization**: Python-based visualization with interactive controls
- **Data analysis tools**: Statistical analysis of traffic flow, congestion detection
- **Extensible architecture**: Easy to extend with new vehicle types or behaviors

## Getting Started

### Prerequisites

- C++17 compatible compiler (GCC 8+, Clang 7+, MSVC 2019+)
- Python 3.8 or higher
- CMake 3.15 or higher
- pybind11 (for C++/Python binding)
- NumPy, Matplotlib, PyQt5 (for Python visualization)

### Installation

1. Clone the repository
   ```bash
   git clone https://github.com/yourusername/traffic-simulation-system.git
   cd traffic-simulation-system
   ```

2. Build the C++ components
   ```bash
   mkdir build && cd build
   cmake ..
   make -j4
   ```

3. Install Python dependencies
   ```bash
   pip install -r requirements.txt
   ```

4. Run the examples
   ```bash
   python examples/basic_simulation.py
   ```

## Quick Example

```python
from traffic_simulation import SimulationController, Visualization

# Initialize the simulation
sim = SimulationController()
sim.load_network("data/maps/grid_city.json")
sim.generate_traffic(vehicle_count=100)

# Create visualization
vis = Visualization(sim)

# Run simulation for 10 minutes
sim.run(minutes=10, real_time_factor=2.0)

# Analyze results
stats = sim.get_statistics()
print(f"Average speed: {stats.avg_speed} km/h")
print(f"Traffic flow: {stats.flow_rate} vehicles/hour")
```

## Documentation

Comprehensive documentation is available in the `docs` directory:

- [User Guide](docs/usage/user_guide.md)
- [API Reference](docs/api/README.md)
- [Developer Guide](docs/development/developer_guide.md)
- [Examples](examples/README.md)

## Project Structure

```
TrafficSystem/
├── cpp/                  # C++ core components
│   ├── include/          # Header files
│   ├── src/              # Implementation files
│   └── binding/          # Python bindings
├── python/               # Python components
│   ├── visualization/    # Visualization modules
│   └── analysis/         # Data analysis modules
├── data/                 # Data files
├── tests/                # Test suites
├── docs/                 # Documentation
└── examples/             # Example usage
```

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- Inspired by SUMO (Simulation of Urban MObility)
- Traffic flow theories from Highway Capacity Manual
- Visualization techniques from modern GIS applications