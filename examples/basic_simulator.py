#!/usr/bin/env python3
"""
Basic Traffic Simulation Example

This example demonstrates how to set up a basic traffic simulation,
create a road network, generate vehicles, and run the simulation.
"""

import os
import sys
import time
import argparse

# Add project root to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Import traffic simulation modules
from traffic_simulation.simulator import TrafficSimulator
from traffic_simulation.analysis.traffic_analyzer import TrafficAnalyzer
import traffic_system as ts

def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description='Traffic Simulation Example')
    parser.add_argument('--duration', type=float, default=300.0,
                        help='Simulation duration in seconds (default: 300.0)')
    parser.add_argument('--vehicles', type=int, default=100,
                        help='Number of vehicles to generate (default: 100)')
    parser.add_argument('--speed', type=float, default=2.0,
                        help='Simulation speed factor (default: 2.0)')
    parser.add_argument('--visualization', action='store_true',
                        help='Enable visualization')
    parser.add_argument('--output', type=str, default='simulation_results',
                        help='Output directory for results (default: simulation_results)')
    
    return parser.parse_args()

def main():
    """Main function"""
    args = parse_args()
    
    print("Traffic Simulation Example")
    print(f"Running simulation for {args.duration} seconds with {args.vehicles} vehicles")
    print(f"Simulation speed: {args.speed}x")
    
    # Create simulator
    simulator = TrafficSimulator()
    
    # Create traffic analyzer
    analyzer = TrafficAnalyzer(simulator)
    
    # Build road network
    print("Building road network...")
    simulator.build_sample_network()
    
    # Generate traffic
    print(f"Generating {args.vehicles} vehicles...")
    simulator.generate_traffic(args.vehicles)
    
    # Start analyzer
    analyzer.start_recording()
    
    # Start visualization if requested
    if args.visualization:
        try:
            # Import visualization module
            from traffic_simulation.visualization.renderer import Renderer
            
            # Create and run renderer in a separate thread
            renderer = Renderer(simulation=simulator.controller)
            
            # Start simulation
            simulator.start()
            
            # Run renderer (this will block until window is closed)
            renderer.run()
            
            # Stop simulation when renderer is closed
            simulator.stop()
            
        except ImportError:
            print("Visualization module not available. Running without visualization.")
            run_simulation(simulator, args.duration, args.speed)
    else:
        # Run simulation without visualization
        run_simulation(simulator, args.duration, args.speed)
    
    # Stop analyzer
    analyzer.stop_recording()
    
    # Generate and print report
    print("\nTraffic Report:")
    print(analyzer.generate_report())
    
    # Export analysis
    analyzer.export_analysis(args.output)
    
    print(f"\nSimulation complete. Results saved to {args.output}")

def run_simulation(simulator, duration, speed_factor):
    """Run simulation for the specified duration
    
    Args:
        simulator: TrafficSimulator instance
        duration: Simulation duration in seconds
        speed_factor: Simulation speed factor
    """
    # Start simulation
    simulator.start()
    
    # Track progress
    start_time = time.time()
    last_update = 0
    
    try:
        # Run simulation until duration is reached
        while simulator.controller.getCurrentTime() < duration:
            # Update simulation (small steps for better responsiveness)
            simulator.controller.step(0.1 * speed_factor)
            
            # Print progress every second
            elapsed = time.time() - start_time
            if elapsed - last_update >= 1.0:
                last_update = elapsed
                sim_time = simulator.controller.getCurrentTime()
                stats = simulator.controller.getStatistics()
                
                # Calculate and print progress
                progress = min(100.0, sim_time / duration * 100.0)
                print(f"Progress: {progress:.1f}% - Simulation time: {sim_time:.1f}s - "
                      f"Vehicles: {stats.activeVehicles}/{stats.totalVehicles} - "
                      f"Avg Speed: {stats.averageSpeed * 3.6:.1f} km/h", end='\r')
        
        print("\nSimulation completed successfully!")
        
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user")
    finally:
        # Stop simulation
        simulator.stop()

if __name__ == "__main__":
    main()