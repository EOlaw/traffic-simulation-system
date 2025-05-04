#!/usr/bin/env python3
"""
Traffic Simulation System

Main entry point for the traffic simulation system.
"""

import os
import sys
import argparse
import logging
import time
from pathlib import Path

from .simulator import TrafficSimulator
from .analysis.traffic_analyzer import TrafficAnalyzer

# Try to import visualization module (optional)
try:
    from .visualization.renderer import Renderer
    VISUALIZATION_AVAILABLE = True
except ImportError:
    VISUALIZATION_AVAILABLE = False


def setup_logging(log_level=logging.INFO, log_file=None):
    """Set up logging configuration
    
    Args:
        log_level: Logging level
        log_file: Optional log file path
    """
    handlers = [logging.StreamHandler()]
    
    if log_file:
        # Create directory if it doesn't exist
        log_dir = os.path.dirname(log_file)
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)
        
        # Add file handler
        handlers.append(logging.FileHandler(log_file))
    
    # Configure logging
    logging.basicConfig(
        level=log_level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=handlers
    )


def parse_args():
    """Parse command line arguments
    
    Returns:
        Parsed arguments
    """
    parser = argparse.ArgumentParser(description='Traffic Simulation System')
    
    # Basic options
    parser.add_argument('--config', '-c', type=str, 
                        help='Path to configuration file')
    parser.add_argument('--output', '-o', type=str, default='simulation_results',
                        help='Output directory for results')
    parser.add_argument('--log-file', type=str,
                        help='Log file path')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Enable verbose logging')
    
    # Subcommands
    subparsers = parser.add_subparsers(dest='command', help='Command to execute')
    
    # Run command
    run_parser = subparsers.add_parser('run', help='Run simulation')
    run_parser.add_argument('--network', '-n', type=str,
                           help='Path to network file')
    run_parser.add_argument('--scenario', '-s', type=str,
                           help='Path to scenario file')
    run_parser.add_argument('--duration', '-d', type=float, default=300.0,
                           help='Simulation duration in seconds')
    run_parser.add_argument('--vehicles', type=int, default=100,
                           help='Number of vehicles to generate (if no scenario)')
    run_parser.add_argument('--speed', type=float, default=1.0,
                           help='Simulation speed factor')
    run_parser.add_argument('--visualization', action='store_true',
                           help='Enable visualization')
    run_parser.add_argument('--analysis', action='store_true',
                           help='Enable traffic analysis')
    
    # Create network command
    create_parser = subparsers.add_parser('create', help='Create network')
    create_parser.add_argument('--output', '-o', type=str, required=True,
                             help='Output network file path')
    create_parser.add_argument('--grid', '-g', type=int, default=3,
                             help='Grid size for sample network')
    
    # Analyze command
    analyze_parser = subparsers.add_parser('analyze', help='Analyze simulation results')
    analyze_parser.add_argument('--input', '-i', type=str, required=True,
                              help='Input directory with simulation results')
    analyze_parser.add_argument('--output', '-o', type=str,
                              help='Output directory for analysis results')
    
    return parser.parse_args()


def run_simulation(args):
    """Run traffic simulation
    
    Args:
        args: Command line arguments
    """
    # Create simulator
    simulator = TrafficSimulator(args.config)
    
    # Create traffic analyzer if requested
    analyzer = None
    if args.analysis:
        analyzer = TrafficAnalyzer(simulator)
        analyzer.start_recording()
    
    try:
        # Load network if specified
        if args.network:
            logging.info(f"Loading network from {args.network}")
            if not simulator.load_network(args.network):
                logging.error("Failed to load network")
                return False
        else:
            # Build sample network
            logging.info("Building sample network")
            simulator.build_sample_network()
        
        # Load scenario or generate traffic
        if args.scenario:
            logging.info(f"Loading scenario from {args.scenario}")
            if not simulator.load_scenario(args.scenario):
                logging.error("Failed to load scenario")
                return False
        else:
            # Generate random traffic
            logging.info(f"Generating {args.vehicles} vehicles")
            simulator.generate_traffic(args.vehicles)
        
        # Run with visualization if requested
        if args.visualization:
            if not VISUALIZATION_AVAILABLE:
                logging.warning("Visualization not available, running without it")
                run_headless(simulator, analyzer, args)
            else:
                run_with_visualization(simulator, analyzer, args)
        else:
            # Run without visualization
            run_headless(simulator, analyzer, args)
        
        return True
        
    except Exception as e:
        logging.exception(f"Error running simulation: {str(e)}")
        return False
    finally:
        # Stop analyzer if active
        if analyzer:
            analyzer.stop_recording()
            
            # Export analysis results
            output_dir = args.output
            os.makedirs(output_dir, exist_ok=True)
            analyzer.export_analysis(output_dir)
            
            # Print report
            print("\nTraffic Simulation Report")
            print("========================")
            print(analyzer.generate_report())


def run_headless(simulator, analyzer, args):
    """Run simulation without visualization
    
    Args:
        simulator: Simulator instance
        analyzer: Analyzer instance
        args: Command line arguments
    """
    # Start simulation
    simulator.start()
    
    # Track progress
    start_time = time.time()
    last_update = 0
    
    try:
        # Run simulation until duration is reached
        while simulator.controller.getCurrentTime() < args.duration:
            # Update simulation (small steps for better responsiveness)
            simulator.controller.step(0.1 * args.speed)
            
            # Print progress every second
            elapsed = time.time() - start_time
            if elapsed - last_update >= 1.0:
                last_update = elapsed
                sim_time = simulator.controller.getCurrentTime()
                stats = simulator.controller.getStatistics()
                
                # Calculate and print progress
                progress = min(100.0, sim_time / args.duration * 100.0)
                print(f"Progress: {progress:.1f}% - Simulation time: {sim_time:.1f}s - "
                     f"Vehicles: {stats.activeVehicles}/{stats.totalVehicles} - "
                     f"Avg Speed: {stats.averageSpeed * 3.6:.1f} km/h", end='\r')
        
        print("\nSimulation completed successfully!")
        
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user")
    finally:
        # Stop simulation
        simulator.stop()


def run_with_visualization(simulator, analyzer, args):
    """Run simulation with visualization
    
    Args:
        simulator: Simulator instance
        analyzer: Analyzer instance
        args: Command line arguments
    """
    try:
        # Create and run renderer
        renderer = Renderer(simulation=simulator.controller)
        
        # Start simulation
        simulator.start()
        
        # Run renderer (this will block until window is closed)
        renderer.run()
        
        # Stop simulation when renderer is closed
        simulator.stop()
        
    except Exception as e:
        logging.exception(f"Error in visualization: {str(e)}")
        simulator.stop()


def create_network(args):
    """Create a road network
    
    Args:
        args: Command line arguments
    """
    try:
        # Create simulator
        simulator = TrafficSimulator()
        
        # Build custom network based on command line options
        # For now, we just create a grid network with the specified size
        logging.info(f"Building grid network of size {args.grid}x{args.grid}")
        
        # Build sample network (will be a grid)
        simulator.build_sample_network()
        
        # Save network to file
        logging.info(f"Saving network to {args.output}")
        if simulator.save_network(args.output):
            logging.info(f"Network saved successfully")
            return True
        else:
            logging.error(f"Failed to save network")
            return False
            
    except Exception as e:
        logging.exception(f"Error creating network: {str(e)}")
        return False


def analyze_results(args):
    """Analyze simulation results
    
    Args:
        args: Command line arguments
    """
    try:
        # Determine output directory
        output_dir = args.output
        if not output_dir:
            output_dir = args.input + "_analysis"
        
        logging.info(f"Analyzing simulation results from {args.input}")
        
        # TODO: Implement offline analysis of simulation results
        # For now, just print a message
        print(f"Analyzing simulation results from {args.input}")
        print(f"Output will be saved to {output_dir}")
        print("This feature is not yet implemented")
        
        return True
        
    except Exception as e:
        logging.exception(f"Error analyzing results: {str(e)}")
        return False


def main():
    """Main entry point"""
    # Parse command line arguments
    args = parse_args()
    
    # Set up logging
    log_level = logging.DEBUG if args.verbose else logging.INFO
    setup_logging(log_level, args.log_file)
    
    logging.info("Traffic Simulation System")
    
    # Execute command
    if args.command == 'run':
        success = run_simulation(args)
    elif args.command == 'create':
        success = create_network(args)
    elif args.command == 'analyze':
        success = analyze_results(args)
    else:
        # No command specified, show help
        print("No command specified. Use --help for available commands.")
        success = False
    
    # Exit with appropriate code
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()