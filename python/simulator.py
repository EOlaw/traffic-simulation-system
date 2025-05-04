"""
Traffic Simulation Manager

This module provides a high-level Python interface for managing the traffic simulation.
It serves as a bridge between the C++ simulation core and Python visualization/analysis.
"""

import os
import time
import threading
import logging
from typing import Optional, List, Dict, Tuple, Callable, Union

# Import C++ bindings
import traffic_system as ts

class TrafficSimulator:
    """
    Main simulation manager that provides a high-level Python interface
    for controlling and interacting with the C++ simulation core.
    """
    
    def __init__(self, config_file: Optional[str] = None):
        """
        Initialize the traffic simulator
        
        Args:
            config_file: Optional path to configuration file
        """
        # Set up logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler("traffic_simulation.log"),
                logging.StreamHandler()
            ]
        )
        self.logger = logging.getLogger("TrafficSimulator")
        
        # Get simulation controller singleton
        self.controller = ts.SimulationController.getInstance()
        
        # Initialize with config if provided
        if config_file:
            self.initialize(config_file)
        else:
            self.initialize()
        
        # Simulation control attributes
        self._running = False
        self._simulation_thread = None
        self._stop_event = threading.Event()
        
        # Current network and scenario info
        self._current_network_file = None
        self._current_scenario_file = None
        
        # Python callbacks for simulation events
        self._update_callbacks = {}
        self._next_callback_id = 0
        
        # Register core callback to forward to Python callbacks
        self._core_callback_id = self.controller.registerUpdateCallback(self._forward_update_event)
        
        self.logger.info("TrafficSimulator initialized")
    
    def __del__(self):
        """Cleanup resources"""
        if hasattr(self, 'controller') and hasattr(self, '_core_callback_id'):
            self.controller.unregisterUpdateCallback(self._core_callback_id)
    
    def initialize(self, config_file: Optional[str] = None) -> bool:
        """
        Initialize the simulation
        
        Args:
            config_file: Optional path to configuration file
            
        Returns:
            True if initialization was successful
        """
        try:
            # Initialize simulation controller
            if config_file:
                success = self.controller.initialize(config_file)
            else:
                success = self.controller.initialize("")
            
            if success:
                self.logger.info("Simulation initialized successfully")
            else:
                self.logger.error("Failed to initialize simulation")
            
            return success
            
        except Exception as e:
            self.logger.error(f"Error during initialization: {str(e)}")
            return False
    
    def load_network(self, network_file: str) -> bool:
        """
        Load road network from file
        
        Args:
            network_file: Path to network file
            
        Returns:
            True if loading was successful
        """
        try:
            success = self.controller.loadNetwork(network_file)
            
            if success:
                self._current_network_file = network_file
                self.logger.info(f"Network loaded from {network_file}")
            else:
                self.logger.error(f"Failed to load network from {network_file}")
            
            return success
            
        except Exception as e:
            self.logger.error(f"Error loading network: {str(e)}")
            return False
    
    def save_network(self, network_file: str) -> bool:
        """
        Save current road network to file
        
        Args:
            network_file: Output file path
            
        Returns:
            True if saving was successful
        """
        try:
            success = self.controller.saveNetwork(network_file)
            
            if success:
                self._current_network_file = network_file
                self.logger.info(f"Network saved to {network_file}")
            else:
                self.logger.error(f"Failed to save network to {network_file}")
            
            return success
            
        except Exception as e:
            self.logger.error(f"Error saving network: {str(e)}")
            return False
    
    def load_scenario(self, scenario_file: str) -> bool:
        """
        Load traffic scenario from file
        
        Args:
            scenario_file: Path to scenario file
            
        Returns:
            True if loading was successful
        """
        try:
            success = self.controller.loadScenario(scenario_file)
            
            if success:
                self._current_scenario_file = scenario_file
                self.logger.info(f"Scenario loaded from {scenario_file}")
            else:
                self.logger.error(f"Failed to load scenario from {scenario_file}")
            
            return success
            
        except Exception as e:
            self.logger.error(f"Error loading scenario: {str(e)}")
            return False
    
    def start(self) -> bool:
        """
        Start the simulation
        
        Returns:
            True if started successfully
        """
        if self._running:
            self.logger.warning("Simulation already running")
            return False
        
        try:
            # Start simulation controller
            self.controller.start()
            
            # Set running flag
            self._running = True
            self._stop_event.clear()
            
            self.logger.info("Simulation started")
            return True
            
        except Exception as e:
            self.logger.error(f"Error starting simulation: {str(e)}")
            return False
    
    def stop(self) -> bool:
        """
        Stop the simulation
        
        Returns:
            True if stopped successfully
        """
        if not self._running:
            self.logger.warning("Simulation not running")
            return False
        
        try:
            # Stop simulation controller
            self.controller.stop()
            
            # Set running flag
            self._running = False
            self._stop_event.set()
            
            # Wait for thread to finish if running in a thread
            if self._simulation_thread and self._simulation_thread.is_alive():
                self._simulation_thread.join(timeout=1.0)
            
            self.logger.info("Simulation stopped")
            return True
            
        except Exception as e:
            self.logger.error(f"Error stopping simulation: {str(e)}")
            return False
    
    def pause(self) -> bool:
        """
        Pause the simulation
        
        Returns:
            True if paused successfully
        """
        try:
            self.controller.pause()
            self.logger.info("Simulation paused")
            return True
            
        except Exception as e:
            self.logger.error(f"Error pausing simulation: {str(e)}")
            return False
    
    def resume(self) -> bool:
        """
        Resume the simulation
        
        Returns:
            True if resumed successfully
        """
        try:
            self.controller.resume()
            self.logger.info("Simulation resumed")
            return True
            
        except Exception as e:
            self.logger.error(f"Error resuming simulation: {str(e)}")
            return False
    
    def reset(self) -> bool:
        """
        Reset the simulation
        
        Returns:
            True if reset successfully
        """
        try:
            # Stop first if running
            if self._running:
                self.stop()
            
            # Reset simulation controller
            self.controller.reset()
            
            self.logger.info("Simulation reset")
            return True
            
        except Exception as e:
            self.logger.error(f"Error resetting simulation: {str(e)}")
            return False
    
    def run_for(self, duration: float, real_time_factor: float = 1.0) -> bool:
        """
        Run simulation for specified duration
        
        Args:
            duration: Simulation duration in seconds
            real_time_factor: Simulation speed multiplier (1.0 = real-time)
            
        Returns:
            True if completed successfully
        """
        try:
            # Run simulation directly
            self.controller.run(duration, real_time_factor)
            
            self.logger.info(f"Completed simulation run for {duration} seconds "
                           f"at {real_time_factor}x speed")
            return True
            
        except Exception as e:
            self.logger.error(f"Error during simulation run: {str(e)}")
            return False
    
    def run_in_thread(self, 
                     real_time_factor: float = 1.0,
                     step_size: float = 0.1,
                     update_interval: float = 0.05) -> bool:
        """
        Run simulation in a separate thread
        
        Args:
            real_time_factor: Simulation speed multiplier (1.0 = real-time)
            step_size: Simulation step size in seconds
            update_interval: Interval between updates in seconds
            
        Returns:
            True if thread started successfully
        """
        if self._running:
            self.logger.warning("Simulation already running")
            return False
        
        # Start simulation
        if not self.start():
            return False
        
        # Define simulation thread function
        def simulation_loop():
            try:
                self.logger.info(f"Starting simulation thread with "
                               f"real-time factor: {real_time_factor}, "
                               f"step size: {step_size}")
                
                # Calculate time adjustment
                time_multiplier = real_time_factor
                simulation_step = step_size
                
                # Run until stopped
                while self._running and not self._stop_event.is_set():
                    # Record start time
                    start_time = time.time()
                    
                    # Update simulation
                    self.controller.step(simulation_step)
                    
                    # Sleep to maintain real-time factor
                    elapsed = time.time() - start_time
                    target_elapsed = simulation_step / time_multiplier
                    
                    if elapsed < target_elapsed:
                        time.sleep(target_elapsed - elapsed)
            
            except Exception as e:
                self.logger.error(f"Error in simulation thread: {str(e)}")
                self._running = False
            
            finally:
                self.logger.info("Simulation thread stopped")
        
        # Create and start thread
        self._simulation_thread = threading.Thread(target=simulation_loop)
        self._simulation_thread.daemon = True
        self._simulation_thread.start()
        
        return True
    
    def generate_traffic(self, vehicle_count: int, seed: int = 0) -> bool:
        """
        Generate random traffic
        
        Args:
            vehicle_count: Number of vehicles to generate
            seed: Random seed (0 for time-based)
            
        Returns:
            True if generation was successful
        """
        try:
            self.controller.generateRandomTraffic(vehicle_count, seed)
            
            self.logger.info(f"Generated {vehicle_count} vehicles")
            return True
            
        except Exception as e:
            self.logger.error(f"Error generating traffic: {str(e)}")
            return False
    
    def get_statistics(self) -> ts.SimulationStatistics:
        """
        Get current simulation statistics
        
        Returns:
            Statistics object
        """
        return self.controller.getStatistics()
    
    def register_update_callback(self, callback: Callable[[float], None]) -> int:
        """
        Register callback for simulation updates
        
        Args:
            callback: Function to call after each update
            
        Returns:
            Callback ID for later removal
        """
        callback_id = self._next_callback_id
        self._next_callback_id += 1
        
        self._update_callbacks[callback_id] = callback
        
        return callback_id
    
    def unregister_update_callback(self, callback_id: int) -> bool:
        """
        Unregister update callback
        
        Args:
            callback_id: Callback ID to remove
            
        Returns:
            True if callback was found and removed
        """
        if callback_id in self._update_callbacks:
            del self._update_callbacks[callback_id]
            return True
        
        return False
    
    def _forward_update_event(self, delta_time: float):
        """Forward simulation update events to Python callbacks"""
        for callback in self._update_callbacks.values():
            try:
                callback(delta_time)
            except Exception as e:
                self.logger.error(f"Error in update callback: {str(e)}")
    
    # Helper methods for creating simulation objects
    def create_vehicle(self, 
                      vehicle_type: ts.VehicleType,
                      position: Union[ts.Vector2D, Tuple[float, float]],
                      road = None,
                      lane_index: int = 0) -> ts.Vehicle:
        """
        Create a vehicle
        
        Args:
            vehicle_type: Type of vehicle
            position: Initial position
            road: Initial road (optional)
            lane_index: Initial lane index (optional)
            
        Returns:
            Created vehicle
        """
        # Convert tuple to Vector2D if needed
        if isinstance(position, tuple):
            position = ts.Vector2D(position[0], position[1])
        
        return self.controller.createVehicle(vehicle_type, position, road, lane_index)
    
    def create_road(self,
                   name: str,
                   road_type: ts.RoadType,
                   start_point: Union[ts.Vector2D, Tuple[float, float]],
                   end_point: Union[ts.Vector2D, Tuple[float, float]],
                   lane_count: int = 1,
                   speed_limit: float = 0.0) -> ts.Road:
        """
        Create a road
        
        Args:
            name: Road name
            road_type: Type of road
            start_point: Starting coordinates
            end_point: Ending coordinates
            lane_count: Number of lanes
            speed_limit: Speed limit (0 = use default for road type)
            
        Returns:
            Created road
        """
        # Convert tuples to Vector2D if needed
        if isinstance(start_point, tuple):
            start_point = ts.Vector2D(start_point[0], start_point[1])
        
        if isinstance(end_point, tuple):
            end_point = ts.Vector2D(end_point[0], end_point[1])
        
        return self.controller.createRoad(
            name, road_type, start_point, end_point, lane_count, speed_limit)
    
    def create_intersection(self,
                           position: Union[ts.Vector2D, Tuple[float, float]],
                           name: str = "") -> ts.Intersection:
        """
        Create an intersection
        
        Args:
            position: Position coordinates
            name: Intersection name (optional)
            
        Returns:
            Created intersection
        """
        # Convert tuple to Vector2D if needed
        if isinstance(position, tuple):
            position = ts.Vector2D(position[0], position[1])
        
        return self.controller.createIntersection(position, name)
    
    def build_sample_network(self) -> bool:
        """
        Build a sample road network
        
        Returns:
            True if successful
        """
        try:
            # Create a grid network of roads
            grid_size = 3
            spacing = 200.0  # 200 meters between intersections
            
            # Create intersections
            intersections = {}
            for i in range(grid_size):
                for j in range(grid_size):
                    pos = (i * spacing, j * spacing)
                    name = f"Intersection {i}-{j}"
                    intersection = self.create_intersection(pos, name)
                    intersections[(i, j)] = intersection
            
            # Create horizontal roads
            for i in range(grid_size):
                for j in range(grid_size - 1):
                    name = f"Road H{i}-{j}"
                    
                    # Main roads are larger
                    if i == grid_size // 2:
                        road_type = ts.RoadType.ARTERIAL
                        lanes = 2
                    else:
                        road_type = ts.RoadType.URBAN
                        lanes = 1
                    
                    # Create road between intersections
                    start = intersections[(j, i)].getPosition()
                    end = intersections[(j+1, i)].getPosition()
                    
                    road = self.create_road(name, road_type, start, end, lanes)
                    
                    # Connect to intersections
                    road.setStartIntersection(intersections[(j, i)])
                    road.setEndIntersection(intersections[(j+1, i)])
            
            # Create vertical roads
            for i in range(grid_size):
                for j in range(grid_size - 1):
                    name = f"Road V{i}-{j}"
                    
                    # Main roads are larger
                    if i == grid_size // 2:
                        road_type = ts.RoadType.ARTERIAL
                        lanes = 2
                    else:
                        road_type = ts.RoadType.URBAN
                        lanes = 1
                    
                    # Create road between intersections
                    start = intersections[(i, j)].getPosition()
                    end = intersections[(i, j+1)].getPosition()
                    
                    road = self.create_road(name, road_type, start, end, lanes)
                    
                    # Connect to intersections
                    road.setStartIntersection(intersections[(i, j)])
                    road.setEndIntersection(intersections[(i, j+1)])
            
            self.logger.info(f"Built sample network with "
                           f"{len(intersections)} intersections and "
                           f"{len(self.controller.getAllRoads())} roads")
            return True
            
        except Exception as e:
            self.logger.error(f"Error building sample network: {str(e)}")
            return False