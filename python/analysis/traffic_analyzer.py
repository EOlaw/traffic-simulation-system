"""
Traffic Analysis Module

Provides tools for analyzing traffic patterns, generating statistics,
and visualizing simulation results.
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from typing import Dict, List, Tuple, Optional, Union
import os
import csv
from datetime import datetime

# Import C++ bindings
import traffic_system as ts

class TrafficData:
    """Container for collected simulation data over time"""
    
    def __init__(self):
        """Initialize data structure"""
        self.time_points = []
        self.vehicle_counts = []
        self.active_vehicle_counts = []
        self.average_speeds = []
        self.average_densities = []
        self.average_flows = []
        self.congestion_times = []
        self.trip_times = []
        
        # Road-specific data
        self.road_speeds = {}  # road_id -> [speeds]
        self.road_densities = {}  # road_id -> [densities]
        self.road_flows = {}  # road_id -> [flows]
        
        # Vehicle-specific data
        self.vehicle_positions = {}  # vehicle_id -> [(x, y)]
        self.vehicle_speeds = {}  # vehicle_id -> [speeds]
        
    def add_stats_record(self, stats: ts.SimulationStatistics):
        """Add a statistics record
        
        Args:
            stats: Simulation statistics snapshot
        """
        self.time_points.append(stats.simulationTime)
        self.vehicle_counts.append(stats.totalVehicles)
        self.active_vehicle_counts.append(stats.activeVehicles)
        self.average_speeds.append(stats.averageSpeed)
        self.average_densities.append(stats.averageDensity)
        self.average_flows.append(stats.averageFlow)
        self.congestion_times.append(stats.totalCongestionTime)
        self.trip_times.append(stats.averageTripTime)
    
    def add_road_record(self, road_id: str, speed: float, density: float, flow: float):
        """Add a road statistics record
        
        Args:
            road_id: Road identifier
            speed: Average speed on road (m/s)
            density: Traffic density (vehicles/km)
            flow: Traffic flow (vehicles/hour)
        """
        if road_id not in self.road_speeds:
            self.road_speeds[road_id] = []
            self.road_densities[road_id] = []
            self.road_flows[road_id] = []
        
        self.road_speeds[road_id].append(speed)
        self.road_densities[road_id].append(density)
        self.road_flows[road_id].append(flow)
    
    def add_vehicle_record(self, vehicle_id: str, position: Tuple[float, float], speed: float):
        """Add a vehicle statistics record
        
        Args:
            vehicle_id: Vehicle identifier
            position: Position (x, y)
            speed: Current speed (m/s)
        """
        if vehicle_id not in self.vehicle_positions:
            self.vehicle_positions[vehicle_id] = []
            self.vehicle_speeds[vehicle_id] = []
        
        self.vehicle_positions[vehicle_id].append(position)
        self.vehicle_speeds[vehicle_id].append(speed)
    
    def to_dataframe(self) -> pd.DataFrame:
        """Convert global statistics to pandas DataFrame
        
        Returns:
            DataFrame with time series data
        """
        data = {
            'time': self.time_points,
            'vehicle_count': self.vehicle_counts,
            'active_vehicles': self.active_vehicle_counts,
            'average_speed': self.average_speeds,
            'average_density': self.average_densities,
            'average_flow': self.average_flows,
            'congestion_time': self.congestion_times,
            'trip_time': self.trip_times
        }
        
        return pd.DataFrame(data)
    
    def road_data_to_dataframe(self, road_id: str) -> pd.DataFrame:
        """Convert road statistics to pandas DataFrame
        
        Args:
            road_id: Road identifier
            
        Returns:
            DataFrame with time series data for the road
        """
        if road_id not in self.road_speeds:
            return pd.DataFrame()
        
        data = {
            'time': self.time_points,
            'speed': self.road_speeds[road_id],
            'density': self.road_densities[road_id],
            'flow': self.road_flows[road_id]
        }
        
        return pd.DataFrame(data)
    
    def vehicle_data_to_dataframe(self, vehicle_id: str) -> pd.DataFrame:
        """Convert vehicle statistics to pandas DataFrame
        
        Args:
            vehicle_id: Vehicle identifier
            
        Returns:
            DataFrame with time series data for the vehicle
        """
        if vehicle_id not in self.vehicle_speeds:
            return pd.DataFrame()
        
        data = {
            'time': self.time_points[:len(self.vehicle_speeds[vehicle_id])],
            'speed': self.vehicle_speeds[vehicle_id]
        }
        
        # Add position data
        positions = self.vehicle_positions[vehicle_id]
        if positions:
            data['pos_x'] = [pos[0] for pos in positions]
            data['pos_y'] = [pos[1] for pos in positions]
        
        return pd.DataFrame(data)
    
    def save_to_csv(self, output_dir: str):
        """Save all data to CSV files
        
        Args:
            output_dir: Output directory
        """
        # Create directory if it doesn't exist
        os.makedirs(output_dir, exist_ok=True)
        
        # Save global statistics
        global_df = self.to_dataframe()
        global_df.to_csv(os.path.join(output_dir, 'global_statistics.csv'), index=False)
        
        # Save road data
        for road_id in self.road_speeds:
            road_df = self.road_data_to_dataframe(road_id)
            road_df.to_csv(os.path.join(output_dir, f'road_{road_id}.csv'), index=False)
        
        # Save vehicle data
        for vehicle_id in self.vehicle_speeds:
            vehicle_df = self.vehicle_data_to_dataframe(vehicle_id)
            vehicle_df.to_csv(os.path.join(output_dir, f'vehicle_{vehicle_id}.csv'), index=False)


class TrafficAnalyzer:
    """Analyzes traffic patterns and generates statistics"""
    
    def __init__(self, simulator=None):
        """Initialize traffic analyzer
        
        Args:
            simulator: Optional simulator instance
        """
        self.simulator = simulator
        self.data = TrafficData()
        self.sampling_interval = 1.0  # seconds
        self.last_sample_time = 0.0
        self.recording = False
        self._callback_id = None
    
    def start_recording(self, sampling_interval: float = 1.0):
        """Start recording traffic data
        
        Args:
            sampling_interval: Time between samples in seconds
        """
        if self.recording:
            return
        
        self.sampling_interval = sampling_interval
        self.last_sample_time = 0.0
        self.recording = True
        
        # Register callback if simulator is provided
        if self.simulator:
            self._callback_id = self.simulator.register_update_callback(self._update_callback)
    
    def stop_recording(self):
        """Stop recording traffic data"""
        if not self.recording:
            return
        
        self.recording = False
        
        # Unregister callback if registered
        if self.simulator and self._callback_id is not None:
            self.simulator.unregister_update_callback(self._callback_id)
            self._callback_id = None
    
    def reset(self):
        """Reset recorded data"""
        self.data = TrafficData()
        self.last_sample_time = 0.0
    
    def _update_callback(self, delta_time: float):
        """Update callback for simulation
        
        Args:
            delta_time: Time elapsed since last update
        """
        if not self.recording:
            return
        
        # Get current simulation time
        current_time = self.simulator.controller.getCurrentTime()
        
        # Check if it's time to sample
        if current_time - self.last_sample_time >= self.sampling_interval:
            self.record_sample()
            self.last_sample_time = current_time
    
    def record_sample(self):
        """Record a data sample from current simulation state"""
        if not self.simulator:
            return
        
        # Get global statistics
        stats = self.simulator.controller.getStatistics()
        self.data.add_stats_record(stats)
        
        # Record road data
        for road in self.simulator.controller.getAllRoads():
            road_id = road.getId().toString()
            speed = road.getAverageSpeed()
            density = road.getAverageDensity()
            flow = road.getAverageFlow()
            
            self.data.add_road_record(road_id, speed, density, flow)
        
        # Record vehicle data
        for vehicle in self.simulator.controller.getAllVehicles():
            vehicle_id = vehicle.getId().toString()
            pos = vehicle.getPosition()
            speed = vehicle.getCurrentSpeed()
            
            self.data.add_vehicle_record(vehicle_id, (pos.x, pos.y), speed)
    
    def calculate_congestion_points(self, threshold: float = 0.3) -> List[Tuple[str, float]]:
        """Calculate points of congestion
        
        Args:
            threshold: Speed threshold relative to speed limit (0.0-1.0)
            
        Returns:
            List of (road_id, congestion_value) tuples sorted by severity
        """
        congestion_points = []
        
        if not self.simulator:
            return congestion_points
        
        # Check each road for congestion
        for road in self.simulator.controller.getAllRoads():
            road_id = road.getId().toString()
            speed_limit = road.getSpeedLimit()
            current_speed = road.getAverageSpeed()
            
            # Skip roads with no vehicles
            if current_speed <= 0:
                continue
            
            # Calculate congestion value (0 = free flow, 1 = complete jam)
            if speed_limit > 0:
                congestion = 1.0 - (current_speed / speed_limit)
            else:
                congestion = 0.0
            
            # Add to list if above threshold
            if congestion > threshold:
                congestion_points.append((road_id, congestion))
        
        # Sort by congestion value (descending)
        congestion_points.sort(key=lambda x: x[1], reverse=True)
        
        return congestion_points
    
    def generate_report(self, output_file: Optional[str] = None) -> str:
        """Generate a text report of traffic statistics
        
        Args:
            output_file: Optional file to write report to
            
        Returns:
            Report text
        """
        # Create DataFrame for analysis
        df = self.data.to_dataframe()
        
        if df.empty:
            return "No data available for report"
        
        # Generate report
        report = "Traffic Simulation Report\n"
        report += "========================\n\n"
        
        # Summary statistics
        report += "Summary Statistics:\n"
        report += f"Simulation duration: {df['time'].max() - df['time'].min():.2f} seconds\n"
        report += f"Average vehicle count: {df['vehicle_count'].mean():.2f}\n"
        report += f"Average speed: {df['average_speed'].mean() * 3.6:.2f} km/h\n"
        report += f"Average density: {df['average_density'].mean():.2f} vehicles/km\n"
        report += f"Average flow: {df['average_flow'].mean():.2f} vehicles/hour\n\n"
        
        # Congestion analysis
        congestion_points = self.calculate_congestion_points()
        if congestion_points:
            report += "Congestion Points:\n"
            for road_id, value in congestion_points[:5]:  # Top 5 congestion points
                report += f"Road {road_id}: {value * 100:.1f}% congestion\n"
        else:
            report += "No significant congestion detected\n"
        
        # Write to file if specified
        if output_file:
            with open(output_file, 'w') as f:
                f.write(report)
        
        return report
    
    def plot_global_statistics(self, 
                              output_file: Optional[str] = None,
                              show_plot: bool = True):
        """Plot global traffic statistics
        
        Args:
            output_file: Optional file to save plot to
            show_plot: Whether to display the plot
        """
        df = self.data.to_dataframe()
        
        if df.empty:
            print("No data available for plotting")
            return
        
        # Create figure with subplots
        fig, axs = plt.subplots(2, 2, figsize=(12, 8))
        fig.suptitle('Traffic Simulation Statistics', fontsize=16)
        
        # Plot vehicle count
        axs[0, 0].plot(df['time'], df['vehicle_count'], 'b-', label='Total')
        axs[0, 0].plot(df['time'], df['active_vehicles'], 'g-', label='Active')
        axs[0, 0].set_xlabel('Time (s)')
        axs[0, 0].set_ylabel('Vehicle Count')
        axs[0, 0].legend()
        axs[0, 0].grid(True)
        
        # Plot average speed
        axs[0, 1].plot(df['time'], df['average_speed'] * 3.6, 'r-')  # Convert to km/h
        axs[0, 1].set_xlabel('Time (s)')
        axs[0, 1].set_ylabel('Average Speed (km/h)')
        axs[0, 1].grid(True)
        
        # Plot density
        axs[1, 0].plot(df['time'], df['average_density'], 'g-')
        axs[1, 0].set_xlabel('Time (s)')
        axs[1, 0].set_ylabel('Average Density (veh/km)')
        axs[1, 0].grid(True)
        
        # Plot flow
        axs[1, 1].plot(df['time'], df['average_flow'], 'b-')
        axs[1, 1].set_xlabel('Time (s)')
        axs[1, 1].set_ylabel('Average Flow (veh/h)')
        axs[1, 1].grid(True)
        
        # Adjust layout
        plt.tight_layout()
        
        # Save if output file specified
        if output_file:
            plt.savefig(output_file, dpi=150)
        
        # Show if requested
        if show_plot:
            plt.show()
        else:
            plt.close()
    
    def plot_road_comparison(self, 
                            road_ids: List[str],
                            metric: str = 'speed',
                            output_file: Optional[str] = None,
                            show_plot: bool = True):
        """Plot comparison of roads for a specific metric
        
        Args:
            road_ids: List of road IDs to compare
            metric: Metric to compare ('speed', 'density', or 'flow')
            output_file: Optional file to save plot to
            show_plot: Whether to display the plot
        """
        if not road_ids:
            print("No roads specified for comparison")
            return
        
        # Create plot
        plt.figure(figsize=(10, 6))
        
        # Plot each road
        for road_id in road_ids:
            df = self.data.road_data_to_dataframe(road_id)
            
            if df.empty:
                print(f"No data available for road {road_id}")
                continue
            
            if metric == 'speed':
                plt.plot(df['time'], df['speed'] * 3.6, label=f'Road {road_id}')  # Convert to km/h
                plt.ylabel('Speed (km/h)')
            elif metric == 'density':
                plt.plot(df['time'], df['density'], label=f'Road {road_id}')
                plt.ylabel('Density (veh/km)')
            elif metric == 'flow':
                plt.plot(df['time'], df['flow'], label=f'Road {road_id}')
                plt.ylabel('Flow (veh/h)')
            else:
                print(f"Unknown metric: {metric}")
                return
        
        plt.xlabel('Time (s)')
        plt.title(f'Road Comparison - {metric.capitalize()}')
        plt.legend()
        plt.grid(True)
        
        # Save if output file specified
        if output_file:
            plt.savefig(output_file, dpi=150)
        
        # Show if requested
        if show_plot:
            plt.show()
        else:
            plt.close()
    
    def plot_vehicle_trajectory(self, 
                               vehicle_id: str,
                               output_file: Optional[str] = None,
                               show_plot: bool = True):
        """Plot trajectory of a vehicle
        
        Args:
            vehicle_id: Vehicle ID
            output_file: Optional file to save plot to
            show_plot: Whether to display the plot
        """
        df = self.data.vehicle_data_to_dataframe(vehicle_id)
        
        if df.empty or 'pos_x' not in df.columns or 'pos_y' not in df.columns:
            print(f"No position data available for vehicle {vehicle_id}")
            return
        
        # Create figure with subplots
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), gridspec_kw={'height_ratios': [3, 1]})
        
        # Plot trajectory
        scatter = ax1.scatter(df['pos_x'], df['pos_y'], c=df['time'], cmap='viridis')
        ax1.set_xlabel('X Position (m)')
        ax1.set_ylabel('Y Position (m)')
        ax1.set_title(f'Vehicle {vehicle_id} Trajectory')
        ax1.grid(True)
        ax1.set_aspect('equal')
        
        # Add colorbar
        cbar = plt.colorbar(scatter, ax=ax1)
        cbar.set_label('Time (s)')
        
        # Plot speed
        ax2.plot(df['time'], df['speed'] * 3.6)  # Convert to km/h
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Speed (km/h)')
        ax2.grid(True)
        
        # Adjust layout
        plt.tight_layout()
        
        # Save if output file specified
        if output_file:
            plt.savefig(output_file, dpi=150)
        
        # Show if requested
        if show_plot:
            plt.show()
        else:
            plt.close()
    
    def export_analysis(self, output_dir: str):
        """Export all analysis data to files
        
        Args:
            output_dir: Output directory
        """
        # Create timestamped directory
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        output_path = os.path.join(output_dir, f'traffic_analysis_{timestamp}')
        os.makedirs(output_path, exist_ok=True)
        
        # Save data to CSV files
        self.data.save_to_csv(output_path)
        
        # Generate report
        report_file = os.path.join(output_path, 'traffic_report.txt')
        self.generate_report(report_file)
        
        # Generate plots
        self.plot_global_statistics(
            os.path.join(output_path, 'global_statistics.png'),
            show_plot=False
        )
        
        # Plot congestion points
        congestion_points = self.calculate_congestion_points()
        if congestion_points:
            road_ids = [cp[0] for cp in congestion_points[:5]]  # Top 5 congestion points
            self.plot_road_comparison(
                road_ids,
                'speed',
                os.path.join(output_path, 'congestion_comparison.png'),
                show_plot=False
            )
        
        # Optionally plot some vehicle trajectories
        df = self.data.to_dataframe()
        if not df.empty and 'vehicle_speeds' in dir(self.data) and self.data.vehicle_speeds:
            # Get some vehicle IDs
            vehicle_ids = list(self.data.vehicle_speeds.keys())
            
            # Plot trajectories for a few vehicles
            for vehicle_id in vehicle_ids[:5]:  # First 5 vehicles
                self.plot_vehicle_trajectory(
                    vehicle_id,
                    os.path.join(output_path, f'vehicle_{vehicle_id}_trajectory.png'),
                    show_plot=False
                )
        
        print(f"Analysis exported to {output_path}")