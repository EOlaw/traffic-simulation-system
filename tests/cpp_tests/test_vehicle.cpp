#include <iostream>
#include <cassert>
#include <memory>
#include "../../cpp/include/models/Vehicle.h"
#include "../../cpp/include/models/Road.h"
#include "../../cpp/include/utils/Vector2D.h"

using namespace TrafficSim;

// Simple test framework
#define TEST(name) void test_##name()
#define RUN_TEST(name) std::cout << "Running test: " << #name << "... "; test_##name(); std::cout << "PASSED\n"
#define ASSERT(condition) if (!(condition)) { std::cerr << "\nAssertion failed: " << #condition << " at line " << __LINE__ << std::endl; exit(1); }

TEST(vehicle_creation) {
    // Test basic vehicle creation
    Vehicle vehicle(VehicleType::CAR, 4.5, 2.0, 30.0);
    
    // Check initial state
    ASSERT(vehicle.getType() == VehicleType::CAR);
    ASSERT(vehicle.getLength() == 4.5);
    ASSERT(vehicle.getWidth() == 2.0);
    ASSERT(vehicle.getMaxSpeed() == 30.0);
    ASSERT(vehicle.getCurrentSpeed() == 0.0);
}

TEST(vehicle_movement) {
    // Create a vehicle
    Vehicle vehicle(VehicleType::CAR, 4.5, 2.0, 30.0);
    
    // Set initial state
    Vector2D initialPosition(100.0, 200.0);
    vehicle.setPosition(initialPosition);
    vehicle.setHeading(0.0);  // East direction
    vehicle.setCurrentSpeed(10.0);  // 10 m/s
    
    // Update vehicle (1 second)
    vehicle.update(1.0);
    
    // Check new position (should have moved east by 10 meters)
    Vector2D expectedPosition(110.0, 200.0);
    Vector2D actualPosition = vehicle.getPosition();
    
    ASSERT(std::abs(actualPosition.x - expectedPosition.x) < 0.001);
    ASSERT(std::abs(actualPosition.y - expectedPosition.y) < 0.001);
}

TEST(speed_limits) {
    // Create a vehicle
    Vehicle vehicle(VehicleType::CAR, 4.5, 2.0, 30.0);
    
    // Test speed limits
    vehicle.setCurrentSpeed(10.0);
    ASSERT(vehicle.getCurrentSpeed() == 10.0);
    
    // Test over max speed
    vehicle.setCurrentSpeed(50.0);
    ASSERT(vehicle.getCurrentSpeed() == 30.0);  // Clamped to max speed
    
    // Test negative speed
    vehicle.setCurrentSpeed(-5.0);
    ASSERT(vehicle.getCurrentSpeed() == 0.0);  // Clamped to 0
}

TEST(acceleration_and_deceleration) {
    // Create a vehicle
    Vehicle vehicle(VehicleType::CAR, 4.5, 2.0, 30.0);
    
    // Test acceleration
    vehicle.setCurrentSpeed(0.0);
    vehicle.accelerate(5.0);
    vehicle.update(1.0);
    ASSERT(vehicle.getCurrentSpeed() > 0.0);
    
    // Test deceleration
    double initialSpeed = vehicle.getCurrentSpeed();
    vehicle.brake(5.0);
    vehicle.update(1.0);
    ASSERT(vehicle.getCurrentSpeed() < initialSpeed);
    
    // Test stop
    vehicle.setCurrentSpeed(10.0);
    vehicle.stop();
    vehicle.update(1.0);
    ASSERT(vehicle.getCurrentSpeed() < 10.0);
}

TEST(collision_detection) {
    // Create two vehicles
    Vehicle vehicle1(VehicleType::CAR, 4.5, 2.0, 30.0);
    Vehicle vehicle2(VehicleType::CAR, 4.5, 2.0, 30.0);
    
    // Position vehicles far apart
    vehicle1.setPosition(Vector2D(0.0, 0.0));
    vehicle2.setPosition(Vector2D(100.0, 100.0));
    
    // Check no collision
    ASSERT(!vehicle1.detectCollision(vehicle2));
    
    // Position vehicles close together
    vehicle1.setPosition(Vector2D(0.0, 0.0));
    vehicle2.setPosition(Vector2D(3.0, 0.0));
    
    // Check collision (vehicles are 4.5m long, so there should be overlap)
    ASSERT(vehicle1.detectCollision(vehicle2));
}

TEST(vehicle_types) {
    // Create different vehicle types
    Car car(30.0);
    Bus bus(25.0);
    Truck truck(20.0, 5000.0);
    EmergencyVehicle ambulance(40.0);
    
    // Check types
    ASSERT(car.getType() == VehicleType::CAR);
    ASSERT(bus.getType() == VehicleType::BUS);
    ASSERT(truck.getType() == VehicleType::TRUCK);
    ASSERT(ambulance.getType() == VehicleType::EMERGENCY);
    
    // Check specialized behavior
    bus.loadPassengers(20);
    ASSERT(bus.getPassengerCount() == 20);
    
    truck.setCargoWeight(10000.0);
    ASSERT(truck.getCargoWeight() == 10000.0);
    
    ambulance.activateSiren(true);
    ASSERT(ambulance.isSirenActive());
}

int main() {
    std::cout << "Running Vehicle Tests\n";
    std::cout << "=====================\n";
    
    RUN_TEST(vehicle_creation);
    RUN_TEST(vehicle_movement);
    RUN_TEST(speed_limits);
    RUN_TEST(acceleration_and_deceleration);
    RUN_TEST(collision_detection);
    RUN_TEST(vehicle_types);
    
    std::cout << "\nAll tests passed!\n";
    return 0;
}