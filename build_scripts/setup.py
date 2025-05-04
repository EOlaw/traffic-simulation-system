#!/usr/bin/env python3

from setuptools import setup, find_packages
import os
import sys

# Add python directory to path for package discovery
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

setup(
    name="traffic_simulation",
    version="@PROJECT_VERSION@",
    description="Traffic Simulation System",
    author="Your Name",
    author_email="your.email@example.com",
    url="https://github.com/yourusername/traffic-simulation-system",
    
    # Find all Python packages
    packages=find_packages(),
    
    # Include C++ extension module
    py_modules=["traffic_system"],
    
    # Dependencies
    install_requires=[
        "numpy>=1.20.0",
        "pandas>=1.3.0",
        "matplotlib>=3.4.0", 
        "PyQt5>=5.15.0",
    ],
    
    # Optional dependencies
    extras_require={
        "dev": [
            "pytest>=6.0.0",
            "pytest-cov>=2.10.0",
            "black>=22.0.0",
            "flake8>=4.0.0",
            "sphinx>=4.0.0",
        ],
    },
    
    # Console scripts
    entry_points={
        "console_scripts": [
            "traffic-sim=traffic_simulation.main:main",
        ],
    },
    
    # Package data
    include_package_data=True,
    package_data={
        "traffic_simulation": ["data/*"],
    },
    
    # Metadata
    keywords="traffic, simulation, transportation",
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: C++",
        "Topic :: Scientific/Engineering :: Visualization",
        "Topic :: Scientific/Engineering :: Information Analysis",
    ],
    
    # Python requirements
    python_requires=">=3.8",
)