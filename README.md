# Matlab-Python Wrapper for Dynamixel

## Important Note

**Make sure to set your USB-to-Serial device's LATENCY TIMER to 1 ms.**  
For proper and responsive Dynamixel communication, the latency timer (often found as a setting for FTDI or similar USB-serial chipsets) must be set to 1 ms. Failure to do so may result in significant communication delays or timeouts.

## Overview

This repository provides a simple wrapper to allow control of Dynamixel motors from both MATLAB and Python environments. You can choose to use the Python implementation or interact with Dynamixel motors directly from MATLAB using a class interface.

## Files

- **Dynamixel.py**: Python program to control Dynamixel motors. This file can also be used independently in any Python environment.
- **PyDynamixel.m**: MATLAB class file that acts as a wrapper for the Python code, allowing MATLAB scripts to interface with Dynamixel motors.
- **test_PyDynamixel.m**: MATLAB sample script demonstrating how to use the MATLAB class.

## Usage

### Python

To control Dynamixel motors directly from Python, simply run or import `Dynamixel.py`.

```bash
python Dynamixel.py
```
or use its classes/functions in your own Python program.

### MATLAB

To control Dynamixel motors from MATLAB:

1. Make sure your MATLAB environment has access to Python and the required dependencies for Dynamixel.
2. Run `test_PyDynamixel.m` for a sample usage of the MATLAB class interface.

```matlab
test_PyDynamixel
```

## Requirements

- Python (with access to Dynamixel SDK for Python)
- MATLAB (with Python integration enabled)

## License

This project is licensed under the MIT license.
