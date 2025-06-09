# Unitree L2 Python/Windows Compatible Tools

This repository contains a set of Python tools for capturing, processing, and visualizing 3D LiDAR data over UDP from the Unitree 4D L2 LiDAR. At the time of me developing this, Unitree had tools available for Linux written in C/C++, but I needed something for Windows (and it would have been nice to have it in Python). This is more or less a refactor plus some additional bells and whistles for visualization.

It is very important to note that these scripts will only decode packets if your sensor is set to: 3D point packet type, no IMU, normal mode (not negative angle). If you deviate, it is likely that the scripts will not correctly decode incoming packets. 

## decode_lidar_3d.py

A utility module that decodes raw LiDAR packets from UDP data. It parses the binary packet format, extracting header information, calibration parameters, and point data. This module is used by the other scripts and provides the core functionality for interpreting the LiDAR sensor data. You can pass packets into this function and do whatever you want with what it returns.

## main_3d.py

A real-time visualization tool that connects to a LiDAR sensor over UDP, receives data packets, decodes them using decode_lidar_3d.py, and displays the 3D point cloud in real-time using matplotlib. Points persist for a configurable duration to create a continuous visualization of the environment. This is nice to get a sanity check if you're scanning anything, but I suggest the save.py into unpack_save.py route.

## save.py

A data capture utility that connects to a LiDAR sensor, collects data for a specified duration, and saves the processed point cloud data to a pickle file. It includes filtering capabilities to limit the field of view to a specific angular range.

## unpack_save.py

A post-processing tool that reads the pickle file created by save.py and can export the data to different formats:
- Generates a PLY file for use with 3D visualization software (I just use Blender, which can also be scripted with Python)
- Optionally can export to JSON format

## Network Info

The default network configuration is set for a LiDAR sensor at IPv4 192.168.1.62:6101 with the host at 192.168.1.2:6201. Host mask 255.255.255.0, gateway 192.168.1.1, if prompted you can set your DNS to anything you prefer.
