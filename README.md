# LiDAR 3D Obstacle Detection – Point Cloud Processing

This repository contains a **C++ implementation of a LiDAR-based 3D obstacle detection pipeline** developed using the **Point Cloud Library (PCL)**. The project processes raw LiDAR point cloud data to detect and visualize obstacles in a simulated urban driving environment.

The system demonstrates core **3D perception techniques** used in **autonomous driving**, **robotics**, and **environment understanding**, with key algorithms implemented from scratch.

**Author:** Bhagyath Badduri

---

## 🎥 Demo – LiDAR Obstacle Detection

![LiDAR Obstacle Detection](lidar_obstacle_detection.gif)

---

## 📌 Project Overview

- End-to-end **LiDAR perception pipeline**
- Real-time obstacle detection in a city-block simulation
- Ground plane separation and obstacle segmentation
- 3D bounding box visualization for detected objects
- Custom implementations of core perception algorithms

---

## 🧠 Technical Approach

**Point Cloud Processing**
- Voxel grid filtering to downsample raw point clouds
- Region of interest cropping to remove irrelevant data

**Segmentation**
- Ground plane extraction using a **custom RANSAC algorithm**
- Separation of road surface and obstacle points

**Clustering**
- **KD-tree–based Euclidean clustering** for obstacle grouping
- Individual cluster extraction for detected objects

**Visualization**
- Axis-aligned **3D bounding boxes**
- Real-time visualization using **PCL Visualizer**

---

## 🛠️ Technologies Used

- C++
- Point Cloud Library (PCL)
- Eigen
- CMake
- LiDAR point cloud data (PCD files)

---

## Build & Run

**Requirements**
- C++11 or later  
- CMake ≥ 3.5  
- PCL (Point Cloud Library)  
- Eigen  

**Build and Execute**
```bash
mkdir build
cd build
cmake ..
make
./environment
