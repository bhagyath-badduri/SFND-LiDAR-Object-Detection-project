# LiDAR 3D Obstacle Detection – Point Cloud Processing Pipeline

This repository implements a complete **LiDAR-based 3D obstacle detection pipeline** using C++ and the **Point Cloud Library (PCL)**.

The system processes raw LiDAR point cloud data to **segment the ground plane, detect obstacles, and generate 3D bounding boxes** in a simulated urban driving environment.

This project represents a core component of a **perception and sensor fusion pipeline** used in autonomous driving and robotics for environment understanding.

---

## 🎥 Demo – LiDAR Obstacle Detection

![LiDAR Obstacle Detection](lidar_obstacle_detection.gif)

The demo shows a city-block scenario where LiDAR data is processed in real time to detect obstacles such as vehicles and objects on the road.

---

## 📌 Project Overview

In autonomous systems, LiDAR provides precise 3D spatial information but requires processing to extract meaningful structure.

This project implements an end-to-end perception pipeline that:

- processes raw LiDAR point clouds
- separates ground from obstacles
- groups obstacle points into clusters
- estimates object boundaries using 3D bounding boxes
- visualizes results in real time

---

## Sensor Fusion Context

This project is part of a broader **sensor fusion and perception stack**, where LiDAR is used to:

- provide accurate 3D geometry of the environment  
- complement camera-based perception  
- enable object detection independent of lighting conditions  

The obstacle detection output can be integrated with:

- tracking systems (e.g., Kalman Filter / UKF)
- camera-based object detection
- motion prediction modules

---

## 🧠 Technical Approach

The pipeline consists of the following stages:

---

### 1️⃣ Point Cloud Filtering

Raw LiDAR data is first preprocessed to reduce noise and computational cost.

- **Voxel Grid Filtering**  
  Downsamples the point cloud by grouping points into voxels and replacing them with centroids.

- **Region of Interest (ROI) Cropping**  
  Removes points outside the relevant driving area to focus on useful data.

---

### 2️⃣ Ground Plane Segmentation

The road surface is separated from obstacles using a **custom RANSAC algorithm**.

- Random samples are selected to estimate a plane model
- Points close to the plane are classified as ground
- Remaining points are treated as obstacles

👉 This step is critical for isolating objects above the road surface.

---

### 3️⃣ Obstacle Clustering

Obstacle points are grouped into individual objects using:

- **KD-tree for efficient nearest-neighbor search**
- **Euclidean clustering algorithm**

Each cluster corresponds to a potential obstacle such as a vehicle or object.

---

### 4️⃣ Bounding Box Estimation

For each detected cluster:

- Axis-aligned **3D bounding boxes** are generated
- Bounding boxes represent object size and position

This allows downstream systems to:

- estimate object dimensions
- perform tracking
- assess collision risk

---

### 5️⃣ Visualization

The processed results are visualized using **PCL Visualizer**:

- Ground plane and obstacles are color-coded
- Clusters are displayed individually
- Bounding boxes highlight detected objects

---

## Results

The system successfully detects and localizes obstacles in a simulated urban environment.

### Key observations:

- Ground plane segmentation effectively removes road points  
- Clustering accurately groups object points  
- Bounding boxes provide clear spatial representation of obstacles  
- Filtering significantly reduces noise and improves stability  

This demonstrates the effectiveness of classical LiDAR processing pipelines for real-time perception.

---

## What This Project Demonstrates

This project demonstrates practical understanding of:

- LiDAR point cloud processing  
- 3D environment perception  
- RANSAC-based plane fitting  
- KD-tree and spatial search  
- Euclidean clustering  
- Real-time obstacle detection  
- 3D bounding box estimation  

---

## 🧰 Tools and Environment

- **C++**
- **Point Cloud Library (PCL)**
- **Eigen**
- **CMake**
- LiDAR point cloud data (PCD files)

