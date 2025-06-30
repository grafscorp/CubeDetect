# CubeDetect
![cube](https://github.com/user-attachments/assets/7d408a88-2a63-4ef1-aed7-7ae633035304)

This project is a solution for detecting a cube from an image using C++, OpenCV. 
The algorithm identifies cube vertices, edges, and faces.

## Key Features
- Advanced edge detection with adaptive thresholding
- Line clustering for identifying cube edges
- Vertex detection at line intersections
- Geometric verification to filter false positives
- 3D cube visualization with semi-transparent faces
- Shadow compensation for reliable detection in low-light areas
## Requirements
- C++17 compatible compiler
- CMake 3.10+
- OpenCV 4.0+
## Installation
```bash
# Clone the repository
git clone https://github.com/grafscorp/CubeDetect.git
cd CubeDetect

# Create build directory
mkdir build && cd build

# Configure and build
cmake ..
cmake --build . --config Release
```
## Usage
```bash
CubeDetect path/to/image/or/video
```
## Sample results

<div style="text-align:center">
  <img src="https://github.com/user-attachments/assets/c4821cc8-88d5-436a-8a19-ade2b9df7459" height="300" width="300" />
  <img src="https://github.com/user-attachments/assets/77df5497-f8a8-447a-a9d7-ee4c912eee99" height="300" width="300" />
</div>


<div style="text-align:center">
  <img src="https://github.com/user-attachments/assets/6654479a-92f3-4e9e-aea1-62e7848f5af2" height="300" width="300" />
  <img src="https://github.com/user-attachments/assets/fb1e47f8-9220-48c3-8c51-601eac6cc2e5" height="300" width="300" />
</div>
<div style="text-align:center">
  <img src="https://github.com/user-attachments/assets/4914b6d8-15fb-4f1b-85ff-4c6ef6e8f4d8" height="300" width="300" />
  <img src="https://github.com/user-attachments/assets/46f56322-a5ab-40e1-be94-19cbda22a20d" height="300" width="300" />
</div>


## Problems


- Can't work with multiple cubes
- When the background is loaded, the results are distorted
- There may be artifacts in the nude during the video.
