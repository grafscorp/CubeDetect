# CubeDetect
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


## Problems

- Can't work with multiple cubes
- When the background is loaded, the results are distorted
- There may be artifacts in the nude during the video.
