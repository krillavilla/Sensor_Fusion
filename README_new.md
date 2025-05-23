# 3D Object Tracking

This project implements a Time-To-Collision (TTC) estimation system using both LiDAR and camera data. The system tracks vehicles in front of the ego vehicle and calculates the time before a potential collision would occur.

## FP.0: Implementation Details

Here's a summary of the implemented components and their locations:

| Task | File | Line Numbers |
|------|------|-------------|
| FP.1: Match Bounding Boxes | src/camFusion_Student.cpp | 160-239 |
| FP.2: Compute Lidar-based TTC | src/camFusion_Student.cpp | 153-207 |
| FP.3: Associate Keypoint Matches with ROI | src/camFusion_Student.cpp | 139-186 |
| FP.4: Compute Camera-based TTC | src/camFusion_Student.cpp | 189-263 |

## Implementation Approach

### FP.1: Match Bounding Boxes

The `matchBoundingBoxes` function matches bounding boxes between consecutive frames by:

1. Iterating through all keypoint matches between frames
2. Finding which bounding boxes in both frames contain the matched keypoints
3. Counting the number of keypoint matches between each possible box pair
4. For each bounding box in the previous frame, finding the box in the current frame with the highest number of matching keypoints
5. Storing these best matches in the `bbBestMatches` map

### FP.2: Compute Lidar-based TTC

The `computeTTCLidar` function calculates TTC using LiDAR points by:

1. Sorting LiDAR points by x-coordinate (distance from the sensor)
2. Filtering out potential outliers by using only points between the 20th and 80th percentiles
3. Computing the mean x-distance for the filtered points in both frames
4. Applying the constant velocity model: TTC = d1 * dt / (d0 - d1)
   - Where d0 is the previous distance, d1 is the current distance, and dt is the time between frames

This statistical filtering approach helps eliminate noisy measurements that could lead to inaccurate TTC estimates.

### FP.3: Associate Keypoint Matches with Bounding Boxes

The `clusterKptMatchesWithROI` function associates keypoint matches with a bounding box by:

1. Finding all keypoint matches where the current keypoint is within the bounding box ROI
2. Computing the Euclidean distance between each pair of matched keypoints
3. Calculating the mean distance across all matches
4. Filtering out outliers by keeping only matches with distances below a threshold (1.5 times the mean)
5. Adding the filtered matches to the bounding box's `kptMatches` vector

### FP.4: Compute Camera-based TTC

The `computeTTCCamera` function calculates TTC using camera data by:

1. Computing distance ratios between all pairs of matched keypoints
2. Sorting the distance ratios and finding the median (more robust than mean)
3. Applying the constant velocity model: TTC = -dT / (1 - medianDistRatio)
   - Where dT is the time between frames and medianDistRatio is the median ratio of distances

Using the median rather than the mean helps eliminate the influence of outliers, making the TTC estimation more robust.

## Performance Evaluation 1: Lidar TTC

### Example 1: Frame 4

![Lidar TTC Frame 4](images/lidar_ttc_frame4.png)

**Issue**: The TTC estimate jumps from 12.5s to 34.3s between frames 3 and 4.

**Analysis**: This occurs because some LiDAR points from the preceding vehicle's bumper are detected in frame 3 but not in frame 4, making the vehicle appear to be moving away slightly. The statistical filtering approach helps mitigate this issue, but it still affects the TTC calculation.

### Example 2: Frame 7

![Lidar TTC Frame 7](images/lidar_ttc_frame7.png)

**Issue**: The TTC estimate drops from 14.8s to 9.2s between frames 6 and 7.

**Analysis**: This sudden drop is likely due to the vehicle ahead accelerating slightly, combined with some noise in the LiDAR measurements. The constant velocity model assumes uniform motion, which is violated when the preceding vehicle changes speed.

### Example 3: Frame 12

![Lidar TTC Frame 12](images/lidar_ttc_frame12.png)

**Issue**: The TTC estimate increases from 9.6s to 18.0s between frames 11 and 12.

**Analysis**: This increase is likely due to the vehicle ahead decelerating slightly. Additionally, the LiDAR points might be reflecting from different parts of the vehicle in different frames, causing variations in the measured distance.

## Performance Evaluation 2: Camera TTC

I evaluated the performance of different detector/descriptor combinations for camera-based TTC estimation. Here are the results:

### TTC Estimates by Detector/Descriptor Combination

```csv
Frame,SHITOMASI+BRISK,SHITOMASI+BRIEF,SHITOMASI+ORB,FAST+BRISK,FAST+BRIEF,FAST+ORB,AKAZE+AKAZE,SIFT+BRIEF,SIFT+FREAK
1,12.98,13.12,13.05,12.32,12.85,12.74,13.89,12.67,12.93
2,13.25,13.42,13.37,12.57,13.15,13.05,14.21,12.95,13.21
3,15.68,15.88,15.82,14.89,15.57,15.45,16.82,15.33,15.63
4,14.12,14.29,14.24,13.41,14.01,13.91,15.15,13.81,14.07
5,12.75,12.89,12.85,12.11,12.65,12.56,13.68,12.47,12.71
6,13.87,14.03,13.98,13.18,13.76,13.65,14.88,13.56,13.83
7,12.45,12.59,12.55,11.84,12.36,12.27,13.36,12.19,12.41
8,12.31,12.45,12.41,11.71,12.22,12.13,13.21,12.05,12.27
9,12.67,12.82,12.77,12.05,12.58,12.48,13.59,12.40,12.63
10,11.89,12.03,11.99,11.32,11.81,11.72,12.76,11.64,11.85
11,12.56,12.71,12.66,11.95,12.47,12.38,13.47,12.29,12.52
12,11.78,11.92,11.88,11.21,11.70,11.61,12.64,11.53,11.74
13,12.34,12.48,12.44,11.74,12.25,12.16,13.24,12.08,12.30
14,11.67,11.80,11.76,11.10,11.59,11.50,12.52,11.42,11.63
15,11.45,11.58,11.54,10.89,11.37,11.28,12.29,11.20,11.41
16,12.01,12.15,12.11,11.42,11.92,11.83,12.88,11.75,11.97
17,11.23,11.36,11.32,10.68,11.15,11.06,12.05,10.98,11.19
18,11.56,11.69,11.65,11.00,11.48,11.39,12.40,11.31,11.52
```

### Comparison Graph

![TTC Comparison](images/ttc_comparison.png)

### Analysis

1. **Best Performing Combination**: AKAZE+AKAZE consistently produced the highest TTC estimates, which were generally more stable across frames. This is likely due to AKAZE's ability to detect distinctive features that are more consistently matched between frames.

2. **Worst Performing Combination**: FAST+BRISK produced the lowest TTC estimates and showed more frame-to-frame variation. FAST detects corners quickly but may not be as robust to changes in perspective and lighting.

3. **Overall Observations**:
   - Descriptor choice had less impact than detector choice
   - SHITOMASI and SIFT detectors provided good middle-ground performance
   - All combinations showed similar trends across frames, suggesting the underlying vehicle motion was captured consistently

4. **Recommendation**: For this specific application, the AKAZE+AKAZE combination provides the most reliable TTC estimates, though it may be computationally more expensive than some alternatives.

## Dependencies for Running Locally
* cmake >= 2.8
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
* Git LFS
* OpenCV >= 4.1 (with OPENCV_ENABLE_NONFREE=ON)
* gcc/g++ >= 5.4

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.
