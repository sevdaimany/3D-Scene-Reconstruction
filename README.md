
# 3D Scene Reconstruction

This project is the final lab evaluation for the 3D Vision for Multiple and Moving Cameras (3DVMMC) module at Universidad Autónoma de Madrid. The goal is to perform a full 3D reconstruction pipeline on a real-world object or scene using multiple views, starting with camera calibration, then keypoint matching, and finally producing a 3D point cloud.
## objective
To reconstruct a 3D scene from multiple views using techniques and toolboxes developed throughout the 3DVMMC course, involving:

- Camera calibration
- Feature detection and matching
- Fundamental matrix estimation
- Projective and Euclidean reconstruction


---

## 🧪 Session 1: Camera Calibration

**Objective**: Estimate the intrinsic matrix (A) of a chosen camera.

### ✅ Steps:

1. **Calibration with a digital checkerboard**:
   - Used a 1080px checkerboard displayed on a monitor.
   - Physical size on screen: **193.5 x 193.5 mm**
   - Captured image resolution: **960 × 1280 pixels**
   - Intrinsic matrix A:
     ```
     | 1041.31  -12.19   504.94 |
     |    0     1045.72  630.84 |
     |    0        0        1   |
     ```

2. **Geometric analysis**:
   - **Pixel squareness**: 0.42% deviation → nearly square pixels
   - **Principal point vs. image center**: Shifted ~10px horizontally, ~15px vertically
   - **Skew (axis orthogonality)**: Slight skew with value -12.19

3. **Physical calibration pattern**:
   - Used a square paper (240×240 mm) with 9 marked points
   - Intrinsic matrix A′:
     ```
     | 1098.16  -21.15   472.46 |
     |    0     1102.63  615.88 |
     |    0        0       1    |
     ```
   - Results consistent with digital calibration; differences due to manual point selection and pattern accuracy

---

## 🔍 Session 2: Feature Detection & Matching

**Objective**: Detect and match local features across multiple views using different detectors and matching algorithms.

### ✅ Scene Capture

- Captured 8 views of a 3-object scene under two lighting conditions:
  - **Natural daylight**
  - **Artificial lamp light** (to challenge feature robustness)

### ✅ Algorithms Evaluated:

- **Detectors**: MATLAB SIFT, VLFeat SIFT
- **Matchers**: MATLAB `matchFeatures`, VLFeat `vl_ubcmatch`, custom NNDR matcher
- **Evaluation metrics**: Inlier ratios for both Fundamental Matrix (F) and Homography (H)

### 🔢 Example Evaluation Results:

| Setup | Inliers (F) | Inlier Ratio (F) | Inliers (H) | Inlier Ratio (H) |
|-------|-------------|------------------|-------------|------------------|
| VLFeat + NNDR (0.2) | 214 | 0.926 | 206 | 0.891 |
| MATLAB + NNDR (0.2) | 78  | 0.918 | 72  | 0.847 |

- **Best Setup**: **VLFeat SIFT + Custom NNDR (Threshold: 0.2)**  
- **Validation Tools**: `vgg_gui_F.m` for visual epipolar geometry verification
- **Homography results**: Confirmed alignment on planar surfaces, highlighting 3D parallax limits

---

## 🧱 Session 3: 3D Reconstruction

**Objective**: Reconstruct a 3D point cloud from matched features using projective and Euclidean techniques.

### ✅ Pipeline Steps:

1. **Multi-view point matching**:
   - Used `n_view_matching` to identify consistent features across multiple views

2. **Initial projective reconstruction**:
   - Estimated Fundamental Matrix from 2 views using 8-point algorithm
   - Built initial 3D structure with projective geometry

3. **Resectioning and Projective Bundle Adjustment**:
   - Refined structure by optimizing all camera parameters and 3D point positions
   - Reprojection error improved:
     - Initial reconstruction: **10314**
     - After adding more views (before optimization): **36141**
     - After Bundle Adjustment: **1457**

4. **Euclidean Upgrade**:
   - Used intrinsic matrix from Session 1 to compute Essential Matrix
   - Upgraded projective structure to Euclidean using two calibrated views

5. **Euclidean Resection**:
   - Estimated Euclidean projection matrices for all cameras
   - Residual reprojection error after resectioning: **962**

6. **Final Euclidean Bundle Adjustment**:
   - Joint optimization of camera parameters and 3D points in Euclidean space
   - Final reprojection error: **very close to zero**, confirming accurate geometry





## 🖼️ Visual Results – Session 3: 3D Reconstruction

This section provides screenshots that illustrate the outcomes of the 3D reconstruction pipeline.

### 🔹 Reconstructed Scene – Real Object

Below is the result of the full Euclidean 3D reconstruction using the captured images of the real-world scene. 

<p float="left">
  <img src="https://github.com/sevdaimany/3D-Scene-Reconstruction/blob/master/screenshots/reconstructed_1.png" width="150" />
  <img src="https://github.com/sevdaimany/3D-Scene-Reconstruction/blob/master/screenshots/reconstructed_2.png" width="150" /> 
</p>

### 🔹 Toy Data Example – Cube Reconstruction

To better visualize the effectiveness and structure of the reconstruction process, a synthetic dataset of a cube was also reconstructed. 

![Cube Reconstruction](https://github.com/sevdaimany/3D-Scene-Reconstruction/blob/master/screenshots/reconstructed_3.png)

### 🔹 Additional Suggested Screenshots



- **Matched Features Between Views**  

  ![Feature Matches](https://github.com/sevdaimany/3D-Scene-Reconstruction/blob/master/screenshots/section3_initial.jpg)

- **Epipolar Geometry Visualization**  
  Screenshot from `vgg_gui_F.m` showing epipolar line and matching accuracy.  
  ![Epipolar Geometry](https://github.com/sevdaimany/3D-Scene-Reconstruction/blob/master/screenshots/vgg_gui_result.png)


## 🧠 Insights & Learnings

This project was a comprehensive application of multi-view geometry concepts in computer vision. Below is a summary of the most important ideas and methods applied:

### 🎯 Camera Calibration (Session 1)

- **Intrinsic Parameters**: Describes the internal geometry of the camera, including focal length, principal point, pixel skew, and pixel aspect ratio. These are captured in the **intrinsic matrix A**.
- **Zhang's Calibration Method**: A technique that uses multiple views of a known planar pattern (e.g., a checkerboard) to estimate A by solving a set of linear and non-linear equations using homographies.

### 🔍 Feature Detection and Matching (Session 2)

- **Local Features**: Keypoints like corners or blobs are extracted using SIFT detector. These points are robust to changes in scale and rotation.
- **Descriptors**: Compact representations of local image patches (e.g., SIFT descriptors) allow comparing features across images.
- **Matching Strategies**: Used nearest neighbor ratio (NNDR), custom thresholds, and VLFeat/MATLAB toolboxes to match keypoints.


### 🧱 3D Reconstruction (Session 3)

- **Homography vs. Fundamental Matrix**:
  - **Homography** models a planar transformation between views.
  - **Fundamental Matrix (F)** Encodes the **epipolar geometry** between two views of a general 3D scene. It relates corresponding points in two images using **epipolar lines**: if a point exists in one image, its match must lie along a specific line in the other.
  

- **Epipolar Geometry**: Encodes the geometric relationship between two calibrated views. Each point in one image maps to an epipolar line in the other. The **Fundamental Matrix F** models this, while the **Essential Matrix E** is used in the Euclidean case (with known intrinsics).
- **Projective Reconstruction**: Reconstructs scene geometry from two or more views without using any metric information. This is up to a projective transformation.
- **Resectioning**: The process of computing camera projection matrices (P) given known 3D points and their image correspondences. Used in both projective and Euclidean steps.
- **Bundle Adjustment**: A nonlinear optimization that refines camera parameters and 3D points to minimize reprojection error. 
- **Upgrading to Euclidean**: Using the known intrinsic matrix, we convert the projective reconstruction into a Euclidean one, allowing for metric measurements (like angles and lengths).

