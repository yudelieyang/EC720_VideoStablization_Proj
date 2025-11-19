# Video Stabilization
### Prerequisites


```
ffmpeg
matlab
cvx
vlfeat
```

### How-To

1. Extract frames to a directory.


```
    ffmpeg -i test.mp4 frames/out-%05d.jpg
```

2. Change vlfeat and cvx directory paths in `main.m` and `optimizeAffineTransforms.m`.
3. Change original and output directory for frames in `main.m`.
4. Run `main.m`.
5. Merge frames.
```
    bash create_video.sh
```


## Method Overview

This repository implements the L1-optimal camera path smoothing algorithm
by Grundmann et al. (CVPR 2011), using:
- VLFeat SIFT for feature detection and matching,
- RANSAC-based similarity motion estimation between consecutive frames,
- CVX to solve the L1-regularized path smoothing problem,
- FFmpeg for frame extraction and video reconstruction.

Note: We do **not** implement the full probabilistic Kalman-filter-based
method of Litvin et al. (2003); that paper is only used for comparison
and discussion in our report.


