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

