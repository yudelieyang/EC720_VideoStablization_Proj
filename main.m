%% L1Stabilizer main.m (Windows version for example4_car_input)
run('C:\Users\24832\Documents\vlfeat-0.9.21-bin.tar\vlfeat-0.9.21-bin\vlfeat-0.9.21\toolbox\vl_setup.m');
run('C:\Users\24832\Documents\cvx\cvx\cvx_setup.m');
clear;

% ===== 参数区 =====
frame_dir = 'C:\Users\24832\Documents\MATLAB\L1Stabilizer-master\L1Stabilizer-master\frames\';      % 输入帧目录
out_dir   = 'C:\Users\24832\Documents\MATLAB\L1Stabilizer-master\L1Stabilizer-master\frames_opt\';  % 输出帧目录
im_size   = [360 640];     % 与 readImages 中的 resize 保持一致
crop_ratio = 0.8;          % 越小放大越多，黑边更少
num_frames = Inf;          % 让读取函数自动使用目录中的全部帧

if ~exist(out_dir,'dir'), mkdir(out_dir); end

% ===== 读取帧序列 =====
im_array = readImages(frame_dir, num_frames);
fprintf('Loaded %d frames from %s\n', numel(im_array), frame_dir);
if numel(im_array) < 5
    error('Need at least 5 frames, got %d.', numel(im_array));
end

% ===== 提取 SIFT 特征 =====
[features, descriptors] = extractSIFT(im_array);
fprintf('SIFT sets: %d (should equal frames)\n', numel(features));

% ===== 获取原始相机路径 =====
t_transforms = getTransforms(im_array, features, descriptors);
fprintf('Got %d transforms (expected frames-1 = %d)\n', numel(t_transforms), max(0, numel(im_array)-1));

if numel(t_transforms) < 4
    error('Not enough transforms (n=%d). Need at least 4. Check frames & features.', numel(t_transforms));
end

% ===== 优化平滑相机路径 =====
n_transforms = optimizeTransforms(t_transforms, im_size);

% ===== 可视化轨迹对比 =====
plotPath(t_transforms, n_transforms);

% ===== 按新轨迹重新映射并裁剪 =====
n_im_array = applyOptimizedTransforms(im_array, n_transforms);
fprintf('Stabilized frames generated: %d\n', numel(n_im_array));

% ===== 保存稳定后帧 =====
Nout = numel(n_im_array);              % 用实际数量，避免越界
for k = 1:Nout
    file_name = fullfile(out_dir, sprintf('%05d.jpg', k));
    imwrite(n_im_array{k}, file_name);
end

disp('✅ 所有稳定帧已保存到 frames_opt 文件夹中。');
disp('接下来请用 ffmpeg 合成为稳定视频:');
disp('ffmpeg -framerate 30 -i frames_opt\\%05d.jpg -c:v libx264 -pix_fmt yuv420p example4_car_input_L1Stabilized.mp4');
