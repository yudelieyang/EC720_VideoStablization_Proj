function evaluate_stabilization(origVideo, stabVideo, varargin)
% Evaluate jitter reduction between original and stabilized videos.
% Metrics:
%  - RMS jitter reduction (camera path residual, pixel units)
%  - (Optional) Corner displacement RMS reduction (image space)
%
% Usage:
%   evaluate_stabilization('orig.mp4','stab.mp4','Win',31,'Step',2);

% ---------- Args ----------
p = inputParser;
addParameter(p,'Win',31,@(x)isnumeric(x)&&mod(x,2)==1 && x>=5);   % detrend window
addParameter(p,'Step',1,@(x)isnumeric(x)&&x>=1);                  % frame step
addParameter(p,'CornerMetric',true,@islogical);                   % corner RMS metric
parse(p,varargin{:});
Win   = p.Results.Win;
Step  = p.Results.Step;
useCorner = p.Results.CornerMetric;

% ---------- Read videos ----------
vr1 = VideoReader(origVideo);
vr2 = VideoReader(stabVideo);

% Align lengths
N = min(floor(vr1.Duration*vr1.FrameRate), floor(vr2.Duration*vr2.FrameRate));
N = floor(N/Step)*Step; % make divisible by Step
assert(N>=30,'Video too short for evaluation.');

% ---------- Estimate paths (translation) ----------
[path1, sz] = estimatePath(vr1, N, Step);
[path2, ~ ] = estimatePath(vr2, N, Step);

% ---------- Detrend (moving average or Savitzky-Golay) ----------
smooth1 = movmean(path1, Win, 1);
smooth2 = movmean(path2, Win, 1);

res1 = path1 - smooth1;              % original residual (jitter)
res2 = path2 - smooth2;              % stabilized residual (remaining jitter)

rms1 = rms(vecnorm(res1,2,2));
rms2 = rms(vecnorm(res2,2,2));
reduction_path = (1 - rms2/max(rms1,eps))*100;
% ---------- Per-frame RMS (jitter magnitude per frame) ----------
perFrameRMS_orig = vecnorm(res1, 2, 2);   % 每一帧的 sqrt(dx^2 + dy^2)
perFrameRMS_stab = vecnorm(res2, 2, 2);

frames = (0:size(res1,1)-1)*Step;

figure('Name','Per-frame RMS jitter');
plot(frames, perFrameRMS_orig, 'r-', ...
     frames, perFrameRMS_stab, 'g-', 'LineWidth', 1);
grid on;
xlabel('Frame index');
ylabel('Per-frame RMS jitter (px)');
legend('Original','Stabilized','Location','best');
title('Per-frame RMS of camera-path residuals');

fprintf('--- Path-space RMS jitter ---\n');
fprintf('Original RMS:   %.3f px\n', rms1);
fprintf('Stabilized RMS: %.3f px\n', rms2);
fprintf('Reduction:      %.2f %%\n', reduction_path);

% ---------- Optional: corner displacement RMS metric ----------
if useCorner
    corners = [1 1; sz(2) 1; sz(2) sz(1); 1 sz(1)]; % [x y]: TL, TR, BR, BL
    crms1 = cornerRMS(vr1, N, Step, corners);
    crms2 = cornerRMS(vr2, N, Step, corners);
    reduction_corner = (1 - crms2/max(crms1,eps))*100;
    fprintf('\n--- Image-space corner RMS ---\n');
    fprintf('Original:   %.3f px\n', crms1);
    fprintf('Stabilized: %.3f px\n', crms2);
    fprintf('Reduction:  %.2f %%\n', reduction_corner);
end

% ---------- Plots ----------
t = (0:size(path1,1)-1)*Step;
figure('Name','Camera Path (translation)');
subplot(2,1,1);
plot(t, path1(:,1),'k-', t, smooth1(:,1),'r--', t, res1(:,1),'b:','LineWidth',1);
xlabel('Frame'); ylabel('X (px)'); legend('orig','trend','residual'); grid on; title('Original Path');
subplot(2,1,2);
plot(t, path2(:,1),'k-', t, smooth2(:,1),'r--', t, res2(:,1),'b:','LineWidth',1);
xlabel('Frame'); ylabel('X (px)'); legend('stab','trend','residual'); grid on; title('Stabilized Path');

figure('Name','Residual Norm');
plot(t, vecnorm(res1,2,2),'r-', t, vecnorm(res2,2,2),'g-','LineWidth',1);
xlabel('Frame'); ylabel('||residual|| (px)'); legend('orig','stab'); grid on; title(sprintf('RMS Reduction: %.2f%%', reduction_path));

end

% ===== Helper: estimate per-frame translation path via similarity model =====
function [path, sz] = estimatePath(vr, N, Step)
% Returns T-by-2 path of cumulative translations [tx, ty] in pixels.
% Uses built-in features + RANSAC similarity estimation and accumulates transforms.
%reset(vr);
f1 = readFrame(vr); sz = size(f1);
gray1 = prepGray(f1);
Tcum = eye(3);
tx = zeros(floor(N/Step),1); ty = tx;

idx = 1;
for t = 1:Step:N-Step
    f2 = readFrame(vr);
    for s = 2:Step, if hasFrame(vr), f2 = readFrame(vr); end, end
    gray2 = prepGray(f2);

    % Feature detect & match (FAST+BRIEF or SURF fallback)
    [pts1, desc1] = detectAndExtract(gray1);
    [pts2, desc2] = detectAndExtract(gray2);
    if size(desc1,1)<10 || size(desc2,1)<10
        % fallback to previous translation
        % keep Tcum unchanged
    else
        idxPairs = matchFeatures(desc1, desc2, 'MaxRatio',0.8,'Unique',true,'MatchThreshold',100);
        if size(idxPairs,1)>=6
            mp1 = pts1(idxPairs(:,1),:);
            mp2 = pts2(idxPairs(:,2),:);
            try
                tform = estimateGeometricTransform2D(mp2, mp1, 'similarity', ...
                    'MaxDistance', 3, 'MaxNumTrials', 2000, 'Confidence', 99);
                T = tform.T; % maps frame2 -> frame1
                % Accumulate to global (compose)
                Tcum = T * Tcum;
            catch
                % keep previous Tcum
            end
        end
    end
    tx(idx) = Tcum(3,1);
    ty(idx) = Tcum(3,2);
    idx = idx + 1;

    gray1 = gray2;
end
path = [tx ty];
end

% ===== Helper: grayscale & smoothing =====
function g = prepGray(f)
if size(f,3)==3, g = rgb2gray(f); else, g = f; end
g = im2single(g);
end

% ===== Helper: detect + extract with built-ins =====
function [pts, desc] = detectAndExtract(I)
% Try FAST+BRIEF (fast), fallback to SURF if needed.
try
    ptsLoc = detectFASTFeatures(I, 'MinContrast',0.03, 'MinQuality',0.01);
    ptsLoc = ptsLoc.selectStrongest(min(1500, ptsLoc.Count));
    [desc, validPts] = extractFeatures(I, ptsLoc, 'Method','BRISK');
    pts = validPts.Location;
    if size(desc,1)<50
        error('Too few BRISK features, fallback to SURF');
    end
catch
    ptsLoc = detectSURFFeatures(I, 'MetricThreshold',300);
    ptsLoc = ptsLoc.selectStrongest(min(1500, ptsLoc.Count));
    [desc, validPts] = extractFeatures(I, ptsLoc); % SURF descriptors
    pts = validPts.Location;
end
end

% ===== Helper: corner displacement RMS =====
function crms = cornerRMS(vr, N, Step, corners)
%reset(vr);
prev = readFrame(vr);
prevG = im2single(rgb2grayIfNeeded(prev));
traj = zeros(floor(N/Step), size(corners,1));
idx = 1;
Hacc = eye(3);
for t = 1:Step:N-Step
    fr = readFrame(vr);
    for s = 2:Step, if hasFrame(vr), fr = readFrame(vr); end, end
    currG = im2single(rgb2grayIfNeeded(fr));

    [p1,d1] = detectAndExtract(prevG);
    [p2,d2] = detectAndExtract(currG);
    idxPairs = matchFeatures(d1,d2,'MaxRatio',0.8,'Unique',true,'MatchThreshold',100);
    if size(idxPairs,1)>=6
        mp1 = p1(idxPairs(:,1),:);
        mp2 = p2(idxPairs(:,2),:);
        try
            tform = estimateGeometricTransform2D(mp2, mp1, 'projective', ...
                'MaxDistance', 3,'MaxNumTrials',2000,'Confidence',99);
            Hacc = tform.T * Hacc; % accumulate homography
        catch
        end
    end

    c = [corners, ones(size(corners,1),1)];
    cWarp = (Hacc * c')';
    cWarp = cWarp(:,1:2) ./ cWarp(:,3);
    if idx==1
        traj(idx,:) = 0;
    else
        traj(idx,:) = vecnorm(cWarp - cPrev, 2, 2);
    end
    cPrev = cWarp;
    prevG  = currG;
    idx = idx + 1;
end
crms = rms(traj,'all');
end

function g = rgb2grayIfNeeded(f)
if size(f,3)==3, g = rgb2gray(f); else, g = f; end
end
