function n_transforms = optimizeTransforms(t_transforms, im_size)
%% optimizeTransforms
% Solve L1-optimal camera path (CVPR'11 style)
% Inputs:
%   t_transforms : 1xN cell, each is 3x3 transform between adjacent frames
%   im_size      : [H W]
% Output:
%   n_transforms : 1xN cell, optimized transforms (3x3)

    % ---- CVX 环境（已安装即可；也可放在 main.m 里统一跑一次）----
    run('C:\Users\24832\Documents\cvx\cvx\cvx_setup.m');

    % ---- 帧间变换数量 ----
    n = numel(t_transforms);
    if n < 4
        error('Not enough transforms (n=%d). Need at least 4.', n);
    end

    % ---- 裁剪窗口（与论文一致的中心裁剪）----
    crop_ratio = 0.8;                % 0.7~0.9 越小放大越多
    center_x = round(im_size(2) / 2);
    center_y = round(im_size(1) / 2);
    crop_w = round(im_size(2) * crop_ratio);
    crop_h = round(im_size(1) * crop_ratio);
    crop_x = round(center_x - crop_w / 2);
    crop_y = round(center_y - crop_h / 2);
    crop_points = [crop_x           crop_y;
                   crop_x+crop_w    crop_y;
                   crop_x           crop_y+crop_h;
                   crop_x+crop_w    crop_y+crop_h];

    % ---- L1 代价权重（参考论文 Fig.8 的设置）----
    w1 = 10; 
    w2 = 1; 
    w3 = 100;

    % 平移与仿射项的权重（100:1），对应 [tx ty a11 a21 a12 a22]
    c1 = [1 1 100 100 100 100]';
    c2 = c1;
    c3 = c1;

    % ---- Proximity（相机模型接近性）约束的矩阵与上下界 ----
    % 使得仿射线性部分接近旋转+等比尺度，且剪切/非等比缩放受限
    U = zeros(6, 6);
    U(3, 1) = 1; U(6, 4) = 1;
    U(4, 2) = 1; U(5, 3) = 1;
    U(3, 5) = 1; U(6, 5) = -1;
    U(4, 6) = 1; U(5, 6) = 1;

    % 下上界（可按数据情况微调）
    lb = [0.9  -0.1  -0.1   0.9  -0.1  -0.05];
    ub = [1.1   0.1   0.1   1.1   0.1   0.05];

    % ---- CVX 优化 ----
    % p(k,:) = [tx ty a11 a21 a12 a22] 使得仿射矩阵：
    % B = [a11 a12 0; a21 a22 0; tx ty 1]
    cvx_begin quiet
        variable p(n, 6)
        variable e1(n, 6)
        variable e2(n, 6)
        variable e3(n, 6)

        % 目标函数：一阶/二阶/三阶 L1 差分的加权和
        minimize( w1*sum(e1*c1) + w2*sum(e2*c2) + w3*sum(e3*c3) )

        subject to
            % ---- 平滑性约束（一次/二次/三次差分的 L1 范数）----
            for k = 1:(n-3)
                % 当前与后续 3 帧的参数化仿射
                B_t  = [p(k,3)   p(k,5)   0;
                        p(k,4)   p(k,6)   0;
                        p(k,1)   p(k,2)   1];
                B_t1 = [p(k+1,3) p(k+1,5) 0;
                        p(k+1,4) p(k+1,6) 0;
                        p(k+1,1) p(k+1,2) 1];
                B_t2 = [p(k+2,3) p(k+2,5) 0;
                        p(k+2,4) p(k+2,6) 0;
                        p(k+2,1) p(k+2,2) 1];
                B_t3 = [p(k+3,3) p(k+3,5) 0;
                        p(k+3,4) p(k+3,6) 0;
                        p(k+3,1) p(k+3,2) 1];

                % 把原始相机路径（t_transforms）对 B_t* 做约束
                R0 = t_transforms{k+1} * B_t1 - B_t;    % 一阶
                R1 = t_transforms{k+2} * B_t2 - B_t1;   % 二阶
                R2 = t_transforms{k+3} * B_t3 - B_t2;   % 三阶

                % 抽成 [tx ty a11 a21 a12 a22] 顺序
                r0 = [R0(3,1) R0(3,2) R0(1,1) R0(2,1) R0(1,2) R0(2,2)];
                r1 = [R1(3,1) R1(3,2) R1(1,1) R1(2,1) R1(1,2) R1(2,2)];
                r2 = [R2(3,1) R2(3,2) R2(1,1) R2(2,1) R2(1,2) R2(2,2)];

                % L1 松弛：|r| <= e
                -e1(k,:) <= r0 <= e1(k,:);
                -e2(k,:) <= (r1 - r0) <= e2(k,:);
                -e3(k,:) <= (r2 - 2*r1 + r0) <= e3(k,:);
            end

            % ---- 末端约束（避免最后若干帧的自由漂移）----
            for k = max(1, n-3):n
                p(k,6) == p(n,6);
            end

            % ---- 非负性约束 ----
            for k = 1:n
                e1(k,:) >= 0;
                e2(k,:) >= 0;
                e3(k,:) >= 0;
            end

            % ---- Proximity 约束（限制剪切/非等比）----
            for k = 1:n
                lb <= p(k,:) * U <= ub;
            end

            % ---- Inclusion 约束（裁剪窗口落在图像内）----
            for i = 1:4
                for k = 1:n
                    0 <= [1 0 crop_points(i,1) crop_points(i,2) 0 0] * p(k,:)' <= im_size(2);
                    0 <= [0 1 0 0 crop_points(i,1) crop_points(i,2)] * p(k,:)' <= im_size(1);
                end
            end
    cvx_end

    % ---- 将参数形式转换为 3x3 仿射矩阵列表 ----
    n_transforms = parToMat(p);
end
