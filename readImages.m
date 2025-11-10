function images_array = readImages(image_dir, maxFrames)
% Robust reader: jpg/jpeg/png, natural sort, optional frame cap.

    % 收集所有图像
    files = [dir(fullfile(image_dir,'*.jpg')); ...
             dir(fullfile(image_dir,'*.jpeg')); ...
             dir(fullfile(image_dir,'*.png'))];

    if isempty(files)
        error('No images found in %s (jpg/jpeg/png).', image_dir);
    end

    % ---- 自然排序（按文件名中的数字排序）----
    names = {files.name}';
    tokens = regexp(names,'\d+','match');
    key = zeros(numel(names),1);
    for i=1:numel(names)
        if ~isempty(tokens{i})
            key(i) = str2double(tokens{i}{end});
        else
            key(i) = i; % 无数字的按出现顺序
        end
    end
    [~, idx] = sortrows([key, (1:numel(key))']);  % 数字相同按原序
    files = files(idx);

    % ---- 帧数上限 ----
    if nargin < 2 || isempty(maxFrames) || isinf(maxFrames) || maxFrames<=0
        maxFrames = numel(files);
    else
        maxFrames = min(maxFrames, numel(files));
    end

    % 预分配并读取
    images_array = cell(maxFrames,1);
    for k = 1:maxFrames
        full_name = fullfile(image_dir, files(k).name);
        fprintf('Now reading %s\n', full_name);
        im = imread(full_name);
        % 保持你原来的尺寸设定（若要跟 im_size 保持一致可改成参数传入）
        im = imresize(im, [360 640]);
        images_array{k} = im;
    end
end
