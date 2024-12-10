clear all;
clear; clc;
tic;
% 清除所有串口对象
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

try
    % 初始化串口连接
    s = serial('COM5', 'BaudRate', 115200, 'Terminator', 'LF');
    fopen(s);
    disp('串口已打开。');

    % 定义全局变量用于捕捉按键和校准
    global userPressedQ userPressedC initialOffset R_calib;
    userPressedQ = false;
    userPressedC = false;
    initialOffset = zeros(6,1);
    R_calib = eye(3);

    % 设置 figure 的 KeyPressFcn 以捕捉按键
    fig = figure('Name', 'Forearm Orientation Test', 'NumberTitle', 'off', ...
        'KeyPressFcn', @figureKeyPress);

    % 初始校准
    performCalibration(s);

    % 定义目标 α 角度（度）
    alpha_targets = [0, -30, -15, -35,45,10]; % 您可以根据需要调整这些值
    % 定义目标 α 角度（度）
    % alpha_targets = [0, -45, 40, -5,35,10]; % 您可以根据需要调整这些值
    % 角度差异阈值（度）
    threshold_alpha = 5;

    % 主测试循环，对于每个目标 α 角度
    for idx = 1:length(alpha_targets)
        alpha_target = alpha_targets(idx);
        disp(['正在测试目标前臂方向角度 α = ', num2str(alpha_target), '°']);

        % 初始化变量
        testComplete = false;

        while ishandle(fig) && ~userPressedQ && ~testComplete
            % 检查是否需要重新校准
            if userPressedC
                performCalibration(s);
                userPressedC = false;
                disp('重新校准完成。');
            end

            % 读取并处理可用的数据
            while s.BytesAvailable > 0
                line = fgetl(s); % 读取一行数据
                data = sscanf(line, '%f,%f,%f,%f,%f,%f'); % 解析六个浮点数

                if numel(data) == 6
                    % 解析数据
                    yaw1 = data(1) - initialOffset(1);
                    pitch1 = data(2) - initialOffset(2);
                    roll1 = data(3) - initialOffset(3);
                    yaw2 = data(4) - initialOffset(4);
                    pitch2 = data(5) - initialOffset(5);
                    roll2 = data(6) - initialOffset(6);

                    % 将角度转换为弧度
                    eul1 = deg2rad([yaw1, pitch1, roll1]); % 上臂 IMU
                    eul2 = deg2rad([yaw2, pitch2, roll2]); % 前臂 IMU

                    % 计算旋转矩阵（ZYX 顺序：Yaw-Pitch-Roll）
                    Ru = eul2rotm(eul2, 'ZYX');
                    Rv = eul2rotm(eul1, 'ZYX');

                    % 应用校准旋转矩阵
                    Ru_aligned = R_calib * Ru;
                    Rv_aligned = R_calib * Rv;

                    % 计算前臂方向角度 α
                    eul_forearm = rotm2eul(Rv_aligned, 'ZYX');
                    alpha_current = rad2deg(eul_forearm(3)); % 绕 X 轴的旋转角度

                    % 计算当前 α 与目标 α 的差异
                    diff_alpha = alpha_target - alpha_current;

                    % 实时输出当前 α 角度和差异
                    disp(['当前 α = ', num2str(alpha_current), '°, 目标 α = ', num2str(alpha_target), '°, 差异 = ', num2str(diff_alpha), '°']);

                    % 检查差异是否在阈值内
                    if abs(diff_alpha) <= threshold_alpha
                        disp('前臂方向角度已在阈值范围内。');
                        testComplete = true;
                        break; % 退出内层 while 循环
                    else
                        % 向用户提供反馈（例如，通过串口通信）
                        % 您可以根据需要向 Arduino 发送命令
                        % 例如，发送 'e' 表示逆时针旋转，'c' 表示顺时针旋转
                        if diff_alpha > 0
                            fwrite(s, 'c'); % 用户需要顺时针旋转
                        else
                            fwrite(s, 'e'); % 用户需要逆时针旋转
                        end
                    end

                    % 如果需要，更新图形
                    drawnow limitrate;
                end
            end

            % 小延迟以减轻 CPU 负担
            pause(0.01);
        end

        if userPressedQ
            break; % 退出 for 循环
        end

        disp(['α = ', num2str(alpha_target), '° 的测试完成。']);

        % 在下一次测试前稍作停顿
        pause(1);
    end

    disp('所有测试已完成。');

    % 清理资源
    if exist('s', 'var') && strcmp(s.Status, 'open')
        fclose(s);
        delete(s);
    end
    clear s;
    disp('程序已结束。');

catch ME
    % 在出现错误时确保串口被关闭
    if exist('s', 'var') && strcmp(s.Status, 'open')
        fclose(s);
        delete(s);
    end
    disp('程序异常终止。');
    rethrow(ME);
end
toc;
% 按键回调函数
function figureKeyPress(~, event)
    global userPressedQ userPressedC;
    if strcmp(event.Key, 'q')
        userPressedQ = true;
    elseif strcmp(event.Key, 'c')
        userPressedC = true;
    end
end

% 校准函数
function performCalibration(s)
    global initialOffset R_calib;

    disp('开始校准...');
    disp('请将手臂置于校准位置（手臂水平伸直，手掌朝内，IMU 佩戴面朝右），并保持静止...');
    disp('正在收集数据，请稍候...');

    calibrationData = zeros(6,1);
    numSamples = 50; % 采集50个样本进行平均
    samplesCollected = 0;

    while samplesCollected < numSamples
        if s.BytesAvailable > 0
            line = fgetl(s); % 读取一行数据
            data = sscanf(line, '%f,%f,%f,%f,%f,%f'); % 解析六个浮点数
            if numel(data) == 6
                calibrationData = calibrationData + data;
                samplesCollected = samplesCollected + 1;
            end
        end
        pause(0.01); % 等待10ms
    end
    initialOffset = calibrationData / numSamples;
    disp('校准数据已收集：');
    disp(initialOffset);

    % 计算校准旋转矩阵
    eul1_calib = deg2rad([initialOffset(1), initialOffset(2), initialOffset(3)]);
    Ru_calib = eul2rotm(eul1_calib, 'ZYX');

    % 定义期望的旋转矩阵（手臂水平伸直，沿 X 轴）
    Ru_desired = eye(3); % 单位矩阵

    % 计算校准旋转矩阵
    R_calib = Ru_desired * Ru_calib';

    disp('校准旋转矩阵已更新。');
end

