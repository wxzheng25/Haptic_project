clear all 
clear;clc
% 清除所有串口对象
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end
tic
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
    fig = figure('Name', 'Real-Time Arm Motion Visualization', 'NumberTitle', 'off', ...
        'KeyPressFcn', @figureKeyPress);

    % 初始校准
    performCalibration(s);

    % 初始化绘图
    hold on;
    grid on;
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('Real-Time Arm Motion Visualization');
    axis equal;
    view([-30, 30]); % 固定视角
    axis([-1.5 1.5 -1.5 1.5 -1 2]); % 增大绘图范围
    set(gca, 'Color', [0.9 0.9 0.9]); % 设置背景色
    set(gca, 'XDir', 'reverse'); % 修正左右运动方向

    % 添加光源和材质
    camlight;
    lighting gouraud;
    material shiny;

    % 定义臂长
    Lu = 0.3; % 上臂长度，单位米
    Lf = 0.25; % 前臂长度，单位米

    %定义目标输入
    Pe_target = [Lu*sind(30); Lu*cosd(30); 0]; % Elbow target position
    alpha_target = -30-90; % 前臂方向角度，单位度 (initial=-90,-)
    theta_target = 30; % 肘关节角度，单位度(initial=)
%     
%     % 定义目标输入
%     Pe_target =[Lu*cosd(30); 0; Lu*sind(30)]; % Elbow target position
%     alpha_target = 30-90; % 前臂方向角度，单位度 (initial=-90)
%     theta_target = 30; % 肘关节角度，单位度(initial=-)
    
    
    % 计算目标手腕位置 Pw_target
    % 计算上臂方向向量（从肩部指向肘部）
    upper_arm_direction = Pe_target / norm(Pe_target);

    % 根据肘关节角度 theta_target 和前臂方向角度 alpha_target，计算前臂方向向量
    % 首先，在上臂坐标系中，定义前臂方向向量
    theta_rad = deg2rad(theta_target); % 将角度转换为弧度
    alpha_rad = deg2rad(alpha_target);

    % 计算前臂在上臂坐标系中的方向
    % 绕上臂的 Y 轴旋转肘关节角度 theta_rad，然后绕自身的 X 轴旋转 alpha_rad
    R_elbow = axang2rotm([0 1 0 theta_rad]); % 绕 Y 轴旋转 theta_rad
    R_forearm = axang2rotm([1 0 0 alpha_rad]); % 绕 X 轴旋转 alpha_rad
   forearm_direction_local = R_forearm * R_elbow * [1; 0; 0];

    % 将前臂方向转换到全局坐标系
    % 首先计算上臂的旋转矩阵
    Ru_target = vrrotvec2mat(vrrotvec([1; 0; 0], upper_arm_direction)); % 将上臂方向对齐到 Pe_target

    % 前臂方向在全局坐标系中的方向向量
    forearm_direction_global = Ru_target * forearm_direction_local;

    % 计算目标手腕位置
    Pw_target = Pe_target + Lf * (forearm_direction_global / norm(forearm_direction_global));

    %初始化绘图对象
    shoulderPlot = plot3(0, 0, 0, 'kd', 'MarkerSize', 12, 'MarkerFaceColor', 'k'); % 肩部标记
    elbowPlot = plot3(0, 0, 0, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    wristPlot = plot3(0, 0, 0, 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    elbowTargetPlot = plot3(Pe_target(1), Pe_target(2), Pe_target(3), 'r*', 'MarkerSize', 12);
    wristTargetPlot = plot3(Pw_target(1), Pw_target(2), Pw_target(3), 'b*', 'MarkerSize', 12);
    armLine = plot3([0, 0], [0, 0], [0, 0], 'k-', 'LineWidth', 4);
    forearmLine = plot3([0, 0], [0, 0], [0, 0], 'b-', 'LineWidth', 4);

    % 添加黑色实线，连接目标原点、肘关节目标位置和手腕目标位置
    targetLine = plot3([0, Pe_target(1), Pw_target(1)], [0, Pe_target(2), Pw_target(2)], [0, Pe_target(3), Pw_target(3)], 'k-', 'LineWidth', 2, 'LineStyle', '--');

    % 主循环提示
    disp('Starting real-time plotting. Press ''q'' to exit, press ''c'' to recalibrate.');

    % 初始化校准阶段
    calibrationStage = 1; % 1: 校准上臂，2: 校准前臂方向角度，3: 校准肘关节角度，4: 校准完成
    threshold_angle = 1; % 角度阈值，单位度
    threshold_position = 0.01; % 位置阈值，单位米

    % 主循环
    while ishandle(fig) && ~userPressedQ
        % 检查是否需要重新校准
        if userPressedC
            performCalibration(s);
            userPressedC = false;
            disp('Recalibration complete.');
            calibrationStage = 1; % 重新开始上臂校准
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

                % 计算上臂的终点（肘部）位置
                Pe = Ru_aligned * [Lu; 0; 0];

                % 计算前臂的终点（手腕）位置
                Pw = Pe + Rv_aligned * [Lf; 0; 0];

                % 更新绘图对象
                set(elbowPlot, 'XData', Pe(1), 'YData', Pe(2), 'ZData', Pe(3));
                set(wristPlot, 'XData', Pw(1), 'YData', Pw(2), 'ZData', Pw(3));
                set(armLine, 'XData', [0, Pe(1)], 'YData', [0, Pe(2)], 'ZData', [0, Pe(3)]);
                set(forearmLine, 'XData', [Pe(1), Pw(1)], 'YData', [Pe(2), Pw(2)], 'ZData', [Pe(3), Pw(3)]);

                % 更新目标线的位置
                set(targetLine, 'XData', [0, Pe_target(1), Pw_target(1)], 'YData', [0, Pe_target(2), Pw_target(2)], 'ZData', [0, Pe_target(3), Pw_target(3)]);

                % 计算上臂和前臂的方向向量
                upper_arm_vector = Ru_aligned * [1; 0; 0];
                forearm_vector = Rv_aligned * [1; 0; 0];

                % 计算肘关节角度（θ），即上臂和前臂之间的夹角
                cos_theta = dot(upper_arm_vector, forearm_vector) / (norm(upper_arm_vector) * norm(forearm_vector));
                theta_current = acosd(cos_theta);

                % **修改部分开始**
                % 计算前臂方向角度（α），仅基于前臂的 IMU
                eul_forearm = rotm2eul(Rv_aligned, 'ZYX');
                alpha_current = rad2deg(eul_forearm(3)); % 绕 X 轴的旋转角度
                % **修改部分结束**

                % 根据校准阶段进行操作
                if calibrationStage == 1
                    % 计算肘部位置差值
                    diff_Pe = Pe - Pe_target;
                    disp(['Upper arm position difference: x=', num2str(diff_Pe(1)), ', y=', num2str(diff_Pe(2)), ', z=', num2str(diff_Pe(3))]);

                    % 提示用户调整上臂位置
                    % 上臂校准部分的振子控制保持与原函数一致
                    % 初始设置校准子阶段（x 轴校准：1，y 轴校准：2）
                    if ~exist('calibrationSubStage', 'var') || calibrationSubStage == 0
                        calibrationSubStage = 1;
                    end

                    % 校准上臂的 x 轴
                    if calibrationSubStage == 1
                        % 显示提示
                        if ~exist('textHandle', 'var')
                            textHandle = text(0, 0, 1.5, 'Calibrating Upper arm X-axis...', 'Color', 'r', 'FontSize', 14);
                        else
                            set(textHandle, 'String', 'Calibrating Upper arm X-axis...');
                        end

                        % 根据差值方向激活对应的振子
                        if diff_Pe(1) > threshold_position
                            fwrite(s, 'r'); % 正 x 方向
                        elseif diff_Pe(1) < -threshold_position
                            fwrite(s, 'l'); % 负 x 方向
                        else
                            fwrite(s, 'n'); % 停止振动
                            disp('Upper arm X-axis calibration successful.');
                            calibrationSubStage = 2; % 切换到 y 轴校准
                        end
                    end

                    % 校准上臂的 y 轴
                    if calibrationSubStage == 2
                        % 显示提示
                        set(textHandle, 'String', 'Calibrating Upper arm Y-axis...');

                        % 根据差值方向激活对应的振子
                        if diff_Pe(3) > threshold_position
                            fwrite(s, 'x'); % 正 y 方向
                        elseif diff_Pe(3) < -threshold_position
                            fwrite(s, 'u'); % 负 y 方向
                        else
                            fwrite(s, 'n'); % 停止振动
                            disp('Upper arm Y-axis calibration successful.');
                            calibrationSubStage = 0; % 子阶段完成
                            calibrationStage = 2; % 进入下一个校准阶段
                            set(textHandle, 'String', 'Upper arm calibration complete!');
                        end
                    end
                    % 记录初始角度
                    alpha_current_ini = alpha_current;
                    theta_current_ini = theta_current;
                elseif calibrationStage == 2
                    % 调整前臂方向角度 α
                     diff_alpha = alpha_current-alpha_current_ini - (-alpha_target-90);
                    disp(['Current forearm orientation angle α = ', num2str(alpha_current-alpha_current_ini), '°, Target angle α = ', num2str((-alpha_target-90)), '°, Difference = ', num2str(diff_alpha), '°']);

                    % 根据差值方向发送指令给 Arduino
                    if diff_alpha > threshold_angle
                        fwrite(s, 'e'); % 逆时针旋转
                        disp('Please rotate your forearm counterclockwise.');
                    elseif diff_alpha < -threshold_angle
                        fwrite(s, 'c'); % 顺时针旋转
                        disp('Please rotate your forearm clockwise.');
                    else
                        fwrite(s, 'n'); % 停止旋转
                        disp('Forearm orientation angle calibration successful.');
                        calibrationStage = 3; % 进入下一个校准阶段
                    end

                elseif calibrationStage == 3
                    % 调整肘关节角度 θ
                    diff_theta = theta_current - theta_current_ini - theta_target;
                    disp(['Current elbow joint angle θ = ', num2str(theta_current - theta_current_ini), '°, Target angle θ = ', num2str(theta_target), '°, Difference = ', num2str(diff_theta), '°']);

                    % 根据差值方向激活振子
                    if abs(diff_theta) > threshold_angle
      
                        fwrite(s, 'm'); % 激活振子，引导用户调整肘关节角度
                        disp('Please adjust your elbow joint angle.');
                    else
                        fwrite(s, 'n'); % 停止振动
                        disp('Elbow joint angle calibration successful.');
                        calibrationStage = 4; % 校准完成
                        disp('Calibration complete!');
                        %%set(textHandle, 'String', 'Calibration complete!');
                        
                    end

                end
                disp(norm(Pw-Pw_target))
 
                % 更新图形
                drawnow limitrate;
            end
        end

        % 小延迟以减轻 CPU 负担
        pause(0.001);
    end

    % 清理资源
    if exist('s', 'var') && strcmp(s.Status, 'open')
        fclose(s);
        delete(s);
    end
    clear s;
    disp('Program terminated.');
    toc

catch ME
    % 在出现错误时确保串口被关闭
    if exist('s', 'var') && strcmp(s.Status, 'open')
        fclose(s);
        delete(s);
    end
    disp('Program terminated unexpectedly.');
    rethrow(ME);
end

% 回调函数定义
function figureKeyPress(~, event)
    global userPressedQ userPressedC;
    if strcmp(event.Key, 'q')
        userPressedQ = true;
    elseif strcmp(event.Key, 'c')
        userPressedC = true;
    end
end

% 校准函数定义
function performCalibration(s)
    global initialOffset R_calib;

    disp('Starting calibration...');
    disp('Please position your arm in the calibration position (arm extended horizontally, palm inward, IMU worn facing right), and remain still...');
    disp('Collecting data, please wait...');

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
    disp('Calibration data collected:');
    disp(initialOffset);

    % 计算校准旋转矩阵
    eul1_calib = deg2rad([initialOffset(1), initialOffset(2), initialOffset(3)]);
    Ru_calib = eul2rotm(eul1_calib, 'ZYX');

    % 定义期望的旋转矩阵（手臂水平伸直，沿 X 轴）
    Ru_desired = eye(3); % 单位矩阵

    % 计算校准旋转矩阵
    R_calib = Ru_desired * Ru_calib';

    disp('Calibration rotation matrix updated.');
end
