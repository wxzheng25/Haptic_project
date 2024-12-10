clear all 
clear;clc
% ������д��ڶ���
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end
tic
try
    % ��ʼ����������
    s = serial('COM5', 'BaudRate', 115200, 'Terminator', 'LF');
    fopen(s);
    disp('�����Ѵ򿪡�');

    
    % ����ȫ�ֱ������ڲ�׽������У׼
    global userPressedQ userPressedC initialOffset R_calib;
    userPressedQ = false;
    userPressedC = false;
    initialOffset = zeros(6,1);
    R_calib = eye(3);

    % ���� figure �� KeyPressFcn �Բ�׽����
    fig = figure('Name', 'Real-Time Arm Motion Visualization', 'NumberTitle', 'off', ...
        'KeyPressFcn', @figureKeyPress);

    % ��ʼУ׼
    performCalibration(s);

    % ��ʼ����ͼ
    hold on;
    grid on;
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('Real-Time Arm Motion Visualization');
    axis equal;
    view([-30, 30]); % �̶��ӽ�
    axis([-1.5 1.5 -1.5 1.5 -1 2]); % �����ͼ��Χ
    set(gca, 'Color', [0.9 0.9 0.9]); % ���ñ���ɫ
    set(gca, 'XDir', 'reverse'); % ���������˶�����

    % ��ӹ�Դ�Ͳ���
    camlight;
    lighting gouraud;
    material shiny;

    % ����۳�
    Lu = 0.3; % �ϱ۳��ȣ���λ��
    Lf = 0.25; % ǰ�۳��ȣ���λ��

    %����Ŀ������
    Pe_target = [Lu*sind(30); Lu*cosd(30); 0]; % Elbow target position
    alpha_target = -30-90; % ǰ�۷���Ƕȣ���λ�� (initial=-90,-)
    theta_target = 30; % ��ؽڽǶȣ���λ��(initial=)
%     
%     % ����Ŀ������
%     Pe_target =[Lu*cosd(30); 0; Lu*sind(30)]; % Elbow target position
%     alpha_target = 30-90; % ǰ�۷���Ƕȣ���λ�� (initial=-90)
%     theta_target = 30; % ��ؽڽǶȣ���λ��(initial=-)
    
    
    % ����Ŀ������λ�� Pw_target
    % �����ϱ۷����������Ӽ粿ָ���ⲿ��
    upper_arm_direction = Pe_target / norm(Pe_target);

    % ������ؽڽǶ� theta_target ��ǰ�۷���Ƕ� alpha_target������ǰ�۷�������
    % ���ȣ����ϱ�����ϵ�У�����ǰ�۷�������
    theta_rad = deg2rad(theta_target); % ���Ƕ�ת��Ϊ����
    alpha_rad = deg2rad(alpha_target);

    % ����ǰ�����ϱ�����ϵ�еķ���
    % ���ϱ۵� Y ����ת��ؽڽǶ� theta_rad��Ȼ��������� X ����ת alpha_rad
    R_elbow = axang2rotm([0 1 0 theta_rad]); % �� Y ����ת theta_rad
    R_forearm = axang2rotm([1 0 0 alpha_rad]); % �� X ����ת alpha_rad
   forearm_direction_local = R_forearm * R_elbow * [1; 0; 0];

    % ��ǰ�۷���ת����ȫ������ϵ
    % ���ȼ����ϱ۵���ת����
    Ru_target = vrrotvec2mat(vrrotvec([1; 0; 0], upper_arm_direction)); % ���ϱ۷�����뵽 Pe_target

    % ǰ�۷�����ȫ������ϵ�еķ�������
    forearm_direction_global = Ru_target * forearm_direction_local;

    % ����Ŀ������λ��
    Pw_target = Pe_target + Lf * (forearm_direction_global / norm(forearm_direction_global));

    %��ʼ����ͼ����
    shoulderPlot = plot3(0, 0, 0, 'kd', 'MarkerSize', 12, 'MarkerFaceColor', 'k'); % �粿���
    elbowPlot = plot3(0, 0, 0, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    wristPlot = plot3(0, 0, 0, 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    elbowTargetPlot = plot3(Pe_target(1), Pe_target(2), Pe_target(3), 'r*', 'MarkerSize', 12);
    wristTargetPlot = plot3(Pw_target(1), Pw_target(2), Pw_target(3), 'b*', 'MarkerSize', 12);
    armLine = plot3([0, 0], [0, 0], [0, 0], 'k-', 'LineWidth', 4);
    forearmLine = plot3([0, 0], [0, 0], [0, 0], 'b-', 'LineWidth', 4);

    % ��Ӻ�ɫʵ�ߣ�����Ŀ��ԭ�㡢��ؽ�Ŀ��λ�ú�����Ŀ��λ��
    targetLine = plot3([0, Pe_target(1), Pw_target(1)], [0, Pe_target(2), Pw_target(2)], [0, Pe_target(3), Pw_target(3)], 'k-', 'LineWidth', 2, 'LineStyle', '--');

    % ��ѭ����ʾ
    disp('Starting real-time plotting. Press ''q'' to exit, press ''c'' to recalibrate.');

    % ��ʼ��У׼�׶�
    calibrationStage = 1; % 1: У׼�ϱۣ�2: У׼ǰ�۷���Ƕȣ�3: У׼��ؽڽǶȣ�4: У׼���
    threshold_angle = 1; % �Ƕ���ֵ����λ��
    threshold_position = 0.01; % λ����ֵ����λ��

    % ��ѭ��
    while ishandle(fig) && ~userPressedQ
        % ����Ƿ���Ҫ����У׼
        if userPressedC
            performCalibration(s);
            userPressedC = false;
            disp('Recalibration complete.');
            calibrationStage = 1; % ���¿�ʼ�ϱ�У׼
        end

        % ��ȡ��������õ�����
        while s.BytesAvailable > 0
            line = fgetl(s); % ��ȡһ������
            data = sscanf(line, '%f,%f,%f,%f,%f,%f'); % ��������������

            if numel(data) == 6
                % ��������
                yaw1 = data(1) - initialOffset(1);
                pitch1 = data(2) - initialOffset(2);
                roll1 = data(3) - initialOffset(3);
                yaw2 = data(4) - initialOffset(4);
                pitch2 = data(5) - initialOffset(5);
                roll2 = data(6) - initialOffset(6);

                % ���Ƕ�ת��Ϊ����
                eul1 = deg2rad([yaw1, pitch1, roll1]); % �ϱ� IMU
                eul2 = deg2rad([yaw2, pitch2, roll2]); % ǰ�� IMU

                % ������ת����ZYX ˳��Yaw-Pitch-Roll��
                Ru = eul2rotm(eul2, 'ZYX');
                Rv = eul2rotm(eul1, 'ZYX');

                % Ӧ��У׼��ת����
                Ru_aligned = R_calib * Ru;
                Rv_aligned = R_calib * Rv;

                % �����ϱ۵��յ㣨�ⲿ��λ��
                Pe = Ru_aligned * [Lu; 0; 0];

                % ����ǰ�۵��յ㣨����λ��
                Pw = Pe + Rv_aligned * [Lf; 0; 0];

                % ���»�ͼ����
                set(elbowPlot, 'XData', Pe(1), 'YData', Pe(2), 'ZData', Pe(3));
                set(wristPlot, 'XData', Pw(1), 'YData', Pw(2), 'ZData', Pw(3));
                set(armLine, 'XData', [0, Pe(1)], 'YData', [0, Pe(2)], 'ZData', [0, Pe(3)]);
                set(forearmLine, 'XData', [Pe(1), Pw(1)], 'YData', [Pe(2), Pw(2)], 'ZData', [Pe(3), Pw(3)]);

                % ����Ŀ���ߵ�λ��
                set(targetLine, 'XData', [0, Pe_target(1), Pw_target(1)], 'YData', [0, Pe_target(2), Pw_target(2)], 'ZData', [0, Pe_target(3), Pw_target(3)]);

                % �����ϱۺ�ǰ�۵ķ�������
                upper_arm_vector = Ru_aligned * [1; 0; 0];
                forearm_vector = Rv_aligned * [1; 0; 0];

                % ������ؽڽǶȣ��ȣ������ϱۺ�ǰ��֮��ļн�
                cos_theta = dot(upper_arm_vector, forearm_vector) / (norm(upper_arm_vector) * norm(forearm_vector));
                theta_current = acosd(cos_theta);

                % **�޸Ĳ��ֿ�ʼ**
                % ����ǰ�۷���Ƕȣ�������������ǰ�۵� IMU
                eul_forearm = rotm2eul(Rv_aligned, 'ZYX');
                alpha_current = rad2deg(eul_forearm(3)); % �� X �����ת�Ƕ�
                % **�޸Ĳ��ֽ���**

                % ����У׼�׶ν��в���
                if calibrationStage == 1
                    % �����ⲿλ�ò�ֵ
                    diff_Pe = Pe - Pe_target;
                    disp(['Upper arm position difference: x=', num2str(diff_Pe(1)), ', y=', num2str(diff_Pe(2)), ', z=', num2str(diff_Pe(3))]);

                    % ��ʾ�û������ϱ�λ��
                    % �ϱ�У׼���ֵ����ӿ��Ʊ�����ԭ����һ��
                    % ��ʼ����У׼�ӽ׶Σ�x ��У׼��1��y ��У׼��2��
                    if ~exist('calibrationSubStage', 'var') || calibrationSubStage == 0
                        calibrationSubStage = 1;
                    end

                    % У׼�ϱ۵� x ��
                    if calibrationSubStage == 1
                        % ��ʾ��ʾ
                        if ~exist('textHandle', 'var')
                            textHandle = text(0, 0, 1.5, 'Calibrating Upper arm X-axis...', 'Color', 'r', 'FontSize', 14);
                        else
                            set(textHandle, 'String', 'Calibrating Upper arm X-axis...');
                        end

                        % ���ݲ�ֵ���򼤻��Ӧ������
                        if diff_Pe(1) > threshold_position
                            fwrite(s, 'r'); % �� x ����
                        elseif diff_Pe(1) < -threshold_position
                            fwrite(s, 'l'); % �� x ����
                        else
                            fwrite(s, 'n'); % ֹͣ��
                            disp('Upper arm X-axis calibration successful.');
                            calibrationSubStage = 2; % �л��� y ��У׼
                        end
                    end

                    % У׼�ϱ۵� y ��
                    if calibrationSubStage == 2
                        % ��ʾ��ʾ
                        set(textHandle, 'String', 'Calibrating Upper arm Y-axis...');

                        % ���ݲ�ֵ���򼤻��Ӧ������
                        if diff_Pe(3) > threshold_position
                            fwrite(s, 'x'); % �� y ����
                        elseif diff_Pe(3) < -threshold_position
                            fwrite(s, 'u'); % �� y ����
                        else
                            fwrite(s, 'n'); % ֹͣ��
                            disp('Upper arm Y-axis calibration successful.');
                            calibrationSubStage = 0; % �ӽ׶����
                            calibrationStage = 2; % ������һ��У׼�׶�
                            set(textHandle, 'String', 'Upper arm calibration complete!');
                        end
                    end
                    % ��¼��ʼ�Ƕ�
                    alpha_current_ini = alpha_current;
                    theta_current_ini = theta_current;
                elseif calibrationStage == 2
                    % ����ǰ�۷���Ƕ� ��
                     diff_alpha = alpha_current-alpha_current_ini - (-alpha_target-90);
                    disp(['Current forearm orientation angle �� = ', num2str(alpha_current-alpha_current_ini), '��, Target angle �� = ', num2str((-alpha_target-90)), '��, Difference = ', num2str(diff_alpha), '��']);

                    % ���ݲ�ֵ������ָ��� Arduino
                    if diff_alpha > threshold_angle
                        fwrite(s, 'e'); % ��ʱ����ת
                        disp('Please rotate your forearm counterclockwise.');
                    elseif diff_alpha < -threshold_angle
                        fwrite(s, 'c'); % ˳ʱ����ת
                        disp('Please rotate your forearm clockwise.');
                    else
                        fwrite(s, 'n'); % ֹͣ��ת
                        disp('Forearm orientation angle calibration successful.');
                        calibrationStage = 3; % ������һ��У׼�׶�
                    end

                elseif calibrationStage == 3
                    % ������ؽڽǶ� ��
                    diff_theta = theta_current - theta_current_ini - theta_target;
                    disp(['Current elbow joint angle �� = ', num2str(theta_current - theta_current_ini), '��, Target angle �� = ', num2str(theta_target), '��, Difference = ', num2str(diff_theta), '��']);

                    % ���ݲ�ֵ���򼤻�����
                    if abs(diff_theta) > threshold_angle
      
                        fwrite(s, 'm'); % �������ӣ������û�������ؽڽǶ�
                        disp('Please adjust your elbow joint angle.');
                    else
                        fwrite(s, 'n'); % ֹͣ��
                        disp('Elbow joint angle calibration successful.');
                        calibrationStage = 4; % У׼���
                        disp('Calibration complete!');
                        %%set(textHandle, 'String', 'Calibration complete!');
                        
                    end

                end
                disp(norm(Pw-Pw_target))
 
                % ����ͼ��
                drawnow limitrate;
            end
        end

        % С�ӳ��Լ��� CPU ����
        pause(0.001);
    end

    % ������Դ
    if exist('s', 'var') && strcmp(s.Status, 'open')
        fclose(s);
        delete(s);
    end
    clear s;
    disp('Program terminated.');
    toc

catch ME
    % �ڳ��ִ���ʱȷ�����ڱ��ر�
    if exist('s', 'var') && strcmp(s.Status, 'open')
        fclose(s);
        delete(s);
    end
    disp('Program terminated unexpectedly.');
    rethrow(ME);
end

% �ص���������
function figureKeyPress(~, event)
    global userPressedQ userPressedC;
    if strcmp(event.Key, 'q')
        userPressedQ = true;
    elseif strcmp(event.Key, 'c')
        userPressedC = true;
    end
end

% У׼��������
function performCalibration(s)
    global initialOffset R_calib;

    disp('Starting calibration...');
    disp('Please position your arm in the calibration position (arm extended horizontally, palm inward, IMU worn facing right), and remain still...');
    disp('Collecting data, please wait...');

    calibrationData = zeros(6,1);
    numSamples = 50; % �ɼ�50����������ƽ��
    samplesCollected = 0;

    while samplesCollected < numSamples
        if s.BytesAvailable > 0
            line = fgetl(s); % ��ȡһ������
            data = sscanf(line, '%f,%f,%f,%f,%f,%f'); % ��������������
            if numel(data) == 6
                calibrationData = calibrationData + data;
                samplesCollected = samplesCollected + 1;
            end
        end
        pause(0.01); % �ȴ�10ms
    end
    initialOffset = calibrationData / numSamples;
    disp('Calibration data collected:');
    disp(initialOffset);

    % ����У׼��ת����
    eul1_calib = deg2rad([initialOffset(1), initialOffset(2), initialOffset(3)]);
    Ru_calib = eul2rotm(eul1_calib, 'ZYX');

    % ������������ת�����ֱ�ˮƽ��ֱ���� X �ᣩ
    Ru_desired = eye(3); % ��λ����

    % ����У׼��ת����
    R_calib = Ru_desired * Ru_calib';

    disp('Calibration rotation matrix updated.');
end
