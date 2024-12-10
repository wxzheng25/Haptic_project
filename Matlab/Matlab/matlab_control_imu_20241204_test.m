clear all;
clear; clc;
tic;
% ������д��ڶ���
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

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
    fig = figure('Name', 'Forearm Orientation Test', 'NumberTitle', 'off', ...
        'KeyPressFcn', @figureKeyPress);

    % ��ʼУ׼
    performCalibration(s);

    % ����Ŀ�� �� �Ƕȣ��ȣ�
    alpha_targets = [0, -30, -15, -35,45,10]; % �����Ը�����Ҫ������Щֵ
    % ����Ŀ�� �� �Ƕȣ��ȣ�
    % alpha_targets = [0, -45, 40, -5,35,10]; % �����Ը�����Ҫ������Щֵ
    % �ǶȲ�����ֵ���ȣ�
    threshold_alpha = 5;

    % ������ѭ��������ÿ��Ŀ�� �� �Ƕ�
    for idx = 1:length(alpha_targets)
        alpha_target = alpha_targets(idx);
        disp(['���ڲ���Ŀ��ǰ�۷���Ƕ� �� = ', num2str(alpha_target), '��']);

        % ��ʼ������
        testComplete = false;

        while ishandle(fig) && ~userPressedQ && ~testComplete
            % ����Ƿ���Ҫ����У׼
            if userPressedC
                performCalibration(s);
                userPressedC = false;
                disp('����У׼��ɡ�');
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

                    % ����ǰ�۷���Ƕ� ��
                    eul_forearm = rotm2eul(Rv_aligned, 'ZYX');
                    alpha_current = rad2deg(eul_forearm(3)); % �� X �����ת�Ƕ�

                    % ���㵱ǰ �� ��Ŀ�� �� �Ĳ���
                    diff_alpha = alpha_target - alpha_current;

                    % ʵʱ�����ǰ �� �ǶȺͲ���
                    disp(['��ǰ �� = ', num2str(alpha_current), '��, Ŀ�� �� = ', num2str(alpha_target), '��, ���� = ', num2str(diff_alpha), '��']);

                    % �������Ƿ�����ֵ��
                    if abs(diff_alpha) <= threshold_alpha
                        disp('ǰ�۷���Ƕ�������ֵ��Χ�ڡ�');
                        testComplete = true;
                        break; % �˳��ڲ� while ѭ��
                    else
                        % ���û��ṩ���������磬ͨ������ͨ�ţ�
                        % �����Ը�����Ҫ�� Arduino ��������
                        % ���磬���� 'e' ��ʾ��ʱ����ת��'c' ��ʾ˳ʱ����ת
                        if diff_alpha > 0
                            fwrite(s, 'c'); % �û���Ҫ˳ʱ����ת
                        else
                            fwrite(s, 'e'); % �û���Ҫ��ʱ����ת
                        end
                    end

                    % �����Ҫ������ͼ��
                    drawnow limitrate;
                end
            end

            % С�ӳ��Լ��� CPU ����
            pause(0.01);
        end

        if userPressedQ
            break; % �˳� for ѭ��
        end

        disp(['�� = ', num2str(alpha_target), '�� �Ĳ�����ɡ�']);

        % ����һ�β���ǰ����ͣ��
        pause(1);
    end

    disp('���в�������ɡ�');

    % ������Դ
    if exist('s', 'var') && strcmp(s.Status, 'open')
        fclose(s);
        delete(s);
    end
    clear s;
    disp('�����ѽ�����');

catch ME
    % �ڳ��ִ���ʱȷ�����ڱ��ر�
    if exist('s', 'var') && strcmp(s.Status, 'open')
        fclose(s);
        delete(s);
    end
    disp('�����쳣��ֹ��');
    rethrow(ME);
end
toc;
% �����ص�����
function figureKeyPress(~, event)
    global userPressedQ userPressedC;
    if strcmp(event.Key, 'q')
        userPressedQ = true;
    elseif strcmp(event.Key, 'c')
        userPressedC = true;
    end
end

% У׼����
function performCalibration(s)
    global initialOffset R_calib;

    disp('��ʼУ׼...');
    disp('�뽫�ֱ�����У׼λ�ã��ֱ�ˮƽ��ֱ�����Ƴ��ڣ�IMU ����泯�ң��������־�ֹ...');
    disp('�����ռ����ݣ����Ժ�...');

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
    disp('У׼�������ռ���');
    disp(initialOffset);

    % ����У׼��ת����
    eul1_calib = deg2rad([initialOffset(1), initialOffset(2), initialOffset(3)]);
    Ru_calib = eul2rotm(eul1_calib, 'ZYX');

    % ������������ת�����ֱ�ˮƽ��ֱ���� X �ᣩ
    Ru_desired = eye(3); % ��λ����

    % ����У׼��ת����
    R_calib = Ru_desired * Ru_calib';

    disp('У׼��ת�����Ѹ��¡�');
end

