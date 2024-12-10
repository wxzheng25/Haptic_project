clear all 
clear;clc
% Clear all serial objects
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end
tic
try
    % Initialize serial connection
    s = serial('COM5', 'BaudRate', 115200, 'Terminator', 'LF');
    fopen(s);
    disp('Serial port opened.');
    
    
    % Define global variables for capturing key presses and calibration
    global userPressedQ userPressedC initialOffset R_calib;
    userPressedQ = false;
    userPressedC = false;
    initialOffset = zeros(6,1);
    R_calib = eye(3);
    
    % Set figure's KeyPressFcn to capture key presses
    fig = figure('Name', 'Real-Time Arm Motion Visualization', 'NumberTitle', 'off', ...
        'KeyPressFcn', @figureKeyPress);
    
    % Initial calibration
    performCalibration(s);
    
    % Initialize plotting
    hold on;
    grid on;
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('Real-Time Arm Motion Visualization');
    axis equal;
    view([-30, 30]); % Fixed view angle
    axis([-1.5 1.5 -1.5 1.5 -1 2]); % Increase plotting range
    set(gca, 'Color', [0.9 0.9 0.9]); % Set background color
    set(gca, 'XDir', 'reverse'); % Correct left-right movement direction
    
    % Add light and material properties
    camlight;
    lighting gouraud;
    material shiny;
    
    % Define arm lengths
    Lu = 0.3; % Upper arm length in meters
    Lf = 0.25; % Forearm length in meters
    
%     % Define target input
%     Pe_target = [Lu*sind(30); Lu*cosd(30); 0]; % Elbow target position
%     alpha_target = -30-90; % Forearm direction angle in degrees (initial=-90,-)
%     theta_target = 30; % Elbow joint angle in degrees (initial=)
%     
    % Define target input
    Pe_target =[Lu*cosd(30); 0; Lu*sind(30)]; % Elbow target position
    alpha_target = 30-90; % Forearm direction angle in degrees (initial=-90)
    theta_target = 45; % Elbow joint angle in degrees (initial=-)
    
    
    % Calculate target wrist position Pw_target
    % Calculate upper arm direction vector (from shoulder to elbow)
    upper_arm_direction = Pe_target / norm(Pe_target);
    
    % Calculate forearm direction vector based on elbow joint angle theta_target and forearm direction angle alpha_target
    % First, define the forearm direction vector in the upper arm coordinate system
    theta_rad = deg2rad(theta_target); % Convert angle to radians
    alpha_rad = deg2rad(alpha_target);
    
    % Calculate forearm direction in the upper arm coordinate system
    % Rotate around the upper arm's Y axis by theta_rad, then rotate around its own X axis by alpha_rad
    R_elbow = axang2rotm([0 1 0 theta_rad]); % Rotate around Y axis by theta_rad
    R_forearm = axang2rotm([1 0 0 alpha_rad]); % Rotate around X axis by alpha_rad
    forearm_direction_local = R_forearm * R_elbow * [1; 0; 0];
    
    % Convert forearm direction to global coordinate system
    % First, calculate the rotation matrix of the upper arm
    Ru_target = vrrotvec2mat(vrrotvec([1; 0; 0], upper_arm_direction)); % Align upper arm direction to Pe_target
    
    % Forearm direction vector in the global coordinate system
    forearm_direction_global = Ru_target * forearm_direction_local;
    
    % Calculate target wrist position
    Pw_target = Pe_target + Lf * (forearm_direction_global / norm(forearm_direction_global));
    
    % Initialize plot objects
    shoulderPlot = plot3(0, 0, 0, 'kd', 'MarkerSize', 12, 'MarkerFaceColor', 'k'); % Shoulder marker
    elbowPlot = plot3(0, 0, 0, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    wristPlot = plot3(0, 0, 0, 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    elbowTargetPlot = plot3(Pe_target(1), Pe_target(2), Pe_target(3), 'r*', 'MarkerSize', 12);
    wristTargetPlot = plot3(Pw_target(1), Pw_target(2), Pw_target(3), 'b*', 'MarkerSize', 12);
    armLine = plot3([0, 0], [0, 0], [0, 0], 'k-', 'LineWidth', 4);
    forearmLine = plot3([0, 0], [0, 0], [0, 0], 'b-', 'LineWidth', 4);
    
    % Add black dashed line connecting origin, elbow target position, and wrist target position
    targetLine = plot3([0, Pe_target(1), Pw_target(1)], [0, Pe_target(2), Pw_target(2)], [0, Pe_target(3), Pw_target(3)], 'k-', 'LineWidth', 2, 'LineStyle', '--');
    
    % Main loop prompt
    disp('Starting real-time plotting. Press ''q'' to exit, press ''c'' to recalibrate.');
    
    % Initialize calibration stage
    calibrationStage = 1; % 1: Calibrate upper arm, 2: Calibrate forearm direction angle, 3: Calibrate elbow joint angle, 4: Calibration complete
    threshold_angle = 5; % Angle threshold in degrees
    threshold_position = 0.03; % Position threshold in meters
    
    % Main loop
    while ishandle(fig) && ~userPressedQ
        % Check if recalibration is needed
        if userPressedC
            performCalibration(s);
            userPressedC = false;
            disp('Recalibration complete.');
            calibrationStage = 1; % Restart upper arm calibration
        end
    
        % Read and process available data
        while s.BytesAvailable > 0
            line = fgetl(s); % Read a line of data
            data = sscanf(line, '%f,%f,%f,%f,%f,%f'); % Parse six floating-point numbers
    
            if numel(data) == 6
                % Parse data
                yaw1 = data(1) - initialOffset(1);
                pitch1 = data(2) - initialOffset(2);
                roll1 = data(3) - initialOffset(3);
                yaw2 = data(4) - initialOffset(4);
                pitch2 = data(5) - initialOffset(5);
                roll2 = data(6) - initialOffset(6);
    
                % Convert angles to radians
                eul1 = deg2rad([yaw1, pitch1, roll1]); % Upper arm IMU
                eul2 = deg2rad([yaw2, pitch2, roll2]); % Forearm IMU
    
                % Calculate rotation matrices (ZYX order: Yaw-Pitch-Roll)
                Ru = eul2rotm(eul2, 'ZYX');
                Rv = eul2rotm(eul1, 'ZYX');
    
                % Apply calibration rotation matrices
                Ru_aligned = R_calib * Ru;
                Rv_aligned = R_calib * Rv;
    
                % Calculate the end position of the upper arm (elbow)
                Pe = Ru_aligned * [Lu; 0; 0];
    
                % Calculate the end position of the forearm (wrist)
                Pw = Pe + Rv_aligned * [Lf; 0; 0];
    
                % Update plot objects
                set(elbowPlot, 'XData', Pe(1), 'YData', Pe(2), 'ZData', Pe(3));
                set(wristPlot, 'XData', Pw(1), 'YData', Pw(2), 'ZData', Pw(3));
                set(armLine, 'XData', [0, Pe(1)], 'YData', [0, Pe(2)], 'ZData', [0, Pe(3)]);
                set(forearmLine, 'XData', [Pe(1), Pw(1)], 'YData', [Pe(2), Pw(2)], 'ZData', [Pe(3), Pw(3)]);
    
                % Update target line positions
                set(targetLine, 'XData', [0, Pe_target(1), Pw_target(1)], 'YData', [0, Pe_target(2), Pw_target(2)], 'ZData', [0, Pe_target(3), Pw_target(3)]);
    
                % Calculate direction vectors of upper arm and forearm
                upper_arm_vector = Ru_aligned * [1; 0; 0];
                forearm_vector = Rv_aligned * [1; 0; 0];
    
                % Calculate elbow joint angle (¦È), the angle between upper arm and forearm
                cos_theta = dot(upper_arm_vector, forearm_vector) / (norm(upper_arm_vector) * norm(forearm_vector));
                theta_current = acosd(cos_theta);
    
                % **Modification Start**
                % Calculate forearm direction angle (¦Á) based only on the forearm's IMU
                eul_forearm = rotm2eul(Rv_aligned, 'ZYX');
                alpha_current = rad2deg(eul_forearm(3)); % Rotation angle around X axis
                % **Modification End**
    
                % Perform operations based on calibration stage
                if calibrationStage == 1
                    % Calculate elbow position difference
                    diff_Pe = Pe - Pe_target;
                    disp(['Upper arm position difference: x=', num2str(diff_Pe(1)), ', y=', num2str(diff_Pe(2)), ', z=', num2str(diff_Pe(3))]);
    
                    % Prompt user to adjust upper arm position
                    % Maintain vibration control for upper arm calibration as in original function
                    % Initially set calibration sub-stage (1: x-axis calibration, 2: y-axis calibration)
                    if ~exist('calibrationSubStage', 'var') || calibrationSubStage == 0
                        calibrationSubStage = 1;
                    end
    
                    % Calibrate upper arm's x-axis
                    if calibrationSubStage == 1
                        % Display prompt
                        if ~exist('textHandle', 'var')
                            textHandle = text(0, 0, 1.5, 'Calibrating Upper arm X-axis...', 'Color', 'r', 'FontSize', 14);
                        else
                            set(textHandle, 'String', 'Calibrating Upper arm X-axis...');
                        end
    
                        % Activate corresponding vibration based on difference direction
                        if diff_Pe(1) > threshold_position
                            fwrite(s, 'r'); % Positive x direction
                        elseif diff_Pe(1) < -threshold_position
                            fwrite(s, 'l'); % Negative x direction
                        else
                            fwrite(s, 'n'); % Stop vibration
                            disp('Upper arm X-axis calibration successful.');
                            calibrationSubStage = 2; % Switch to y-axis calibration
                        end
                    end
    
                    % Calibrate upper arm's y-axis
                    if calibrationSubStage == 2
                        % Display prompt
                        set(textHandle, 'String', 'Calibrating Upper arm Y-axis...');
    
                        % Activate corresponding vibration based on difference direction
                        if diff_Pe(3) > threshold_position
                            fwrite(s, 'x'); % Positive y direction
                        elseif diff_Pe(3) < -threshold_position
                            fwrite(s, 'u'); % Negative y direction
                        else
                            fwrite(s, 'n'); % Stop vibration
                            disp('Upper arm Y-axis calibration successful.');
                            calibrationSubStage = 0; % Sub-stage complete
                            calibrationStage = 2; % Move to next calibration stage
                            set(textHandle, 'String', 'Upper arm calibration complete!');
                        end
                    end
                    % Record initial angles
                    alpha_current_ini = alpha_current;
                    theta_current_ini = theta_current;
                elseif calibrationStage == 2
                        
                    set(textHandle, 'String', 'Calibrating forearm orientation...');
                    % Adjust forearm direction angle ¦Á
                    diff_alpha = alpha_current - alpha_current_ini - (-alpha_target - 90);
                    disp(['Current forearm orientation angle ¦Á = ', num2str(alpha_current - alpha_current_ini), '¡ã, Target angle ¦Á = ', num2str((-alpha_target - 90)), '¡ã, Difference = ', num2str(diff_alpha), '¡ã']);
    
                    % Send commands to Arduino based on difference direction
                    if diff_alpha > threshold_angle
                        fwrite(s, 'e'); % Rotate counterclockwise
                        disp('Please rotate your forearm counterclockwise.');
                    elseif diff_alpha < -threshold_angle
                        fwrite(s, 'c'); % Rotate clockwise
                        disp('Please rotate your forearm clockwise.');
                    else
                        fwrite(s, 'n'); % Stop rotation
                        disp('Forearm orientation angle calibration successful.');
                        calibrationStage = 3; % Move to next calibration stage
                    end
    
                elseif calibrationStage == 3
                    set(textHandle, 'String', 'Calibrating elbow joint angle ¦È...');
                    % Adjust elbow joint angle ¦È
                    diff_theta = theta_current - theta_current_ini - theta_target;
                    disp(['Current elbow joint angle ¦È = ', num2str(theta_current - theta_current_ini), '¡ã, Target angle ¦È = ', num2str(theta_target), '¡ã, Difference = ', num2str(diff_theta), '¡ã']);
    
                    % Activate vibration based on difference direction
                    if abs(diff_theta) > threshold_angle
                        
                        fwrite(s, 'a'); % Activate vibration to guide user to adjust elbow joint angle
                        disp('Please adjust your elbow joint angle.');
                    else
                        fwrite(s, 'n'); % Stop vibration
                        disp('Elbow joint angle calibration successful.');
                        calibrationStage = 4; % Calibration complete
                        disp('Calibration complete!');
                        %%set(textHandle, 'String', 'Calibration complete!');
                        toc
                    end
    
                end
    
                % Update graphics
                drawnow limitrate;
            end
        end
        
        
        pause(0.001);
    end
    
    % Clean up resources
        if exist('s', 'var') && strcmp(s.Status, 'open')
            fclose(s);
            delete(s);
        end
        clear s;
        disp('Program terminated.');
    
    catch ME
        % Ensure serial port is closed in case of error
        if exist('s', 'var') && strcmp(s.Status, 'open')
            fclose(s);
            delete(s);
        end
        disp('Program terminated unexpectedly.');
        rethrow(ME);
    end
    
    % Callback function definition
    function figureKeyPress(~, event)
        global userPressedQ userPressedC;
        if strcmp(event.Key, 'q')
            userPressedQ = true;
        elseif strcmp(event.Key, 'c')
            userPressedC = true;
        end
    end
    
    % Calibration function definition
    function performCalibration(s)
        global initialOffset R_calib;
    
        disp('Starting calibration...');
        disp('Please position your arm in the calibration position (arm extended horizontally, palm inward, IMU worn facing right), and remain still...');
        disp('Collecting data, please wait...');
    
        calibrationData = zeros(6,1);
        numSamples = 50; % Collect 50 samples for averaging
        samplesCollected = 0;
    
        while samplesCollected < numSamples
            if s.BytesAvailable > 0
                line = fgetl(s); % Read a line of data
                data = sscanf(line, '%f,%f,%f,%f,%f,%f'); % Parse six floating-point numbers
                if numel(data) == 6
                    calibrationData = calibrationData + data;
                    samplesCollected = samplesCollected + 1;
                end
            end
            pause(0.01); % Wait for 10ms
        end
        initialOffset = calibrationData / numSamples;
        disp('Calibration data collected:');
        disp(initialOffset);
    
        % Calculate calibration rotation matrix
        eul1_calib = deg2rad([initialOffset(1), initialOffset(2), initialOffset(3)]);
        Ru_calib = eul2rotm(eul1_calib, 'ZYX');
    
        % Define desired rotation matrix (arm extended horizontally along X axis)
        Ru_desired = eye(3); % Identity matrix
    
        % Calculate calibration rotation matrix
        R_calib = Ru_desired * Ru_calib';
    
        disp('Calibration rotation matrix updated.');
    end
