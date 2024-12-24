% Import manipulator and set up
manipulator = importrobot("robot1113.urdf");
manipulator.DataFormat = "row";
gripper = 'end_effector';

% Set desired XYZ points
targetPoints = [0.2, 0.0, 0.5; % First target
                0.1, 0.2, 0.3; % Second target
                0.3, -0.1, 0.4]; % Third target

% Generalized Inverse Kinematics setup
genIK = generalizedInverseKinematics('RigidBodyTree', manipulator, ...
   'ConstraintInputs', {'position', 'orientation', 'joint'});

% Cartesian position constraint
cartesianConstraint = constraintPositionTarget(gripper);
cartesianConstraint.PositionTolerance = 0.002;

% Orientation constraint
orientationConstraint = constraintOrientationTarget(gripper);
orientationConstraint.OrientationTolerance = deg2rad(1);

% Joint bounds constraint
jointConstraint = constraintJointBounds(manipulator);
jointConstraint.Weights = ones(size(jointConstraint.Weights));

% Solve for each target point
qWaypoints = []; % Store joint configurations
for i = 1:size(targetPoints, 1)
    % Update target position
    cartesianConstraint.TargetPosition = targetPoints(i, :);
    
    % Solve for joint angles using IK
    if i == 1
        qInit = homeConfiguration(manipulator); % Use home position for the first point
    else
        qInit = qWaypoints(end, :); % Use the previous point as the starting position
    end
    [qSolution, solutionInfo] = genIK(qInit, cartesianConstraint, ...
        orientationConstraint, jointConstraint);
    qWaypoints = [qWaypoints; qSolution];
end

% Simulate movement in MATLAB
framerate = 15;
r = rateControl(framerate);
tFinal = 10;
numFrames = tFinal * framerate;
qInterp = pchip(linspace(0, tFinal, size(qWaypoints, 1)), qWaypoints', ...
    linspace(0, tFinal, numFrames))';

figure;
gripperPosition = zeros(numFrames, 3);
for k = 1:numFrames
    show(manipulator, qInterp(k, :), 'PreservePlot', false);
    gripperPosition(k, :) = tform2trvec(getTransform(manipulator, qInterp(k, :), gripper));
    waitfor(r);
end

function writeJointAnglesToRaspberryPi(qWaypoints)
    rpi = raspi('[rpi_url]', '6dof', 'password'); % replace rpi_url and password to your own values

    % Initilize motors
    base = servo(rpi, 13, 'MinPulseDuration', 15e-3, 'MaxPulseDuration', 3e-3);
    link1 = servo(rpi, 19, 'MinPulseDuration', 15e-3, 'MaxPulseDuration', 3e-3);
    link2 = servo(rpi, 26, 'MinPulseDuration', 15e-3, 'MaxPulseDuration', 3e-3);
    link3 = servo(rpi, 20, 'MinPulseDuration', 15e-3, 'MaxPulseDuration', 3e-3);
    end_effect = servo(rpi, 21, 'MinPulseDuration', 15e-3, 'MaxPulseDuration', 3e-3);
    servos = [base, link1, link2, link3, end_effect]
    
    numJoints = size(qWaypoints, 2);
    for i = 1:size(qWaypoints, 1)
        for j = 1:numJoints
            pin = sprintf('D%d', 18 + (j - 1)); 
            angle = rad2deg(qWaypoints(i, j));
            writePWMDutyCycle(rpi, servos[j], angle / 180); % Normalize to 0-1 for PWM
        end
        pause(0.02); % Adjust delay as needed
    end
end

% Export joint angles to Raspberry Pi
writeJointAnglesToRaspberryPi(qWaypoints);
