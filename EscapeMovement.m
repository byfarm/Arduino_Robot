% this is the file for the final escape movement once the lights have been
% located. 

% needed inputs: exitAngle
% expected outputs: movement of the robot

% ================= code ===========================

% for this case the object range is a set value because the robot will be
% in the center of the circle, and the ultrasonic sensor will not be able
% to read the necicary distance. 
escapeRange = 0.5; % m
[trueAngle,trueRange] = ParalaxUnsetArray(exitAngle,exitRange);
trueRange = 0.6; % m
% once the true distance is calculated, turn the robot the correct way and
% move out of the maze
MovementProcess(trueAngle,trueRange)

% =================== local functions =======================

function [trueAngle,trueRange] = ParalaxUnsetArray(sensorRangeArray, sensorAngleArray)
    % ParalaxUnsetArray: Find distance from the measured object to the center of
    % the robot by using law of sines and the sensor's distace and angle from the
    % object. Also to find the angle from the center of the robot to the
    % object by using the measured angle of the sensor by using law of sines 
    % and the calculated distance to the object from the center of wheels. 

    % Inputs: the measured sensor range and measured sensor angle
    % Outputs: The object range from the center of the robot and the 
    % angle from the center of the robot
% =================== Begin Code ======================================
% set dimentions of robot
paralaxOffset = 0.065; % meters
robotRangeOffset = 0.08; % m
sensorRangeOffset = 0.02; % m
% using law of cosines:
trueRange = sqrt(abs((sensorRangeArray + sensorRangeOffset).^2 + paralaxOffset^2 - 2 .* (sensorRangeArray + sensorRangeOffset) .* paralaxOffset .* cosd(180 - sensorAngleArray)));
% using law of sines:
trueAngle = asind(((sensorRangeArray + sensorRangeOffset) .* sind(180 - sensorAngleArray)) ./ trueRange);
% adjust the true range for how long the robot is
trueRange = trueRange - robotRangeOffset;
% =================== End Code ==========================================
end

function MovementProcess(angle,range)
% if the inputed angle is greater than zero, the robot will spin left, if
% greater it will spin right, and if the angle is zero, the robot will move
% forwards. 
 if angle > 0
     LeftSpinMotion(myKAR, enA, enB, in1, in2, in3, in4, angle)
     pause(1)
     ForwardMotion(myKAR, enA, enB, in1, in2, in3, in4, range)
 elseif angle < 0
     RightSpinMotion(myKAR, enA, enB, in1, in2, in3, in4, angle)
     pause(1)
     ForwardMotion(myKAR, enA, enB, in1, in2, in3, in4, range)
 else
    ForwardMotion(myKAR, enA, enB, in1, in2, in3, in4, range)
 end
fprintf('object touched.\n')
end


function ForwardMotion(a, left, right, leftD1, leftD2, rightD1, rightD2, distance)
    % Moves the robot forwards by setting the direction of the motors
    % forwards. It then converts the distance input into the time it takes to
    % move the robot that distance when the motor speed is 1.
    %=============================
    % print that function starts
    fprintf('ForwardMotion has started.\n');
    % Assign Left and Right Rotation Direction
    writeDigitalPin(a,leftD1,0); writeDigitalPin(a,leftD2,1); % A
    writeDigitalPin(a,rightD2,0); writeDigitalPin(a,rightD1,1); % B
    % assign a speed and time for the motors to spin
    speedMotors = 2; 
    % convert the distance to a time
    % the robot has an approxamite velocity of 0.188 m/s when the speed
    % setting is at 2, therefore the distance to time conversion is
    % distance/0.188.
    timeSpin = distance/0.188;
    % Assign Speed to both motors
    writePWMVoltage(a,left,speedMotors); % Left
    writePWMVoltage(a,right,speedMotors); % Right
    % time want motors to spin
    pause(timeSpin);
    % stop motors
    writePWMVoltage(a,left,0); % Left
    writePWMVoltage(a,right,0); % Right
    pause(0.5)
    % state that function has ended
    fprintf('ForwardMotion has ended.\n')
end

function LeftSpinMotion(a, left, right, leftD1, leftD2, rightD1, rightD2, theta)
    % Spins the robot left by setting the direction of the left motor
    % backwards and the direction of the right motor forwards. Then it
    % converts the input angle into an approximate time it takes to spin
    % the robot with a speed of 1 to the desired angle in degrees. It then shuts off
    % the motors.
    %=============================
    % print that function starts
    fprintf('LeftSpinMotion has started.\n');
    % Assign Left and Right Rotation Direction
    writeDigitalPin(a,leftD1,1); writeDigitalPin(a,leftD2,0); % A
    writeDigitalPin(a,rightD2,0); writeDigitalPin(a,rightD1,1); % B
    % assign a speed and time for the motors to spin
    speedMotors = 2; 
    % convert the angle to an expirementally determined time
    % the robot spins at approximatly 114.65 degrees per second. Therefore
    % the conversion from angle to time is theta/114.65
    timeSpin = theta/114.65;
    % Assign Speed to both motors
    writePWMVoltage(a,left,speedMotors); % Left
    writePWMVoltage(a,right,speedMotors); % Right
    % time want motors to spin
    pause(timeSpin);
    % stop motors
    writePWMVoltage(a,left,0); % Left
    writePWMVoltage(a,right,0); % Right
    pause(0.5)
    % state that function has ended
    fprintf('LeftSpinMotion has ended.\n')
end

function RightSpinMotion(a, left, right, leftD1, leftD2, rightD1, rightD2, theta)
    % Spins the robot right by setting the direction of the right motor
    % backwards and the direction of the left motor forwards. Then it
    % converts the input angle into an approximate time it takes to spin
    % the robot with a speed of 1 to the desired angle. It then shuts off
    % the motors.
    %=============================
    % print that function starts
    fprintf('RightSpinMotion has started.\n');
    % Assign Left and Right Rotation Direction
    writeDigitalPin(a,leftD1,0); writeDigitalPin(a,leftD2,1); % A
    writeDigitalPin(a,rightD2,1); writeDigitalPin(a,rightD1,0); % B
    % assign a speed and time for the motors to spin
    speedMotors = 2; 
    % convert the angle to an expirementally determined time
    % the robot spins at approximatly 114.65 degrees per second. Therefore
    % the conversion from angle to time is theta/114.65
    timeSpin = theta/114.65;
    % Assign Speed to both motors
    writePWMVoltage(a,left,speedMotors); % Left
    writePWMVoltage(a,right,speedMotors); % Right
    % time want motors to spin
    pause(timeSpin);
    % stop motors
    writePWMVoltage(a,left,0); % Left
    writePWMVoltage(a,right,0); % Right
    pause(0.5)
    % state that function has ended
    fprintf('RightSpinMotion has ended.\n')
end
