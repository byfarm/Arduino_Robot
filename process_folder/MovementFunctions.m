% this is the code for the movement functions, which move the robot from
% whereever it is stationed to the object, and then back again. We will 
% implement a buzzer for when the robot hits the object, and a thermoster
% check to restart the robot. Also, the paralax function is incorperated
% into this script

% needed inputs: object distance, object angle
% expected outputs: none

% ======================= code =============================
% take the outputs from the scanning portion and put them into the paralax
% function to change them to the true angles
objectRange = 0.7; %m
objectAngle = 90; % deg
[trueAngle,trueRange] = ParalaxUnsetArray(objectRange,objectAngle);
% input trueAngle and trueRange into the movement function
MovementProcess(trueAngle,trueRange,myKAR, enA, enB, in1, in2, in3, in4)
pause(0.5)
% play the tone to show the object has been reached. 
BuzzerPlay(myKAR, tonePin)   
% command to remove the object
fprintf('Remove Object.\n\n');
% make it so that you must warm up the temp sensor in order to continue
TempWait(myKAR, tempPin,ledPin, 0.1,tempThreshold)
% after given the go ahead, move back towards center
BackwardMotion(myKAR, enA, enB, in1, in2, in3, in4, trueRange)
fprintf('Movement section 1 done, object eliminated.\nEscape Process begins now.\n\n')


% ------------------- local functions ----------------------
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
paralaxOffset = 0.017; % meters
robotRangeOffset = 0.016; % m
sensorRangeOffset = 0.003; % m
% using law of cosines:
trueRange = sqrt(abs((sensorRangeArray + sensorRangeOffset).^2 + paralaxOffset^2 - 2 .* (sensorRangeArray + sensorRangeOffset) .* paralaxOffset .* cosd(180 - sensorAngleArray)));
% using law of sines:
trueAngle = asind(((sensorRangeArray + sensorRangeOffset) .* sind(180 - sensorAngleArray)) ./ trueRange);
% adjust the true range for how long the robot is
trueRange = trueRange - robotRangeOffset;
% =================== End Code ==========================================
end

function MovementProcess(angle,range,kar, mA, mB, m1, m2, m3, m4)
% if the inputed angle is greater than zero, the robot will spin left, if
% greater it will spin right, and if the angle is zero, the robot will move
% forwards. 
 if angle > 0
     LeftSpinMotion(kar, mA, mB, m1, m2, m3, m4, angle)
     pause(1)
     ForwardMotion(kar, mA, mB, m1, m2, m3, m4, range)
 elseif angle < 0
     RightSpinMotion(kar, mA, mB, m1, m2, m3, m4, angle)
     pause(1)
     ForwardMotion(kar, mA, mB, m1, m2, m3, m4, range)
 else
    ForwardMotion(kar, mA, mB, m1, m2, m3, m4, range)
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

function BackwardMotion(a, left, right, leftD1, leftD2, rightD1, rightD2, distance)
    % moves the Robot backwards by at a specified speed for a specified
    % amount of time by spinning both wheels backwards at the same time.
    % inputs are myKAR, the voltage pins for the left and right motors, and
    % the pins that determine direction for the left and right motors.
    % output is the robot moving backwards, along with updates when the
    % function has started/ended
    %=============================
    % print that function starts
    fprintf('BackwardMotion has started.\n');
    % Assign Left and Right Rotation Direction
    writeDigitalPin(a,leftD1,1); writeDigitalPin(a,leftD2,0); % A
    writeDigitalPin(a,rightD2,1); writeDigitalPin(a,rightD1,0); % B
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
    fprintf('BackwardMotion has ended.\n')
end

function BuzzerPlay(a, pin)
% plays a tone to notify that it is time to remove the object, and the
% robot is ready to escape.
    toneFreq = 2400; toneTime = 0.5;
    playTone(a, pin, toneFreq, toneTime);
    pause(toneTime); 
end

