% this is all the functions accociated with the light location portion of
% the challenege. This includes the scan function with the LDR incorperated
% into it, the lightCondition, which is a true or false function that
% outputs whether the robot has detected the exit or not, and the Exit
% Locator, which once the robot has registared that is has found the
% lights, will calculate the angle to the exit. 

% necicary inputs: none
% expected outputs: angle to the exit
% ================= begin ===================
exitCondition = false;
% set the exit condition to false and that will change when the exit is
% found, so until the exit is found, have the robot run the scan fucntion,
% and turn 120 degrees if the light is not found. 
while exitCondition == false
% turns 120 degrees to scan a new part of the maze
    degrees = 120;
    ScanTurn(myKAR, enA, enB, in1, in2, in3, in4,degrees);
% scans 180 degrees and collects readings from the LDR on the voltage, and
% records the angle that the LDR reads that voltage at.
    [lightData]=LightScanner(myKAR, frontScanServo, ldrPin);
% analyzes the light data and determines if it has found one light, two
% lights, or no lights. it only updates the exit condition to true if the
% sensors get a certian number of hits.
    [exitCondition]=LightConditionChanger(lightData);
    [light1Angle] = ExitLocator(lightData);
end
% add in here a fuction that spins the robot to the first light
light1Range = 0; % is zero because the distance is set and do not want to move translationally
MovementProcess(light1Angle,light1Range,myKAR, enA, enB, in1, in2, in3, in4)
% have the robot scan again and export the exit angle from there.
[lightData]=LightScanner(myKAR, frontScanServo, ldrPin);
% analyzes the final light data outputed to determine the exit angle.
[exitAngle] = ExitLocator(lightData);


% --------------------- local functions ----------------
function [lightData] = LightScanner(a, servo, LDR)
% sets the servo to five different positions and finds the range of objects
% in those positions. It takes 3 good measurments at each position by
% checking to see if the servo returns inf, if it does, it keeps on taking
% measuremnts till it has 3 finite numbers. Finally, call a polar plotting
% function to plot and display the data on a polar plot. 
% Range measurment and display
% set the center value
centerValue = 0.5; 
%this is the number of angles that the servo is to scan
wantedAngles = 180;
% create targetScanData array
lightData = ones(wantedAngles,2);  
% take the angles you want and convert them so the robot can understand
% them. Set the angle number counter to zero.
degreesAngles = linspace(-90,90,wantedAngles)';
robotAngles = round(-1/180 .* degreesAngles + centerValue,3);
angleNumber = 0;
lightData(:,2) = degreesAngles;
% check center position
writePosition(servo, centerValue);
fprintf('Light Scanning has started.\n\n')
while angleNumber < wantedAngles
% move to angle 1 and take data
angleNumber = angleNumber + 1;
writePosition(servo, robotAngles(angleNumber));
% pause(0.01);
% take the ldr voltage and input it into the lightData array.  
ldrVoltage = readVoltage(a,LDR);
lightData(angleNumber) = ldrVoltage;
end
% move back to center
writePosition (servo, centerValue);
fprintf('Light scanning has ended.\n\n') 
end

function ScanTurn(a, left, right, leftD1, leftD2, rightD1, rightD2, angle)
    % Spins the robot to the left 120 degrees so that it can scan another
    % area of the maze. 
    %============================
    % Assign Left and Right Rotation Direction
    writeDigitalPin(a,leftD1,1); writeDigitalPin(a,leftD2,0); % A
    writeDigitalPin(a,rightD2,0); writeDigitalPin(a,rightD1,1); % B
    % assign a speed and time for the motors to spin
    speedMotors = 2; 
    % convert the angle to an expirementally determined time
    % the robot spins at approximatly 114.65 degrees per second. Therefore
    % the conversion from angle to time is theta/114.65
    timeSpin = angle/114.65;
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

function [exitCondition] = LightConditionChanger(data)
% takes the light data and finds the values under the minimum. if there are
% two lights, there will be greater hits than just 1 light. This function
% checks to see how many hits there are and if there are more than the
% threshold, will respond that the exit condition is within the range of
% the sensor, and the robot should calculate the angle that it needs to
% move to. 
lightThreshold = 1; % volts
lightNumThreshold = 10;
lightData = data(:,1);
lightsIndex = lightData < lightThreshold;
numHits = sum(lightsIndex);
    if numHits > lightNumThreshold
    exitCondition = true;
    else
    exitCondition = false; 
    end
end

 function [exitAngle] = ExitLocator(dataArray)
% takes the data from the light scan and finds the average angle of all the
% hits, which is where the center of the exit should be. 
angleArray = dataArray(:,2);
voltageArray = dataArray(:,1);
% set a voltage minimum that if the voltage reading is under, there is a
% light present. 
voltageMin = 1; % volts
% finds which of the angles have condition true for a light being present
lights = voltageArray < voltageMin;
lightAngles = angleArray(lights);
% outputs the exit angle, which is the mean of the registered light angles.
exitAngle = mean(lightAngles);
end

function ButtonWait(a, b_Pin,l_Pin, d_time)
% Make the program not start until the user presses the button on the
% robot. Does this by reading the digital pin for a certian amount of time
% then starting. 
% print to screen
fprintf('Press the button when youre ready')
buttonSt = 0 ;
while buttonSt == 0
    buttonSt = readDigitalPin(a,b_Pin);
    writeDigitalPin(a,l_Pin,1)
    pause(d_time);
    writeDigitalPin(a,l_Pin,0)
    pause(d_time);
end
fprintf('\nbuttonWait done.\n')
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


%-------- end code -------------