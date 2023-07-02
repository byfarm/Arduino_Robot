% this is the script for the first part of the challenge, where we are
% using the ultrasonic sensor to detect the object.
% needed inputs: none
% Expected Outputs: object range, object angle

% ================= code ====================
% Scan Sorrounding Area and turn 120 degreees when do not find the object
objectCondition = false;
while objectCondition == false
    ScanTurn(myKAR, enA, enB, in1, in2, in3, in4);
    [targetScanData] = ScanSorroundings(frontScanServo, frontUSsensor);
    [objectCondition] = ObjectCheck(targetScanData);
end
[objectAngle,objectRange] = ObjectLoc(targetScanData);


% ------------------------ local functions ------------------
function ScanTurn(a, left, right, leftD1, leftD2, rightD1, rightD2)
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
    timeSpin = 120/114.65;
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
    fprintf('ScanTurn has ended.\n')
end

function [targetScanData] = ScanSorroundings(servo, ultrasonic)
% sets the servo to five different positions and finds the range of objects
% in those positions. It takes 3 good measurments at each position by
% checking to see if the servo returns inf, if it does, it keeps on taking
% measuremnts till it has 3 finite numbers. Finally, call a polar plotting
% function to plot and display the data on a polar plot. 
% Range measurment and display
% set the center value
centerValue = 0.5; 
% Set the max value a range can be
maxValue = .5; % meters
%this is the number of angles that the servo is to scan
wantedAngles = 40;
% create targetScanData array
targetScanData = ones(wantedAngles,2);    
% take the angles you want and convert them so the robot can understand
% them. Set the angle number counter to zero.
degreesAngles = linspace(-90,90,wantedAngles)';
robotAngles = round(-1/180 .* degreesAngles + centerValue,3);
angleNumber = 0;
% check center position
writePosition(servo, centerValue);
pause(0.5)
fprintf('Range Scanning has started.\n\n')
while angleNumber < wantedAngles
% move to angle 1 and take data
angleNumber = angleNumber + 1;
writePosition(servo, robotAngles(angleNumber));
pause(0.0015);
% set up a ones array so that the data can be put into it. 
positionDistanceArray = ones(1,3);
goodMeasurements = 0; % is the number of good measurments taken. 
while goodMeasurements < 3
    distance = readDistance(ultrasonic);
    if distance == inf
        goodMeasurements = goodMeasurements;
    elseif distance > maxValue
        distance = maxValue;
        goodMeasurements = goodMeasurements+1;
        positionDistanceArray(goodMeasurements) = distance;
    elseif distance <= maxValue
        goodMeasurements = goodMeasurements+1;
        positionDistanceArray(goodMeasurements) = distance;
    end
end
positionDistance = sum(positionDistanceArray) / 3;
positionData = [degreesAngles(angleNumber), positionDistance];
targetScanData(angleNumber,:) = positionData;
pause(.5)
end
% move back to center
writePosition (servo, centerValue);
fprintf('Range scanning has ended.\n\n')
% call the function that plots the data
PlotUSPolar(targetScanData)
end


function [objectCondition] = ObjectCheck(dataArray)
% checks to see if any objects have been picked up by the scanner by taking
% the min distance recorded and checking to see if it is closer than the
% wall. Also, the minimum distances must be less than 3 in order. 
distanceThreshold = 0.4; % meters
rangeArray = dataArray(:,2);
minimumDistance = min(rangeArray);
% find where those minimum distances "hit"
distHits = (rangeArray <= minimumDistance + 0.001);
% find the number of objects by summing the amound of hits
numObjects = sum(distHits);
    if (minimumDistance < distanceThreshold) && (numObjects < 3)
    objectCondition = true;
    else
    objectCondition = false; 
    end
end

function [objectAngle,objectDistance] = ObjectLoc(dataArray)
% find the minimum distance of the function, which is the distance of the
% objects. Then uses a logical array to delete all the distances and angles
% that are not the object. outputs the angle and object distance. 
rArray = dataArray(:,2);
aArray = dataArray(:,1);
minDistance = min(rArray);
dHits = (rArray <= minDistance + 0.001);
objectDistance = minDistance;
% find all the points that were not hits and set them equal to 1
zeros = dHits < 0.0001;
% delete all the points that were not hits for both the angle array and the
% distance array
rArray(zeros) = [];
aArray(zeros) = [];
objectAngle = aArray ;
% output sentences and table
fprintf("The table bellow shows the angle and range from the sensor.\n\n")
T = table(objectAngle, objectDistance);
disp(T)
end

function PlotUSPolar(data)
    % Data Setup
    angDegrees = data(:,1);
    rangeCM = data(:,2);
    % Find object local minimums
    objPos = [false;rangeCM(2:end-1) < rangeCM(1:end-2) & rangeCM(2:end-1) < rangeCM(3:end);false];
    % polar plot
    figure('Name',"Ultrasonic Target Scan");
    polarplot(deg2rad(angDegrees),rangeCM, 'b--','lineWidth',2);
    hold on
    polarplot(deg2rad(angDegrees(objPos)), rangeCM(objPos), 'ro','MarkerFaceColor','r');
    pax = gca;
    pax.ThetaZeroLocation = 'top';
    pax.ThetaDir = 'clockwise';
    title('Ultrasonic Target Scan');
    pax.ThetaLim = [-90,90];
    fprintf('polar plot done');
end