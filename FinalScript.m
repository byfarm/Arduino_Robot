% ---------- Begin Code --------------------
%% -------- step one: ----------------
% check the battery voltage, then start the process using button wait
% functions: VoltageCheck, ButtonWait
% needed inputs: none
% outputs: none
% ==============================================
% Check battery voltage
VoltageCheck(myKAR, potPin)
% start with the button start
ButtonWait(myKAR, buttonPin,ledPin, 0.1)
%% --------- step two ----------------
% start scanning for the object
% functions: ScanTurn, ScanSorroundings, ObjectCheck, ObjectLoc
% needed inputs: none
% outputs: objectRange, objectAngle
% ==============================================
% turn the ssr on to show scanning has started 
writeDigitalPin(myKAR, ssrPin, 1); % ON
objectCondition = false;
while objectCondition == false
    ScanTurn(myKAR, enA, enB, in1, in2, in3, in4);
    [targetScanData] = ScanSorroundings(frontScanServo, frontUSsensor);
    [objectCondition] = ObjectCheck(targetScanData);
end
[objectAngle,objectRange] = ObjectLoc(targetScanData);
%turn the ssr off to show scanning has ended
writeDigitalPin(myKAR, ssrPin, 0); % OFF
 fprintf('done scanning for object\n');
%% ---------- step three ----------------
% move and touch the object, play the buzzer then wait for the temp to
% increase enough and move back to center.
% functions: MovementProcess(forward,backward,left,right motions),
% BuzzerPlay, TempWait, ParalaxUnset
% needed inputs: objectRange, objectAngle
% outputs: none
% ==============================================
[trueAngle,trueRange] = ParalaxUnsetArray(objectRange,objectAngle);
% input trueAngle and trueRange into the movement function
MovementProcess(trueAngle,trueRange,myKAR, enA, enB, in1, in2, in3, in4)
pause(0.5)
% play the tone to show the object has been reached. 
BuzzerPlay(myKAR, tonePin)   
% command to remove the object
fprintf('Remove Object.\n\n');
% make it so that you must warm up the temp sensor in order to continue
TempWait(myKAR,tempPin,ledPin, 0.1)
% after given the go ahead, move back towards center
BackwardMotion(myKAR, enA, enB, in1, in2, in3, in4, trueRange)
fprintf('Movement section 1 done, object eliminated.\nEscape Process begins now.\n\n')
%% ----------- step four -------------
% scan for the lights
% functions: ScanTurn, LightConditionChanger, ExitLocator, LightScanner,
% MovementProcess
% needed inputs: none
% outputs: lightAngle
% ==============================================
% set the exit condition to false an d that will change when the exit is
% found, so until the exit is found, have the robot run the scan fucntion,
% and turn 120 degrees if the light is not found. 
exitCondition = false;
while exitCondition == false
% turns 120 degrees to scan a new part of the maze
    ScanTurn(myKAR, enA, enB, in1, in2, in3, in4);
% scans 180 degrees and collects readings from the LDR on the voltage, and
% records the angle that the LDR reads that voltage at.
    [lightData]=LightScanner(myKAR, frontScanServo, ldrPin);
% analyzes the light data and determines if it has found one light, two
% lights, or no lights. it only updates the exit condition to true if the
% sensors get a certian number of hits.
    [exitCondition]=LightConditionChanger(lightData);
    [light1Angle] = ExitLocator(lightData);
end
fprintf('\nlight1 located\n')
% add in here a fuction that spins the robot to the first light
light1Range = 0; % is zero because the distance is set and do not want to move translationally
MovementProcess(light1Angle,light1Range,myKAR, enA, enB, in1, in2, in3, in4)
% have the robot scan again and export the exit angle from there.
[lightData]=LightScanner(myKAR, frontScanServo, ldrPin);
% analyzes the final light data outputed to determine the exit angle.
[exitAngle] = ExitLocator(lightData);
%% --------------- step five -------------------------
% exits the escape room
% functions: ParalaxUnset, MovementProcess
% needed inputs: exitAngle
% outputs: none
% ==============================================
exitRange = 0.6;
[trueAngle,trueRange] = ParalaxUnsetArray(exitAngle,exitRange);
trueRange = 0.6; % m. Is 0.6 so will stop just outside the maze
% once the true distance is calculated, turn the robot the correct way and
% move out of the maze
MovementProcess(trueAngle,trueRange, myKAR, enA, enB, in1, in2, in3, in4)
fprintf('\n\nChallenge Completed!')
%%
% =-=-=-=-=-=-=-=-= Local Functions =-=-=-=-=-=-=-=-=
% =============== object scanning functions ================
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
maxValue = .45; % meters
%this is the number of angles that the servo is to scan
wantedAngles = 20;
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
%T = table(objectAngle, objectDistance);
%disp(T)
end

% ================= light scanning functions ======================
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

function [exitCondition] = LightConditionChanger(data)
% takes the light data and finds the values under the minimum. if there are
% two lights, there will be greater hits than just 1 light. This function
% checks to see how many hits there are and if there are more than the
% threshold, will respond that the exit condition is within the range of
% the sensor, and the robot should calculate the angle that it needs to
% move to. 
lightThreshold = 0.8; % volts
lightNumThreshold = 2;
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

% =============== movement functions ==========================
function MovementProcess(angle,range,kar, mA, mB, m1, m2, m3, m4)
% if the inputed angle is greater than zero, the robot will spin left, if
% greater it will spin right, and if the angle is zero, the robot will move
% forwards. 
 if angle < 0
     LeftSpinMotion(kar, mA, mB, m1, m2, m3, m4, angle)
     pause(1)
     ForwardMotion(kar, mA, mB, m1, m2, m3, m4, range)
 elseif angle > 0
     RightSpinMotion(kar, mA, mB, m1, m2, m3, m4, angle)
     pause(1)
     ForwardMotion(kar, mA, mB, m1, m2, m3, m4, range)
 else
    ForwardMotion(kar, mA, mB, m1, m2, m3, m4, range)
 end
fprintf('Movement Finished.\n')
end

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
    timeSpin = 120/160;
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
     timeSpin = abs(theta)/160;
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
    timeSpin = abs(theta)/160;
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

% =============== msc functions ============================ 
function VoltageCheck(a, pin)
% checks the voltage on the battery to make sure the motors have enough
% power. 
 batteryVoltage = readVoltage(a, pin);
 batteryVoltage = 3 * batteryVoltage;
 fprintf('The battery voltage is %0.2f Volts', batteryVoltage)
 % put in a 2k resistor before the pin and a 1 k resistor after the pin
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

function BuzzerPlay(a, pin)
% plays a tone to notify that it is time to remove the object, and the
% robot is ready to escape.
    toneFreq = 2400; toneTime = 0.5;
    playTone(a, pin, toneFreq, toneTime);
    pause(toneTime); 
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

function TempWait(a,temperature, indicator, pauseT)
tempThreshold = 2.4;
initialVoltage = readVoltage(a, temperature);
fprintf('\n\nInital voltage is %0.2f V.\nThreshold must be greater than initial voltage.\n', initialVoltage);
fprintf('Robot is waiting for temperature to increase...\n');
while initialVoltage < tempThreshold
    initialVoltage = readVoltage(a,temperature);
    writeDigitalPin(a,indicator,1)
    pause(pauseT);
    writeDigitalPin(a,indicator,0)
    pause(pauseT);
end
fprintf('\nRobot running, TempWait done.\n')
end

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
% =-=-=-=-=-=-=-=-= End Local Functions =-=-=-=-=-=-=-=-=
% ----------------- End Code ---------------------------------