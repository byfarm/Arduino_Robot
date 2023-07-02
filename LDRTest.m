

ldrVoltage = readVoltage(myKAR,ldrPin);
 fprintf('LDR voltage = %0.2fV.\n',ldrVoltage);
 pause(0.25);
[lightData]=ScanSorroundings(myKAR, frontScanServo, ldrPin)
[exitCondition]=LightConditionChanger(lightData)
[exitAngle] = LightLocator(lightData)
 function [lightData] = ScanSorroundings(a, servo, LDR)
% sets the servo to five different positions and finds the range of objects
% in those positions. It takes 3 good measurments at each position by
% checking to see if the servo returns inf, if it does, it keeps on taking
% measuremnts till it has 3 finite numbers. Finally, call a polar plotting
% function to plot and display the data on a polar plot. 
% Range measurment and display
% set the center value
centerValue = 0.5; 
% Set the max value a range can be
maxValue = .8; % meters
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
fprintf('Range scanning has ended.\n\n') 
end

function [exitCondition] = LightConditionChanger(data)
% takes the light data and finds the values under the minimum. if there are
% two lights, there will be greater hits than just 1 light. This function
% checks to see how many hits there are and if there are more than the
% threshold, will respond that the exit condition is within the range of
% the sensor, and the robot should calculate the angle that it needs to
% move to. 
lightThreshold = 1; % volts
lightNumThreshold = 20;
lightData = data(:,1);
lightsIndex = lightData < lightThreshold;
    if lightsIndex > lightNumThreshold
    exitCondition = true;
    else
    exitCondition = false; 
    end
end

 function [exitAngle] = LightLocator(dataArray)
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


%-------- end code -------------