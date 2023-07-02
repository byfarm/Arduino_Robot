% this is the code for the actions that must take place before the robot
% starts scanning
% ================ code =============================
% Check battery voltage
VoltageCheck(myKAR, potPin)
% start with the button start
ButtonWait(myKAR, buttonPin,ledPin, 0.1)
% --------------- local functions -----------------

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
