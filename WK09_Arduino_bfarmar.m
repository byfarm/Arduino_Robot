clear;clc;close all;
% USB or Bluetooth?
% Create USB or BlueTooth connection
    % Uncomment next line for USB Connection
        % MUST remove BT device from Sensor Shield
% New tool : switch-case-otherwise
% set control variable
% connType = 'USB';
% switch connType
%     case 'USB'
       
COM = '/dev/cu.usbserial-14130';
myKAR = arduino(COM,'Uno','Libraries',{'Ultrasonic','Servo'});
%     case 'bluetooth'
%             
%            % Uncomment for bluetooth connection
%         % MUST attach BT device and UPDATE name 'Granny'
%         COM = 'Porky Pig';
%             myKAR = arduino(COM);
%     otherwise 
%         fprintf('Error - check connType spelling.\n')
% end
% Pin and Device Configuration
% ButtonWait Setup
    ledPin = 'D12'; configurePin(myKAR,ledPin,'DigitalOutput'); % LED control
    buttonPin = 'D13'; configurePin(myKAR,buttonPin,'DigitalInput'); % push-button input
% Assign and configure pins for servo control
    % Use of Servo library (PWM) - functionality on pins 9 and 10, 
    % whether or not there is a Servo on those pins.
    frontServoPin = 'D10'; % Either D9 or D10 committed, available for servo only 
    configurePin(myKAR,frontServoPin,'Servo'); % servo control
    % Set Servo Characteristics - Pulse Duration Values - servo dependent.
    rotMinPulse = 500*10^-6; rotMaxPulse = 2400*10^-6; % min/max pulse width
    % Servo Object Setup: Servo pins and pulse characteristics assigned to device
    frontScanServo = servo(myKAR,frontServoPin,'MinPulseDuration',rotMinPulse,...
        'MaxPulseDuration', rotMaxPulse );    
% Configure pins for ultrasonic sensor
    usEchoPin = 'A1';  configurePin(myKAR,usEchoPin,'Ultrasonic'); % Ultrasonic echo
    usTriggerPin = 'A0'; configurePin(myKAR,usTriggerPin,'Ultrasonic'); % Ultrasonic trigger   
    % Assign Ultrasonic sensor to Arduino pins for trigger and echo
    frontUSsensor = ultrasonic(myKAR,usTriggerPin,usEchoPin,'OutputFormat','double' );
% Tone pins
    % Use of the playTone() function will interfere with PWM output on 
    % pins 3 and 11 on Uno boards. If using playTone, both are comittted.
    tonePin = 'D11'; configurePin(myKAR, tonePin, 'Tone'); 
% Solid State Relay
    ssrPin = 'A2'; configurePin(myKAR,ssrPin,'DigitalOutput');
% Thermistor Check
    tempPin = 'A3'; configurePin(myKAR,tempPin,'AnalogInput');
% Light Dependent Resistor Check
    ldrPin = 'A4'; configurePin(myKAR,ldrPin,'AnalogInput');
% Potentiometer Check
    potPin = 'A5'; configurePin(myKAR,potPin,'AnalogInput');
% Assign and Configure Pins for Motor Control
% Add Motor Configuration
 enA = 'D5'; in1 = 'D4'; in2 = 'D2'; % Motor A (LEFT) pins
 enB = 'D6'; in3 = 'D8'; in4 = 'D7'; % Motor B (RIGHT) pins
 % Motor A control (LEFT)
 configurePin(myKAR,enA,'PWM');
 configurePin(myKAR,in1,'DigitalOutput'); configurePin(myKAR,in2,'DigitalOutput');
 % Motor B control (RIGHT)
 configurePin(myKAR,enB,'PWM');
 configurePin(myKAR,in4,'DigitalOutput'); configurePin(myKAR,in3,'DigitalOutput');
% Setup Complete
    fprintf("Cleared and configured to use to use %s\n - Ready to run code with hardware configured as included here.\n\n",COM);
    %====END SETUP====  