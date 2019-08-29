%% Script to calculate the different relay operating 
%% times for different relay types. The current values
%% are based on 


% Variables
C = 0;
TMS = 0.5;
I = 10133333;
Is = 400; 
alpha = 0;

% Standard Inverse (SI)
C = 0.14;
alpha = 0.02;
T = (C*TMS)/(((I/Is)^alpha)-1);
fprintf('Standard Inverse Relay Operating Time:')
disp(T)

% Very Inverse (VI)
C = 13.5;
alpha = 1;
T = (C*TMS)/(((I/Is)^alpha)-1);
fprintf('Very Inverse Relay Operating Time:')
disp(T)


% Extremely Inverse (EI)
C = 80;
alpha = 2;
T = (C*TMS)/(((I/Is)^alpha)-1);
fprintf('Extremely Inverse Relay Operating Time:')
disp(T)


% Long Inverse (LI)
C = 120;
alpha = 1;
T = (C*TMS)/(((I/Is)^alpha)-1);
fprintf('Long Inverse Relay Operating Time:')
disp(T)


