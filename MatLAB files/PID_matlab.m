% PID Controller Simulation in MATLAB

% Clear workspace and close all figures
clear;
clc;
close all;

% Define the system parameters
K = 1;          % System gain
tau = 0.5;      % Time constant
num = K;        % Numerator of the transfer function
den = [tau 1];  % Denominator of the transfer function
sys = tf(num, den); % Transfer function of the system

% Define the PID controller parameters
Kp = 5;         % Proportional gain
Ki = 50;         % Integral gain
Kd = 44.5;       % Derivative gain

% Create the PID controller
C = pid(Kp, Ki, Kd);

% Closed-loop system with PID controller
closed_loop_sys = feedback(C * sys, 1);

% Time vector for simulation
t = 0:0.01:10;

% Step response of the closed-loop system
step(closed_loop_sys, t);

% Add labels and title
title('Step Response with PID Controller');
xlabel('Time (s)');
ylabel('Output');
grid on;

% Display the step response information
stepinfo(closed_loop_sys)