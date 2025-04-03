clc; clear; close all;

% Constants
g = 9.81; % Gravity (m/s^2)

% Acceleration vs. Angle (in mm/s²)
angles = 0:1:20; % Angles from 0 to 45 degrees in 3-degree increments
acceleration = g * sind(angles); % Convert to mm/s^2

% figure;
% plot(angles, acceleration, 'LineWidth', 2);
% xlabel('Slope Angle (degrees)');
% ylabel('Acceleration (mm/s²)');
% title("Ball's Acceleration vs. Board'd Slope Angle");
% grid on;
% grid minor;

% Displacement vs. Time & Velocity vs. Time
theta = 20; % Fixed slope angle for motion calculations
a = g * sind(theta); % Acceleration along the slope (m/s²)

time = 0:0.1:10; % Time from 0 to 10 seconds
velocity = a * time; % v = at
displacement = 0.5 * a * time.^2; % s = (1/2) * a * t^2

% figure;
% plot(time, displacement, 'r-', 'LineWidth', 2);
% xlabel('Time (s)');
% ylabel('Displacement (m)');
% title('Displacement vs. Time');
% grid on;
% 
% figure;
% plot(time, velocity, 'g-', 'LineWidth', 2);
% xlabel('Time (s)');
% ylabel('Velocity (m/s)');
% title('Velocity vs. Time');
% grid on;



L = 0.2; % Length of the bar in mm
angles = 0:1:15; % Angles from 0 to 90 degrees

% Calculate height for each angle
height = L * sind(angles); % h = L * sin(theta)

% Create a single figure window
figure;

% % First subplot: Angle vs. Height
% subplot(1,2,1); % 1 row, 2 columns, first plot
% plot(angles, height, 'LineWidth', 2);
% xlabel('Angle (degrees)');
% ylabel('Height (m)');
% title("Board Angle vs. Tilt Height of Board");
% grid on;
% grid minor;
% 
% % Second subplot: Height vs. Servo Angle
% r = 0.020; % Servo radius in mm
% height = 0:0.01:0.05; 
% 
% % Calculate servo angle in degrees
% servo_angle = (height / r) * (180 / pi); % Convert radians to degrees
% 
% subplot(1,2,2); % 1 row, 2 columns, second plot
% plot(height, servo_angle, 'LineWidth', 2);
% xlabel('Height (m)');
% ylabel('Servo Angle (degrees)');
% title('Tilt Height of Board vs. Servo Angle');
% grid on;
% grid minor;



% Define parameters
L = 238;  % Length of the board (mm)
r = 20;   % Servo arm radius (mm)
angles = 0:1:10;  % Board tilt angles in degrees

% Calculate height change due to tilt
height = L * sind(angles);  

% Map height to servo angle
servo_angle = (height / r) * (180 / pi);  % Convert radians to degrees

% Plot Board Angle vs. Servo Angle
figure;
plot(angles, servo_angle, 'LineWidth', 2);
xlabel('Board Tilt Angle (degrees)');
ylabel('Servo Angle (degrees)');
title('Mapping Board Angle to Servo Angle');
grid on;
grid minor;



% Define constants
g = 9.81; % Acceleration due to gravity (m/s^2)
t = 1; % Time duration (seconds) - Adjust as needed
angles = 0:1:10; % Board angles in degrees

% Calculate velocity of the ball
velocity = g * t * sind(angles);

% Plot board angle vs. ball velocity
figure;
plot(angles, velocity, 'LineWidth', 2);
xlabel('Board Tilt Angle (degrees)');
ylabel('Ball Velocity (m/s)');
title('Ball Velocity vs. Board Angle');
grid on;
grid minor;



