% Define serial port and baud rate
serialPort = 'COM4'; % Change to match your Arduino's port
baudRate = 115200;   % Ensure it matches the Arduino code
s = serialport(serialPort, baudRate);

% Set serial properties
configureTerminator(s, "LF"); % Use Line Feed as terminator
flush(s); % Clear any old data
s.Timeout = 10; % Set timeout for serial read

% Number of samples to collect
numSamples = 50;
data = zeros(numSamples, 3); % Preallocate for speed
fprintf('Collecting %d samples from Arduino...\n', numSamples);

% Collect exactly 400 samples
sampleCount = 0;
while sampleCount < numSamples
    try
        line = readline(s); % Read a line from serial
        values = str2double(strsplit(line, ',')); % Convert to numbers
        
        if numel(values) == 3 && all(~isnan(values)) % Ensure correct format
            sampleCount = sampleCount + 1;
            data(sampleCount, :) = values;
        end
    catch
        warning('Error reading data, skipping...');
    end
end

% Close the serial connection
clear s;
fprintf('Data collection complete.\n');

% Save data to CSV
csvFileName = 'pid_data.csv';
writematrix(data, csvFileName);
fprintf('Data saved to %s\n', csvFileName);


% Extract data columns
X1 = data(:, 1);  % Assuming X1 is in the first column
Y1 = data(:, 2);  % Assuming Y1 is in the second column 

% Plot system response
figure;
plot(time, setpoint, 'r--', 'LineWidth', 1.5); hold on;
plot(time, output, 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Response');
legend('Setpoint', 'Output');
title('PID System Response');
grid on;

% Estimate PID gains using system identification
sys_id = tfest(iddata(output, setpoint, mean(diff(time))), 2, 0);

% Approximate PID tuning using Ziegler-Nichols or another heuristic method
Ku = max(output) / max(setpoint); % Ultimate gain estimation
Tu = mean(diff(time)); % Estimate oscillation period (can refine later)

% Use Ziegler-Nichols method for initial estimates
Kp = 0.6 * Ku;
Ki = 1.2 * Ku / Tu;
Kd = 0.075 * Ku * Tu;

% Display results
fprintf('Estimated PID Gains (Ziegler-Nichols Approximation):\n');
fprintf('Kp = %.4f\n', Kp);
fprintf('Ki = %.4f\n', Ki);
fprintf('Kd = %.4f\n', Kd);
