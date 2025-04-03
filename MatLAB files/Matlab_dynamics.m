%% Step 1: Set up Serial Communication with Arduino
serial_port = "COM4"; % Change this to your Arduino port
baud_rate = 115200;
arduino = serialport(serial_port, baud_rate);
flush(arduino); % Clear any old data

%% Step 2: Read Data from Arduino
samples = 1000;  % Number of data points to collect
data = zeros(samples, 3); % Pre-allocate for speed

disp("Collecting Data...");
for i = 1:samples
    raw = readline(arduino); % Read a line of data
    
    if numel(values) == 3
        data(i, :) = values'; % Store in array
    end
end
disp("Data Collection Complete!");

% Convert milliseconds to seconds
time = (data(:,1) - data(1,1)) / 1000;  
input_signal = data(:,2);
position = data(:,3);

% Remove duplicate or non-increasing time values
[time, unique_idx] = unique(time, 'stable');
input_signal = input_signal(unique_idx);
position = position(unique_idx);

% Estimate sampling time
Ts = mean(diff(time));

% If time is irregular, set Ts = []
if Ts <= 0
    Ts = [];
end

% Create iddata object
id_data = iddata(position, input_signal, Ts);


%% Step 7: Simulate Step Response
figure;
step(sys_tf);
title("Estimated System Step Response");

%% Step 8: Plot Raw Data
figure;
plot(time, position, 'b', 'LineWidth', 1.5);
hold on;
plot(time, input_signal, 'r--', 'LineWidth', 1);
legend("Position Output", "Input Signal");
xlabel("Time (s)");
ylabel("Magnitude");
title("Arduino Data - Input vs Output");
grid on;
