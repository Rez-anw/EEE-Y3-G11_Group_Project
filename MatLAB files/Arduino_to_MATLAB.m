clear;
clc;

% **Step 1: Connect to Arduino**
arduino_port = "COM4";  % Change to your Arduino's COM port
baud_rate = 115200;     % Must match the Arduino Serial baud rate

try
    % Create serial connection with timeout
    serialObj = serialport(arduino_port, baud_rate, 'Timeout', 5);
    configureTerminator(serialObj, "LF"); % Ensure correct line termination
    flush(serialObj);  % Clear any old data in buffer
    disp("Successfully connected to Arduino on " + arduino_port);
catch ME
    error("Failed to connect to Arduino: " + ME.message);
end

% **Step 2: Preallocate Data Storage**
num_samples = 500;
data = NaN(num_samples, 4); % Preallocate for [X1, Y1, X2, Y2]
time = NaN(num_samples, 1); % Preallocate time array
index = 1;
t0 = tic;

% **Step 3: Read Data from Arduino**
disp("Reading data from Arduino...");
disp("Make sure Arduino is sending data in format: X1: value, Y1: value, X2: value, Y2: value");

while index <= num_samples
    if serialObj.NumBytesAvailable > 0
        try
            line = strtrim(readline(serialObj));  % Read and trim one line
            fprintf("Raw line: '%s'\n", line); % Debug output
            
            % Regular expression to extract numeric values from the string
            expr = '(?<=[:\s])(-?\d+\.?\d*)';  % Match numbers following ':' or space
            values = str2double(regexp(line, expr, 'match'));  % Extract numeric values
            
            if length(values) == 4 && all(~isnan(values))  % Expect 4 numeric values
                data(index, :) = values';  % Store the 4 variables
                time(index) = toc(t0);
                index = index + 1;
                
                % Print real-time data for debugging
                fprintf("Sample %d: X1: %.2f, Y1: %.2f, X2: %.2f, Y2: %.2f at %.2fs\n", ...
                    index-1, values(1), values(2), values(3), values(4), time(index-1));
            else
                disp("Warning: Invalid data format. Expected 4 numeric values.");
            end
        catch ME
            disp("Error reading data: " + ME.message);
        end
    end
    
    % Add timeout check
    if toc(t0) > 15 && index == 1
        warning("Timeout: No valid data received. Check Arduino connection and data format.");
        break;
    end
    
    % Small pause to prevent CPU overuse
    pause(0.01);
end

% **Step 4: Trim Unused Data**
valid_entries = ~isnan(data(:,1));
data = data(valid_entries, :);
time = time(valid_entries);

% **Step 5: Close Serial Connection**
clear serialObj;
disp("Data collection complete! Collected " + size(data,1) + " samples.");

% **Step 6: Calculate MSE (Mean Squared Error)**
if ~isempty(data)
    % MSE for X and Y axes
    error_x = data(:,1) - data(:,3);  % Difference between current X and desired X
    error_y = data(:,2) - data(:,4);  % Difference between current Y and desired Y
    
    mse_x = mean(error_x.^2);  % MSE for X axis
    mse_y = mean(error_y.^2);  % MSE for Y axis
    
    disp(['MSE for X axis: ', num2str(mse_x)]);
    disp(['MSE for Y axis: ', num2str(mse_y)]);
    
    % Optional: Plot error trends
    figure;
    
    % X-Axis error plot
    subplot(2,1,1);
    plot(time, error_x, 'b', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Error');
    title('Error in X Axis');
    grid on;
    
    % Y-Axis error plot
    subplot(2,1,2);
    plot(time, error_y, 'r', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Error');
    title('Error in Y Axis');
    grid on;
    
    % Save the figure
    saveas(gcf, 'Error_Analysis.png');
else
    disp("No valid data collected. Check Arduino connection and data format.");
end

% **Optional: Save data to CSV for further analysis**
if ~isempty(data)
    csv_data = [time, data];
    csvwrite('tracking_data.csv', csv_data);
    disp("Data saved to 'tracking_data.csv' for further analysis.");
end
