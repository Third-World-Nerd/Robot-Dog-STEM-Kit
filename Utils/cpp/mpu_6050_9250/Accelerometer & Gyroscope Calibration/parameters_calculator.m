clc
clear

mpu_table = readtable('LoggedData.csv');
mpu = table2array(mpu_table);

time = mpu(:,1);           % Extract time column (assumed in seconds or milliseconds)
mpu = mpu(:,2:7);          % Sensor data

accelX = mpu(:,1);
accelY = mpu(:,2);
accelZ = mpu(:,3);

gyroX = mpu(:,4);
gyroY = mpu(:,5);
gyroZ = mpu(:,6);

% Calculate sampling rate
timeInit = time(1);
timeFinal = time(end);
totalTime = timeFinal - timeInit;

numSamples = length(time);
samplingRate = (numSamples - 1) / totalTime * 1000;  % Hz

fprintf('Sampling Rate: %.2f Hz\n', samplingRate);
% calculate the mean of the data
mean_accelX = mean(accelX);
mean_accelY = mean(accelY);
mean_accelZ = mean(accelZ);

mean_gyroX = mean(gyroX);
mean_gyroY = mean(gyroY);
mean_gyroZ = mean(gyroZ);

% Print in Arduino-style #define format
fprintf('#define GYRO_X_OFFSET %.7f\n', mean_gyroX);
fprintf('#define GYRO_Y_OFFSET %.7f\n', mean_gyroY);
fprintf('#define GYRO_Z_OFFSET %.7f\n', mean_gyroZ);

fprintf('\n');

fprintf('#define ACCEL_X_OFFSET %.7f\n', mean_accelX);
fprintf('#define ACCEL_Y_OFFSET %.7f\n', mean_accelY);
fprintf('#define ACCEL_Z_OFFSET %.7f\n', mean_accelZ);

% calculate the difference between each data point and the mean
diff_accelX = accelX-mean_accelX;
diff_accelY = accelY-mean_accelY;
diff_accelZ = accelZ-mean_accelZ;

diff_gyroX = gyroX-mean_gyroX;
diff_gyroY = gyroY-mean_gyroY;
diff_gyroZ = gyroZ-mean_gyroZ;


% calculate the Markov Bias for each difference value
markov_bias_accelX = abs(diff(diff_accelX));
markov_bias_accelY = abs(diff(diff_accelY));
markov_bias_accelZ = abs(diff(diff_accelZ));

markov_bias_gyroX = abs(diff(diff_gyroX));
markov_bias_gyroY = abs(diff(diff_gyroY));
markov_bias_gyroZ = abs(diff(diff_gyroZ));

% calculate the standard deviation of the Markov Bias values
std_markov_bias_accelX = std(markov_bias_accelX);
std_markov_bias_accelY = std(markov_bias_accelY);
std_markov_bias_accelZ = std(markov_bias_accelZ);

std_markov_bias_gyroX = std(markov_bias_gyroX);
std_markov_bias_gyroY = std(markov_bias_gyroY);
std_markov_bias_gyroZ = std(markov_bias_gyroZ);


TAU_AX = sqrt(3)/(2*pi*std_markov_bias_accelX);
TAU_AY = sqrt(3)/(2*pi*std_markov_bias_accelY);
TAU_AZ = sqrt(3)/(2*pi*std_markov_bias_accelZ);

TAU_GX = sqrt(3)/(2*pi*std_markov_bias_gyroX);
TAU_GY = sqrt(3)/(2*pi*std_markov_bias_gyroY);
TAU_GZ = sqrt(3)/(2*pi*std_markov_bias_gyroZ);

% --- Q Matrix (Gyroscope variance) ---
var_gyroX = var(gyroX - mean_gyroX);
var_gyroY = var(gyroY - mean_gyroY);
var_gyroZ = var(gyroZ - mean_gyroZ);

fprintf('// Gyro process noise covariance Q (rad^2/s^2)\n');
fprintf('const double Q[3] = {%.16f, %.16f, %.16f};\n\n', var_gyroX, var_gyroY, var_gyroZ);

% --- R Matrix (Accelerometer + Magnetometer) ---
var_accelX = var(accelX - mean_accelX);
var_accelY = var(accelY - mean_accelY);
var_accelZ = var(accelZ - mean_accelZ);

fprintf('// Measurement noise covariance R (accelerometer + magnetometer)\n');
fprintf('const double R[6] = {%.16f, %.16f, %.16f};\n',var_accelX, var_accelY, var_accelZ);
