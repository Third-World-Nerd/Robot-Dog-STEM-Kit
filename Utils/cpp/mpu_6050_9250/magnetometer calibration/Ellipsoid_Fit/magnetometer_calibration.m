% read magnetometer data from Processing Sketch directory
% Input the path of csv file produced by Collect_Data.py
close all
clear all
clc

M = readmatrix('LoggedData.csv');

timeInit = M(1,1);
timeFinal = M(end,1);
totalTime = timeFinal - timeInit;      % Total duration in seconds (assuming time is in seconds)
numSamples = size(M, 1);               % Total number of samples
samplingRate = (numSamples - 1) / totalTime *1000;  % Hz
fprintf('Sampling Rate: %.2f Hz\n', samplingRate);

x = M(:,2);
y = M(:,3);
z = M(:,4);

% do ellipsoid fitting
[e_center, e_radii, e_eigenvecs, e_algebraic] = ellipsoid_fit([x, y, z]);

% compensate distorted magnetometer data
S = [x - e_center(1), y - e_center(2), z - e_center(3)]'; % translate
scale = [e_radii(1) 0 0; 0 e_radii(2) 0; 0 0 e_radii(3)] \ eye(3) * min(e_radii); % scaling matrix
map = e_eigenvecs';   % rotation to align axes
invmap = e_eigenvecs; % inverse rotation
comp = invmap * scale * map;
S = comp * S; % final compensated data

% output for Arduino
fprintf('In the Razor_AHRS.ino, under "SENSOR CALIBRATION" find the section that reads "Magnetometer (extended calibration)"\n');
fprintf('Replace the existing 2 lines with these:\n\n');
fprintf('const float magn_ellipsoid_center[3] = {%.6g, %.6g, %.6g};\n', e_center);
fprintf('const float magn_ellipsoid_transform[3][3] = {{%.6g, %.6g, %.6g}, {%.6g, %.6g, %.6g}, {%.6g, %.6g, %.6g}};\n', comp);

% draw ellipsoid fit
figure;
hold on;
plot3(x, y, z, '.r'); % original data

maxd = max(e_radii);
step = maxd / 50;
[xp, yp, zp] = meshgrid(-maxd:step:maxd + e_center(1), -maxd:step:maxd + e_center(2), -maxd:step:maxd + e_center(3));

Ellipsoid = e_algebraic(1) *xp.*xp +   e_algebraic(2) * yp.*yp + e_algebraic(3)   * zp.*zp + ...
          2*e_algebraic(4) *xp.*yp + 2*e_algebraic(5) * xp.*zp + 2*e_algebraic(6) * yp.*zp + ...
          2*e_algebraic(7) *xp     + 2*e_algebraic(8) * yp     + 2*e_algebraic(9) * zp;
p = patch(isosurface(xp, yp, zp, Ellipsoid, 1));
set(p, 'FaceColor', 'g', 'EdgeColor', 'none');

alpha(0.5);
view(-70, 40);
axis vis3d;
axis equal;
camlight;
lighting phong;

% draw original and compensated data
figure;
hold on;
plot3(x, y, z, '.r');              % original magnetometer data
plot3(S(1,:), S(2,:), S(3,:), 'b.'); % compensated data
view(-70, 40);
axis vis3d;
axis equal;

%% === Compute R_mag from compensated data ===

mag_norm = vecnorm(S);
var_mag = var(mag_norm);
R_mag = diag([var_mag/3, var_mag/3, var_mag/3]);

fprintf('const double R_mag[3] = {%.16f, %.16f, %.16f};\n',R_mag(1,1), R_mag(2,2), R_mag(3,3));

