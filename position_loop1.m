clear all
close all
clc

% ==========================================
% 1. PARAMETERS & INPUT
% ==========================================
L1 = 0.210; % Ground (Pink) - d
L2 = 0.118; % Crank (Cyan/Red) - a
L3 = 0.210; % Coupler (Blue) - b
L4 = 0.118; % Rocker (Brown/Black) - c

a = L2;
b = L3;
c = L4;
d = L1;

% Ground Offset (Global rotation)
theta1_deg = 0.81;
theta1 = deg2rad(theta1_deg);

% --- INPUT: Link 4 (Brown) ---
q4_global_deg = -102.5; 
q4 = deg2rad(q4_global_deg) - theta1; % Local Angle

% ==========================================
% 2. CALCULATION (Find q2 given q4)
% ==========================================

% Norton Constants (Standard Definitions)
K1 = d/a;
K2 = d/c;
K3 = (a^2 - b^2 + c^2 + d^2)/(2*a*c);

% Coefficients for Quadratic Equation A*t^2 + B*t + C = 0
% Derived for finding q2 when q4 is known (Inverse Kinematics)
% Note: Formula adjusted to solve for q2
A = (1 + K1)*cos(q4) + K2 + K3;
B = -2*sin(q4);
C = (K1 - 1)*cos(q4) - K2 + K3;

% Solve for q2 (2 Solutions: Crossed and Open)
% q2_1 and q2_2
q2_1 = 2*atan((-B + sqrt(B^2-4*A*C))/(2*A));
q2_2 = 2*atan((-B - sqrt(B^2-4*A*C))/(2*A));

% Calculate q3 using Vector Loop Relationship
% b*e^(j*q3) = c*e^(j*q4) + d - a*e^(j*q2)
% We use angle() to find q3 directly consistent with the loop

% For Case 1 (q2_1)
Vec3_1 = c*exp(1j*q4) + d - a*exp(1j*q2_1);
q3_1 = angle(Vec3_1);

% For Case 2 (q2_2)
Vec3_2 = c*exp(1j*q4) + d - a*exp(1j*q2_2);
q3_2 = angle(Vec3_2);

% ==========================================
% 3. CONVERT TO GLOBAL
% ==========================================
q2_1g = q2_1 + theta1;
q3_1g = q3_1 + theta1;

q2_2g = q2_2 + theta1;
q3_2g = q3_2 + theta1;

q4g = q4 + theta1;

% Display Results
fprintf('--- Results ---\n');
fprintf('Case 1 (q2_1): %.2f deg, q3: %.2f deg\n', rad2deg(q2_1g), rad2deg(q3_1g));
fprintf('Case 2 (q2_2): %.2f deg, q3: %.2f deg\n', rad2deg(q2_2g), rad2deg(q3_2g));

% ==========================================
% 4. PLOTTING (MATCHING YOUR FORMAT)
% ==========================================
% Select Solution to Plot (e.g., Case 2 - Open)
q2_plot = q2_2g;
q3_plot = q3_2g;
q4_plot = q4g;

% Vectors in Global Frame
RA = a*exp(1j*q2_plot);          % Link 2 (Red)
RBA2 = b*exp(1j*q3_plot);        % Link 3 (Blue)
RB2 = RA + RBA2;                 % Position B from Origin (Green)

RO4O2 = d*exp(1j*theta1);        % Ground Vector (Black)
RBO42 = c*exp(1j*q4_plot);       % Link 4 Vector (Black - from O4)

% Extract Components
RAx = real(RA); RAy = imag(RA);
RBA2x = real(RBA2); RBA2y = imag(RBA2);
RB2x = real(RB2); RB2y = imag(RB2);
RO4O2x = real(RO4O2); RO4O2y = imag(RO4O2);
RBO42x = real(RBO42); RBO42y = imag(RBO42);

% Plotting
figure;
hold on; 

% 1. Link 2 (Red) - O2 to A
quiver(0, 0, RAx, RAy, 0, 'red', 'MaxHeadSize', 0.5, 'LineWidth', 2); 

% 2. Link 3 (Blue) - A to B
quiver(RAx, RAy, RBA2x, RBA2y, 0, 'blue', 'MaxHeadSize', 0.5, 'LineWidth', 2);

% 3. Position Vector B (Green) - Origin to B
quiver(0, 0, RB2x, RB2y, 0, 'green', 'MaxHeadSize', 0.5, 'LineWidth', 2);

% 4. Ground (Black) - O2 to O4
quiver(0, 0, RO4O2x, RO4O2y, 0, 'black', 'MaxHeadSize', 0.5, 'LineWidth', 2); 

% 5. Link 4 (Black) - O4 to B
quiver(RO4O2x, RO4O2y, RBO42x, RBO42y, 0, 'black', 'MaxHeadSize', 0.5, 'LineWidth', 2); 

axis equal;
grid on;
title('4-Bar Linkage Position (Using format from snippet)');
xlabel('X'); ylabel('Y');
