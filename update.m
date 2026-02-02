clear all
close all
clc

% ==========================================
% 1. PARAMETERS & INPUT
% ==========================================
% Lengths (meters) based on your request
d = 0.210; % Ground (L1) [cite: 522]
a = 0.118; % Link 2 (Crank) [cite: 522]
b = 0.210; % Link 3 (Coupler) [cite: 522]
c = 0.118; % Link 4 (Rocker/Input here) [cite: 522]

% Input Angles
theta4_deg_global = 102.05; % Input angle for Link 4
offset_deg = 0.81;          % Ground offset angle

% Convert to Radians
offset = deg2rad(offset_deg);
% Convert global input to local frame (relative to ground link d)
theta4_local = deg2rad(theta4_deg_global) - offset;

% ==========================================
% 2. CALCULATION (Inverted K-Method)
% ==========================================
% The standard Norton formulas solve for Theta4 given Theta2. [cite: 383, 418]
% Since we have Theta4 and need Theta2, we swap the roles of 'a' and 'c'.
% Imagine Link 4 is the driver (crank) and Link 2 is the follower.

% Temporary lengths for the calculation
a_prime = c; % Link 4 acts as driver
c_prime = a; % Link 2 acts as follower
b_prime = b; % Coupler remains coupler
d_prime = d; % Ground remains ground

% Calculate K constants [cite: 376, 377, 378]
K1 = d_prime / a_prime;
K2 = d_prime / c_prime;
K3 = (a_prime^2 - b_prime^2 + c_prime^2 + d_prime^2) / (2 * a_prime * c_prime);

% Calculate A, B, C coefficients using Theta4 as input [cite: 415, 416]
% Note: Variable is theta4_local because K-equations assume horizontal ground
A_val = cos(theta4_local) - K1 - K2*cos(theta4_local) + K3;
B_val = -2 * sin(theta4_local);
C_val = K1 - (K2 + 1)*cos(theta4_local) + K3;

% Solve for Theta2 (Open and Crossed configurations) [cite: 418]
% We are solving for theta2 because we swapped inputs
det_root = sqrt(B_val^2 - 4*A_val*C_val);

% Solution 1 (Open)
theta2_local_1 = 2 * atan2((-B_val - det_root), (2 * A_val));
% Solution 2 (Crossed)
theta2_local_2 = 2 * atan2((-B_val + det_root), (2 * A_val));

% Calculate Theta3 for both cases using vector components [cite: 51, 56]
% R3 = R1 + R4 - R2  =>  b*e^j*t3 = d + c*e^j*t4 - a*e^j*t2
R1 = d;
R4 = c * exp(1i * theta4_local);

% Case 1
R2_1 = a * exp(1i * theta2_local_1);
R3_vector_1 = R1 + R4 - R2_1;
theta3_local_1 = angle(R3_vector_1);

% Case 2
R2_2 = a * exp(1i * theta2_local_2);
R3_vector_2 = R1 + R4 - R2_2;
theta3_local_2 = angle(R3_vector_2);

% Convert back to Global Angles
theta2_global_1 = rad2deg(theta2_local_1 + offset);
theta3_global_1 = rad2deg(theta3_local_1 + offset);
theta2_global_2 = rad2deg(theta2_local_2 + offset);
theta3_global_2 = rad2deg(theta3_local_2 + offset);

% Display Results
disp('=== RESULTS FOR LOOP 1 ===');
disp(['Input Theta4: ', num2str(theta4_deg_global)]);
disp(' ');
disp('--- Configuration 1 ---');
disp(['Theta2 (Link 2): ', num2str(theta2_global_1)]);
disp(['Theta3 (Link 3): ', num2str(theta3_global_1)]);
disp(' ');
disp('--- Configuration 2 ---');
disp(['Theta2 (Link 2): ', num2str(theta2_global_2)]);
disp(['Theta3 (Link 3): ', num2str(theta3_global_2)]);

% ==========================================
% 3. PLOTTING VECTORS [cite: 9, 219]
% ==========================================
figure(1); clf; hold on; grid on; axis equal;
title(['Four-Bar Loop 1 (Input \theta_4 = ' num2str(theta4_deg_global) '^\circ)']);
xlabel('X (m)'); ylabel('Y (m)');

% Define Pivot Points (Global Frame)
O2 = 0 + 0i;
O4 = d * exp(1i * offset);

% Choose Configuration 1 to plot (Change to _2 to see the crossed circuit)
t2_plot = theta2_local_1 + offset;
t3_plot = theta3_local_1 + offset;
t4_plot = theta4_local + offset;

% Vector positions
R_O2 = O2;              % Start Point
R_A = O2 + a * exp(1i * t2_plot); % Tip of Link 2
R_B = O4 + c * exp(1i * t4_plot); % Tip of Link 4 (calculated from O4)
R_O4 = O4;              % End Point

% Plot Links
plot([real(R_O2), real(R_A)], [imag(R_O2), imag(R_A)], 'g-o', 'LineWidth', 2, 'DisplayName', 'Link 2 (a)');
plot([real(R_A), real(R_B)], [imag(R_A), imag(R_B)], 'b-o', 'LineWidth', 2, 'DisplayName', 'Link 3 (b)');
plot([real(R_O4), real(R_B)], [imag(R_O4), imag(R_B)], 'k-o', 'LineWidth', 2, 'DisplayName', 'Link 4 (c)');
plot([real(R_O2), real(R_O4)], [imag(R_O2), imag(R_O4)], 'm--', 'LineWidth', 2, 'DisplayName', 'Ground (d)');

legend('Location', 'best');
