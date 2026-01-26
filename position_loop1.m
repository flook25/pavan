clear all
close all
clc

% ==========================================
% 1. PARAMETERS (Loop 3 from your code)
% ==========================================
L1 = 0.210; % Ground (Pink)
L2 = 0.118; % Crank (Cyan) - We want to FIND this
L3 = 0.210; % Coupler (Blue) - We want to FIND this
L4 = 0.118; % Rocker (Brown) - We KNOW this (Input)

d = L1;
a = L2;
b = L3;
c = L4;

% Ground Offset
offset_deg = 0.81;
offset = deg2rad(offset_deg);

% ==========================================
% 2. INPUT: Theta 4 (Brown Link)
% ==========================================
q4d_global = 102.5; % Input Angle
q4 = deg2rad(q4d_global) - offset; % Local Angle

% ==========================================
% 3. CALCULATION: Find Theta 2 (Cyan) from Theta 4
% ==========================================

% --- Define K Constants (Swapped for Inverse) ---
% ปกติ K1=d/a แต่เราสลับบทบาท a กับ c เพื่อหา q2 จาก q4
K1_inv = d/c; 
K2_inv = d/a; 
K3_inv = (c^2 - b^2 + a^2 + d^2)/(2*c*a); 

% --- Coefficients for Theta 2 ---
A = cos(q4) - K1_inv - K2_inv*cos(q4) + K3_inv;
B = -2*sin(q4);
C = K1_inv - (K2_inv + 1)*cos(q4) + K3_inv;

% --- Solve for Theta 2 (Cyan) ---
disc = B^2 - 4*A*C;

% Case 1: Parallel Mode (Cyan and Brown usually parallel)
q2_sol1 = 2*atan((-B - sqrt(disc))/(2*A));

% Case 2: Crossed Mode (Cyan and Brown crossed)
q2_sol2 = 2*atan((-B + sqrt(disc))/(2*A));


% ==========================================
% 4. CALCULATION: Find Theta 3 (Blue) from Theta 2
% ==========================================
% Standard Constants for finding q3
K1 = d/a;
K4 = d/b;
K5 = (c^2 - d^2 - a^2 - b^2)/(2*a*b);

% --- Case 1 (Using q2_sol1) ---
D1 = cos(q2_sol1) - K1 + K4*cos(q2_sol1) + K5;
E1 = -2*sin(q2_sol1);
F1 = K1 + (K4 - 1)*cos(q2_sol1) + K5;
% Parallel mode usually uses -sqrt for q3
q3_sol1 = 2*atan((-E1 - sqrt(E1^2 - 4*D1*F1))/(2*D1));

% --- Case 2 (Using q2_sol2) ---
D2 = cos(q2_sol2) - K1 + K4*cos(q2_sol2) + K5;
E2 = -2*sin(q2_sol2);
F2 = K1 + (K4 - 1)*cos(q2_sol2) + K5;
% Crossed mode usually uses +sqrt for q3
q3_sol2 = 2*atan((-E2 + sqrt(E2^2 - 4*D2*F2))/(2*D2));


% Convert to Degrees (Global)
q2_d1 = rad2deg(q2_sol1) + offset_deg;
q3_d1 = rad2deg(q3_sol1) + offset_deg;

q2_d2 = rad2deg(q2_sol2) + offset_deg;
q3_d2 = rad2deg(q3_sol2) + offset_deg;


% ==========================================
% 5. VECTORS & PLOTTING
% ==========================================
RO4O2 = d*exp(j*offset);
RO4O2x = real(RO4O2); RO4O2y = imag(RO4O2);

% --- Vectors Case 1 ---
V_Cyan_1 = a * exp(j*(q2_sol1 + offset));
V_Blue_1 = b * exp(j*(q3_sol1 + offset));
V_Brown_1 = c * exp(j*(q4 + offset)); % Input

% --- Vectors Case 2 ---
V_Cyan_2 = a * exp(j*(q2_sol2 + offset));
V_Blue_2 = b * exp(j*(q3_sol2 + offset));
V_Brown_2 = c * exp(j*(q4 + offset)); % Input

% --- Plot Case 1 ---
figure(1)
title(['Case 1: Parallel Mode (Input Brown = ' num2str(q4d_global) ')']); hold on;
quiver(0,0, RO4O2x, RO4O2y, 0, 'Color', [1 0 1], 'LineWidth', 4, 'MaxHeadSize', 0.5); % Pink
quiver(0,0, real(V_Cyan_1), imag(V_Cyan_1), 0, 'cyan', 'LineWidth', 3, 'MaxHeadSize', 0.5); % Cyan (Found)
quiver(real(V_Cyan_1), imag(V_Cyan_1), real(V_Blue_1), imag(V_Blue_1), 0, 'blue', 'LineWidth', 3, 'MaxHeadSize', 0.5); % Blue (Found)
quiver(RO4O2x, RO4O2y, real(V_Brown_1), imag(V_Brown_1), 0, 'Color', [0.85 0.5 0.1], 'LineWidth', 3, 'MaxHeadSize', 0.5); % Brown (Input)
axis equal; grid on;

% --- Plot Case 2 ---
figure(2)
title(['Case 2: Crossed Mode (Input Brown = ' num2str(q4d_global) ')']); hold on;
quiver(0,0, RO4O2x, RO4O2y, 0, 'Color', [1 0 1], 'LineWidth', 4, 'MaxHeadSize', 0.5); % Pink
quiver(0,0, real(V_Cyan_2), imag(V_Cyan_2), 0, 'cyan', 'LineWidth', 3, 'MaxHeadSize', 0.5); % Cyan (Found)
quiver(real(V_Cyan_2), imag(V_Cyan_2), real(V_Blue_2), imag(V_Blue_2), 0, 'blue', 'LineWidth', 3, 'MaxHeadSize', 0.5); % Blue (Found)
quiver(RO4O2x, RO4O2y, real(V_Brown_2), imag(V_Brown_2), 0, 'Color', [0.85 0.5 0.1], 'LineWidth', 3, 'MaxHeadSize', 0.5); % Brown (Input)
axis equal; grid on;

% Display Results
disp('--- Results from Brown Input ---');
disp(['Input Brown (Theta 4): ', num2str(q4d_global)]);
disp(['Case 1: Cyan (Theta 2) = ', num2str(q2_d1), ', Blue (Theta 3) = ', num2str(q3_d1)]);
disp(['Case 2: Cyan (Theta 2) = ', num2str(q2_d2), ', Blue (Theta 3) = ', num2str(q3_d2)]);
