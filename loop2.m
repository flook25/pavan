clear all
close all
clc

% ==========================================
% 1. PARAMETERS & INPUT
% ==========================================
L1 = 0.210; % Ground (Pink)
L2 = 0.118; % Crank (Cyan) - Input Link
L3 = 0.210; % Coupler (Red)
L4 = 0.118; % Rocker (Grey)

d = L1;
a = L2;
b = L3;
c = L4;

offset_deg = 0.81;
offset = deg2rad(offset_deg);

% Input from Previous Loop
theta_cyan_prev = 386.3814; 

% Rotate 180 degrees (Connected Link)
theta2_global = theta_cyan_prev + 180;

% Convert to Local Frame for Calculation
q_in = deg2rad(theta2_global) - offset; 

% ==========================================
% 2. CALCULATION (Standard Method)
% ==========================================
% Input: q_in (Theta 2 / Cyan)
% Output: Theta 4 (Grey) and Theta 3 (Red)

% Standard K Constants
K1 = d/a;
K2 = d/c;
K3 = (a^2 - b^2 + c^2 + d^2)/(2*a*c);
K4 = d/b;
K5 = (c^2 - d^2 - a^2 - b^2)/(2*a*b);

% Coefficients for Theta 4 (Grey)
A = cos(q_in) - K1 - K2*cos(q_in) + K3;
B = -2*sin(q_in);
C = K1 - (K2+1)*cos(q_in) + K3;

% Coefficients for Theta 3 (Red)
D_val = cos(q_in) - K1 + K4*cos(q_in) + K5;
E_val = -2*sin(q_in);
F_val = K1 + (K4-1)*cos(q_in) + K5;

% Roots
det_AC = sqrt(B^2 - 4*A*C);
det_DF = sqrt(E_val^2 - 4*D_val*F_val);

% --- CASE 1: OPEN ---
% Note: In Standard Method, (-B - sqrt) usually gives Open config
q4_loc_1 = 2*atan2((-B - det_AC), (2*A));
q3_loc_1 = 2*atan2((-E_val - det_DF), (2*D_val));

% --- CASE 2: CROSSED ---
q4_loc_2 = 2*atan2((-B + det_AC), (2*A));
q3_loc_2 = 2*atan2((-E_val + det_DF), (2*D_val));

% ==========================================
% 3. TRANSFORM BACK & DISPLAY
% ==========================================
% Add Offset back to get Global Angles
T4_Open = q4_loc_1 + offset;
T3_Open = q3_loc_1 + offset;

T4_Cross = q4_loc_2 + offset;
T3_Cross = q3_loc_2 + offset;

disp('======================================');
disp(['LOOP 2 RESULTS (Input Cyan = ' num2str(theta_cyan_prev) ' + 180)']);
disp('======================================');
disp('--- CASE 1 (OPEN) ---');
disp(['  Grey (Theta 4): ', num2str(rad2deg(T4_Open))]);
disp(['  Red  (Theta 3): ', num2str(rad2deg(T3_Open))]);
disp(' ');
disp('--- CASE 2 (CROSSED) ---');
disp(['  Grey (Theta 4): ', num2str(rad2deg(T4_Cross))]);
disp(['  Red  (Theta 3): ', num2str(rad2deg(T3_Cross))]);

% ==========================================
% 4. PLOTTING
% ==========================================
Rot = exp(1j * offset);
R1 = d * exp(1j * offset); % Ground

% Input Vector (Cyan - Link 2)
R2_Vec = a * exp(1j * (deg2rad(theta2_global))); % Global directly

% --- Case 1 Vectors ---
R4_Op = c * exp(1j * T4_Open); % Grey (from O4)
R3_Op = (R1 + R4_Op) - R2_Vec; % Red (from A to B)

% --- Case 2 Vectors ---
R4_Cr = c * exp(1j * T4_Cross);
R3_Cr = (R1 + R4_Cr) - R2_Vec;

figure(2); clf;

% Subplot 1: Open
subplot(1,2,1); hold on; grid on; axis equal;
title('Loop 2: Case 1 (Open)');
plot([0 real(R1)], [0 imag(R1)], 'm-', 'LineWidth', 2); % Ground
quiver(0, 0, real(R2_Vec), imag(R2_Vec), 0, 'c', 'LineWidth', 2, 'MaxHeadSize',0.5); % Cyan (Input)
quiver(real(R1), imag(R1), real(R4_Op), imag(R4_Op), 0, 'Color', [0.5 0.5 0.5], 'LineWidth', 2, 'MaxHeadSize',0.5); % Grey
quiver(real(R2_Vec), imag(R2_Vec), real(R3_Op), imag(R3_Op), 0, 'r', 'LineWidth', 2, 'MaxHeadSize',0.5); % Red

% Subplot 2: Crossed
subplot(1,2,2); hold on; grid on; axis equal;
title('Loop 2: Case 2 (Crossed)');
plot([0 real(R1)], [0 imag(R1)], 'm-', 'LineWidth', 2); % Ground
quiver(0, 0, real(R2_Vec), imag(R2_Vec), 0, 'c', 'LineWidth', 2, 'MaxHeadSize',0.5); % Cyan (Input)
quiver(real(R1), imag(R1), real(R4_Cr), imag(R4_Cr), 0, 'Color', [0.5 0.5 0.5], 'LineWidth', 2, 'MaxHeadSize',0.5); % Grey
quiver(real(R2_Vec), imag(R2_Vec), real(R3_Cr), imag(R3_Cr), 0, 'r', 'LineWidth', 2, 'MaxHeadSize',0.5); % Red
