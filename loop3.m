clear all
close all
clc

% ==========================================
% 1. PARAMETERS & INPUT
% ==========================================
L1 = 0.210; % Ground (Pink)
L2 = 0.180; % Crank (Green) - Output
L3 = 0.180; % Coupler (Yellow)
L4 = 0.118; % Rocker (Grey) - Input

d = L1;
a = L2;
b = L3;
c = L4;

theta_grey_deg = 206.3841; 
offset_deg = 0.81;

offset = deg2rad(offset_deg);
q4_global = deg2rad(theta_grey_deg);
q4_local = q4_global - offset;

% ==========================================
% 2. CALCULATION (Inverted Method)
% ==========================================
% Step 1: หมุน Input ไป 180 องศา (Inverted Frame)
q_in_calc = q4_local - pi;

% Step 2: กำหนด Constants (Input=c, Output=a)
d_cal = d;
a_cal = c; % Link 4 is Input
b_cal = b;
c_cal = a; % Link 2 is Output

K1 = d_cal/a_cal;
K2 = d_cal/c_cal;
K3 = (a_cal^2 - b_cal^2 + c_cal^2 + d_cal^2)/(2*a_cal*c_cal);
K4 = d_cal/b_cal;
K5 = (c_cal^2 - d_cal^2 - a_cal^2 - b_cal^2)/(2*a_cal*b_cal);

% Coefficients
A = cos(q_in_calc) - K1 - K2*cos(q_in_calc) + K3;
B = -2*sin(q_in_calc);
C = K1 - (K2+1)*cos(q_in_calc) + K3;

D_val = cos(q_in_calc) - K1 + K4*cos(q_in_calc) + K5;
E_val = -2*sin(q_in_calc);
F_val = K1 + (K4-1)*cos(q_in_calc) + K5;

% Roots
det_AC = sqrt(B^2 - 4*A*C);
det_DF = sqrt(E_val^2 - 4*D_val*F_val);

% Case 1 (Open-like)
q2_loc_1 = 2*atan2((-B - det_AC), (2*A));
q3_loc_1 = 2*atan2((-E_val - det_DF), (2*D_val));

% Case 2 (Crossed-like)
q2_loc_2 = 2*atan2((-B + det_AC), (2*A));
q3_loc_2 = 2*atan2((-E_val + det_DF), (2*D_val));

% Step 3: หมุนกลับ 180 องศา (Global Frame)
T2_Open = q2_loc_1 + pi + offset;
T3_Open = q3_loc_1 + pi + offset;

T2_Cross = q2_loc_2 + pi + offset;
T3_Cross = q3_loc_2 + pi + offset;

% ==========================================
% 3. DISPLAY RESULTS
% ==========================================
disp('======================================');
disp(['LOOP 3 RESULTS (Input Grey = ' num2str(theta_grey_deg) ')']);
disp('======================================');
disp('--- CASE 1 (Open) ---');
disp(['  Green  (Theta 2): ', num2str(rad2deg(T2_Open))]);
disp(['  Yellow (Theta 3): ', num2str(rad2deg(T3_Open))]);
disp(' ');
disp('--- CASE 2 (Crossed) ---');
disp(['  Green  (Theta 2): ', num2str(rad2deg(T2_Cross))]);
disp(['  Yellow (Theta 3): ', num2str(rad2deg(T3_Cross))]);

% ==========================================
% 4. PLOTTING
% ==========================================
Rot = exp(1j * offset);
R1 = d * exp(1j * offset); % Ground

% Input Vector (Grey - Link 4) from O4
R4_Vec = c * exp(1j * q4_global);

% --- Case 1 Vectors ---
R2_Op = a * exp(1j * T2_Open); % Green
vec_B_Op = R1 + R4_Vec; % End of Grey (Point B)
vec_A_Op = R2_Op;       % End of Green (Point A)
R3_Op = vec_B_Op - vec_A_Op; % Yellow (A to B)

% --- Case 2 Vectors ---
R2_Cr = a * exp(1j * T2_Cross); % Green
vec_B_Cr = R1 + R4_Vec;
vec_A_Cr = R2_Cr;
R3_Cr = vec_B_Cr - vec_A_Cr; % Yellow

figure(3); clf;

% Subplot 1: Open
subplot(1,2,1); hold on; grid on; axis equal;
title('Loop 3: Case 1');
plot([0 real(R1)], [0 imag(R1)], 'm-', 'LineWidth', 2); % Ground
quiver(0, 0, real(R2_Op), imag(R2_Op), 0, 'g', 'LineWidth', 2, 'MaxHeadSize',0.5); % Green
quiver(real(R1), imag(R1), real(R4_Vec), imag(R4_Vec), 0, 'Color', [0.5 0.5 0.5], 'LineWidth', 2, 'MaxHeadSize',0.5); % Grey
quiver(real(R2_Op), imag(R2_Op), real(R3_Op), imag(R3_Op), 0, 'y', 'LineWidth', 2, 'MaxHeadSize',0.5); % Yellow

% Subplot 2: Crossed
subplot(1,2,2); hold on; grid on; axis equal;
title('Loop 3: Case 2');
plot([0 real(R1)], [0 imag(R1)], 'm-', 'LineWidth', 2); % Ground
quiver(0, 0, real(R2_Cr), imag(R2_Cr), 0, 'g', 'LineWidth', 2, 'MaxHeadSize',0.5); % Green
quiver(real(R1), imag(R1), real(R4_Vec), imag(R4_Vec), 0, 'Color', [0.5 0.5 0.5], 'LineWidth', 2, 'MaxHeadSize',0.5); % Grey
quiver(real(R2_Cr), imag(R2_Cr), real(R3_Cr), imag(R3_Cr), 0, 'y', 'LineWidth', 2, 'MaxHeadSize',0.5); % Yellow
