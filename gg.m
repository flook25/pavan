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
% หมุน Input ไป -180 (Inverted Frame)
q_in_calc = q4_local - pi;

% Constants (Input=c, Output=a)
K1 = d/c;
K2 = d/a;
K3 = (c^2 - b^2 + a^2 + d^2)/(2*c*a);
K4 = d/b;
K5 = (c^2 - d^2 - a^2 - b^2)/(2*c*b);

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

% --- CASE 1 (Open Formula: -sqrt) ---
q2_loc_1 = 2*atan2((-B - det_AC), (2*A));
q3_loc_1 = 2*atan2((-E_val - det_DF), (2*D_val));

% --- CASE 2 (Crossed Formula: +sqrt) ---
q2_loc_2 = 2*atan2((-B + det_AC), (2*A));
q3_loc_2 = 2*atan2((-E_val + det_DF), (2*D_val));

% หมุนกลับ +180 (Global Frame)
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
disp('--- CASE 2 (Crossed) - PLOTTED FIRST ---');
disp(['  Green  (Theta 2): ', num2str(rad2deg(T2_Cross))]);
disp(['  Yellow (Theta 3): ', num2str(rad2deg(T3_Cross))]);
disp(' ');
disp('--- CASE 1 (Open) - PLOTTED SECOND ---');
disp(['  Green  (Theta 2): ', num2str(rad2deg(T2_Open))]);
disp(['  Yellow (Theta 3): ', num2str(rad2deg(T3_Open))]);

% ==========================================
% 4. PLOTTING (SWAPPED ORDER)
% ==========================================
Rot = exp(1j * offset);
R1 = d * exp(1j * offset); % Ground

% Input Vector (Grey)
R4_Vec = c * exp(1j * q4_global);

% --- Calculate Vectors ---
% Case 2 (Crossed) Vectors
R2_Cr = a * exp(1j * T2_Cross); 
vec_B_Cr = R1 + R4_Vec;
vec_A_Cr = R2_Cr;
R3_Cr = vec_B_Cr - vec_A_Cr; 

% Case 1 (Open) Vectors
R2_Op = a * exp(1j * T2_Open); 
vec_B_Op = R1 + R4_Vec;
vec_A_Op = R2_Op;
R3_Op = vec_B_Op - vec_A_Op; 

figure(3); clf;

% --- PLOT 1: CASE 2 (Crossed) ---
subplot(1,2,1); hold on; grid on; axis equal;
title('Loop 3: Case 2 (Crossed)');
plot([0 real(R1)], [0 imag(R1)], 'm-', 'LineWidth', 2); % Ground
quiver(0, 0, real(R2_Cr), imag(R2_Cr), 0, 'g', 'LineWidth', 2, 'MaxHeadSize',0.5); % Green
quiver(real(R1), imag(R1), real(R4_Vec), imag(R4_Vec), 0, 'Color', [0.5 0.5 0.5], 'LineWidth', 2, 'MaxHeadSize',0.5); % Grey
quiver(real(R2_Cr), imag(R2_Cr), real(R3_Cr), imag(R3_Cr), 0, 'y', 'LineWidth', 2, 'MaxHeadSize',0.5); % Yellow

% --- PLOT 2: CASE 1 (Open) ---
subplot(1,2,2); hold on; grid on; axis equal;
title('Loop 3: Case 1 (Open)');
plot([0 real(R1)], [0 imag(R1)], 'm-', 'LineWidth', 2); % Ground
quiver(0, 0, real(R2_Op), imag(R2_Op), 0, 'g', 'LineWidth', 2, 'MaxHeadSize',0.5); % Green
quiver(real(R1), imag(R1), real(R4_Vec), imag(R4_Vec), 0, 'Color', [0.5 0.5 0.5], 'LineWidth', 2, 'MaxHeadSize',0.5); % Grey
quiver(real(R2_Op), imag(R2_Op), real(R3_Op), imag(R3_Op), 0, 'y', 'LineWidth', 2, 'MaxHeadSize',0.5); % Yellow
