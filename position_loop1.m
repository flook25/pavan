clear all
close all
clc

% ==========================================
% 1. PARAMETERS & INPUT
% ==========================================
L1 = 0.210; % Ground (Pink) - d
L2 = 0.118; % Crank (Cyan) - a
L3 = 0.210; % Coupler (Blue) - b
L4 = 0.118; % Rocker (Brown) - c (Input)

% Assign Canonical Names
d = L1;
a = L2; % (Output in this calculation)
b = L3;
c = L4; % (Input in this calculation)

% Input Parameters
theta4_deg_global = 102.05; 
offset_deg = 0.81;

% Convert to radians and Remove Offset (To Local Frame)
offset = deg2rad(offset_deg);
q4_local = deg2rad(theta4_deg_global) - offset;

% ==========================================
% 2. CALCULATION (Inverted Mechanism)
% ==========================================
% IMPORTANT: Shift Input by -180 degrees (pi) because we are calculating 
% from O4 looking back at O2 (Frame is rotated 180 deg)
q_in_calc = q4_local - pi; 

% Swap Constants for Inverted Calculation (Input=c, Output=a)
d_cal = d;
a_cal = c; % Link 4 is Driver
b_cal = b;
c_cal = a; % Link 2 is Follower

% Freudenstein Constants (Inverted)
K1 = d_cal/a_cal;
K2 = d_cal/c_cal;
K3 = (a_cal^2 - b_cal^2 + c_cal^2 + d_cal^2)/(2*a_cal*c_cal);
K4 = d_cal/b_cal;
K5 = (c_cal^2 - d_cal^2 - a_cal^2 - b_cal^2)/(2*a_cal*b_cal);

% Coefficients for Theta 2 (Output)
A = cos(q_in_calc) - K1 - K2*cos(q_in_calc) + K3;
B = -2*sin(q_in_calc);
C = K1 - (K2+1)*cos(q_in_calc) + K3;

% Coefficients for Theta 3 (Coupler)
D_val = cos(q_in_calc) - K1 + K4*cos(q_in_calc) + K5;
E_val = -2*sin(q_in_calc);
F_val = K1 + (K4-1)*cos(q_in_calc) + K5;

% Solve Quadratic Roots
det_AC = sqrt(B^2 - 4*A*C);
det_DF = sqrt(E_val^2 - 4*D_val*F_val);

% --- CASE 1: OPEN (Standard Formula: -sqrt) ---
% Note: Using (-B - sqrt) aligns with the "Open" config logic in your example
q2_local_calc_1 = 2*atan2((-B - det_AC), (2*A));
q3_local_calc_1 = 2*atan2((-E_val - det_DF), (2*D_val));

% --- CASE 2: CROSSED (Standard Formula: +sqrt) ---
% Note: Using (-B + sqrt) aligns with the "Crossed" config logic
q2_local_calc_2 = 2*atan2((-B + det_AC), (2*A));
q3_local_calc_2 = 2*atan2((-E_val + det_DF), (2*D_val));

% ==========================================
% 3. TRANSFORM BACK TO GLOBAL
% ==========================================
% IMPORTANT: Add +180 degrees (pi) back to rotate frame to Global (O2 origin)
Theta2_Open_Local  = q2_local_calc_1 + pi;
Theta3_Open_Local  = q3_local_calc_1 + pi;

Theta2_Cross_Local = q2_local_calc_2 + pi;
Theta3_Cross_Local = q3_local_calc_2 + pi;

% Add Offset for Final Global Angles
T2_Open = Theta2_Open_Local + offset;
T3_Open = Theta3_Open_Local + offset;

T2_Cross = Theta2_Cross_Local + offset;
T3_Cross = Theta3_Cross_Local + offset;

Theta4_Global = q4_local + offset; % Same as Input

% ==========================================
% 4. VECTOR CONSTRUCTION & PLOTTING
% ==========================================
Rot = exp(1i * offset);

% Ground (Pink)
R1 = d * exp(1i * offset);

% --- CASE 1 Vectors ---
R2_Op = a * exp(1i * T2_Open);           % Link 2 (Cyan)
R4_Op = c * exp(1i * Theta4_Global);     % Link 4 (Brown)
% Calculate R3 (Link 3) from Vector Loop Closure
% Loop: O2->A->B->O4 => R2 + R3 - R4 - R1 = 0 => R3 = R1 + R4 - R2
% Note: R4 is defined from O4, so vector to B is (R1 + R4)
vec_B_Op = R1 + R4_Op; 
vec_A_Op = R2_Op;
R3_Op = vec_B_Op - vec_A_Op;

% --- CASE 2 Vectors ---
R2_Cr = a * exp(1i * T2_Cross);          % Link 2 (Cyan)
R4_Cr = c * exp(1i * Theta4_Global);     % Link 4 (Brown)
vec_B_Cr = R1 + R4_Cr;
vec_A_Cr = R2_Cr;
R3_Cr = vec_B_Cr - vec_A_Cr;

% --- PLOT ---
figure(1); clf;

% Subplot 1: Case 1 (Open)
subplot(1,2,1); hold on; grid on; axis equal;
title('Case 1: Open Circuit');
% Ground
plot([0 real(R1)], [0 imag(R1)], 'm-', 'LineWidth', 2);
% Link 2 (Cyan)
plot([0 real(R2_Op)], [0 imag(R2_Op)], 'c-', 'LineWidth', 2);
% Link 4 (Brown) - from O4
plot([real(R1) real(vec_B_Op)], [imag(R1) imag(vec_B_Op)], 'Color', [0.6 0.3 0], 'LineWidth', 2);
% Link 3 (Blue) - Connects Tips
plot([real(R2_Op) real(vec_B_Op)], [imag(R2_Op) imag(vec_B_Op)], 'b-', 'LineWidth', 2);

% Subplot 2: Case 2 (Crossed)
subplot(1,2,2); hold on; grid on; axis equal;
title('Case 2: Crossed Circuit');
% Ground
plot([0 real(R1)], [0 imag(R1)], 'm-', 'LineWidth', 2);
% Link 2 (Cyan)
plot([0 real(R2_Cr)], [0 imag(R2_Cr)], 'c-', 'LineWidth', 2);
% Link 4 (Brown)
plot([real(R1) real(vec_B_Cr)], [imag(R1) imag(vec_B_Cr)], 'Color', [0.6 0.3 0], 'LineWidth', 2);
% Link 3 (Blue)
plot([real(R2_Cr) real(vec_B_Cr)], [imag(R2_Cr) imag(vec_B_Cr)], 'b-', 'LineWidth', 2);

% --- DISPLAY RESULTS ---
disp('======================================');
disp(['Loop 1 Analysis (Input Theta4 = ' num2str(theta4_deg_global) ' deg)']);
disp('======================================');
disp('--- CASE 1 (OPEN) ---');
disp(['  Link 2 (Cyan):  ', num2str(rad2deg(T2_Open))]);
disp(['  Link 3 (Blue):  ', num2str(rad2deg(angle(R3_Op)))]);
disp(' ');
disp('--- CASE 2 (CROSSED) ---');
disp(['  Link 2 (Cyan):  ', num2str(rad2deg(T2_Cross))]);
disp(['  Link 3 (Blue):  ', num2str(rad2deg(angle(R3_Cr)))]);
