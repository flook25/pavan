clear all
close all
clc

% =========================================================================
%  GLOBAL PARAMETERS
% =========================================================================
offset_deg = 0.81;
offset = deg2rad(offset_deg);
d_global = 0.210; % Shared Ground (Pink)

% =========================================================================
%  LOOP 1: CALCULATION (Inverted Input: Brown L4)
% =========================================================================
L1 = 0.210; L2 = 0.118; L3 = 0.210; L4 = 0.118;
d = L1; a = L2; b = L3; c = L4;

theta4_deg = 102.05; 
q_in_L1 = deg2rad(theta4_deg); 

% Inverted Constants
K1=d/c; K2=d/a; K3=(c^2-b^2+a^2+d^2)/(2*c*a); K4=d/b; K5=(a^2-d^2-c^2-b^2)/(2*c*b);

% Coefficients
A = cos(q_in_L1) - K1 - K2*cos(q_in_L1) + K3;
B = -2*sin(q_in_L1);
C = K1 - (K2+1)*cos(q_in_L1) + K3;
D_val = cos(q_in_L1) - K1 + K4*cos(q_in_L1) + K5;
E_val = -2*sin(q_in_L1);
F_val = K1 + (K4-1)*cos(q_in_L1) + K5;

% Roots (Use max(0, ...) to prevent negative sqrt error)
det_AC = sqrt(max(0, B^2 - 4*A*C));
det_DF = sqrt(max(0, E_val^2 - 4*D_val*F_val));

% --- Loop 1: Case 1 (Open) ---
q2_loc_1 = 2*atan2((-B - det_AC), (2*A));
q3_loc_1 = 2*atan2((-E_val - det_DF), (2*D_val));
T2_Open_L1 = q2_loc_1 + pi + offset; 
T3_Open_L1 = q3_loc_1 + pi + offset; 
T4_Global_L1 = q_in_L1 + pi + offset; 

% --- Loop 1: Case 2 (Crossed) ---
q2_loc_2 = 2*atan2((-B + det_AC), (2*A));
q3_loc_2 = 2*atan2((-E_val + det_DF), (2*D_val));
T2_Cross_L1 = q2_loc_2 + pi + offset; 
T3_Cross_L1 = q3_loc_2 + pi + offset; 


% =========================================================================
%  LOOP 2: CALCULATION (Input: Cyan L2 from Loop 1 + 180)
% =========================================================================
L1 = 0.210; L2 = 0.118; L3 = 0.210; L4 = 0.118;
d = L1; a = L2; b = L3; c = L4;

theta2_global_Open = rad2deg(T2_Open_L1) + 180;
q_in_Open = deg2rad(theta2_global_Open) - offset;

theta2_global_Cross = rad2deg(T2_Cross_L1) + 180;
q_in_Cross = deg2rad(theta2_global_Cross) - offset;

K1=d/a; K2=d/c; K3=(a^2-b^2+c^2+d^2)/(2*a*c); K4=d/b; K5=(c^2-d^2-a^2-b^2)/(2*a*b);

% --- Loop 2: Case 1 (Open Calculation) ---
A=cos(q_in_Open)-K1-K2*cos(q_in_Open)+K3; B=-2*sin(q_in_Open); C=K1-(K2+1)*cos(q_in_Open)+K3;
D_val=cos(q_in_Open)-K1+K4*cos(q_in_Open)+K5; E_val=-2*sin(q_in_Open); F_val=K1+(K4-1)*cos(q_in_Open)+K5;
% Roots
det_AC_2 = sqrt(max(0, B^2 - 4*A*C));
det_DF_2 = sqrt(max(0, E_val^2 - 4*D_val*F_val));

q4_loc_1 = 2*atan2((-B - det_AC_2), (2*A));
q3_loc_1 = 2*atan2((-E_val - det_DF_2), (2*D_val));
T4_Open_L2 = q4_loc_1 + offset; 
T3_Open_L2 = q3_loc_1 + offset; 

% --- Loop 2: Case 2 (Crossed Calculation) ---
A=cos(q_in_Cross)-K1-K2*cos(q_in_Cross)+K3; B=-2*sin(q_in_Cross); C=K1-(K2+1)*cos(q_in_Cross)+K3;
D_val=cos(q_in_Cross)-K1+K4*cos(q_in_Cross)+K5; E_val=-2*sin(q_in_Cross); F_val=K1+(K4-1)*cos(q_in_Cross)+K5;
% Roots
det_AC_2 = sqrt(max(0, B^2 - 4*A*C));
det_DF_2 = sqrt(max(0, E_val^2 - 4*D_val*F_val));

q4_loc_2 = 2*atan2((-B + det_AC_2), (2*A));
q3_loc_2 = 2*atan2((-E_val + det_DF_2), (2*D_val));
T4_Cross_L2 = q4_loc_2 + offset; 
T3_Cross_L2 = q3_loc_2 + offset; 


% =========================================================================
%  LOOP 3: CALCULATION (Inverted Input: Grey L4 from Loop 2)
% =========================================================================
L1 = 0.210; L2 = 0.180; L3 = 0.180; L4 = 0.118;
d = L1; a = L2; b = L3; c = L4;

K1=d/c; K2=d/a; K3=(c^2-b^2+a^2+d^2)/(2*c*a); K4=d/b; K5=(c^2-d^2-a^2-b^2)/(2*c*b);

% --- Loop 3: Case 1 (System Open) ---
% *FIXED: Using 'System Open' but applying CROSSED formula (+sqrt) as requested*
q4_local_Open = T4_Open_L2 - offset;
q_in_calc_Open = q4_local_Open - pi; 

A=cos(q_in_calc_Open)-K1-K2*cos(q_in_calc_Open)+K3; B=-2*sin(q_in_calc_Open); C=K1-(K2+1)*cos(q_in_calc_Open)+K3;
% Determinant check
det_AC_3 = sqrt(max(0, B^2 - 4*A*C)); 

% 1. Find Theta 2 (Green) using CROSSED (+sqrt) formula
q2_loc_SWAP = 2*atan2((-B + det_AC_3), (2*A));
% 2. Find Theta 3 (Yellow) using Vector Method (Prevents Error)
% Vector Eq: R_Coupler = R_Ground + R_Output - R_Input
R_Coupler_Vec = d + a*exp(1i*q2_loc_SWAP) - c*exp(1i*q_in_calc_Open);
q3_loc_SWAP = angle(R_Coupler_Vec);

T2_Open_L3 = q2_loc_SWAP + pi + offset; 
T3_Open_L3 = q3_loc_SWAP + pi + offset; 

% --- Loop 3: Case 2 (System Crossed) ---
% *FIXED: Using 'System Crossed' but applying OPEN formula (-sqrt) as requested*
q4_local_Cross = T4_Cross_L2 - offset;
q_in_calc_Cross = q4_local_Cross - pi; 

A=cos(q_in_calc_Cross)-K1-K2*cos(q_in_calc_Cross)+K3; B=-2*sin(q_in_calc_Cross); C=K1-(K2+1)*cos(q_in_calc_Cross)+K3;
% Determinant check
det_AC_3 = sqrt(max(0, B^2 - 4*A*C));

% 1. Find Theta 2 (Green) using OPEN (-sqrt) formula
q2_loc_SWAP2 = 2*atan2((-B - det_AC_3), (2*A));
% 2. Find Theta 3 (Yellow) using Vector Method
R_Coupler_Vec = d + a*exp(1i*q2_loc_SWAP2) - c*exp(1i*q_in_calc_Cross);
q3_loc_SWAP2 = angle(R_Coupler_Vec);

T2_Cross_L3 = q2_loc_SWAP2 + pi + offset; 
T3_Cross_L3 = q3_loc_SWAP2 + pi + offset; 


% =========================================================================
%  DISPLAY RESULTS
% =========================================================================
disp('===================================================');
disp(['FULL MECHANISM RESULTS (Input Theta4 = ' num2str(theta4_deg) ')']);
disp('===================================================');

disp('--- SYSTEM CASE 2 (CROSSED CHAIN) ---');
disp(['  L1 Brown  (In) : ', num2str(rad2deg(T4_Global_L1))]);
disp(['  L1 Cyan   (Out): ', num2str(rad2deg(T2_Cross_L1))]);
disp('  ----------------');
disp(['  L2 Grey   (Out): ', num2str(rad2deg(T4_Cross_L2))]);
disp(['  L2 Red    (Cou): ', num2str(rad2deg(T3_Cross_L2))]);
disp('  ----------------');
disp(['  L3 Green  (Out): ', num2str(rad2deg(T2_Cross_L3))]);
disp(['  L3 Yellow (Cou): ', num2str(rad2deg(T3_Cross_L3))]);
disp(' ');

disp('--- SYSTEM CASE 1 (OPEN CHAIN) ---');
disp(['  L1 Brown  (In) : ', num2str(rad2deg(T4_Global_L1))]);
disp(['  L1 Cyan   (Out): ', num2str(rad2deg(T2_Open_L1))]);
disp('  ----------------');
disp(['  L2 Grey   (Out): ', num2str(rad2deg(T4_Open_L2))]);
disp(['  L2 Red    (Cou): ', num2str(rad2deg(T3_Open_L2))]);
disp('  ----------------');
disp(['  L3 Green  (Out): ', num2str(rad2deg(T2_Open_L3))]);
disp(['  L3 Yellow (Cou): ', num2str(rad2deg(T3_Open_L3))]);


% =========================================================================
%  PLOTTING
% =========================================================================
figure(1); clf;

Rot = exp(1i * offset);
R1 = d_global * exp(1i * offset); % Shared Ground

% -------------------------
% PLOT 1: SYSTEM CROSSED
% -------------------------
subplot(1,2,1); hold on; grid on; axis equal;
title('System Configuration 2 (Crossed)');
plot([0 real(R1)], [0 imag(R1)], 'm-', 'LineWidth', 2); % Ground

% Loop 1
L2_Loop1 = 0.118; L4_Loop1 = 0.118;
V_Cyan_L1 = L2_Loop1 * exp(1i * T2_Cross_L1);
V_Brown_L1 = L4_Loop1 * exp(1i * T4_Global_L1);
V_Blue_L1 = (R1 + V_Brown_L1) - V_Cyan_L1;
quiver(0, 0, real(V_Cyan_L1), imag(V_Cyan_L1), 0, 'c', 'LineWidth', 2, 'MaxHeadSize',0.5); 
quiver(real(R1), imag(R1), real(V_Brown_L1), imag(V_Brown_L1), 0, 'Color',[0.6 0.3 0], 'LineWidth', 2, 'MaxHeadSize',0.5); 
quiver(real(V_Cyan_L1), imag(V_Cyan_L1), real(V_Blue_L1), imag(V_Blue_L1), 0, 'b', 'LineWidth', 2, 'MaxHeadSize',0.5); 

% Loop 2
L2_Loop2 = 0.118; L4_Loop2 = 0.118;
V_Cyan_L2 = L2_Loop2 * exp(1i * deg2rad(rad2deg(T2_Cross_L1) + 180)); 
V_Grey_L2 = L4_Loop2 * exp(1i * T4_Cross_L2);
V_Red_L2 = (R1 + V_Grey_L2) - V_Cyan_L2;
quiver(0, 0, real(V_Cyan_L2), imag(V_Cyan_L2), 0, 'c', 'LineWidth', 2, 'MaxHeadSize',0.5); 
quiver(real(R1), imag(R1), real(V_Grey_L2), imag(V_Grey_L2), 0, 'Color', [0.5 0.5 0.5], 'LineWidth', 2, 'MaxHeadSize',0.5); 
quiver(real(V_Cyan_L2), imag(V_Cyan_L2), real(V_Red_L2), imag(V_Red_L2), 0, 'r', 'LineWidth', 2, 'MaxHeadSize',0.5); 

% Loop 3
L2_Loop3 = 0.180; L4_Loop3 = 0.118;
V_Grey_sys = L4_Loop3 * exp(1i * T4_Cross_L2); 
V_Green_sys = L2_Loop3 * exp(1i * T2_Cross_L3); 
V_Yellow_sys = (R1 + V_Grey_sys) - V_Green_sys;
quiver(0, 0, real(V_Green_sys), imag(V_Green_sys), 0, 'g', 'LineWidth', 2, 'MaxHeadSize',0.5); 
quiver(real(V_Green_sys), imag(V_Green_sys), real(V_Yellow_sys), imag(V_Yellow_sys), 0, 'y', 'LineWidth', 2, 'MaxHeadSize',0.5); 


% -------------------------
% PLOT 2: SYSTEM OPEN
% -------------------------
subplot(1,2,2); hold on; grid on; axis equal;
title('System Configuration 1 (Open)');
plot([0 real(R1)], [0 imag(R1)], 'm-', 'LineWidth', 2); % Ground

% Loop 1
V_Cyan_L1 = L2_Loop1 * exp(1i * T2_Open_L1);
V_Brown_L1 = L4_Loop1 * exp(1i * T4_Global_L1);
V_Blue_L1 = (R1 + V_Brown_L1) - V_Cyan_L1;
quiver(0, 0, real(V_Cyan_L1), imag(V_Cyan_L1), 0, 'c', 'LineWidth', 2, 'MaxHeadSize',0.5); 
quiver(real(R1), imag(R1), real(V_Brown_L1), imag(V_Brown_L1), 0, 'Color',[0.6 0.3 0], 'LineWidth', 2, 'MaxHeadSize',0.5); 
quiver(real(V_Cyan_L1), imag(V_Cyan_L1), real(V_Blue_L1), imag(V_Blue_L1), 0, 'b', 'LineWidth', 2, 'MaxHeadSize',0.5); 

% Loop 2
V_Cyan_L2 = L2_Loop2 * exp(1i * deg2rad(rad2deg(T2_Open_L1) + 180)); 
V_Grey_L2 = L4_Loop2 * exp(1i * T4_Open_L2);
V_Red_L2 = (R1 + V_Grey_L2) - V_Cyan_L2;
quiver(0, 0, real(V_Cyan_L2), imag(V_Cyan_L2), 0, 'c', 'LineWidth', 2, 'MaxHeadSize',0.5); 
quiver(real(R1), imag(R1), real(V_Grey_L2), imag(V_Grey_L2), 0, 'Color', [0.5 0.5 0.5], 'LineWidth', 2, 'MaxHeadSize',0.5); 
quiver(real(V_Cyan_L2), imag(V_Cyan_L2), real(V_Red_L2), imag(V_Red_L2), 0, 'r', 'LineWidth', 2, 'MaxHeadSize',0.5); 

% Loop 3
V_Grey_sys = L4_Loop3 * exp(1i * T4_Open_L2);
V_Green_sys = L2_Loop3 * exp(1i * T2_Open_L3);
V_Yellow_sys = (R1 + V_Grey_sys) - V_Green_sys;
quiver(0, 0, real(V_Green_sys), imag(V_Green_sys), 0, 'g', 'LineWidth', 2, 'MaxHeadSize',0.5); 
quiver(real(V_Green_sys), imag(V_Green_sys), real(V_Yellow_sys), imag(V_Yellow_sys), 0, 'y', 'LineWidth', 2, 'MaxHeadSize',0.5);
