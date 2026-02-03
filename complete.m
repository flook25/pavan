clear all
close all
clc

% =========================================================================
%  GLOBAL PARAMETERS (Defined once for consistency)
% =========================================================================
offset_deg = 0.81;
offset = deg2rad(offset_deg);
d_global = 0.210; % Shared Ground (Pink)

% =========================================================================
%  LOOP 1: CALCULATION (Inverted Input: Brown L4)
% =========================================================================
% Parameters Loop 1
L1 = 0.210; L2 = 0.118; L3 = 0.210; L4 = 0.118;
d = L1; a = L2; b = L3; c = L4;

theta4_deg = 102.05; 
q_in_L1 = deg2rad(theta4_deg); % Local Input (relative to O4->O2)

% Inverted Constants (Link 4 is Crank, Link 2 is Rocker)
K1=d/c; K2=d/a; K3=(c^2-b^2+a^2+d^2)/(2*c*a); K4=d/b; K5=(a^2-d^2-c^2-b^2)/(2*c*b);

% Coefficients
A = cos(q_in_L1) - K1 - K2*cos(q_in_L1) + K3;
B = -2*sin(q_in_L1);
C = K1 - (K2+1)*cos(q_in_L1) + K3;
D_val = cos(q_in_L1) - K1 + K4*cos(q_in_L1) + K5;
E_val = -2*sin(q_in_L1);
F_val = K1 + (K4-1)*cos(q_in_L1) + K5;

% Roots
det_AC = sqrt(B^2 - 4*A*C);
det_DF = sqrt(E_val^2 - 4*D_val*F_val);

% --- Loop 1: Case 1 (Open) ---
q2_loc_1 = 2*atan2((-B - det_AC), (2*A));
q3_loc_1 = 2*atan2((-E_val - det_DF), (2*D_val));
% Transform to Global (+180)
T2_Open_L1 = q2_loc_1 + pi + offset; % Cyan
T3_Open_L1 = q3_loc_1 + pi + offset; % Blue
T4_Global_L1 = q_in_L1 + pi + offset; % Brown (Input)

% --- Loop 1: Case 2 (Crossed) ---
q2_loc_2 = 2*atan2((-B + det_AC), (2*A));
q3_loc_2 = 2*atan2((-E_val + det_DF), (2*D_val));
% Transform to Global (+180)
T2_Cross_L1 = q2_loc_2 + pi + offset; % Cyan
T3_Cross_L1 = q3_loc_2 + pi + offset; % Blue


% =========================================================================
%  LOOP 2: CALCULATION (Input: Cyan L2 from Loop 1 + 180)
% =========================================================================
% Parameters Loop 2
L1 = 0.210; L2 = 0.118; L3 = 0.210; L4 = 0.118;
d = L1; a = L2; b = L3; c = L4;

% --- Setup Input for Case 1 (Open Chain) ---
theta2_global_Open = rad2deg(T2_Open_L1) + 180;
q_in_Open = deg2rad(theta2_global_Open) - offset;

% --- Setup Input for Case 2 (Crossed Chain) ---
theta2_global_Cross = rad2deg(T2_Cross_L1) + 180;
q_in_Cross = deg2rad(theta2_global_Cross) - offset;

% Constants (Standard)
K1=d/a; K2=d/c; K3=(a^2-b^2+c^2+d^2)/(2*a*c); K4=d/b; K5=(c^2-d^2-a^2-b^2)/(2*a*b);

% --- Loop 2: Case 1 (Open Calculation) ---
A=cos(q_in_Open)-K1-K2*cos(q_in_Open)+K3; B=-2*sin(q_in_Open); C=K1-(K2+1)*cos(q_in_Open)+K3;
D_val=cos(q_in_Open)-K1+K4*cos(q_in_Open)+K5; E_val=-2*sin(q_in_Open); F_val=K1+(K4-1)*cos(q_in_Open)+K5;
q4_loc_1 = 2*atan2((-B - sqrt(B^2 - 4*A*C)), (2*A));
q3_loc_1 = 2*atan2((-E_val - sqrt(E_val^2 - 4*D_val*F_val)), (2*D_val));
T4_Open_L2 = q4_loc_1 + offset; % Grey
T3_Open_L2 = q3_loc_1 + offset; % Red

% --- Loop 2: Case 2 (Crossed Calculation) ---
A=cos(q_in_Cross)-K1-K2*cos(q_in_Cross)+K3; B=-2*sin(q_in_Cross); C=K1-(K2+1)*cos(q_in_Cross)+K3;
D_val=cos(q_in_Cross)-K1+K4*cos(q_in_Cross)+K5; E_val=-2*sin(q_in_Cross); F_val=K1+(K4-1)*cos(q_in_Cross)+K5;
q4_loc_2 = 2*atan2((-B + sqrt(B^2 - 4*A*C)), (2*A));
q3_loc_2 = 2*atan2((-E_val + sqrt(E_val^2 - 4*D_val*F_val)), (2*D_val));
T4_Cross_L2 = q4_loc_2 + offset; % Grey
T3_Cross_L2 = q3_loc_2 + offset; % Red


% =========================================================================
%  LOOP 3: CALCULATION (Inverted Input: Grey L4 from Loop 2)
% =========================================================================
% Parameters Loop 3
L1 = 0.210; L2 = 0.180; L3 = 0.180; L4 = 0.118;
d = L1; a = L2; b = L3; c = L4;

% Constants (Inverted)
K1=d/c; K2=d/a; K3=(c^2-b^2+a^2+d^2)/(2*c*a); K4=d/b; K5=(c^2-d^2-a^2-b^2)/(2*c*b);

% --- Loop 3: Case 1 (Open Chain) ---
% Input from Loop 2 Open output
q4_local_Open = T4_Open_L2 - offset;
q_in_calc_Open = q4_local_Open - pi; % Inverted Shift

A=cos(q_in_calc_Open)-K1-K2*cos(q_in_calc_Open)+K3; B=-2*sin(q_in_calc_Open); C=K1-(K2+1)*cos(q_in_calc_Open)+K3;
D_val=cos(q_in_calc_Open)-K1+K4*cos(q_in_calc_Open)+K5; E_val=-2*sin(q_in_calc_Open); F_val=K1+(K4-1)*cos(q_in_calc_Open)+K5;
% Using Open Formula (-sqrt)
q2_loc_1 = 2*atan2((-B - sqrt(B^2 - 4*A*C)), (2*A));
q3_loc_1 = 2*atan2((-E_val - sqrt(E_val^2 - 4*D_val*F_val)), (2*D_val));
T2_Open_L3 = q2_loc_1 + pi + offset; % Green
T3_Open_L3 = q3_loc_1 + pi + offset; % Yellow

% --- Loop 3: Case 2 (Crossed Chain) ---
% Input from Loop 2 Crossed output
q4_local_Cross = T4_Cross_L2 - offset;
q_in_calc_Cross = q4_local_Cross - pi; % Inverted Shift

A=cos(q_in_calc_Cross)-K1-K2*cos(q_in_calc_Cross)+K3; B=-2*sin(q_in_calc_Cross); C=K1-(K2+1)*cos(q_in_calc_Cross)+K3;
D_val=cos(q_in_calc_Cross)-K1+K4*cos(q_in_calc_Cross)+K5; E_val=-2*sin(q_in_calc_Cross); F_val=K1+(K4-1)*cos(q_in_calc_Cross)+K5;
% Using Crossed Formula (+sqrt)
q2_loc_2 = 2*atan2((-B + sqrt(B^2 - 4*A*C)), (2*A));
q3_loc_2 = 2*atan2((-E_val + sqrt(E_val^2 - 4*D_val*F_val)), (2*D_val));
T2_Cross_L3 = q2_loc_2 + pi + offset; % Green
T3_Cross_L3 = q3_loc_2 + pi + offset; % Yellow


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

% Loop 1 (Crossed)
L2_Loop1 = 0.118; L4_Loop1 = 0.118;
V_Cyan_L1 = L2_Loop1 * exp(1i * T2_Cross_L1);
V_Brown_L1 = L4_Loop1 * exp(1i * T4_Global_L1);
V_Blue_L1 = (R1 + V_Brown_L1) - V_Cyan_L1;
quiver(0, 0, real(V_Cyan_L1), imag(V_Cyan_L1), 0, 'c', 'LineWidth', 2, 'MaxHeadSize',0.5); % Cyan
quiver(real(R1), imag(R1), real(V_Brown_L1), imag(V_Brown_L1), 0, 'Color',[0.6 0.3 0], 'LineWidth', 2, 'MaxHeadSize',0.5); % Brown
quiver(real(V_Cyan_L1), imag(V_Cyan_L1), real(V_Blue_L1), imag(V_Blue_L1), 0, 'b', 'LineWidth', 2, 'MaxHeadSize',0.5); % Blue

% Loop 2 (Crossed)
L2_Loop2 = 0.118; L4_Loop2 = 0.118;
V_Cyan_L2 = L2_Loop2 * exp(1i * deg2rad(rad2deg(T2_Cross_L1) + 180)); % Input L2
V_Grey_L2 = L4_Loop2 * exp(1i * T4_Cross_L2);
V_Red_L2 = (R1 + V_Grey_L2) - V_Cyan_L2;
quiver(0, 0, real(V_Cyan_L2), imag(V_Cyan_L2), 0, 'c', 'LineWidth', 2, 'MaxHeadSize',0.5); % Cyan (Rotated)
quiver(real(R1), imag(R1), real(V_Grey_L2), imag(V_Grey_L2), 0, 'Color', [0.5 0.5 0.5], 'LineWidth', 2, 'MaxHeadSize',0.5); % Grey
quiver(real(V_Cyan_L2), imag(V_Cyan_L2), real(V_Red_L2), imag(V_Red_L2), 0, 'r', 'LineWidth', 2, 'MaxHeadSize',0.5); % Red

% Loop 3 (Crossed) - Swapped View (Origin at O4)
L2_Loop3 = 0.180; L4_Loop3 = 0.118;
O4_Plot = 0; % Shift Origin to O4 for visualization
O2_Plot = d_global * exp(1i * (offset - pi)); % Ground from O4 View
V_Grey_L3 = L4_Loop3 * exp(1i * (T4_Cross_L2 - pi)); % Input Grey
V_Green_L3 = L2_Loop3 * exp(1i * (T2_Cross_L3 - pi)); % Output Green
Point_B = O4_Plot + V_Grey_L3;
Point_A = O2_Plot + V_Green_L3;
% Note: Plotting Loop 3 separately on the side as per snippet style, or overlaid?
% Snippet 3 plotted it standalone. I will plot it relative to Ground R1 to attach to mechanism.
% Re-attaching to Mechanism:
% Loop 3 Input is Grey Link (Attached to O4). Output is Green (Attached to O2).
% Let's plot it in place on top of the others.
V_Grey_sys = L4_Loop3 * exp(1i * T4_Cross_L2); % Matches Loop 2 Grey
V_Green_sys = L2_Loop3 * exp(1i * T2_Cross_L3); 
V_Yellow_sys = (R1 + V_Grey_sys) - V_Green_sys;
% Plot Loop 3 (Overlaid on Mechanism)
quiver(0, 0, real(V_Green_sys), imag(V_Green_sys), 0, 'g', 'LineWidth', 2, 'MaxHeadSize',0.5); % Green
% Grey is already plotted in Loop 2, but let's overwrite to show connection if needed
quiver(real(V_Green_sys), imag(V_Green_sys), real(V_Yellow_sys), imag(V_Yellow_sys), 0, 'y', 'LineWidth', 2, 'MaxHeadSize',0.5); % Yellow


% -------------------------
% PLOT 2: SYSTEM OPEN
% -------------------------
subplot(1,2,2); hold on; grid on; axis equal;
title('System Configuration 1 (Open)');
plot([0 real(R1)], [0 imag(R1)], 'm-', 'LineWidth', 2); % Ground

% Loop 1 (Open)
V_Cyan_L1 = L2_Loop1 * exp(1i * T2_Open_L1);
V_Brown_L1 = L4_Loop1 * exp(1i * T4_Global_L1);
V_Blue_L1 = (R1 + V_Brown_L1) - V_Cyan_L1;
quiver(0, 0, real(V_Cyan_L1), imag(V_Cyan_L1), 0, 'c', 'LineWidth', 2, 'MaxHeadSize',0.5); 
quiver(real(R1), imag(R1), real(V_Brown_L1), imag(V_Brown_L1), 0, 'Color',[0.6 0.3 0], 'LineWidth', 2, 'MaxHeadSize',0.5); 
quiver(real(V_Cyan_L1), imag(V_Cyan_L1), real(V_Blue_L1), imag(V_Blue_L1), 0, 'b', 'LineWidth', 2, 'MaxHeadSize',0.5); 

% Loop 2 (Open)
V_Cyan_L2 = L2_Loop2 * exp(1i * deg2rad(rad2deg(T2_Open_L1) + 180)); 
V_Grey_L2 = L4_Loop2 * exp(1i * T4_Open_L2);
V_Red_L2 = (R1 + V_Grey_L2) - V_Cyan_L2;
quiver(0, 0, real(V_Cyan_L2), imag(V_Cyan_L2), 0, 'c', 'LineWidth', 2, 'MaxHeadSize',0.5); 
quiver(real(R1), imag(R1), real(V_Grey_L2), imag(V_Grey_L2), 0, 'Color', [0.5 0.5 0.5], 'LineWidth', 2, 'MaxHeadSize',0.5); 
quiver(real(V_Cyan_L2), imag(V_Cyan_L2), real(V_Red_L2), imag(V_Red_L2), 0, 'r', 'LineWidth', 2, 'MaxHeadSize',0.5); 

% Loop 3 (Open)
V_Grey_sys = L4_Loop3 * exp(1i * T4_Open_L2);
V_Green_sys = L2_Loop3 * exp(1i * T2_Open_L3);
V_Yellow_sys = (R1 + V_Grey_sys) - V_Green_sys;
quiver(0, 0, real(V_Green_sys), imag(V_Green_sys), 0, 'g', 'LineWidth', 2, 'MaxHeadSize',0.5); 
quiver(real(V_Green_sys), imag(V_Green_sys), real(V_Yellow_sys), imag(V_Yellow_sys), 0, 'y', 'LineWidth', 2, 'MaxHeadSize',0.5);
