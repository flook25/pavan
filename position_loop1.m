clear all
close all
clc

% ==========================================
% SECTION 1: PARAMETERS & INPUT
% ==========================================
% --- Dimensions (Meters) ---
L1 = 0.210; % Ground (d) - Pink

% Loop 1
L2_Loop1 = 0.180; % Crank Loop 1 (a) - Green
L3_Loop1 = 0.180; % Coupler Loop 1 (b) - Yellow
L4_Shared = 0.118; % Shared Rocker (c) - Grey

% Loop 2
L2_Loop2 = 0.118; % Crank Loop 2 (Cyan Down)
L3_Loop2 = 0.210; % Coupler Loop 2 (Red)

% Loop 3
L2_Loop3 = 0.118; % Crank Loop 3 (Cyan Up - Extended)
L3_Loop3 = 0.210; % Coupler Loop 3 (Blue)
L4_Loop3 = 0.118; % Rocker Loop 3 (Brown - INPUT)

d = L1; % Common Ground

% --- Global Input ---
offset_deg = 0.81;
offset = deg2rad(offset_deg);

% *** NEW INPUT: Brown Link (Loop 3) ***
q4_Brown_deg = 102.5;
q4_Brown = deg2rad(q4_Brown_deg) - offset;

% ==========================================
% SECTION 2: SOLVE LOOP 3 (Backwards)
% Input: Brown (q4), Find: Cyan Up (q_in_L3) & Blue (q3)
% ==========================================
% Standard 4-bar: Ground(d)-Cyan(a)-Blue(b)-Brown(c)
% We know 'c' (Brown angle), want to find 'a' (Cyan angle)
% Use Inverse Kinematics logic (Swap a and c)

a = L2_Loop3; b = L3_Loop3; c = L4_Loop3;

% K Constants (Input c - Brown)
K1_L3 = d/c;
K2_L3 = d/a;
K3_L3 = (c^2 - b^2 + a^2 + d^2)/(2*c*a);

% Forward K for q3 later
K1_L3_fwd = d/a;
K4_L3_fwd = d/b;
K5_L3_fwd = (c^2 - d^2 - a^2 - b^2)/(2*a*b);

% Coeffs for Cyan (q2 equivalent)
A_L3 = cos(q4_Brown) - K1_L3 - K2_L3*cos(q4_Brown) + K3_L3;
B_L3 = -2*sin(q4_Brown);
C_L3 = K1_L3 - (K2_L3+1)*cos(q4_Brown) + K3_L3;

% Solve Cyan Up Angle (q_in_L3) - Two Cases
disc_L3 = B_L3^2 - 4*A_L3*C_L3;
q_Cyan_Up_1 = 2*atan((-B_L3 - sqrt(disc_L3))/(2*A_L3)); % Case 1 (e.g., Parallel)
q_Cyan_Up_2 = 2*atan((-B_L3 + sqrt(disc_L3))/(2*A_L3)); % Case 2 (e.g., Crossed)

% Find Blue Angle (q3) for each case
% Case 1
D1 = cos(q_Cyan_Up_1) - K1_L3_fwd + K4_L3_fwd*cos(q_Cyan_Up_1) + K5_L3_fwd;
E1 = -2*sin(q_Cyan_Up_1);
F1 = K1_L3_fwd + (K4_L3_fwd - 1)*cos(q_Cyan_Up_1) + K5_L3_fwd;
q3_Blue_1 = 2*atan((-E1 - sqrt(E1^2 - 4*D1*F1))/(2*D1));

% Case 2
D2 = cos(q_Cyan_Up_2) - K1_L3_fwd + K4_L3_fwd*cos(q_Cyan_Up_2) + K5_L3_fwd;
E2 = -2*sin(q_Cyan_Up_2);
F2 = K1_L3_fwd + (K4_L3_fwd - 1)*cos(q_Cyan_Up_2) + K5_L3_fwd;
q3_Blue_2 = 2*atan((-E2 + sqrt(E2^2 - 4*D2*F2))/(2*D2));


% ==========================================
% SECTION 3: TRANSFER TO LOOP 2
% Cyan Up (q_Cyan_Up) -> Cyan Down (q_Cyan_Down)
% Relation: Straight line, so q_Cyan_Down = q_Cyan_Up - pi
% ==========================================
q_Cyan_Down_1 = q_Cyan_Up_1 - pi;
q_Cyan_Down_2 = q_Cyan_Up_2 - pi;


% ==========================================
% SECTION 4: SOLVE LOOP 2 (Forward)
% Input: Cyan Down (q2), Find: Grey (q4) & Red (q3)
% ==========================================
a = L2_Loop2; b = L3_Loop2; c = L4_Shared;

% We know 'a' (Cyan), find 'c' (Grey) -> Standard Forward Kinematics
K1_L2 = d/a;
K2_L2 = d/c;
K3_L2 = (a^2 - b^2 + c^2 + d^2)/(2*a*c);
K4_L2 = d/b;
K5_L2 = (c^2 - d^2 - a^2 - b^2)/(2*a*b);

% --- Case 1 Calculation ---
q_in_L2_1 = q_Cyan_Down_1;
A_L2_1 = cos(q_in_L2_1) - K1_L2 - K2_L2*cos(q_in_L2_1) + K3_L2;
B_L2_1 = -2*sin(q_in_L2_1);
C_L2_1 = K1_L2 - (K2_L2+1)*cos(q_in_L2_1) + K3_L2;
% Grey (q4)
q4_Grey_1 = 2*atan((-B_L2_1 - sqrt(B_L2_1^2 - 4*A_L2_1*C_L2_1))/(2*A_L2_1)); % Using -sqrt
% Red (q3)
D_L2_1 = cos(q_in_L2_1) - K1_L2 + K4_L2*cos(q_in_L2_1) + K5_L2;
E_L2_1 = -2*sin(q_in_L2_1);
F_L2_1 = K1_L2 + (K4_L2 - 1)*cos(q_in_L2_1) + K5_L2;
q3_Red_1 = 2*atan((-E_L2_1 - sqrt(E_L2_1^2 - 4*D_L2_1*F_L2_1))/(2*D_L2_1));

% --- Case 2 Calculation ---
q_in_L2_2 = q_Cyan_Down_2;
A_L2_2 = cos(q_in_L2_2) - K1_L2 - K2_L2*cos(q_in_L2_2) + K3_L2;
B_L2_2 = -2*sin(q_in_L2_2);
C_L2_2 = K1_L2 - (K2_L2+1)*cos(q_in_L2_2) + K3_L2;
% Grey (q4)
q4_Grey_2 = 2*atan((-B_L2_2 + sqrt(B_L2_2^2 - 4*A_L2_2*C_L2_2))/(2*A_L2_2)); % Using +sqrt
% Red (q3)
D_L2_2 = cos(q_in_L2_2) - K1_L2 + K4_L2*cos(q_in_L2_2) + K5_L2;
E_L2_2 = -2*sin(q_in_L2_2);
F_L2_2 = K1_L2 + (K4_L2 - 1)*cos(q_in_L2_2) + K5_L2;
q3_Red_2 = 2*atan((-E_L2_2 + sqrt(E_L2_2^2 - 4*D_L2_2*F_L2_2))/(2*D_L2_2));


% ==========================================
% SECTION 5: SOLVE LOOP 1 (Backwards)
% Input: Grey (q4 from Loop 2), Find: Green (q2) & Yellow (q3)
% ==========================================
a = L2_Loop1; b = L3_Loop1; c = L4_Shared;

% We know 'c' (Grey), find 'a' (Green) -> Inverse Kinematics
K1_L1 = d/c;
K2_L1 = d/a;
K3_L1 = (c^2 - b^2 + a^2 + d^2)/(2*c*a);

K1_L1_fwd = d/a;
K4_L1_fwd = d/b;
K5_L1_fwd = (c^2 - d^2 - a^2 - b^2)/(2*a*b);

% --- Case 1 Calculation ---
q_in_L1_1 = q4_Grey_1;
A_L1_1 = cos(q_in_L1_1) - K1_L1 - K2_L1*cos(q_in_L1_1) + K3_L1;
B_L1_1 = -2*sin(q_in_L1_1);
C_L1_1 = K1_L1 - (K2_L1+1)*cos(q_in_L1_1) + K3_L1;
% Green (q2)
q2_Green_1 = 2*atan((-B_L1_1 - sqrt(B_L1_1^2 - 4*A_L1_1*C_L1_1))/(2*A_L1_1));
% Yellow (q3)
D_L1_1 = cos(q2_Green_1) - K1_L1_fwd + K4_L1_fwd*cos(q2_Green_1) + K5_L1_fwd;
E_L1_1 = -2*sin(q2_Green_1);
F_L1_1 = K1_L1_fwd + (K4_L1_fwd - 1)*cos(q2_Green_1) + K5_L1_fwd;
q3_Yellow_1 = 2*atan((-E_L1_1 - sqrt(E_L1_1^2 - 4*D_L1_1*F_L1_1))/(2*D_L1_1));

% --- Case 2 Calculation ---
q_in_L1_2 = q4_Grey_2;
A_L1_2 = cos(q_in_L1_2) - K1_L1 - K2_L1*cos(q_in_L1_2) + K3_L1;
B_L1_2 = -2*sin(q_in_L1_2);
C_L1_2 = K1_L1 - (K2_L1+1)*cos(q_in_L1_2) + K3_L1;
% Green (q2)
q2_Green_2 = 2*atan((-B_L1_2 + sqrt(B_L1_2^2 - 4*A_L1_2*C_L1_2))/(2*A_L1_2));
% Yellow (q3)
D_L1_2 = cos(q2_Green_2) - K1_L1_fwd + K4_L1_fwd*cos(q2_Green_2) + K5_L1_fwd;
E_L1_2 = -2*sin(q2_Green_2);
F_L1_2 = K1_L1_fwd + (K4_L1_fwd - 1)*cos(q2_Green_2) + K5_L1_fwd;
q3_Yellow_2 = 2*atan((-E_L1_2 + sqrt(E_L1_2^2 - 4*D_L1_2*F_L1_2))/(2*D_L1_2));


% ==========================================
% SECTION 6: VECTORS & PLOTTING
% ==========================================
RO4O2 = d*exp(j*offset);
RO4O2x = real(RO4O2); RO4O2y = imag(RO4O2);

% --- Calculate Vectors Case 1 ---
V_Green_1 = L2_Loop1 * exp(j*(q2_Green_1 + offset));
V_Yellow_1 = L3_Loop1 * exp(j*(q3_Yellow_1 + offset));
V_Grey_1 = L4_Shared * exp(j*(q4_Grey_1 + offset));

V_Cyan_Down_1 = L2_Loop2 * exp(j*(q_Cyan_Down_1 + offset));
V_Red_1 = L3_Loop2 * exp(j*(q3_Red_1 + offset));

V_Cyan_Up_1 = L2_Loop3 * exp(j*(q_Cyan_Up_1 + offset));
V_Blue_1 = L3_Loop3 * exp(j*(q3_Blue_1 + offset));
V_Brown_1 = L4_Loop3 * exp(j*(q4_Brown + offset)); % Input

% --- Calculate Vectors Case 2 ---
V_Green_2 = L2_Loop1 * exp(j*(q2_Green_2 + offset));
V_Yellow_2 = L3_Loop1 * exp(j*(q3_Yellow_2 + offset));
V_Grey_2 = L4_Shared * exp(j*(q4_Grey_2 + offset));

V_Cyan_Down_2 = L2_Loop2 * exp(j*(q_Cyan_Down_2 + offset));
V_Red_2 = L3_Loop2 * exp(j*(q3_Red_2 + offset));

V_Cyan_Up_2 = L2_Loop3 * exp(j*(q_Cyan_Up_2 + offset));
V_Blue_2 = L3_Loop3 * exp(j*(q3_Blue_2 + offset));
V_Brown_2 = L4_Loop3 * exp(j*(q4_Brown + offset)); % Input


% --- Plot Case 1 ---
figure(1)
title('Case 1: Open Circuit'); hold on;
quiver(0,0, RO4O2x, RO4O2y, 0, 'Color', [1 0 1], 'LineWidth', 4, 'MaxHeadSize', 0.5); % Pink
% Loop 1
quiver(0,0, real(V_Green_1), imag(V_Green_1), 0, 'green', 'LineWidth', 3, 'MaxHeadSize', 0.5);
quiver(real(V_Green_1), imag(V_Green_1), real(V_Yellow_1), imag(V_Yellow_1), 0, 'yellow', 'LineWidth', 3, 'MaxHeadSize', 0.5);
quiver(RO4O2x, RO4O2y, real(V_Grey_1), imag(V_Grey_1), 0, 'Color', [0.5 0.5 0.5], 'LineWidth', 3, 'MaxHeadSize', 0.5);
% Loop 2
quiver(0,0, real(V_Cyan_Down_1), imag(V_Cyan_Down_1), 0, 'cyan', 'LineWidth', 3, 'MaxHeadSize', 0.5);
quiver(real(V_Cyan_Down_1), imag(V_Cyan_Down_1), real(V_Red_1), imag(V_Red_1), 0, 'red', 'LineWidth', 3, 'MaxHeadSize', 0.5);
% Loop 3
quiver(0,0, real(V_Cyan_Up_1), imag(V_Cyan_Up_1), 0, 'cyan', 'LineWidth', 3, 'MaxHeadSize', 0.5, 'LineStyle', ':');
quiver(real(V_Cyan_Up_1), imag(V_Cyan_Up_1), real(V_Blue_1), imag(V_Blue_1), 0, 'blue', 'LineWidth', 3, 'MaxHeadSize', 0.5);
quiver(RO4O2x, RO4O2y, real(V_Brown_1), imag(V_Brown_1), 0, 'Color', [0.85 0.5 0.1], 'LineWidth', 3, 'MaxHeadSize', 0.5);
axis equal; grid on;

% --- Plot Case 2 ---
figure(2)
title('Case 2: Crossed Circuit'); hold on;
quiver(0,0, RO4O2x, RO4O2y, 0, 'Color', [1 0 1], 'LineWidth', 4, 'MaxHeadSize', 0.5); % Pink
% Loop 1
quiver(0,0, real(V_Green_2), imag(V_Green_2), 0, 'green', 'LineWidth', 3, 'MaxHeadSize', 0.5);
quiver(real(V_Green_2), imag(V_Green_2), real(V_Yellow_2), imag(V_Yellow_2), 0, 'yellow', 'LineWidth', 3, 'MaxHeadSize', 0.5);
quiver(RO4O2x, RO4O2y, real(V_Grey_2), imag(V_Grey_2), 0, 'Color', [0.5 0.5 0.5], 'LineWidth', 3, 'MaxHeadSize', 0.5);
% Loop 2
quiver(0,0, real(V_Cyan_Down_2), imag(V_Cyan_Down_2), 0, 'cyan', 'LineWidth', 3, 'MaxHeadSize', 0.5);
quiver(real(V_Cyan_Down_2), imag(V_Cyan_Down_2), real(V_Red_2), imag(V_Red_2), 0, 'red', 'LineWidth', 3, 'MaxHeadSize', 0.5);
% Loop 3
quiver(0,0, real(V_Cyan_Up_2), imag(V_Cyan_Up_2), 0, 'cyan', 'LineWidth', 3, 'MaxHeadSize', 0.5, 'LineStyle', ':');
quiver(real(V_Cyan_Up_2), imag(V_Cyan_Up_2), real(V_Blue_2), imag(V_Blue_2), 0, 'blue', 'LineWidth', 3, 'MaxHeadSize', 0.5);
quiver(RO4O2x, RO4O2y, real(V_Brown_2), imag(V_Brown_2), 0, 'Color', [0.85 0.5 0.1], 'LineWidth', 3, 'MaxHeadSize', 0.5);
axis equal; grid on;

% Results
disp('--- Results (From Brown Input) ---');
disp(['Case 1 Green: ', num2str(rad2deg(q2_Green_1)+offset_deg)]);
disp(['Case 2 Green: ', num2str(rad2deg(q2_Green_2)+offset_deg)]);
