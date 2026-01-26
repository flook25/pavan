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
% Input: Cyan Down (q2), Find: Grey (q4) & Red (q
