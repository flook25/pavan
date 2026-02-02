clear all
close all
clc

% ==========================================
% 1. PARAMETERS & INPUT
% ==========================================
L1 = 0.210; % Ground (Pink) - d
L2 = 0.118; % Crank (Cyan) - a (Output in this calc)
L3 = 0.210; % Coupler (Blue) - b
L4 = 0.118; % Rocker (Brown) - c (Input in this calc)

% Assign names for calculation
d = L1;
a = L2; 
b = L3; 
c = L4; 

% Input Parameters
theta4_deg_global = 102.05; 
offset_deg = 0.81;

% Convert to radians
offset = deg2rad(offset_deg);
% Global Input Angle relative to Ground
theta4_local = deg2rad(theta4_deg_global) - offset; 

% ==========================================
% 2. POSITION ANALYSIS (Inverted Mechanism)
% ==========================================
% Concept: ย้ายจุดอ้างอิงไปที่ O4 (Link 4 เป็น Input)
% ต้องหมุนแกน Input ไป 180 องศา (pi) เพื่อให้ตรงกับ Frame การคำนวณมาตรฐาน
q_in_calc = theta4_local - pi; 

% Set parameters for calculation (Input=c, Output=a)
d_calc = d;
a_calc = c; % Link 4 acts as driver
b_calc = b;
c_calc = a; % Link 2 acts as follower

% Freudenstein K-Constants
K1 = d_calc/a_calc;
K2 = d_calc/c_calc;
K3 = (a_calc^2 - b_calc^2 + c_calc^2 + d_calc^2)/(2*a_calc*c_calc);
K4 = d_calc/b_calc;
K5 = (c_calc^2 - d_calc^2 - a_calc^2 - b_calc^2)/(2*a_calc*b_calc);

% Coefficients (A, B, C, D, E, F)
A = cos(q_in_calc) - K1 - K2*cos(q_in_calc) + K3;
B = -2*sin(q_in_calc);
C = K1 - (K2+1)*cos(q_in_calc) + K3;

D_val = cos(q_in_calc) - K1 + K4*cos(q_in_calc) + K5;
E_val = -2*sin(q_in_calc);
F_val = K1 + (K4-1)*cos(q_in_calc) + K5;

% --- SOLVE QUADRATIC (Follow Example Logic) ---
det_AC = sqrt(B^2 - 4*A*C);
det_DF = sqrt(E_val^2 - 4*D_val*F_val);

% --- CASE 1: OPEN (Matches Example's (-B - sqrt)) ---
% Output Angle (Link 2)
q_out_1_calc = 2*atan2((-B - det_AC), (2*A)); 
% Coupler Angle (Link 3)
q3_1_calc    = 2*atan2((-E_val - det_DF), (2*D_val));

% --- CASE 2: CROSSED (Matches Example's (-B + sqrt)) ---
% Output Angle (Link 2)
q_out_2_calc = 2*atan2((-B + det_AC), (2*A));
% Coupler Angle (Link 3)
q3_2_calc    = 2*atan2((-E_val + det_DF), (2*D_val));

% ==========================================
% 3. TRANSFORM BACK TO GLOBAL
% ==========================================
% หมุนค่ากลับ 180 องศา (+pi) เพื่อคืนสู่ Global Frame เดิม
Theta2_Open = q_out_1_calc + pi;
Theta3_Open = q3_1_calc + pi;

Theta2_Cross = q_out_2_calc + pi;
Theta3_Cross = q3_2_calc + pi;

Theta4_Final = theta4_local; % Input remains same

% ==========================================
% 4. DISPLAY RESULTS
% ==========================================
disp('======================================');
disp('       POSITION RESULTS (Degrees)');
disp('======================================');
disp(['Input Theta 4: ', num2str(theta4_deg_global)]);
disp(' ');
disp('--- CASE 1 (OPEN) ---');
disp(['  Link 2 (Cyan):  ', num2str(rad2deg(Theta2_Open) + offset_deg)]);
disp(['  Link 3 (Blue):  ', num2str(rad2deg(Theta3_Open) + offset_deg)]);
disp(' ');
disp('--- CASE 2 (CROSSED) ---');
disp(['  Link 2 (Cyan):  ', num2str(rad2deg(Theta2_Cross) + offset_deg)]);
disp(['  Link 3 (Blue):  ', num2str(rad2deg(Theta3_Cross) + offset_deg)]);

% ==========================================
% 5. PLOTTING VECTORS
% ==========================================
% Global Rotation due to Offset
Rot = exp(1i * offset);

% Ground Vector (Pink)
RO4O2 = d * exp(1i * 0) * Rot; 

% --- CASE 1: OPEN ---
% Link 2 (Cyan)
R_Cyan_1 = a * exp(1i * Theta2_Open) * Rot;
% Link 4 (Brown)
R_Brown_1 = c * exp(1i * Theta4_Final) * Rot;
% Link 3 (Blue) - Connecting Tip of 2 to Tip of 4
% Logic: R_Ground + R_Link4 = R_Link2 + R_Link3
% So: R_Link3 = (R_Ground + R_Link4) - R_Link2
Vec_B_1 = RO4O2 + R_Brown_1; % Tip of Link 4 (Point B)
Vec_A_1 = R_Cyan_1;          % Tip of Link 2 (Point A) Note: Link 2 starts at (0,0)
R_Blue_1 = Vec_B_1 - Vec_A_1;

% --- CASE 2: CROSSED ---
R_Cyan_2 = a * exp(1i * Theta2_Cross) * Rot;
R_Brown_2 = c * exp(1i * Theta4_Final) * Rot;

Vec_B_2 = RO4O2 + R_Brown_2;
Vec_A_2 = R_Cyan_2;
R_Blue_2 = Vec_B_2 - Vec_A_2;

% --- PLOT SETUP ---
figure(1); clf;

% Subplot 1: Open
subplot(1,2,1); hold on; grid on; axis equal;
title('Case 1: Open');
% Plot Ground
plot([0 real(RO4O2)], [0 imag(RO4O2)], 'm-', 'LineWidth', 2); 
% Plot Link 2 (Cyan)
plot([0 real(R_Cyan_1)], [0 imag(R_Cyan_1)], 'c-', 'LineWidth', 2);
% Plot Link 4 (Brown) - Starts from end of Ground
plot([real(RO4O2) real(RO4O2)+real(R_Brown_1)], ...
     [imag(RO4O2) imag(RO4O2)+imag(R_Brown_1)], 'Color', [0.6 0.3 0], 'LineWidth', 2);
% Plot Link 3 (Blue) - Connects Cyan to Brown
plot([real(R_Cyan_1) real(R_Cyan_1)+real(R_Blue_1)], ...
     [imag(R_Cyan_1) imag(R_Cyan_1)+imag(R_Blue_1)], 'b-', 'LineWidth', 2);

% Subplot 2: Crossed
subplot(1,2,2); hold on; grid on; axis equal;
title('Case 2: Crossed');
% Plot Ground
plot([0 real(RO4O2)], [0 imag(RO4O2)], 'm-', 'LineWidth', 2);
% Plot Link 2 (Cyan)
plot([0 real(R_Cyan_2)], [0 imag(R_Cyan_2)], 'c-', 'LineWidth', 2);
% Plot Link 4 (Brown)
plot([real(RO4O2) real(RO4O2)+real(R_Brown_2)], ...
     [imag(RO4O2) imag(RO4O2)+imag(R_Brown_2)], 'Color', [0.6 0.3 0], 'LineWidth', 2);
% Plot Link 3 (Blue)
plot([real(R_Cyan_2) real(R_Cyan_2)+real(R_Blue_2)], ...
     [imag(R_Cyan_2) imag(R_Cyan_2)+imag(R_Blue_2)], 'b-', 'LineWidth', 2);

% Use Quiver as requested for vectors (Optional overlay)
quiver(0,0, real(R_Cyan_1), imag(R_Cyan_1), 0, 'c', 'LineWidth', 1);
