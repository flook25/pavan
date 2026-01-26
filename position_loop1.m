clear all
close all
clc

% 1. Parameter Definitions 
L1 = 210; % Link 1 (Ground) - Pink
L2 = 118; % Link 2 (Input) - Light Blue
L3 = 210; % Link 3 (Coupler) - Blue
L4 = 118; % Link 4 (Output) - Brown

% Mapping to standard variables
d = L1;
a = L2;
b = L3;
c = L4;

% Ground angle (Lifted up by 0.81 degrees)
theta1_deg = 0.81;
theta1 = deg2rad(theta1_deg); 

% Input: Theta 4 (Global)
q4_global_deg = 102.5;
q4_global = deg2rad(q4_global_deg);

% Convert to Local Frame (Relative to Ground Link)
q4 = q4_global - theta1;

% ---------------------------------------------------------
% STEP 1: Find Theta 2 given Theta 4 (Inverse Kinematics)
% ---------------------------------------------------------

% K Constants (Swapping a and c)
K1 = d/c; 
K2 = d/a; 
K3 = (c^2 - b^2 + a^2 + d^2)/(2*c*a); 

% Coefficients A, B, C for finding q2
A = cos(q4) - K1 - K2*cos(q4) + K3;
B = -2*sin(q4);
C = K1 - (K2 + 1)*cos(q4) + K3;

% Solve for q2 (Inverse Kinematics has 2 solutions)
disc = B^2 - 4*A*C;

% q2 Candidate 1 (User's Code Sol 1)
q2_local_1 = 2*atan((-B - sqrt(disc))/(2*A));

% q2 Candidate 2 (User's Code Sol 2)
q2_local_2 = 2*atan((-B + sqrt(disc))/(2*A));

% ---------------------------------------------------------
% STEP 2: Find Theta 3 for CASE 1
% (ใช้ q2_local_1 มาคำนวณ D, E, F)
% ---------------------------------------------------------
q2_target_1 = q2_local_1; 

K1_fwd = d/a;
K4_fwd = d/b;
K5_fwd = (c^2 - d^2 - a^2 - b^2)/(2*a*b);

D1 = cos(q2_target_1) - K1_fwd + K4_fwd*cos(q2_target_1) + K5_fwd;
E1 = -2*sin(q2_target_1);
F1 = K1_fwd + (K4_fwd - 1)*cos(q2_target_1) + K5_fwd;

disc_q3_1 = E1^2 - 4*D1*F1;

% User's formula (+sqrt)
q3_local_1 = 2*atan((-E1 + sqrt(disc_q3_1))/(2*D1)); 

% ---------------------------------------------------------
% STEP 3: Find Theta 3 for CASE 2 
% (ใช้ q2_local_2 มาคำนวณ D, E, F ใหม่! เพื่อให้ Theta 3 เปลี่ยน)
% ---------------------------------------------------------
q2_target_2 = q2_local_2; 

D2 = cos(q2_target_2) - K1_fwd + K4_fwd*cos(q2_target_2) + K5_fwd;
E2 = -2*sin(q2_target_2);
F2 = K1_fwd + (K4_fwd - 1)*cos(q2_target_2) + K5_fwd;

disc_q3_2 = E2^2 - 4*D2*F2;

% ใช้สูตรเดิมกับชุดตัวแปรใหม่
q3_local_2 = 2*atan((-E2 + sqrt(disc_q3_2))/(2*D2)); 

% ---------------------------------------------------------
% Convert to Global Angles
% ---------------------------------------------------------
% Case 1
q2_global_1 = rad2deg(q2_target_1 + theta1);
q3_global_1 = rad2deg(q3_local_1 + theta1);

% Case 2
q2_global_2 = rad2deg(q2_target_2 + theta1);
q3_global_2 = rad2deg(q3_local_2 + theta1);

% ---------------------------------------------------------
% PLOTTING PREPARATION
% ---------------------------------------------------------
% Define Colors
color_Pink = [1, 0.07, 0.57];  
color_LBlue = [0.2, 0.8, 1];   
color_Blue = [0, 0, 1];        
color_Brown = [0.6, 0.4, 0.2]; 

% Ground Vector (Common)
RO4O2 = d*exp(j*theta1); 
O2 = [0, 0];
O4 = [real(RO4O2), imag(RO4O2)];

% --- Vectors for Case 1 ---
RA1 = a*exp(j*(q2_target_1 + theta1));
RBA1 = b*exp(j*(q3_local_1 + theta1));
RBO4_1 = c*exp(j*(q4 + theta1));

A1 = [real(RA1), imag(RA1)];
B1_from_A = [real(RA1 + RBA1), imag(RA1 + RBA1)];

% --- Vectors for Case 2 ---
RA2 = a*exp(j*(q2_target_2 + theta1));
RBA2 = b*exp(j*(q3_local_2 + theta1));
RBO4_2 = c*exp(j*(q4 + theta1)); % q4 same input

A2 = [real(RA2), imag(RA2)];
B2_from_A = [real(RA2 + RBA2), imag(RA2 + RBA2)];

% ---------------------------------------------------------
% FIGURE 1: Case 1 (User's validated case)
% ---------------------------------------------------------
figure(1);
hold on;
% Ground (Pink)
quiver(O2(1), O2(2), O4(1)-O2(1), O4(2)-O2(2), 0, 'Color', color_Pink, 'LineWidth', 4, 'MaxHeadSize', 0.5);
% Link 2 (Light Blue)
quiver(O2(1), O2(2), A1(1)-O2(1), A1(2)-O2(2), 0, 'Color', color_LBlue, 'LineWidth', 3, 'MaxHeadSize', 0.5);
% Link 3 (Blue)
quiver(A1(1), A1(2), B1_from_A(1)-A1(1), B1_from_A(2)-A1(2), 0, 'Color', color_Blue, 'LineWidth', 3, 'MaxHeadSize', 0.5);
% Link 4 (Brown)
quiver(O4(1), O4(2), B1_from_A(1)-O4(1), B1_from_A(2)-O4(2), 0, 'Color', color_Brown, 'LineWidth', 3, 'MaxHeadSize', 0.5);
axis equal; grid on;
title(['Case 1: First Solution, \theta_4 = ' num2str(q4_global_deg)]);
xlabel('x'); ylabel('y');


% ---------------------------------------------------------
% FIGURE 2: Case 2 (The other solution)
% ---------------------------------------------------------
figure(2);
hold on;
% Ground (Pink)
quiver(O2(1), O2(2), O4(1)-O2(1), O4(2)-O2(2), 0, 'Color', color_Pink, 'LineWidth', 4, 'MaxHeadSize', 0.5);
% Link 2 (Light Blue)
quiver(O2(1), O2(2), A2(1)-O2(1), A2(2)-O2(2), 0, 'Color', color_LBlue, 'LineWidth', 3, 'MaxHeadSize', 0.5);
% Link 3 (Blue)
quiver(A2(1), A2(2), B2_from_A(1)-A2(1), B2_from_A(2)-A2(2), 0, 'Color', color_Blue, 'LineWidth', 3, 'MaxHeadSize', 0.5);
% Link 4 (Brown)
quiver(O4(1), O4(2), B2_from_A(1)-O4(1), B2_from_A(2)-O4(2), 0, 'Color', color_Brown, 'LineWidth', 3, 'MaxHeadSize', 0.5);
axis equal; grid on;
title(['Case 2: Second Solution, \theta_4 = ' num2str(q4_global_deg)]);
xlabel('x'); ylabel('y');


% Display Results
fprintf('--- Case 1 ---\n');
fprintf('Theta 2 (Global) = %.4f deg\n', q2_global_1);
fprintf('Theta 3 (Global) = %.4f deg\n', q3_global_1);
fprintf('\n--- Case 2 ---\n');
fprintf('Theta 2 (Global) = %.4f deg\n', q2_global_2);
fprintf('Theta 3 (Global) = %.4f deg\n', q3_global_2);
