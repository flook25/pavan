clear all
close all
clc

% 1. Parameter Definitions
L1 = 210; % Length of Link1 d (Ground) - Pink
L2 = 118; % Length of Link2 a (Input) - Light Blue
L3 = 210; % Length of Link3 b (Coupler) - Blue
L4 = 118; % Length of Link4 c (Output) - Brown

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

% K Constants (Swapping a and c for Inverse Kinematics)
K1_inv = d/c; 
K2_inv = d/a; 
K3_inv = (c^2 - b^2 + a^2 + d^2)/(2*c*a); 

% Coefficients A, B, C for finding q2
A = cos(q4) - K1_inv - K2_inv*cos(q4) + K3_inv;
B = -2*sin(q4);
C = K1_inv - (K2_inv + 1)*cos(q4) + K3_inv;

% Solve for q2 (Two Solutions)
disc = B^2 - 4*A*C;

% *** กำหนด Case 1 ให้เป็นแบบ "ชี้ขึ้น" (Up) โดยใช้ -sqrt ***
q2_local_1 = 2*atan((-B - sqrt(disc))/(2*A));

% *** กำหนด Case 2 ให้เป็นแบบ "ชี้ลง" (Down) โดยใช้ +sqrt ***
q2_local_2 = 2*atan((-B + sqrt(disc))/(2*A));

% ---------------------------------------------------------
% STEP 2: Find Theta 3 (Calculate separate q3 for EACH case)
% ---------------------------------------------------------
K1 = d/a;
K4 = d/b;
K5 = (c^2 - d^2 - a^2 - b^2)/(2*a*b);

% --- Calculate Theta 3 for Case 1 (Using q2_local_1) ---
D1 = cos(q2_local_1) - K1 + K4*cos(q2_local_1) + K5;
E1 = -2*sin(q2_local_1);
F1 = K1 + (K4 - 1)*cos(q2_local_1) + K5;
% สำหรับ Case ขึ้น (Parallel) ปกติใช้ -sqrt เพื่อให้ q3 ขนาน
q3_local_1 = 2*atan((-E1 - sqrt(E1^2 - 4*D1*F1))/(2*D1));

% --- Calculate Theta 3 for Case 2 (Using q2_local_2) ---
D2 = cos(q2_local_2) - K1 + K4*cos(q2_local_2) + K5;
E2 = -2*sin(q2_local_2);
F2 = K1 + (K4 - 1)*cos(q2_local_2) + K5;
% สำหรับ Case ลง (Crossed) ใช้ +sqrt เพื่อให้ปิดลูปได้ถูกต้อง
q3_local_2 = 2*atan((-E2 + sqrt(E2^2 - 4*D2*F2))/(2*D2));


% Calculate Global Angles for display
q2_global_1 = rad2deg(q2_local_1 + theta1);
q3_global_1 = rad2deg(q3_local_1 + theta1);

q2_global_2 = rad2deg(q2_local_2 + theta1);
q3_global_2 = rad2deg(q3_local_2 + theta1);


% ---------------------------------------------------------
% VECTOR CALCULATION (Pre-calculate for Plotting)
% ---------------------------------------------------------
% Define Colors
color_Pink = [1, 0.07, 0.57];  
color_LBlue = [0.2, 0.8, 1];   
color_Blue = [0, 0, 1];        
color_Brown = [0.6, 0.4, 0.2]; 

% Ground Vector (Same for both)
RO4O2 = d*exp(j*theta1); 
RO4O2x = real(RO4O2); RO4O2y = imag(RO4O2);
O4x = RO4O2x; O4y = RO4O2y;

% --- Vectors for Case 1 (Up) ---
RA1 = a*exp(j*(q2_local_1 + theta1));
RBA1 = b*exp(j*(q3_local_1 + theta1));
% End points Case 1
Ax1 = real(RA1); Ay1 = imag(RA1);
Bx1 = real(RA1 + RBA1); By1 = imag(RA1 + RBA1);

% --- Vectors for Case 2 (Down) ---
RA2 = a*exp(j*(q2_local_2 + theta1));
RBA2 = b*exp(j*(q3_local_2 + theta1));
% End points Case 2
Ax2 = real(RA2); Ay2 = imag(RA2);
Bx2 = real(RA2 + RBA2); By2 = imag(RA2 + RBA2);

% Link 4 Tip check (should land on Bx, By)
% RBO4 = c*exp(j*(q4+theta1));


% ---------------------------------------------------------
% PLOTTING FIGURE 1 (Case 1: Up Configuration)
% ---------------------------------------------------------
figure(1);
hold on;
% 1. Ground (Pink)
quiver(0, 0, O4x, O4y, 0, 'Color', color_Pink, 'LineWidth', 4, 'MaxHeadSize', 0.5);
% 2. Link 2 (Light Blue) - UP
quiver(0, 0, Ax1, Ay1, 0, 'Color', color_LBlue, 'LineWidth', 3, 'MaxHeadSize', 0.5);
% 3. Link 3 (Blue)
quiver(Ax1, Ay1, Bx1-Ax1, By1-Ay1, 0, 'Color', color_Blue, 'LineWidth', 3, 'MaxHeadSize', 0.5);
% 4. Link 4 (Brown)
quiver(O4x, O4y, Bx1-O4x, By1-O4y, 0, 'Color', color_Brown, 'LineWidth', 3, 'MaxHeadSize', 0.5);

axis equal; grid on;
title(['Case 1: Up Configuration (Parallel), \theta_4 = ' num2str(q4_global_deg)]);
xlabel('x'); ylabel('y');


% ---------------------------------------------------------
% PLOTTING FIGURE 2 (Case 2: Down Configuration)
% ---------------------------------------------------------
figure(2);
hold on;
% 1. Ground (Pink)
quiver(0, 0, O4x, O4y, 0, 'Color', color_Pink, 'LineWidth', 4, 'MaxHeadSize', 0.5);
% 2. Link 2 (Light Blue) - DOWN
quiver(0, 0, Ax2, Ay2, 0, 'Color', color_LBlue, 'LineWidth', 3, 'MaxHeadSize', 0.5);
% 3. Link 3 (Blue)
quiver(Ax2, Ay2, Bx2-Ax2, By2-Ay2, 0, 'Color', color_Blue, 'LineWidth', 3, 'MaxHeadSize', 0.5);
% 4. Link 4 (Brown)
quiver(O4x, O4y, Bx2-O4x, By2-O4y, 0, 'Color', color_Brown, 'LineWidth', 3, 'MaxHeadSize', 0.5);

axis equal; grid on;
title(['Case 2: Down Configuration (Crossed), \theta_4 = ' num2str(q4_global_deg)]);
xlabel('x'); ylabel('y');


% Display Results
fprintf('--- Results ---\n');
fprintf('Case 1 (Up):   Theta 2 = %.4f deg, Theta 3 = %.4f deg\n', q2_global_1, q3_global_1);
fprintf('Case 2 (Down): Theta 2 = %.4f deg, Theta 3 = %.4f deg\n', q2_global_2, q3_global_2);
