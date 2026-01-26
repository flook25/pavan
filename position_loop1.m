clear all
close all
clc

% 1. Parameter Definitions
L1 = 210; % Link 1 (Ground) - Pink
L2 = 118; % Link 2 (Input) - Light Blue
L3 = 210; % Link 3 (Coupler) - Blue
L4 = 118; % Link 4 (Output) - Brown

a = L2;
b = L3;
c = L4;
d = L1;

% Ground angle (Lifted up by 0.81 degrees)
theta1_deg = 0.81;
theta1 = deg2rad(theta1_deg);

% Input: Theta 4 (Global)
q4_global_deg = 102.5;
q4_global = deg2rad(q4_global_deg);

% Convert to Local Frame
q4 = q4_global - theta1;

% ---------------------------------------------------------
% STEP 1: Find Theta 2 given Theta 4 (Inverse Kinematics)
% ---------------------------------------------------------
% K Constants (Inverse: Swapping a and c)
K1_inv = d/c; 
K2_inv = d/a; 
K3_inv = (c^2 - b^2 + a^2 + d^2)/(2*c*a); 

% Coefficients A, B, C for q2
A = cos(q4) - K1_inv - K2_inv*cos(q4) + K3_inv;
B = -2*sin(q4);
C = K1_inv - (K2_inv + 1)*cos(q4) + K3_inv;

% Solve for q2 (Two Solutions)
disc = B^2 - 4*A*C;

% q2 Solution 1 (Parallel/Open)
q2_local_1 = 2*atan((-B - sqrt(disc))/(2*A));

% q2 Solution 2 (Crossed/Anti-Parallel)
q2_local_2 = 2*atan((-B + sqrt(disc))/(2*A));


% ---------------------------------------------------------
% STEP 2: Find Theta 3 given Theta 2 (Calculated for BOTH Cases)
% ---------------------------------------------------------
% Standard K Constants for Forward Kinematics (finding q3)
K1 = d/a;
K4 = d/b;
K5 = (c^2 - d^2 - a^2 - b^2)/(2*a*b);

% --- Case 1: Find q3_1 using q2_local_1 ---
D1 = cos(q2_local_1) - K1 + K4*cos(q2_local_1) + K5;
E1 = -2*sin(q2_local_1);
F1 = K1 + (K4 - 1)*cos(q2_local_1) + K5;

disc_q3_1 = E1^2 - 4*D1*F1;
% Open circuit usually uses -sqrt
q3_local_1 = 2*atan((-E1 - sqrt(disc_q3_1))/(2*D1)); 

% --- Case 2: Find q3_2 using q2_local_2 ---
D2 = cos(q2_local_2) - K1 + K4*cos(q2_local_2) + K5;
E2 = -2*sin(q2_local_2);
F2 = K1 + (K4 - 1)*cos(q2_local_2) + K5;

disc_q3_2 = E2^2 - 4*D2*F2;
% Crossed circuit usually uses +sqrt (or alternate sign to close loop)
q3_local_2 = 2*atan((-E2 + sqrt(disc_q3_2))/(2*D2)); 


% Calculate Global Angles
q2_global_1 = rad2deg(q2_local_1 + theta1);
q3_global_1 = rad2deg(q3_local_1 + theta1);

q2_global_2 = rad2deg(q2_local_2 + theta1);
q3_global_2 = rad2deg(q3_local_2 + theta1);


% ---------------------------------------------------------
% VECTOR CALCULATION & PLOTTING PREP
% ---------------------------------------------------------
% Colors
color_Pink = [1, 0.07, 0.57];
color_LBlue = [0.2, 0.8, 1];
color_Blue = [0, 0, 1];
color_Brown = [0.6, 0.4, 0.2];

% Ground Vector (Same for both)
RO4O2 = d*exp(j*theta1);
RO4O2x = real(RO4O2); RO4O2y = imag(RO4O2);

% Link 4 Vector (Fixed Input)
RBO4 = c*exp(j*(q4 + theta1)); 
RBO4x = real(RBO4); RBO4y = imag(RBO4);
O4x = RO4O2x; O4y = RO4O2y;

% --- Case 1 Vectors ---
RA1 = a*exp(j*(q2_local_1 + theta1));
RBA1 = b*exp(j*(q3_local_1 + theta1));

RA1x = real(RA1); RA1y = imag(RA1);
RBA1x = real(RBA1); RBA1y = imag(RBA1);
Bx1 = RA1x + RBA1x; By1 = RA1y + RBA1y;

% --- Case 2 Vectors ---
RA2 = a*exp(j*(q2_local_2 + theta1));
RBA2 = b*exp(j*(q3_local_2 + theta1));

RA2x = real(RA2); RA2y = imag(RA2);
RBA2x = real(RBA2); RBA2y = imag(RBA2);
Bx2 = RA2x + RBA2x; By2 = RA2y + RBA2y;


% ---------------------------------------------------------
% PLOTTING
% ---------------------------------------------------------

% --- Figure 1: Case 1 (Open Circuit) ---
figure(1);
hold on;
% Ground (Pink)
quiver(0, 0, O4x, O4y, 0, 'Color', color_Pink, 'LineWidth', 4, 'MaxHeadSize', 0.5);
% Link 2 (Light Blue)
quiver(0, 0, RA1x, RA1y, 0, 'Color', color_LBlue, 'LineWidth', 3, 'MaxHeadSize', 0.5);
% Link 3 (Blue) - A to B
quiver(RA1x, RA1y, RBA1x, RBA1y, 0, 'Color', color_Blue, 'LineWidth', 3, 'MaxHeadSize', 0.5);
% Link 4 (Brown) - O4 to B
quiver(O4x, O4y, Bx1-O4x, By1-O4y, 0, 'Color', color_Brown, 'LineWidth', 3, 'MaxHeadSize', 0.5);

axis equal; grid on;
title(['Case 1: Open Circuit, \theta_4 = ' num2str(q4_global_deg)]);
xlabel('x'); ylabel('y');


% --- Figure 2: Case 2 (Crossed Circuit) ---
figure(2);
hold on;
% Ground (Pink)
quiver(0, 0, O4x, O4y, 0, 'Color', color_Pink, 'LineWidth', 4, 'MaxHeadSize', 0.5);
% Link 2 (Light Blue)
quiver(0, 0, RA2x, RA2y, 0, 'Color', color_LBlue, 'LineWidth', 3, 'MaxHeadSize', 0.5);
% Link 3 (Blue) - A to B
quiver(RA2x, RA2y, RBA2x, RBA2y, 0, 'Color', color_Blue, 'LineWidth', 3, 'MaxHeadSize', 0.5);
% Link 4 (Brown) - O4 to B
quiver(O4x, O4y, Bx2-O4x, By2-O4y, 0, 'Color', color_Brown, 'LineWidth', 3, 'MaxHeadSize', 0.5);

axis equal; grid on;
title(['Case 2: Crossed Circuit, \theta_4 = ' num2str(q4_global_deg)]);
xlabel('x'); ylabel('y');

% Display Results
fprintf('--- Results ---\n');
fprintf('Case 1: Theta 2 = %.4f, Theta 3 = %.4f\n', q2_global_1, q3_global_1);
fprintf('Case 2: Theta 2 = %.4f, Theta 3 = %.4f\n', q2_global_2, q3_global_2);
