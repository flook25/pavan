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

% q2 Case 1 (Typically Open/Parallel)
q2_local_1 = 2*atan((-B - sqrt(disc))/(2*A));
% q2 Case 2 (Typically Crossed/Anti-Parallel)
q2_local_2 = 2*atan((-B + sqrt(disc))/(2*A));

% ---------------------------------------------------------
% STEP 2: Find Theta 3 given Theta 2 (For BOTH Cases)
% ---------------------------------------------------------
K1 = d/a;
K4 = d/b;
K5 = (c^2 - d^2 - a^2 - b^2)/(2*a*b);

% --- Calculate q3 for Case 1 ---
D1 = cos(q2_local_1) - K1 + K4*cos(q2_local_1) + K5;
E1 = -2*sin(q2_local_1);
F1 = K1 + (K4 - 1)*cos(q2_local_1) + K5;
q3_local_1 = 2*atan((-E1 - sqrt(E1^2 - 4*D1*F1))/(2*D1)); 

% --- Calculate q3 for Case 2 ---
D2 = cos(q2_local_2) - K1 + K4*cos(q2_local_2) + K5;
E2 = -2*sin(q2_local_2);
F2 = K1 + (K4 - 1)*cos(q2_local_2) + K5;
q3_local_2 = 2*atan((-E2 + sqrt(E2^2 - 4*D2*F2))/(2*D2)); 

% Calculate Global Angles for display
q2_global_1 = rad2deg(q2_local_1 + theta1);
q3_global_1 = rad2deg(q3_local_1 + theta1);

q2_global_2 = rad2deg(q2_local_2 + theta1);
q3_global_2 = rad2deg(q3_local_2 + theta1);

% ---------------------------------------------------------
% VECTOR CALCULATION (Pre-calculate for Plotting)
% ---------------------------------------------------------

% Ground Vector (Same for both)
RO4O2 = d*exp(j*theta1); 
RO4O2x = real(RO4O2); RO4O2y = imag(RO4O2);

% Link 4 Vector (Fixed Input)
RBO4 = c*exp(j*(q4 + theta1));
RBO4x = real(RBO4); RBO4y = imag(RBO4);
O4x = RO4O2x; O4y = RO4O2y;

% --- Vectors for Case 1 (Open/Parallel) ---
RA1 = a*exp(j*(q2_local_1 + theta1));
RBA1 = b*exp(j*(q3_local_1 + theta1));

RA1x = real(RA1); RA1y = imag(RA1);
RBA1x = real(RBA1); RBA1y = imag(RBA1);
% End points B calculated from A path
RB1x = RA1x + RBA1x; RB1y = RA1y + RBA1y; 

% --- Vectors for Case 2 (Crossed/Anti-Parallel) ---
RA2 = a*exp(j*(q2_local_2 + theta1));
RBA2 = b*exp(j*(q3_local_2 + theta1));

RA2x = real(RA2); RA2y = imag(RA2);
RBA2x = real(RBA2); RBA2y = imag(RBA2);
% End points B calculated from A path
RB2x = RA2x + RBA2x; RB2y = RA2y + RBA2y;

% ---------------------------------------------------------
% PLOTTING
% ---------------------------------------------------------
% Define Colors
color_Pink = [1, 0.07, 0.57];  
color_LBlue = [0.2, 0.8, 1];   
color_Blue = [0, 0, 1];        
color_Brown = [0.6, 0.4, 0.2]; 

% --- FIGURE 1: Case 1 (Open Circuit) ---
figure(1);
hold on;
% 1. Ground (Pink)
quiver(0, 0, O4x, O4y, 0, 'Color', color_Pink, 'LineWidth', 4, 'MaxHeadSize', 0.5);
% 2. Link 2 (Light Blue)
quiver(0, 0, RA1x, RA1y, 0, 'Color', color_LBlue, 'LineWidth', 3, 'MaxHeadSize', 0.5);
% 3. Link 3 (Blue) - From A to B
quiver(RA1x, RA1y, RBA1x, RBA1y, 0, 'Color', color_Blue, 'LineWidth', 3, 'MaxHeadSize', 0.5);
% 4. Link 4 (Brown) - From O4 to B
quiver(O4x, O4y, RB1x-O4x, RB1y-O4y, 0, 'Color', color_Brown, 'LineWidth', 3, 'MaxHeadSize', 0.5);

axis equal; grid on;
title(['Case 1: Open Circuit, \theta_4 = ' num2str(q4_global_deg)]);
xlabel('x'); ylabel('y');


% --- FIGURE 2: Case 2 (Crossed Circuit) ---
figure(2);
hold on;
% 1. Ground (Pink)
quiver(0, 0, O4x, O4y, 0, 'Color', color_Pink, 'LineWidth', 4, 'MaxHeadSize', 0.5);
% 2. Link 2 (Light Blue)
quiver(0, 0, RA2x, RA2y, 0, 'Color', color_LBlue, 'LineWidth', 3, 'MaxHeadSize', 0.5);
% 3. Link 3 (Blue) - From A to B
quiver(RA2x, RA2y, RBA2x, RBA2y, 0, 'Color', color_Blue, 'LineWidth', 3, 'MaxHeadSize', 0.5);
% 4. Link 4 (Brown) - From O4 to B
quiver(O4x, O4y, RB2x-O4x, RB2y-O4y, 0, 'Color', color_Brown, 'LineWidth', 3, 'MaxHeadSize', 0.5);

axis equal; grid on;
title(['Case 2: Crossed Circuit, \theta_4 = ' num2str(q4_global_deg)]);
xlabel('x'); ylabel('y');

% Display Results
fprintf('--- Results ---\n');
fprintf('Case 1 (Fig 1): Theta 2 = %.4f deg, Theta 3 = %.4f deg\n', q2_global_1, q3_global_1);
fprintf('Case 2 (Fig 2): Theta 2 = %.4f deg, Theta 3 = %.4f deg\n', q2_global_2, q3_global_2);
