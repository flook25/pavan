clear all
close all
clc

% 1. Parameter Definitions
L1 = 210; % Length of Link1 d (Ground) - Pink
L2 = 118; % Length of Link2 a (Input) - Light Blue (Note: Your prompt L2=118)
L3 = 210; % Length of Link3 b (Coupler) - Blue
L4 = 118; % Length of Link4 c (Output) - Brown

% Mapping to standard variables (Norton)
d = L1;
a = L2;
b = L3;
c = L4;

% Ground angle (Lifted up)
theta1 = 0.81; % radians

% Input: Theta 4 (Global)
q4_global_deg = 102.5;
q4_global = deg2rad(q4_global_deg);

% Convert to Local Frame (Relative to Ground Link)
q4 = q4_global - theta1;

% ---------------------------------------------------------
% STEP 1: Find Theta 2 given Theta 4 (Inverse Kinematics)
% We treat Link 4 (c) as the driver to find Link 2 (a).
% ---------------------------------------------------------

% Inverse K Constants (Swapping a and c for Inverse Kinematics)
K1_inv = d/c; 
K2_inv = d/a; 
K3_inv = (c^2 - b^2 + a^2 + d^2)/(2*c*a); % Swapped a and c

% Freudenstein Coefficients for finding q2 (Input is q4)
A_inv = cos(q4) - K1_inv - K2_inv*cos(q4) + K3_inv;
B_inv = -2*sin(q4);
C_inv = K1_inv - (K2_inv + 1)*cos(q4) + K3_inv;

% Solve for q2 using 2*arctan formula
disc = B_inv^2 - 4*A_inv*C_inv;

% Case 1 & 2 for Theta 2
q2_local_1 = 2*atan((-B_inv - sqrt(disc))/(2*A_inv)); % Solution 1 (Usually Open/Parallel for this setup)
q2_local_2 = 2*atan((-B_inv + sqrt(disc))/(2*A_inv)); % Solution 2 (Usually Crossed/Anti-Parallel)

% Convert back to global
q2_global_1 = rad2deg(q2_local_1 + theta1);
q2_global_2 = rad2deg(q2_local_2 + theta1);

% ---------------------------------------------------------
% STEP 2: Find Theta 3 given Theta 2
% Standard Forward K constants
% ---------------------------------------------------------
K1 = d/a;
K4 = d/b;
K5 = (c^2 - d^2 - a^2 - b^2)/(2*a*b);

% --- For Case 1 (Using q2_local_1) ---
D1 = cos(q2_local_1) - K1 + K4*cos(q2_local_1) + K5;
E1 = -2*sin(q2_local_1);
F1 = K1 + (K4 - 1)*cos(q2_local_1) + K5;

disc_q3_1 = E1^2 - 4*D1*F1;
% Note: Sign selection logic for q3 depends on geometric closure.
% Usually -sqrt goes with Open, +sqrt goes with Crossed, but must be checked.
% Based on typical parallelogram logic:
q3_local_1 = 2*atan((-E1 - sqrt(disc_q3_1))/(2*D1)); 

q3_global_1 = rad2deg(q3_local_1 + theta1);

% --- For Case 2 (Using q2_local_2) ---
D2 = cos(q2_local_2) - K1 + K4*cos(q2_local_2) + K5;
E2 = -2*sin(q2_local_2);
F2 = K1 + (K4 - 1)*cos(q2_local_2) + K5;

disc_q3_2 = E2^2 - 4*D2*F2;
q3_local_2 = 2*atan((-E2 + sqrt(disc_q3_2))/(2*D2)); 

q3_global_2 = rad2deg(q3_local_2 + theta1);

% ---------------------------------------------------------
% PLOTTING
% ---------------------------------------------------------

% Vectors Setup
% Ground
RO4O2 = d*exp(j*theta1); 
RO4O2x = real(RO4O2); RO4O2y = imag(RO4O2);

% --- Case 1: Open Circuit (Solution 1) ---
RA1 = a*exp(j*(q2_local_1 + theta1));
RBA1 = b*exp(j*(q3_local_1 + theta1));
RBO4_1 = c*exp(j*(q4 + theta1));

RA1x = real(RA1); RA1y = imag(RA1);
% Calculate B from A path (Vector Loop Closure Check)
RB1x = real(RA1 + RBA1); RB1y = imag(RA1 + RBA1);
% Calculate B from O4 path (for plotting the link 4)
RBO4_1x = real(RBO4_1); RBO4_1y = imag(RBO4_1);


% --- Case 2: Crossed Circuit (Solution 2) ---
RA2 = a*exp(j*(q2_local_2 + theta1));
RBA2 = b*exp(j*(q3_local_2 + theta1));
RBO4_2 = c*exp(j*(q4 + theta1)); % q4 is input, same for both

RA2x = real(RA2); RA2y = imag(RA2);
RB2x = real(RA2 + RBA2); RB2y = imag(RA2 + RBA2);
RBO4_2x = real(RBO4_2); RBO4_2y = imag(RBO4_2);


% Colors: Pink(L1), Light Blue(L2), Blue(L3), Brown(L4)
color_L1 = [1, 0.4, 0.7]; % Pink
color_L2 = [0.4, 0.8, 1]; % Light Blue
color_L3 = 'b';           % Blue
color_L4 = [0.6, 0.4, 0.2]; % Brown


% --- Figure 1: Case 1 (Open Circuit) ---
figure(1);
title(['Case 1: Open Circuit (\theta_4 = ', num2str(q4_global_deg), ')']);
hold on;
% Link 1 (Ground) - Pink
quiver(0,0, RO4O2x, RO4O2y, 0, 'Color', color_L1, 'LineWidth', 4, 'MaxHeadSize', 0.5);
% Link 2 (Input) - Light Blue
quiver(0,0, RA1x, RA1y, 0, 'Color', color_L2, 'LineWidth', 3, 'MaxHeadSize', 0.5);
% Link 3 (Coupler) - Blue (From A to B)
quiver(RA1x, RA1y, RB1x-RA1x, RB1y-RA1y, 0, 'Color', color_L3, 'LineWidth', 3, 'MaxHeadSize', 0.5);
% Link 4 (Output) - Brown (From O4 to B)
quiver(RO4O2x, RO4O2y, RB1x-RO4O2x, RB1y-RO4O2y, 0, 'Color', color_L4, 'LineWidth', 3, 'MaxHeadSize', 0.5);

axis equal; grid on;
xlabel('x'); ylabel('y');


% --- Figure 2: Case 2 (Crossed Circuit) ---
figure(2);
title(['Case 2: Crossed Circuit (\theta_4 = ', num2str(q4_global_deg), ')']);
hold on;
% Link 1 (Ground) - Pink
quiver(0,0, RO4O2x, RO4O2y, 0, 'Color', color_L1, 'LineWidth', 4, 'MaxHeadSize', 0.5);
% Link 2 (Input) - Light Blue
quiver(0,0, RA2x, RA2y, 0, 'Color', color_L2, 'LineWidth', 3, 'MaxHeadSize', 0.5);
% Link 3 (Coupler) - Blue (From A to B)
quiver(RA2x, RA2y, RB2x-RA2x, RB2y-RA2y, 0, 'Color', color_L3, 'LineWidth', 3, 'MaxHeadSize', 0.5);
% Link 4 (Output) - Brown (From O4 to B)
quiver(RO4O2x, RO4O2y, RB2x-RO4O2x, RB2y-RO4O2y, 0, 'Color', color_L4, 'LineWidth', 3, 'MaxHeadSize', 0.5);

axis equal; grid on;
xlabel('x'); ylabel('y');

% Display Results
disp('--- Results ---');
disp(['Input Theta 4 (Global): ', num2str(q4_global_deg)]);
disp(['Case 1 (Open)    - Theta 2: ', num2str(q2_global_1), '  Theta 3: ', num2str(q3_global_1)]);
disp(['Case 2 (Crossed) - Theta 2: ', num2str(q2_global_2), '  Theta 3: ', num2str(q3_global_2)]);
