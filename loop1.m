clear all
close all
clc

% 1. Parameter Definitions
L1 = 210; % Length of Link1 d (Ground) - Pink
L2 = 118; % Length of Link2 a (Input) - Light Blue
L3 = 210; % Length of Link3 b (Coupler) - Blue
L4 = 118; % Length of Link4 c (Output) - Brown

a = L2;
b = L3;
c = L4;
d = L1;

% Ground angle (Lifted up)
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

% K Constants (Swapping a and c logic for Inverse)
K1_inv = d/c; 
K2_inv = d/a; 
K3_inv = (c^2 - b^2 + a^2 + d^2)/(2*c*a); 

% Coefficients A, B, C for finding q2
A = cos(q4) - K1_inv - K2_inv*cos(q4) + K3_inv;
B = -2*sin(q4);
C = K1_inv - (K2_inv + 1)*cos(q4) + K3_inv;

% Solve for q2 (Two Solutions)
disc = B^2 - 4*A*C;

% q2 Case 1 & Case 2
q2_local_1 = 2*atan((-B - sqrt(disc))/(2*A)); % Usually Open
q2_local_2 = 2*atan((-B + sqrt(disc))/(2*A)); % Usually Crossed

% ---------------------------------------------------------
% STEP 2: Find Theta 3 given Theta 2 (For both cases)
% ---------------------------------------------------------
% Standard K for finding q3
K1 = d/a;
K4 = d/b;
K5 = (c^2 - d^2 - a^2 - b^2)/(2*a*b);

% --- Case 1 Calculation ---
D1 = cos(q2_local_1) - K1 + K4*cos(q2_local_1) + K5;
E1 = -2*sin(q2_local_1);
F1 = K1 + (K4 - 1)*cos(q2_local_1) + K5;
q3_local_1 = 2*atan((-E1 - sqrt(E1^2 - 4*D1*F1))/(2*D1)); 

% --- Case 2 Calculation ---
D2 = cos(q2_local_2) - K1 + K4*cos(q2_local_2) + K5;
E2 = -2*sin(q2_local_2);
F2 = K1 + (K4 - 1)*cos(q2_local_2) + K5;
q3_local_2 = 2*atan((-E2 + sqrt(E2^2 - 4*D2*F2))/(2*D2)); 

% ---------------------------------------------------------
% Vector Calculation (Pre-calculation for Plotting)
% ---------------------------------------------------------

% Ground Vector (Lifted)
RO4O2 = d*exp(j*theta1);

% Case 1 Vectors (Open)
RA1 = a*exp(j*(q2_local_1 + theta1));
RBA1 = b*exp(j*(q3_local_1 + theta1));
RBO4_1 = c*exp(j*(q4 + theta1)); % Fixed Input

% Case 2 Vectors (Crossed)
RA2 = a*exp(j*(q2_local_2 + theta1));
RBA2 = b*exp(j*(q3_local_2 + theta1));
RBO4_2 = c*exp(j*(q4 + theta1)); % Fixed Input

% Extract Components for Quiver
% Ground
RO4O2x = real(RO4O2); RO4O2y = imag(RO4O2);

% Case 1
RA1x = real(RA1); RA1y = imag(RA1);
RBA1x = real(RBA1); RBA1y = imag(RBA1);
RB1x = real(RA1 + RBA1); RB1y = imag(RA1 + RBA1); % Position B
RBO4_1x = real(RBO4_1); RBO4_1y = imag(RBO4_1);

% Case 2
RA2x = real(RA2); RA2y = imag(RA2);
RBA2x = real(RBA2); RBA2y = imag(RBA2);
RB2x = real(RA2 + RBA2); RB2y = imag(RA2 + RBA2); % Position B
RBO4_2x = real(RBO4_2); RBO4_2y = imag(RBO4_2);


% ---------------------------------------------------------
% PLOTTING
% ---------------------------------------------------------
figure(1);
hold on;

% Define Colors
color_Pink = [1, 0.07, 0.57];
color_LBlue = [0.2, 0.8, 1];
color_Blue = [0, 0, 1];
color_Brown = [0.6, 0.4, 0.2];

% --- Plot Case 1: Open Circuit (Solid Lines) ---
% Ground (Pink)
quiver(0, 0, RO4O2x, RO4O2y, 0, 'Color', color_Pink, 'LineWidth', 4, 'MaxHeadSize', 0.5);
% Link 2 (Light Blue)
quiver(0, 0, RA1x, RA1y, 0, 'Color', color_LBlue, 'LineWidth', 3, 'MaxHeadSize', 0.5);
% Link 3 (Blue)
quiver(RA1x, RA1y, RBA1x, RBA1y, 0, 'Color', color_Blue, 'LineWidth', 3, 'MaxHeadSize', 0.5);
% Link 4 (Brown)
quiver(RO4O2x, RO4O2y, RBO4_1x, RBO4_1y, 0, 'Color', color_Brown, 'LineWidth', 3, 'MaxHeadSize', 0.5);

% --- Plot Case 2: Crossed Circuit (Dashed Lines) ---
% Link 2 (Light Blue)
quiver(0, 0, RA2x, RA2y, 0, 'Color', color_LBlue, 'LineWidth', 2, 'LineStyle', '--', 'MaxHeadSize', 0.5);
% Link 3 (Blue)
quiver(RA2x, RA2y, RBA2x, RBA2y, 0, 'Color', color_Blue, 'LineWidth', 2, 'LineStyle', '--', 'MaxHeadSize', 0.5);
% Link 4 (Brown) - Same position but functionally distinct
quiver(RO4O2x, RO4O2y, RBO4_2x, RBO4_2y, 0, 'Color', color_Brown, 'LineWidth', 2, 'LineStyle', '--', 'MaxHeadSize', 0.5);

axis equal; 
grid on;
title(['Fourbar Mechanism: \theta_4 = ' num2str(q4_global_deg)]);
xlabel('x'); ylabel('y');

% Display Results
fprintf('--- Results ---\n');
fprintf('Case 1 (Solid): Theta2=%.2f, Theta3=%.2f\n', rad2deg(q2_local_1+theta1), rad2deg(q3_local_1+theta1));
fprintf('Case 2 (Dash) : Theta2=%.2f, Theta3=%.2f\n', rad2deg(q2_local_2+theta1), rad2deg(q3_local_2+theta1));
