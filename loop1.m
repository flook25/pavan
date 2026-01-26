clear all
close all
clc

% 1. Parameter Definitions
L1 = 210; % Length of Link1 d (Ground) - Pink
L2 = 118; % Length of Link2 a (Input) - Light Blue
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
% We swap 'a' (Link 2) and 'c' (Link 4) in the K formulas
% effectively treating Link 4 as the driver to find Link 2.
% ---------------------------------------------------------

% Inverse K Constants (Swapping a and c)
K1_inv = d/c; 
K2_inv = d/a; 
K3_inv = (c^2 - b^2 + a^2 + d^2)/(2*c*a); % Swapped a and c

% Freudenstein Coefficients for finding q2 (Input is q4)
A_inv = cos(q4) - K1_inv - K2_inv*cos(q4) + K3_inv;
B_inv = -2*sin(q4);
C_inv = K1_inv - (K2_inv + 1)*cos(q4) + K3_inv;

% Solve for q2 using 2*arctan formula
% q2_sol = 2*atan( (-B +/- sqrt(B^2 - 4AC)) / 2A )
disc = B_inv^2 - 4*A_inv*C_inv;

q2_local_1 = 2*atan((-B_inv + sqrt(disc))/(2*A_inv));
q2_local_2 = 2*atan((-B_inv - sqrt(disc))/(2*A_inv));

% Choose the solution that matches the parallelogram configuration (usually Solution 1 or 2)
% Here we calculate for both, but usually one is the open/parallel mode.
q2_target = q2_local_2; % Selecting the one that matches q2 = q4 logic for parallelogram
% Note: You can check q2_local_1 as well.

% Convert q2 back to Global for reporting
theta2_global = rad2deg(q2_target + theta1); 
% For parallelogram, usually q2_global = q4_global

% ---------------------------------------------------------
% STEP 2: Find Theta 3 given Theta 2
% Now we use the standard Forward K constants
% ---------------------------------------------------------
K1 = d/a;
K4 = d/b;
K5 = (c^2 - d^2 - a^2 - b^2)/(2*a*b);

% Coefficients D, E, F for q3 (depend on q2)
D = cos(q2_target) - K1 + K4*cos(q2_target) + K5;
E = -2*sin(q2_target);
F = K1 + (K4 - 1)*cos(q2_target) + K5;

% Solve for q3
disc_q3 = E^2 - 4*D*F;
q3_local = 2*atan((-E - sqrt(disc_q3))/(2*D)); 
% Note: Sign choice (-sqrt) usually gives the correct closure for this config

theta3_global = rad2deg(q3_local + theta1);

% Display Results
fprintf('Theta 2 (Global) = %.4f degrees\n', theta2_global);
fprintf('Theta 3 (Global) = %.4f degrees\n', theta3_global);

% ---------------------------------------------------------
% PLOTTING
% ---------------------------------------------------------

% Position Vectors (Global)
% Ground Origin O2 at (0,0)
RO2 = 0; 
RO4 = d*exp(j*theta1); % Ground Vector (Lifted)

% Link Vectors
RA = a*exp(j*(q2_target + theta1));      % Link 2
RBA = b*exp(j*(q3_local + theta1));      % Link 3
RBO4 = c*exp(j*(q4 + theta1));           % Link 4

% Points
O2x = 0; O2y = 0;
O4x = real(RO4); O4y = imag(RO4);
Ax = real(RA); Ay = imag(RA);
Bx = real(RA + RBA); By = imag(RA + RBA); % B calculated from A
% B_check = real(RO4 + RBO4); % Check closure

% Colors: Pink(L1), Light Blue(L2), Blue(L3), Brown(L4)
color_L1 = [1, 0.4, 0.7]; % Pink
color_L2 = [0.4, 0.8, 1]; % Light Blue
color_L3 = 'b';           % Blue
color_L4 = [0.6, 0.4, 0.2]; % Brown

figure;
hold on;
% Link 1 (Ground) - Pink
quiver(O2x, O2y, O4x-O2x, O4y-O2y, 0, 'Color', color_L1, 'LineWidth', 3, 'MaxHeadSize', 0.5);
% Link 2 (Input) - Light Blue
quiver(O2x, O2y, Ax-O2x, Ay-O2y, 0, 'Color', color_L2, 'LineWidth', 3, 'MaxHeadSize', 0.5);
% Link 3 (Coupler) - Blue
quiver(Ax, Ay, Bx-Ax, By-Ay, 0, 'Color', color_L3, 'LineWidth', 3, 'MaxHeadSize', 0.5);
% Link 4 (Output) - Brown
quiver(O4x, O4y, Bx-O4x, By-O4y, 0, 'Color', color_L4, 'LineWidth', 3, 'MaxHeadSize', 0.5);

axis equal;
grid on;
title(['Fourbar Mechanism: \theta_4 = ', num2str(q4_global_deg)]);
legend('Link 1 (Pink)', 'Link 2 (Lt Blue)', 'Link 3 (Blue)', 'Link 4 (Brown)');
