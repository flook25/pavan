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

%  K Constants (Swapping a and c)
K1 = d/c; 
K2 = d/a; 
K3 = (c^2 - b^2 + a^2 + d^2)/(2*c*a); 

% Coefficients A, B, C for finding q2
A = cos(q4) - K1 - K2*cos(q4) + K3;
B = -2*sin(q4);
C = K1 - (K2 + 1)*cos(q4) + K3;

% Solve for q2
disc = B^2 - 4*A*C;

% Sol 1 (Parallel/Open)
q2_local_1 = 2*atan((-B_inv - sqrt(disc))/(2*A_inv));
% Sol 2 (Crossed/Anti-Parallel) 
q2_local_2 = 2*atan((-B_inv + sqrt(disc))/(2*A_inv));


q2_target = q2_local_1; 

% ---------------------------------------------------------
% STEP 2: Find Theta 3 given Theta 2
% ---------------------------------------------------------
K1 = d/a;
K4 = d/b;
K5 = (c^2 - d^2 - a^2 - b^2)/(2*a*b);

D = cos(q2_target) - K1 + K4*cos(q2_target) + K5;
E = -2*sin(q2_target);
F = K1 + (K4 - 1)*cos(q2_target) + K5;

% Solve for q3
disc_q3 = E^2 - 4*D*F;

q3_local = 2*atan((-E + sqrt(disc_q3))/(2*D)); 

% Calculate Global
q2_global = rad2deg(q2_target + theta1);
q3_global = rad2deg(q3_local + theta1);

% ---------------------------------------------------------
% PLOTTING
% ---------------------------------------------------------
% Define Colors
color_Pink = [1, 0.07, 0.57];  
color_LBlue = [0.2, 0.8, 1];   
color_Blue = [0, 0, 1];        
color_Brown = [0.6, 0.4, 0.2]; 

% Vectors
RO4O2 = d*exp(j*theta1); 
RA = a*exp(j*(q2_target + theta1));
RBA = b*exp(j*(q3_local + theta1));
RBO4 = c*exp(j*(q4 + theta1)); % Fixed input

% Points for plotting
O2 = [0, 0];
O4 = [real(RO4O2), imag(RO4O2)];
A  = [real(RA), imag(RA)];
B_from_A = [real(RA + RBA), imag(RA + RBA)];

figure(1);
hold on;

% 1. Ground (Pink) - Link 1
quiver(O2(1), O2(2), O4(1)-O2(1), O4(2)-O2(2), 0, 'Color', color_Pink, 'LineWidth', 4, 'MaxHeadSize', 0.5);

% 2. Input Crank (Light Blue) - Link 2 (บังคับอยู่บน)
quiver(O2(1), O2(2), A(1)-O2(1), A(2)-O2(2), 0, 'Color', color_LBlue, 'LineWidth', 3, 'MaxHeadSize', 0.5);

% 3. Coupler (Blue) - Link 3 (เปลี่ยนทิศ)
quiver(A(1), A(2), B_from_A(1)-A(1), B_from_A(2)-A(2), 0, 'Color', color_Blue, 'LineWidth', 3, 'MaxHeadSize', 0.5);

% 4. Output Rocker (Brown) - Link 4
quiver(O4(1), O4(2), B_from_A(1)-O4(1), B_from_A(2)-O4(2), 0, 'Color', color_Brown, 'LineWidth', 3, 'MaxHeadSize', 0.5);

axis equal; grid on;
title(['Fourbar: Changed Direction (Crossed/Flip), \theta_4 = ' num2str(q4_global_deg)]);
xlabel('x'); ylabel('y');

% Display Results
fprintf('--- Results (Changed Direction) ---\n');
fprintf('Theta 2 (Global) = %.4f deg\n', q2_global);
fprintf('Theta 3 (Global) = %.4f deg\n', q3_global);
