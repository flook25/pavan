clear all
close all
clc

% 1. Parameter Definitions
L1 = 210; % Ground (Pink)
L2 = 118; % Link 2 (Light Blue)
L3 = 210; % Link 3 (Blue)
L4 = 118; % Link 4 (Brown)

a = L2;
b = L3;
c = L4;
d = L1;

% Ground angle
theta1_deg = 0.81;
theta1 = deg2rad(theta1_deg);

% Input: Theta 4
q4_global_deg = 102.5;
q4_global = deg2rad(q4_global_deg);

% Local Theta 4
q4 = q4_global - theta1;

% ---------------------------------------------------------
% STEP 1: Find Theta 2 given Theta 4
% ---------------------------------------------------------
% Inverse K constants
K1_inv = d/c; 
K2_inv = d/a; 
K3_inv = (c^2 - b^2 + a^2 + d^2)/(2*c*a); 

% Coefficients A, B, C for q2
A = cos(q4) - K1_inv - K2_inv*cos(q4) + K3_inv;
B = -2*sin(q4);
C = K1_inv - (K2_inv + 1)*cos(q4) + K3_inv;

disc = B^2 - 4*A*C;

% --- Calculate Both Solutions for q2 ---
% Solution A: (+sqrt) -> Usually Down/Crossed
q2_sol_A = 2*atan((-B + sqrt(disc))/(2*A));

% Solution B: (-sqrt) -> Usually Up/Parallel
q2_sol_B = 2*atan((-B - sqrt(disc))/(2*A));


% ---------------------------------------------------------
% STEP 2: Find Theta 3 for EACH Case
% ---------------------------------------------------------
% Forward K constants
K1 = d/a;
K4 = d/b;
K5 = (c^2 - d^2 - a^2 - b^2)/(2*a*b);

% --- Case 1 Calculation (Using q2_sol_A -> Down) ---
D1 = cos(q2_sol_A) - K1 + K4*cos(q2_sol_A) + K5;
E1 = -2*sin(q2_sol_A);
F1 = K1 + (K4 - 1)*cos(q2_sol_A) + K5;

% Choose sign for q3 that closes the loop correctly (usually + for this mode)
q3_sol_A = 2*atan((-E1 + sqrt(E1^2 - 4*D1*F1))/(2*D1)); 


% --- Case 2 Calculation (Using q2_sol_B -> Up) ---
D2 = cos(q2_sol_B) - K1 + K4*cos(q2_sol_B) + K5;
E2 = -2*sin(q2_sol_B);
F2 = K1 + (K4 - 1)*cos(q2_sol_B) + K5;

% Choose sign for q3 (usually - for this mode)
q3_sol_B = 2*atan((-E2 - sqrt(E2^2 - 4*D2*F2))/(2*D2));


% ---------------------------------------------------------
% ASSIGN CASES & CALCULATE GLOBAL ANGLES
% ---------------------------------------------------------

% Case 1: Down Configuration
q2_target_1 = q2_sol_A;
q3_local_1  = q3_sol_A;

% Case 2: Up Configuration
q2_target_2 = q2_sol_B;
q3_local_2  = q3_sol_B;

% Convert to Global
q2_global_1 = rad2deg(q2_target_1 + theta1);
q3_global_1 = rad2deg(q3_local_1 + theta1);

q2_global_2 = rad2deg(q2_target_2 + theta1);
q3_global_2 = rad2deg(q3_local_2 + theta1);


% ---------------------------------------------------------
% VECTOR CALCULATION & PLOTTING
% ---------------------------------------------------------
% Colors
color_Pink = [1, 0.07, 0.57];
color_LBlue = [0.2, 0.8, 1];
color_Blue = [0, 0, 1];
color_Brown = [0.6, 0.4, 0.2];

% Ground
RO4O2 = d*exp(j*theta1); 
O4x = real(RO4O2); O4y = imag(RO4O2);

% --- Case 1 Vectors ---
RA1 = a*exp(j*(q2_target_1 + theta1));
RBA1 = b*exp(j*(q3_local_1 + theta1));
Ax1 = real(RA1); Ay1 = imag(RA1);
Bx1 = real(RA1 + RBA1); By1 = imag(RA1 + RBA1);

% --- Case 2 Vectors ---
RA2 = a*exp(j*(q2_target_2 + theta1));
RBA2 = b*exp(j*(q3_local_2 + theta1));
Ax2 = real(RA2); Ay2 = imag(RA2);
Bx2 = real(RA2 + RBA2); By2 = imag(RA2 + RBA2);


% --- FIGURE 1: Case 1 (Down/Crossed) ---
figure(1);
hold on;
quiver(0, 0, O4x, O4y, 0, 'Color', color_Pink, 'LineWidth', 4, 'MaxHeadSize', 0.5); % Ground
quiver(0, 0, Ax1, Ay1, 0, 'Color', color_LBlue, 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 2
quiver(Ax1, Ay1, Bx1-Ax1, By1-Ay1, 0, 'Color', color_Blue, 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 3
quiver(O4x, O4y, Bx1-O4x, By1-O4y, 0, 'Color', color_Brown, 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 4
axis equal; grid on;
title(['Case 1: Down Configuration, \theta_4 = ' num2str(q4_global_deg)]);
xlabel('x'); ylabel('y');


% --- FIGURE 2: Case 2 (Up/Parallel) ---
figure(2);
hold on;
quiver(0, 0, O4x, O4y, 0, 'Color', color_Pink, 'LineWidth', 4, 'MaxHeadSize', 0.5); % Ground
quiver(0, 0, Ax2, Ay2, 0, 'Color', color_LBlue, 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 2
quiver(Ax2, Ay2, Bx2-Ax2, By2-Ay2, 0, 'Color', color_Blue, 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 3
quiver(O4x, O4y, Bx2-O4x, By2-O4y, 0, 'Color', color_Brown, 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 4
axis equal; grid on;
title(['Case 2: Up Configuration, \theta_4 = ' num2str(q4_global_deg)]);
xlabel('x'); ylabel('y');


% Display Results
fprintf('--- Results ---\n');
fprintf('Case 1 (Down): Theta 2 = %.4f deg, Theta 3 = %.4f deg\n', q2_global_1, q3_global_1);
fprintf('Case 2 (Up)  : Theta 2 = %.4f deg, Theta 3 = %.4f deg\n', q2_global_2, q3_global_2);
