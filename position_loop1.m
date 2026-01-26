clear all
close all
clc

% --- 1. Define Parameters (Unit: meters) ---
L1 = 0.210; % Ground (d) - Pink
L2 = 0.118; % Crank (a) - Cyan (Link 2)
L3 = 0.210; % Coupler (b) - Red (Link 3)
L4 = 0.118; % Rocker (c) - Grey (Link 4)

a = L2;
b = L3;
c = L4;
d = L1;

% Ground Offset
offset_deg = 0.81;
offset = deg2rad(offset_deg);

% --- 2. Define K Constants (Inverse Analysis: Link 4 is Input) ---
K1 = d/c; 
K2 = d/a;
K3 = (c^2 - b^2 + a^2 + d^2)/(2*c*a); 

% ==========================================
% CASE 1: Open/Parallel (Input Theta 4 = -102.5)
% (Link 2 will be DOWN)
% ==========================================
q4d = -102.5; % Using negative to force Link 4 DOWN
q4 = deg2rad(q4d) - offset; 

% --- Calculate Coefficients for Theta 2 ---
A = cos(q4) - K1 - K2*cos(q4) + K3;
B = -2*sin(q4);
C = K1 - (K2+1)*cos(q4) + K3;

% --- Calculate Coefficients for Theta 3 ---
P = -2*b*(d + c*cos(q4));
Q = -2*b*c*sin(q4);
R = d^2 + c^2 + b^2 - a^2 + 2*d*c*cos(q4);

D_coef = R - P;
E_coef = 2*Q;
F_coef = R + P;

% --- Solve for Theta 2 & 3 (Case 1: Parallel/Down) ---
% Using -sqrt for q2 (Usually gives the parallel/down solution)
q2_sol1 = 2*atan((-B - sqrt(B^2 - 4*A*C))/(2*A));
% Using -sqrt for q3 (Geometric match for parallel)
q3_sol1 = 2*atan((-E_coef - sqrt(E_coef^2 - 4*D_coef*F_coef))/(2*D_coef));

q2_d1 = rad2deg(q2_sol1) + offset_deg;
q3_d1 = rad2deg(q3_sol1) + offset_deg;


% ==========================================
% CASE 2: Crossed (Input Theta 4 = -102.5)
% (Link 2 will be UP)
% ==========================================
% q4 is the same

% --- Solve for Theta 2 & 3 (Case 2: Crossed/Up) ---
% Using +sqrt for q2 (Usually gives the crossed/up solution)
q2_sol2 = 2*atan((-B + sqrt(B^2 - 4*A*C))/(2*A));
% Using +sqrt for q3 (Geometric match for crossed)
q3_sol2 = 2*atan((-E_coef + sqrt(E_coef^2 - 4*D_coef*F_coef))/(2*D_coef));

q2_d2 = rad2deg(q2_sol2) + offset_deg;
q3_d2 = rad2deg(q3_sol2) + offset_deg;


% ==========================================
% Vector Calculation & Plotting
% ==========================================
% Ground Vector
RO4O2 = d*exp(j*offset); 
RO4O2x = real(RO4O2); RO4O2y = imag(RO4O2);

% --- Vector Case 1 (Open/Down) ---
RA1 = a*exp(j*(q2_sol1 + offset));        
RBA1 = b*exp(j*(q3_sol1 + offset));        
RBO4_1 = c*exp(j*(q4 + offset));    

RA1x = real(RA1); RA1y = imag(RA1);
RBA1x = real(RBA1); RBA1y = imag(RBA1); 
RBO4_1x = real(RBO4_1); RBO4_1y = imag(RBO4_1);

% --- Vector Case 2 (Crossed/Up) ---
RA2 = a*exp(j*(q2_sol2 + offset));        
RBA2 = b*exp(j*(q3_sol2 + offset));
RBO4_2 = c*exp(j*(q4 + offset));    

RA2x = real(RA2); RA2y = imag(RA2);
RBA2x = real(RBA2); RBA2y = imag(RBA2); 
RBO4_2x = real(RBO4_2); RBO4_2y = imag(RBO4_2);

% --- Plotting Case 1 ---
figure(1)
title(['Case 1: Open/Parallel (q4 = ' num2str(q4d) ')']);
hold on;
% Ground (Pink)
quiver(0,0, RO4O2x, RO4O2y, 0, 'Color', [1 0 1], 'MaxHeadSize', 0.5, 'LineWidth', 4); 
% Crank (Cyan) - Link 2
quiver(0,0, RA1x, RA1y, 0, 'cyan', 'MaxHeadSize', 0.5, 'LineWidth', 3); 
% Coupler (Red) - Link 3
quiver(RA1x, RA1y, RBA1x, RBA1y, 0, 'red', 'MaxHeadSize', 0.5, 'LineWidth', 3); 
% Rocker (Grey) - Link 4
quiver(RO4O2x, RO4O2y, RBO4_1x, RBO4_1y, 0, 'Color', [0.5 0.5 0.5], 'MaxHeadSize', 0.5, 'LineWidth', 3); 
axis equal; grid on;
xlabel('x (m)'); ylabel('y (m)');

% --- Plotting Case 2 ---
figure(2)
title(['Case 2: Crossed/Up (q4 = ' num2str(q4d) ')']);
hold on;
% Ground (Pink)
quiver(0,0, RO4O2x, RO4O2y, 0, 'Color', [1 0 1], 'MaxHeadSize', 0.5, 'LineWidth', 4); 
% Crank (Cyan) - Link 2
quiver(0,0, RA2x, RA2y, 0, 'cyan', 'MaxHeadSize', 0.5, 'LineWidth', 3); 
% Coupler (Red) - Link 3
quiver(RA2x, RA2y, RBA2x, RBA2y, 0, 'red', 'MaxHeadSize', 0.5, 'LineWidth', 3); 
% Rocker (Grey) - Link 4
quiver(RO4O2x, RO4O2y, RBO4_2x, RBO4_2y, 0, 'Color', [0.5 0.5 0.5], 'MaxHeadSize', 0.5, 'LineWidth', 3); 
axis equal; grid on;
xlabel('x (m)'); ylabel('y (m)');

% Display Results
disp('--- Results ---');
disp(['Case 1 (Open/Down): Theta 2 = ', num2str(q2_d1), ', Theta 3 = ', num2str(q3_d1)]);
disp(['Case 2 (Crossed/Up): Theta 2 = ', num2str(q2_d2), ', Theta 3 = ', num2str(q3_d2)]);
