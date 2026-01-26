clear all
close all
clc

% --- 1. Define Parameters (Unit: meters) ---
L1 = 0.210; % Ground (d) - Pink
L2 = 0.118; % Crank (a) - Cyan (Output we want to find)
L3 = 0.210; % Coupler (b) - Blue
L4 = 0.118; % Rocker (c) - Brown (Input we know)

a = L2;
b = L3;
c = L4;
d = L1;

% Ground Offset
offset_deg = 0.81;
offset = deg2rad(offset_deg);

% --- 2. Define K Constants (Inverse Analysis: Link 4 is Input) ---
% Swapping 'a' and 'c' because we assume Link 4 is the driver
K1_inv = d/c; 
K2_inv = d/a; 
K3_inv = (c^2 - b^2 + a^2 + d^2)/(2*c*a); 

% Forward K Constants (For finding Theta 3 later)
K1_fwd = d/a;
K4_fwd = d/b;
K5_fwd = (c^2 - d^2 - a^2 - b^2)/(2*a*b);

% =========================================================
% CASE 1 & 2 SETUP (Input Theta 4 = 102.5)
% =========================================================
q4d = 102.5; 
q4 = deg2rad(q4d) - offset; % Local Theta 4

% --- Calculate Coefficients for Theta 2 ---
% We use q4 to find q2
A = cos(q4) - K1_inv - K2_inv*cos(q4) + K3_inv;
B = -2*sin(q4);
C = K1_inv - (K2_inv + 1)*cos(q4) + K3_inv;

% --- Solve for Theta 2 (Two Solutions) ---
disc = B^2 - 4*A*C;

% q2 Solution 1 (Parallel/Open - usually matches q4 direction)
q2_sol1 = 2*atan((-B - sqrt(disc))/(2*A));

% q2 Solution 2 (Crossed - usually opposite direction)
q2_sol2 = 2*atan((-B + sqrt(disc))/(2*A));


% =========================================================
% FIND THETA 3 (Using Theta 2)
% =========================================================

% --- For Case 1 (Using q2_sol1) ---
D1 = cos(q2_sol1) - K1_fwd + K4_fwd*cos(q2_sol1) + K5_fwd;
E1 = -2*sin(q2_sol1);
F1 = K1_fwd + (K4_fwd - 1)*cos(q2_sol1) + K5_fwd;
% Check closure: Parallel often uses -sqrt for q3
q3_sol1 = 2*atan((-E1 - sqrt(E1^2 - 4*D1*F1))/(2*D1));

% --- For Case 2 (Using q2_sol2) ---
D2 = cos(q2_sol2) - K1_fwd + K4_fwd*cos(q2_sol2) + K5_fwd;
E2 = -2*sin(q2_sol2);
F2 = K1_fwd + (K4_fwd - 1)*cos(q2_sol2) + K5_fwd;
% Check closure: Crossed often uses +sqrt (or -sqrt depending on geometry)
% For this config, let's calculate the valid q3
q3_sol2 = 2*atan((-E2 + sqrt(E2^2 - 4*D2*F2))/(2*D2));


% Convert to Degrees (Global)
q2_d1 = rad2deg(q2_sol1) + offset_deg;
q3_d1 = rad2deg(q3_sol1) + offset_deg;

q2_d2 = rad2deg(q2_sol2) + offset_deg;
q3_d2 = rad2deg(q3_sol2) + offset_deg;


% =========================================================
% Vector Calculation
% =========================================================
RO4O2 = d*exp(j*offset); 
RO4O2x = real(RO4O2); RO4O2y = imag(RO4O2);

% --- Vector Case 1 (Parallel) ---
RA1 = a*exp(j*(q2_sol1 + offset));        
RBA1 = b*exp(j*(q3_sol1 + offset));        
RBO4_1 = c*exp(j*(q4 + offset));    

RA1x = real(RA1); RA1y = imag(RA1);
RBA1x = real(RBA1); RBA1y = imag(RBA1); 
RBO4_1x = real(RBO4_1); RBO4_1y = imag(RBO4_1);

% --- Vector Case 2 (Crossed) ---
RA2 = a*exp(j*(q2_sol2 + offset));        
RBA2 = b*exp(j*(q3_sol2 + offset));
RBO4_2 = c*exp(j*(q4 + offset));    

RA2x = real(RA2); RA2y = imag(RA2);
RBA2x = real(RBA2); RBA2y = imag(RBA2); 
RBO4_2x = real(RBO4_2); RBO4_2y = imag(RBO4_2);


% =========================================================
% Plotting
% =========================================================

% --- Plotting Case 1 ---
figure(1)
title(['Case 1: Open/Parallel (Input q4 = ' num2str(q4d) ')']);
hold on;
% Ground (Pink)
quiver(0,0, RO4O2x, RO4O2y, 0, 'Color', [1 0 1], 'MaxHeadSize', 0.5, 'LineWidth', 4); 
% Crank (Cyan) - Link 2
quiver(0,0, RA1x, RA1y, 0, 'cyan', 'MaxHeadSize', 0.5, 'LineWidth', 3); 
% Coupler (Blue) - Link 3
quiver(RA1x, RA1y, RBA1x, RBA1y, 0, 'blue', 'MaxHeadSize', 0.5, 'LineWidth', 3); 
% Rocker (Brown) - Link 4 (Input)
quiver(RO4O2x, RO4O2y, RBO4_1x, RBO4_1y, 0, 'Color', [0.85 0.5 0.1], 'MaxHeadSize', 0.5, 'LineWidth', 3); 
axis equal; grid on;
xlabel('x (m)'); ylabel('y (m)');

% --- Plotting Case 2 ---
figure(2)
title(['Case 2: Crossed (Input q4 = ' num2str(q4d) ')']);
hold on;
% Ground (Pink)
quiver(0,0, RO4O2x, RO4O2y, 0, 'Color', [1 0 1], 'MaxHeadSize', 0.5, 'LineWidth', 4); 
% Crank (Cyan) - Link 2
quiver(0,0, RA2x, RA2y, 0, 'cyan', 'MaxHeadSize', 0.5, 'LineWidth', 3); 
% Coupler (Blue) - Link 3
quiver(RA2x, RA2y, RBA2x, RBA2y, 0, 'blue', 'MaxHeadSize', 0.5, 'LineWidth', 3); 
% Rocker (Brown) - Link 4 (Input)
quiver(RO4O2x, RO4O2y, RBO4_2x, RBO4_2y, 0, 'Color', [0.85
