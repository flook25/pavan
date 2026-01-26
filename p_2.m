clear all
close all
clc

% ==========================================
% 1. PARAMETERS & INPUT
% ==========================================
L1 = 0.210; % Ground (Pink) - d
L2 = 0.118; % Crank (Cyan) - a (Input Link from Loop 1)
L3 = 0.210; % Coupler (Red) - b
L4 = 0.118; % Rocker (Grey) - c

% Assign canonical names
d = L1;
a = L2;
b = L3;
c = L4;

% Ground Offset
theta1_deg = 0.81;
theta1 = deg2rad(theta1_deg);

% Norton's K Constants
K1 = d/a;
K2 = d/c;
K3 = (a^2 - b^2 + c^2 + d^2)/(2*a*c);

% --- INPUT (Link 2 from Loop 1) ---
q2_global_deg = 39.8576 -180 ;
q2 = deg2rad(q2_global_deg) - theta1; % Local Angle

% ==========================================
% 2. CALCULATION (Find q4 given q2)
% ==========================================

% Coefficients A, B, C for finding q4 (Derived from Input q2)
A = cos(q2) - K1 - K2*cos(q2) + K3;
B = -2*sin(q2);
C = K1 - (K2+1)*cos(q2) + K3;

det_val = B^2 - 4*A*C;

% Solve for q4 (2 Cases: Open & Crossed)
q4_sol1 = 2*atan((-B + sqrt(det_val))/(2*A)); 
q4_sol2 = 2*atan((-B - sqrt(det_val))/(2*A));

% Calculate q3 using Vector Sum Logic
% Case 1 (Config 1)
Vec3_1 = c*exp(1j*q4_sol1) + d - a*exp(1j*q2);
q3_sol1 = angle(Vec3_1);

% Case 2 (Config 2)
Vec3_2 = c*exp(1j*q4_sol2) + d - a*exp(1j*q2);
q3_sol2 = angle(Vec3_2);

% ==========================================
% 3. CONVERT TO GLOBAL & DISPLAY
% ==========================================
q2g = q2 + theta1;

q3_1g = q3_sol1 + theta1;
q4_1g = q4_sol1 + theta1;

q3_2g = q3_sol2 + theta1;
q4_2g = q4_sol2 + theta1;

fprintf('--- Results for Input Theta2 = %.4f deg ---\n', q2_global_deg);
fprintf('Configuration 1:\n Theta3: %.4f deg, Theta4: %.4f deg\n', rad2deg(q3_1g), rad2deg(q4_1g));
fprintf('Configuration 2:\n Theta3: %.4f deg, Theta4: %.4f deg\n', rad2deg(q3_2g), rad2deg(q4_2g));

% ==========================================
% 4. PLOTTING
% ==========================================
figure('Name', 'Loop 2 Analysis: Single Input', 'NumberTitle', 'off');

% Define Colors
col_ground = [1, 0.4, 0.7]; % Pink
col_L2 = 'c';               % Cyan
col_L3 = 'r';               % Red
col_L4 = [0.5, 0.5, 0.5];   % Grey


% --- PLOT CASE 1 ---
subplot(1, 2, 2); 
hold on; axis equal; grid on;

RBA_2 = b*exp(1j*q3_2g);
RB_2 = RA + RBA_2;

Bx_2=real(RB_2); By_2=imag(RB_2);

quiver(O2x, O2y, O4x-O2x, O4y-O2y, 0, 'Color', col_ground, 'LineWidth', 2, 'MaxHeadSize', 0.5); % Ground
quiver(O2x, O2y, Ax-O2x, Ay-O2y, 0, 'Color', col_L2, 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 2
quiver(Ax, Ay, Bx_2-Ax, By_2-Ay, 0, 'Color', col_L3, 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 3
quiver(O4x, O4y, Bx_2-O4x, By_2-O4y, 0, 'Color', col_L4, 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 4


title(['Config 1']);
xlabel('X'); ylabel('Y');


subplot(1, 2, 1); 
hold on; axis equal; grid on;

RA = a*exp(1j*q2g);
RBA_1 = b*exp(1j*q3_1g);
RB_1 = RA + RBA_1;
RO4O2 = d*exp(1j*theta1);

O2x=0; O2y=0;
O4x=real(RO4O2); O4y=imag(RO4O2);
Ax=real(RA); Ay=imag(RA);
Bx_1=real(RB_1); By_1=imag(RB_1);

quiver(O2x, O2y, O4x-O2x, O4y-O2y, 0, 'Color', col_ground, 'LineWidth', 2, 'MaxHeadSize', 0.5); % Ground
quiver(O2x, O2y, Ax-O2x, Ay-O2y, 0, 'Color', col_L2, 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 2
quiver(Ax, Ay, Bx_1-Ax, By_1-Ay, 0, 'Color', col_L3, 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 3
quiver(O4x, O4y, Bx_1-O4x, By_1-O4y, 0, 'Color', col_L4, 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 4

title(['Case 2']);
xlabel('X'); ylabel('Y');

