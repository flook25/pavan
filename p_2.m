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

% Norton's K Constants (Fixed for the mechanism)
K1 = d/a;
K2 = d/c;
K3 = (a^2 - b^2 + c^2 + d^2)/(2*a*c);

% #########################################################################
% PART A: Analysis for Input Theta 2 = -102.5000 deg
% #########################################################################

% --- INPUT 1 ---
q2_global_A = -102.5000;
q2_A = deg2rad(q2_global_A) - theta1; % Local Angle

% --- CALCULATION INPUT 1 (Find q4 given q2) ---
A_A = cos(q2_A) - K1 - K2*cos(q2_A) + K3;
B_A = -2*sin(q2_A);
C_A = K1 - (K2+1)*cos(q2_A) + K3;

det_val_A = B_A^2 - 4*A_A*C_A;
if det_val_A < 0
    error('No real solution for Input A');
end

% Solve for q4 (2 Cases)
q4_sol1_A = 2*atan((-B_A + sqrt(det_val_A))/(2*A_A)); 
q4_sol2_A = 2*atan((-B_A - sqrt(det_val_A))/(2*A_A));

% Calculate q3 using Vector Sum Logic
% Case 1 (Config 1)
Vec3_1_A = c*exp(1j*q4_sol1_A) + d - a*exp(1j*q2_A);
q3_sol1_A = angle(Vec3_1_A);

% Case 2 (Config 2)
Vec3_2_A = c*exp(1j*q4_sol2_A) + d - a*exp(1j*q2_A);
q3_sol2_A = angle(Vec3_2_A);

% --- CONVERT TO GLOBAL INPUT 1 ---
q3_1g_A = q3_sol1_A + theta1;
q4_1g_A = q4_sol1_A + theta1;

q3_2g_A = q3_sol2_A + theta1;
q4_2g_A = q4_sol2_A + theta1;

q2g_A = q2_A + theta1;

fprintf('--- Results for Input -102.5 deg ---\n');
fprintf('Case 1: Theta3 = %.4f deg, Theta4 = %.4f deg\n', rad2deg(q3_1g_A), rad2deg(q4_1g_A));
fprintf('Case 2: Theta3 = %.4f deg, Theta4 = %.4f deg\n', rad2deg(q3_2g_A), rad2deg(q4_2g_A));

% --- PLOTTING INPUT 1 ---
figure('Name', 'Loop 2 Analysis: Input -102.5 deg');

% Subplot 1: Case 1
subplot(1, 2, 1); hold on; axis equal; grid on;
RA_1A = a*exp(1j*q2g_A);
RBA_1A = b*exp(1j*q3_1g_A);
RB_1A = RA_1A + RBA_1A;
RO4O2 = d*exp(1j*theta1);

O2x=0; O2y=0;
O4x=real(RO4O2); O4y=imag(RO4O2);
Ax_1A=real(RA_1A); Ay_1A=imag(RA_1A);
Bx_1A=real(RB_1A); By_1A=imag(RB_1A);

quiver(O2x, O2y, O4x-O2x, O4y-O2y, 0, 'Color', [1 0.4 0.7], 'LineWidth', 2, 'MaxHeadSize', 0.5); % Ground (Pink)
quiver(O2x, O2y, Ax_1A-O2x, Ay_1A-O2y, 0, 'c', 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 2 (Cyan)
quiver(Ax_1A, Ay_1A, Bx_1A-Ax_1A, By_1A-Ay_1A, 0, 'r', 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 3 (Red)
quiver(O4x, O4y, Bx_1A-O4x, By_1A-O4y, 0, 'Color', [0.5 0.5 0.5], 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 4 (Grey)
title('Input -102.5: Case 1');

% Subplot 2: Case 2
subplot(1, 2, 2); hold on; axis equal; grid on;
RA_2A = a*exp(1j*q2g_A);
RBA_2A = b*exp(1j*q3_2g_A);
RB_2A = RA_2A + RBA_2A;

Ax_2A=real(RA_2A); Ay_2A=imag(RA_2A);
Bx_2A=real(RB_2A); By_2A=imag(RB_2A);

quiver(O2x, O2y, O4x-O2x, O4y-O2y, 0, 'Color', [1 0.4 0.7], 'LineWidth', 2, 'MaxHeadSize', 0.5); % Ground
quiver(O2x, O2y, Ax_2A-O2x, Ay_2A-O2y, 0, 'c', 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 2
quiver(Ax_2A, Ay_2A, Bx_2A-Ax_2A, By_2A-Ay_2A, 0, 'r', 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 3
quiver(O4x, O4y, Bx_2A-O4x, By_2A-O4y, 0, 'Color', [0.5 0.5 0.5], 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 4
title('Input -102.5: Case 2');


% #########################################################################
% PART B: Analysis for Input Theta 2 = 39.8576 deg
% #########################################################################

% --- INPUT 2 ---
q2_global_B = 39.8576;
q2_B = deg2rad(q2_global_B) - theta1; % Local Angle

% --- CALCULATION INPUT 2 ---
A_B = cos(q2_B) - K1 - K2*cos(q2_B) + K3;
B_B = -2*sin(q2_B);
C_B = K1 - (K2+1)*cos(q2_B) + K3;

det_val_B = B_B^2 - 4*A_B*C_B;
if det_val_B < 0
    error('No real solution for Input B');
end

% Solve for q4
q4_sol1_B = 2*atan((-B_B + sqrt(det_val_B))/(2*A_B)); 
q4_sol2_B = 2*atan((-B_B - sqrt(det_val_B))/(2*A_B));

% Calculate q3
Vec3_1_B = c*exp(1j*q4_sol1_B) + d - a*exp(1j*q2_B);
q3_sol1_B = angle(Vec3_1_B);

Vec3_2_B = c*exp(1j*q4_sol2_B) + d - a*exp(1j*q2_B);
q3_sol2_B = angle(Vec3_2_B);

% --- CONVERT TO GLOBAL INPUT 2 ---
q3_1g_B = q3_sol1_B + theta1;
q4_1g_B = q4_sol1_B + theta1;

q3_2g_B = q3_sol2_B + theta1;
q4_2g_B = q4_sol2_B + theta1;

q2g_B = q2_B + theta1;

fprintf('\n--- Results for Input 39.8576 deg ---\n');
fprintf('Case 1: Theta3 = %.4f deg, Theta4 = %.4f deg\n', rad2deg(q3_1g_B), rad2deg(q4_1g_B));
fprintf('Case 2: Theta3 = %.4f deg, Theta4 = %.4f deg\n', rad2deg(q3_2g_B), rad2deg(q4_2g_B));

% --- PLOTTING INPUT 2 ---
figure('Name', 'Loop 2 Analysis: Input 39.8576 deg');

% Subplot 1: Case 1
subplot(1, 2, 1); hold on; axis equal; grid on;
RA_1B = a*exp(1j*q2g_B);
RBA_1B = b*exp(1j*q3_1g_B);
RB_1B = RA_1B + RBA_1B;

Ax_1B=real(RA_1B); Ay_1B=imag(RA_1B);
Bx_1B=real(RB_1B); By_1B=imag(RB_1B);

quiver(O2x, O2y, O4x-O2x, O4y-O2y, 0, 'Color', [1 0.4 0.7], 'LineWidth', 2, 'MaxHeadSize', 0.5); % Ground
quiver(O2x, O2y, Ax_1B-O2x, Ay_1B-O2y, 0, 'c', 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 2
quiver(Ax_1B, Ay_1B, Bx_1B-Ax_1B, By_1B-Ay_1B, 0, 'r', 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 3
quiver(O4x, O4y, Bx_1B-O4x, By_1B-O4y, 0, 'Color', [0.5 0.5 0.5], 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 4
title('Input 39.86: Case 1');

% Subplot 2: Case 2
subplot(1, 2, 2); hold on; axis equal; grid on;
RA_2B = a*exp(1j*q2g_B);
RBA_2B = b*exp(1j*q3_2g_B);
RB_2B = RA_2B + RBA_2B;

Ax_2B=real(RA_2B); Ay_2B=imag(RA_2B);
Bx_2B=real(RB_2B); By_2B=imag(RB_2B);

quiver(O2x, O2y, O4x-O2x, O4y-O2y, 0, 'Color', [1 0.4 0.7], 'LineWidth', 2, 'MaxHeadSize', 0.5); % Ground
quiver(O2x, O2y, Ax_2B-O2x, Ay_2B-O2y, 0, 'c', 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 2
quiver(Ax_2B, Ay_2B, Bx_2B-Ax_2B, By_2B-Ay_2B, 0, 'r', 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 3
quiver(O4x, O4y, Bx_2B-O4x, By_2B-O4y, 0, 'Color', [0.5 0.5 0.5], 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 4
title('Input 39.86: Case 2');

% Legend (Optional)
legend('Ground', 'Link 2 (Crank)', 'Link 3 (Coupler)', 'Link 4 (Rocker)', 'Location', 'southoutside');
