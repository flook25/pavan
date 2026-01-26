clear all
close all
clc

% ==========================================
% 1. PARAMETERS & INPUT
% ==========================================
L1 = 0.210; % Ground (Pink) - d
L2 = 0.118; % Crank (Cyan/Red) - a
L3 = 0.210; % Coupler (Blue) - b
L4 = 0.118; % Rocker (Brown) - c

% Assign canonical names
d = L1;
a = L2;
b = L3;
c = L4;

% Ground Offset
theta1_deg = 0.81;
theta1 = deg2rad(theta1_deg);

% --- INPUT: Link 4 (Brown) ---
q4_global_deg = -102.5; 
q4 = deg2rad(q4_global_deg) - theta1; % Local Angle

% ==========================================
% 2. CALCULATION (Find q2, q3 from q4)
% ==========================================

% Norton's K Constants
K1 = d/a;
K2 = d/c;
K3 = (a^2 - b^2 + c^2 + d^2)/(2*a*c);

% Coefficients A, B, C for finding q2 (Derived from Input q4)
A = -((1 + K1)*cos(q4) + K2 + K3);
B = 2*sin(q4);
C = (1 - K1)*cos(q4) + K2 - K3;

% Solve for q2 (2 Cases)
det_val = B^2 - 4*A*C;

% Case 1
q2_sol1 = 2*atan((-B + sqrt(det_val))/(2*A)); 
% Case 2
q2_sol2 = 2*atan((-B - sqrt(det_val))/(2*A));

% Calculate q3 using Vector Sum Logic
% Case 1
Vec3_1 = c*exp(1j*q4) + d - a*exp(1j*q2_sol1);
q3_sol1 = angle(Vec3_1);

% Case 2
Vec3_2 = c*exp(1j*q4) + d - a*exp(1j*q2_sol2);
q3_sol2 = angle(Vec3_2);

% ==========================================
% 3. CONVERT TO GLOBAL
% ==========================================
% Global Angles
q2_1g = q2_sol1 + theta1;
q3_1g = q3_sol1 + theta1;

q2_2g = q2_sol2 + theta1;
q3_2g = q3_sol2 + theta1;

q4g = q4 + theta1;

fprintf('--- Results ---\n');
fprintf('Case 1: Theta2 = %.4f deg, Theta3 = %.4f deg\n', rad2deg(q2_1g), rad2deg(q3_1g));
fprintf('Case 2: Theta2 = %.4f deg, Theta3 = %.4f deg\n', rad2deg(q2_2g), rad2deg(q3_2g));

% ==========================================
% 4. PLOTTING BOTH CASES
% ==========================================
figure('Name', '4-Bar Linkage Analysis: Both Solutions', 'NumberTitle', 'off');

% --- PLOT CASE 1 ---
subplot(1, 2, 1); 
hold on; axis equal; grid on;

% Vectors Case 1
RA_1 = a*exp(1j*q2_1g);
RBA_1 = b*exp(1j*q3_1g);
RB_1 = RA_1 + RBA_1; % Position B from Origin
RO4O2 = d*exp(1j*theta1);
RBO4 = c*exp(1j*q4g);

% Components
O2x=0; O2y=0;
O4x=real(RO4O2); O4y=imag(RO4O2);
Ax_1=real(RA_1); Ay_1=imag(RA_1);
Bx_1=real(RB_1); By_1=imag(RB_1);

% Quivers Case 1
quiver(O2x, O2y, O4x-O2x, O4y-O2y, 0, 'Color', [1 0.4 0.7], 'LineWidth', 2, 'MaxHeadSize', 0.5); % Ground
quiver(O2x, O2y, Ax_1-O2x, Ay_1-O2y, 0, 'Color', [0.4 0.7 1], 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 2
quiver(Ax_1, Ay_1, Bx_1-Ax_1, By_1-Ay_1, 0, 'b', 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 3
quiver(O4x, O4y, Bx_1-O4x, By_1-O4y, 0, 'Color', [0.6, 0.3, 0], 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 4

title(['Case 1: \theta_2 = ' num2str(rad2deg(q2_1g), '%.1f') '^\circ']);
xlabel('X'); ylabel('Y');

% --- PLOT CASE 2 ---
subplot(1, 2, 2); 
hold on; axis equal; grid on;

% Vectors Case 2
RA_2 = a*exp(1j*q2_2g);
RBA_2 = b*exp(1j*q3_2g);
RB_2 = RA_2 + RBA_2; % Position B from Origin

% Components
Ax_2=real(RA_2); Ay_2=imag(RA_2);
Bx_2=real(RB_2); By_2=imag(RB_2);

% Quivers Case 2
quiver(O2x, O2y, O4x-O2x, O4y-O2y, 0,  'Color', [1 0.4 0.7], 'LineWidth', 2, 'MaxHeadSize', 0.5); % Ground
quiver(O2x, O2y, Ax_2-O2x, Ay_2-O2y, 0, 'Color', [0.4 0.7 1], 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 2
 % Link 2
quiver(Ax_2, Ay_2, Bx_2-Ax_2, By_2-Ay_2, 0, 'b', 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 3
quiver(O4x, O4y, Bx_2-O4x, By_2-O4y, 0, 'Color', [0.6, 0.3, 0], 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 4

title(['Case 2: \theta_2 = ' num2str(rad2deg(q2_2g), '%.1f') '^\circ']);
xlabel('X'); ylabel('Y');

