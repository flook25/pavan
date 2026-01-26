clear all
close all
clc

% --- 1. Parameter Definitions ---
L1 = 210; % Length of Link1 d (Ground) - Pink
L2 = 118; % Length of Link2 a (Input/Crank) - Light Blue
L3 = 210; % Length of Link3 b (Coupler) - Blue
L4 = 118; % Length of Link4 c (Output/Rocker) - Brown

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

% --- 2. Calculate Constants (For Inverse Kinematics: Find q2 from q4) ---
% Swapping 'a' and 'c' logic strictly for K calculation
K1 = d/c; 
K2 = d/a; 
K3 = (c^2 - b^2 + a^2 + d^2)/(2*c*a); 

% Coefficients A, B, C
A = cos(q4) - K1 - K2*cos(q4) + K3;
B = -2*sin(q4);
C = K1 - (K2 + 1)*cos(q4) + K3;

% --- 3. Calculate Angles (Theta 2) ---
% Solution 1 and 2 for Theta 2
q2_1 = 2*atan((-B - sqrt(B^2 - 4*A*C))/(2*A));
q2_2 = 2*atan((-B + sqrt(B^2 - 4*A*C))/(2*A));

% --- 4. Calculate Theta 3 (Dependent on Theta 2) ---
% Need Forward K constants to find q3
K1_fwd = d/a;
K4_fwd = d/b;
K5_fwd = (c^2 - d^2 - a^2 - b^2)/(2*a*b);

% For Solution 1
D1 = cos(q2_1) - K1_fwd + K4_fwd*cos(q2_1) + K5_fwd;
E1 = -2*sin(q2_1);
F1 = K1_fwd + (K4_fwd - 1)*cos(q2_1) + K5_fwd;
q3_1 = 2*atan((-E1 - sqrt(E1^2 - 4*D1*F1))/(2*D1)); % Open logic

% For Solution 2
D2 = cos(q2_2) - K1_fwd + K4_fwd*cos(q2_2) + K5_fwd;
E2 = -2*sin(q2_2);
F2 = K1_fwd + (K4_fwd - 1)*cos(q2_2) + K5_fwd;
q3_2 = 2*atan((-E2 + sqrt(E2^2 - 4*D2*F2))/(2*D2)); % Crossed logic

% --- 5. Vector Calculations (Complex Numbers) ---
% Ground Vector (Lifted)
RO4O2 = d*exp(j*theta1);

% Link 4 Vector (Fixed Input for both cases)
RBO4 = c*exp(j*(q4 + theta1)); 

% Solution 1 Vectors (Open)
RA1 = a*exp(j*(q2_1 + theta1));
RBA1 = b*exp(j*(q3_1 + theta1));
RB1 = RA1 + RBA1; % Calculate B from A side for closure check

% Solution 2 Vectors (Crossed)
RA2 = a*exp(j*(q2_2 + theta1));
RBA2 = b*exp(j*(q3_2 + theta1));
RB2 = RA2 + RBA2; 

% --- 6. Extract Real and Imaginary Components ---
% Ground
RO4O2x = real(RO4O2);
RO4O2y = imag(RO4O2);

% Link 4
RBO4x = real(RBO4);
RBO4y = imag(RBO4);

% Solution 1 Components
RA1x = real(RA1);
RA1y = imag(RA1);
RBA1x = real(RBA1);
RBA1y = imag(RBA1);
RB1x = real(RB1);
RB1y = imag(RB1);

% Solution 2 Components
RA2x = real(RA2);
RA2y = imag(RA2);
RBA2x = real(RBA2);
RBA2y = imag(RBA2);
RB2x = real(RB2);
RB2y = imag(RB2);

% --- 7. Plotting using Quiver (Strict Format) ---
figure(1);
hold on;

% Plot Ground (Pink) - L1
quiver(0, 0, RO4O2x, RO4O2y, 0, 'Color', [1, 0.07, 0.57], 'LineWidth', 4, 'MaxHeadSize', 0.5);

% --- Plot Case 1: Open Circuit (Solid Lines) ---
% Link 2 (Light Blue)
quiver(0, 0, RA1x, RA1y, 0, 'Color', [0.2, 0.8, 1], 'LineWidth', 2, 'MaxHeadSize', 0.5);
% Link 3 (Blue) - From A to B
quiver(RA1x, RA1y, RBA1x, RBA1y, 0, 'Color', 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);
% Link 4 (Brown) - From O4 to B
quiver(RO4O2x, RO4O2y, RBO4x, RBO4y, 0, 'Color', [0.6, 0.4, 0.2], 'LineWidth', 2, 'MaxHeadSize', 0.5);

% --- Plot Case 2: Crossed Circuit (Dashed Lines for distinction) ---
% Link 2 (Light Blue)
quiver(0, 0, RA2x, RA2y, 0, 'Color', [0.2, 0.8, 1], 'LineWidth', 2, 'LineStyle', '--', 'MaxHeadSize', 0.5);
% Link 3 (Blue)
quiver(RA2x, RA2y, RBA2x, RBA2y, 0, 'Color', 'b', 'LineWidth', 2, 'LineStyle', '--', 'MaxHeadSize', 0.5);
% Link 4 (Brown) - Same position but logically part of loop 2
quiver(RO4O2x, RO4O2y, RBO4x, RBO4y, 0, 'Color', [0.6, 0.4, 0.2], 'LineWidth', 2, 'LineStyle', '--', 'MaxHeadSize', 0.5);

axis equal;
grid on;
title(['Fourbar Mechanism: \theta_4 = ', num2str(q4_global_deg)]);
xlabel('x'); ylabel('y');

% Display Values
fprintf('Case 1 (Solid): Theta2 = %.2f, Theta3 = %.2f\n', rad2deg(q2_1+theta1), rad2deg(q3_1+theta1));
fprintf('Case 2 (Dashed): Theta2 = %.2f, Theta3 = %.2f\n', rad2deg(q2_2+theta1), rad2deg(q3_2+theta1));
