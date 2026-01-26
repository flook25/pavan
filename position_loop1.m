clear all
close all
clc

% ==========================================
% 1. PARAMETERS & INPUT
% ==========================================
L1 = 0.210; % Ground (Pink) - d
L2 = 0.118; % Crank (Cyan)  - a
L3 = 0.210; % Coupler (Blue)- b
L4 = 0.118; % Rocker (Brown)- c

a = L2;
b = L3;
c = L4;
d = L1;

% Ground Offset (Global rotation)
theta1_deg = 0.81;
theta1 = deg2rad(theta1_deg);

% --- INPUT: Link 4 (Brown) ---
q4_global_deg = -102.5; 
q4 = deg2rad(q4_global_deg) - theta1; % Convert to Local Angle

% ==========================================
% 2. CALCULATION (Given q4, Find q2 and q3)
% ==========================================

% Calculate K constants (Norton's Definitions)
K1 = d/a;
K2 = d/c;
K3 = (a^2 - b^2 + c^2 + d^2)/(2*a*c);
K4 = d/b;
% K5_inv: Specific constant for finding q3 from q4 input
K5_inv = (a^2 - d^2 - c^2 - b^2)/(2*c*b);

% --- Find Theta 2 (Crank) ---
% Coefficients A, B, C derived for Inverted Kinematics (Input q4)
A = cos(q4) - K2 - K1*cos(q4) + K3;
B = -2*sin(q4);
C = K2 - (K1+1)*cos(q4) + K3;

% Solve for q2 (2 Solutions: Open and Crossed)
q2_1 = 2*atan((-B + sqrt(B^2-4*A*C))/(2*A));
q2_2 = 2*atan((-B - sqrt(B^2-4*A*C))/(2*A));

% --- Find Theta 3 (Coupler) ---
% Coefficients D, E, F derived for q3 from q4
D = cos(q4) - K2 + K4*cos(q4) + K5_inv;
E = -2*sin(q4);
F = K2 + (K4-1)*cos(q4) + K5_inv;

% Solve for q3 (2 Solutions)
q3_1 = 2*atan((-E + sqrt(E^2-4*D*F))/(2*D));
q3_2 = 2*atan((-E - sqrt(E^2-4*D*F))/(2*D));

% ==========================================
% 3. CONVERT TO GLOBAL & DISPLAY
% ==========================================

% Add theta1 offset back to get Global angles
q2_1_global = q2_1 + theta1;
q3_1_global = q3_1 + theta1;

q2_2_global = q2_2 + theta1;
q3_2_global = q3_2 + theta1;

fprintf('--- Results ---\n');
fprintf('Configuration 1 (Crossed):\n');
fprintf('Theta2: %.4f deg\n', rad2deg(q2_1_global));
fprintf('Theta3: %.4f deg\n', rad2deg(q3_1_global));
fprintf('\nConfiguration 2 (Open):\n');
fprintf('Theta2: %.4f deg\n', rad2deg(q2_2_global));
fprintf('Theta3: %.4f deg\n', rad2deg(q3_2_global));

% ==========================================
% 4. PLOTTING
% ==========================================
% Select Configuration to plot (e.g., Config 2 - Open)
q2_plot = q2_2; 
q3_plot = q3_2;

% Calculate Vectors in Global Frame for Plotting
% Ground (Link 1)
RO2 = 0;
RO4 = d * exp(1j * theta1);

% Link 2 (Crank) - Cyan
RA = a * exp(1j * (q2_plot + theta1));

% Link 3 (Coupler) - Blue
RBA = b * exp(1j * (q3_plot + theta1));

% Link 4 (Rocker) - Brown (From Input q4)
RB_from_O4 = c * exp(1j * (q4 + theta1));

% Calculate Points
Pos_O2 = RO2;
Pos_O4 = RO4;
Pos_A = RO2 + RA;
Pos_B = RO4 + RB_from_O4; % Calculate B from O4 side to verify

% Check closure error (Optional verification)
Pos_B_check = Pos_A + RBA; 
% difference = abs(Pos_B - Pos_B_check) % Should be near 0

% Extract Components for Quiver
O2x = real(Pos_O2); O2y = imag(Pos_O2);
O4x = real(Pos_O4); O4y = imag(Pos_O4);
Ax = real(Pos_A);   Ay = imag(Pos_A);
Bx = real(Pos_B);   By = imag(Pos_B);

% Plotting
figure;
hold on;
axis equal;
grid on;

% 1. Ground Link (Pink/Black) - O2 to O4
% Note: Using Black for visibility as per standard, or Pink as requested
quiver(O2x, O2y, O4x-O2x, O4y-O2y, 0, 'Color', 'k', 'LineWidth', 2, 'MaxHeadSize', 0.5); 
% If need strictly Pink: 'Color', [1, 0.4, 0.7]

% 2. Link 2 (Crank) - Cyan - O2 to A
quiver(O2x, O2y, Ax-O2x, Ay-O2y, 0, 'Color', 'c', 'LineWidth', 3, 'MaxHeadSize', 0.5);

% 3. Link 3 (Coupler) - Blue - A to B
quiver(Ax, Ay, Bx-Ax, By-Ay, 0, 'Color', 'b', 'LineWidth', 3, 'MaxHeadSize', 0.5);

% 4. Link 4 (Rocker) - Brown - O4 to B
brown_color = [0.6, 0.3, 0];
quiver(O4x, O4y, Bx-O4x, By-O4y, 0, 'Color', brown_color, 'LineWidth', 3, 'MaxHeadSize', 0.5);

% Add Labels
text(O2x, O2y, ' O_2');
text(O4x, O4y, ' O_4');
text(Ax, Ay, ' A');
text(Bx, By, ' B');

xlabel('X Position (m)');
ylabel('Y Position (m)');
title('4-Bar Linkage Position Analysis (Input \theta_4)');
legend('Ground', 'Link 2 (Crank)', 'Link 3 (Coupler)', 'Link 4 (Rocker)');
