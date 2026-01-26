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
% Logic เดิมของคุณที่ถูกต้อง แต่นำมาใส่ตัวแปร A, B, C
A = -((1 + K1)*cos(q4) + K2 + K3);
B = 2*sin(q4);
C = (1 - K1)*cos(q4) + K2 - K3;

% Check determinant
det_val = B^2 - 4*A*C;
if det_val < 0
    error('No solution');
end

% Solve for q2 (2 Cases: Open & Crossed) using 2*atan
q2_sol1 = 2*atan((-B + sqrt(det_val))/(2*A)); 
q2_sol2 = 2*atan((-B - sqrt(det_val))/(2*A));

% Calculate q3 using Vector Sum Logic (To ensure loop closure)
% Case 1 (Matches q2_sol1)
Vec3_1 = c*exp(1j*q4) + d - a*exp(1j*q2_sol1);
q3_sol1 = angle(Vec3_1);

% Case 2 (Matches q2_sol2)
Vec3_2 = c*exp(1j*q4) + d - a*exp(1j*q2_sol2);
q3_sol2 = angle(Vec3_2);

% ==========================================
% 3. CONVERT TO GLOBAL & DISPLAY
% ==========================================

% Convert back to Global angles
q2_1_deg = rad2deg(q2_sol1 + theta1);
q3_1_deg = rad2deg(q3_sol1 + theta1);

q2_2_deg = rad2deg(q2_sol2 + theta1);
q3_2_deg = rad2deg(q3_sol2 + theta1);

fprintf('--- Results ---\n');
fprintf('Configuration 1:\n Theta2: %.4f deg, Theta3: %.4f deg\n', q2_1_deg, q3_1_deg);
fprintf('Configuration 2:\n Theta2: %.4f deg, Theta3: %.4f deg\n', q2_2_deg, q3_2_deg);

% ==========================================
% 4. PLOTTING
% ==========================================
% Select Configuration to plot (e.g., Sol 2)
q2_plot = q2_sol2; 
q3_plot = q3_sol2;

% Global Angles for Plotting
q2g = q2_plot + theta1;
q3g = q3_plot + theta1;
q4g = q4 + theta1;

% Calculate Vectors in Global Frame
RO2 = 0; % Origin at O2
RO4 = d*exp(1j*theta1); % Ground Vector

RA  = a*exp(1j*q2g);      % Link 2 Vector
RBA = b*exp(1j*q3g);      % Link 3 Vector
RBO4 = c*exp(1j*q4g);     % Link 4 Vector (from O4)

% Points
Pos_O2 = RO2;
Pos_O4 = RO4;
Pos_A  = RO2 + RA;
Pos_B  = Pos_A + RBA; % or RO4 + RBO4

% Components for Quiver
O2x = real(Pos_O2); O2y = imag(Pos_O2);
O4x = real(Pos_O4); O4y = imag(Pos_O4);
Ax  = real(Pos_A);  Ay  = imag(Pos_A);
Bx  = real(Pos_B);  By  = imag(Pos_B);

% Create Plot
figure;
hold on; axis equal; grid on;

% 1. Ground (Pink/Black) - O2 -> O4
quiver(O2x, O2y, O4x-O2x, O4y-O2y, 0, 'k', 'LineWidth', 2, 'MaxHeadSize', 0.5);

% 2. Link 2 Crank (Cyan) - O2 -> A
quiver(O2x, O2y, Ax-O2x, Ay-O2y, 0, 'c', 'LineWidth', 3, 'MaxHeadSize', 0.5);

% 3. Link 3 Coupler (Blue) - A -> B
quiver(Ax, Ay, Bx-Ax, By-Ay, 0, 'b', 'LineWidth', 3, 'MaxHeadSize', 0.5);

% 4. Link 4 Rocker (Brown) - O4 -> B
% Brown color RGB: [0.6, 0.3, 0]
quiver(O4x, O4y, Bx-O4x, By-O4y, 0, 'Color', [0.6, 0.3, 0], 'LineWidth', 3, 'MaxHeadSize', 0.5);

title('4-Bar Linkage Position Analysis');
xlabel('X Position (m)'); ylabel('Y Position (m)');
legend('Ground', 'Link 2 (Crank)', 'Link 3 (Coupler)', 'Link 4 (Rocker)');
