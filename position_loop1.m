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

% Assign canonical names for formulas
d = L1;
a = L2;
b = L3;
c = L4;

% Ground Offset (Global rotation)
theta1_deg = 0.81;
theta1 = deg2rad(theta1_deg);

% --- INPUT: Link 4 (Brown) ---
q4_global_deg = -102.5; 
% Convert to Local frame (relative to Link 1)
q4 = deg2rad(q4_global_deg) - theta1; 

% ==========================================
% 2. CALCULATION (Inverted Kinematics: Given q4, find q2, q3)
% ==========================================

% Calculate K constants (Norton's Definitions)
K1 = d/a;
K2 = d/c;
K3 = (a^2 - b^2 + c^2 + d^2)/(2*a*c);

% Coefficients for Quadratic Equation A*t^2 + B*t + C = 0
% Derived from Vector Loop for Input q4
A_quad = -((1 + K1)*cos(q4) + K2 + K3);
B_quad = 2*sin(q4);
C_quad = (1 - K1)*cos(q4) + K2 - K3;

% Solve for t = tan(theta2 / 2)
det_val = B_quad^2 - 4*A_quad*C_quad;

if det_val < 0
    error('No real solution exists for the given input angle.');
end

t_1 = (-B_quad + sqrt(det_val)) / (2*A_quad);
t_2 = (-B_quad - sqrt(det_val)) / (2*A_quad);

% Calculate theta2 (Local frame)
q2_1 = 2*atan(t_1);
q2_2 = 2*atan(t_2);

% Calculate theta3 (Local frame) using Vector Sum
% Vector equation: b*e^(j*q3) = c*e^(j*q4) + d - a*e^(j*q2)
Vec3_1 = c*exp(1j*q4) + d - a*exp(1j*q2_1);
Vec3_2 = c*exp(1j*q4) + d - a*exp(1j*q2_2);

q3_1 = angle(Vec3_1);
q3_2 = angle(Vec3_2);

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
% You can change this to use q2_1/q3_1 for Crossed
q2_plot = q2_2; 
q3_plot = q3_2;
% Note: Using Local angles for vector calculation relative to ground at 0, 
% then rotating the whole plot by theta1 is equivalent to using global angles.
% Below we use Global Angles for direct plotting.

% Vectors in Global Frame
RA = a * exp(1j * (q2_plot + theta1));
RB = c * exp(1j * (q4 + theta1)); % Re-derived from q4 local
% Or directly from input:
% RB = c * exp(1j * deg2rad(q4_global_deg)); 
% (Should be offset by d? No, RB is from O4. Position of B from origin is different)

% Let's use the standard loop: Origin -> O2 -> A -> B -> O4 -> Origin
% Position O2 = 0 (or some offset if needed, usually 0)
O2 = 0;
O4 = d * exp(1j * theta1); % Ground Vector

% Points for Config 2
PA = O2 + a * exp(1j * (q2_2 + theta1)); % Point A
PB = O4 + c * exp(1j * (q4 + theta1));   % Point B from O4
% Check closure:
PB_check = PA + b * exp(1j * (q3_2 + theta1));
% Error check
err = abs(PB - PB_check);

% Plotting
figure;
hold on;
axis equal;
grid on;

% Plot Ground (O2 to O4)
plot([real(O2), real(O4)], [imag(O2), imag(O4)], 'k--o', 'LineWidth', 2, 'DisplayName', 'Ground');

% Plot Link 2 (Crank - Cyan)
plot([real(O2), real(PA)], [imag(O2), imag(PA)], 'c-o', 'LineWidth', 3, 'DisplayName', 'Link 2 (Crank)');

% Plot Link 3 (Coupler - Blue)
plot([real(PA), real(PB)], [imag(PA), imag(PB)], 'b-o', 'LineWidth', 3, 'DisplayName', 'Link 3 (Coupler)');

% Plot Link 4 (Rocker - Brown)
plot([real(O4), real(PB)], [imag(O4), imag(PB)], 'color', [0.6, 0.3, 0], 'LineWidth', 3, 'DisplayName', 'Link 4 (Rocker)');

% Quiver style as requested in prompt
quiver(real(O2), imag(O2), real(PA), imag(PA), 0, 'c', 'LineWidth', 2);
quiver(real(PA), imag(PA), real(PB)-real(PA), imag(PB)-imag(PA), 0, 'b', 'LineWidth', 2);
quiver(real(O4), imag(O4), real(PB)-real(O4), imag(PB)-imag(O4), 0, 'color', [0.6, 0.3, 0], 'LineWidth', 2);

xlabel('X Position (m)');
ylabel('Y Position (m)');
title('4-Bar Linkage Position Analysis (Input \theta_4)');
legend('show');
