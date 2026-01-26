clear all
close all
clc

% ==========================================
% 1. PARAMETERS & INPUT
% ==========================================
L1 = 210; % Ground (Pink)
L2 = 118; % Crank (Light Blue)
L3 = 210; % Coupler (Blue)
L4 = 118; % Rocker (Brown)

d = L1;
a = L2;
b = L3;
c = L4;

% Ground Offset
theta1_deg = 0.81;
theta1 = deg2rad(theta1_deg);

% Input: Theta 4 (Global) -> Link 4 (Brown)
q4_global_deg = 102.5;
q4_global = deg2rad(q4_global_deg);

% Local Theta 4 (Relative to ground line)
q4 = q4_global - theta1;

% ==========================================
% 2. SOLVE INVERSE KINEMATICS (Find q2 from q4)
% ==========================================
% K Constants (Swapped 'a' and 'c' because q4 is input)
K1_inv = d/c; 
K2_inv = d/a; 
K3_inv = (c^2 - b^2 + a^2 + d^2)/(2*c*a); 

% Coefficients for q2
A = cos(q4) - K1_inv - K2_inv*cos(q4) + K3_inv;
B = -2*sin(q4);
C = K1_inv - (K2_inv + 1)*cos(q4) + K3_inv;

% Calculate Discriminant
disc = B^2 - 4*A*C;

% Find 2 possible angles for Link 2 (q2)
% Solution 1 (Usually Parallel/Open)
q2_sol1 = 2*atan((-B - sqrt(disc))/(2*A));

% Solution 2 (Usually Crossed)
q2_sol2 = 2*atan((-B + sqrt(disc))/(2*A));

% ==========================================
% 3. FIND CORRESPONDING THETA 3 (Find q3 for EACH q2)
% ==========================================
% Standard Constants for finding q3
K1 = d/a;
K4 = d/b;
K5 = (c^2 - d^2 - a^2 - b^2)/(2*a*b);

% --- CASE 1: Using q2_sol1 ---
D1 = cos(q2_sol1) - K1 + K4*cos(q2_sol1) + K5;
E1 = -2*sin(q2_sol1);
F1 = K1 + (K4 - 1)*cos(q2_sol1) + K5;
% For Parallel setup, usually requires -sqrt
q3_sol1 = 2*atan((-E1 - sqrt(E1^2 - 4*D1*F1))/(2*D1));

% --- CASE 2: Using q2_sol2 ---
D2 = cos(q2_sol2) - K1 + K4*cos(q2_sol2) + K5;
E2 = -2*sin(q2_sol2);
F2 = K1 + (K4 - 1)*cos(q2_sol2) + K5;
% For Crossed setup, usually requires +sqrt (or -sqrt depending on geometry)
% Let's try +sqrt to force the other closure
q3_sol2 = 2*atan((-E2 + sqrt(E2^2 - 4*D2*F2))/(2*D2));


% ==========================================
% 4. CONVERT TO GLOBAL & VECTORS
% ==========================================
% Case 1 Global Angles
q2_global_1 = rad2deg(q2_sol1 + theta1);
q3_global_1 = rad2deg(q3_sol1 + theta1);

% Case 2 Global Angles
q2_global_2 = rad2deg(q2_sol2 + theta1);
q3_global_2 = rad2deg(q3_sol2 + theta1);

% Ground Vector (Same for both)
RO4O2 = d*exp(j*theta1);
RO4O2x = real(RO4O2); RO4O2y = imag(RO4O2);
O4x = RO4O2x; O4y = RO4O2y;

% --- Vectors Case 1 ---
RA1 = a*exp(j*(q2_sol1 + theta1));
RBA1 = b*exp(j*(q3_sol1 + theta1));
RBO4_1 = c*exp(j*(q4 + theta1)); % Input Fixed

RA1x = real(RA1); RA1y = imag(RA1);
RBA1x = real(RBA1); RBA1y = imag(RBA1);
RBO4_1x = real(RBO4_1); RBO4_1y = imag(RBO4_1);

% --- Vectors Case 2 ---
RA2 = a*exp(j*(q2_sol2 + theta1));
RBA2 = b*exp(j*(q3_sol2 + theta1));
RBO4_2 = c*exp(j*(q4 + theta1)); % Input Fixed

RA2x = real(RA2); RA2y = imag(RA2);
RBA2x = real(RBA2); RBA2y = imag(RBA2);
RBO4_2x = real(RBO4_2); RBO4_2y = imag(RBO4_2);


% ==========================================
% 5. PLOTTING (Separated)
% ==========================================
color_Pink = [1, 0.07, 0.57];
color_LBlue = [0.2, 0.8, 1];
color_Blue = [0, 0, 1];
color_Brown = [0.6, 0.4, 0.2];

% --- Figure 1: Case 1 ---
figure(1);
hold on;
quiver(0,0, O4x, O4y, 0, 'Color', color_Pink, 'LineWidth', 4, 'MaxHeadSize', 0.5); % Ground
quiver(0,0, RA1x, RA1y, 0, 'Color', color_LBlue, 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 2
quiver(RA1x, RA1y, RBA1x, RBA1y, 0, 'Color', color_Blue, 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 3
quiver(O4x, O4y, RBO4_1x, RBO4_1y, 0, 'Color', color_Brown, 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 4
axis equal; grid on;
title(['Case 1: Solution 1 (Parallel/Open), \theta_4 = ' num2str(q4_global_deg)]);
xlabel('x'); ylabel('y');

% --- Figure 2: Case 2 ---
figure(2);
hold on;
quiver(0,0, O4x, O4y, 0, 'Color', color_Pink, 'LineWidth', 4, 'MaxHeadSize', 0.5); % Ground
quiver(0,0, RA2x, RA2y, 0, 'Color', color_LBlue, 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 2
quiver(RA2x, RA2y, RBA2x, RBA2y, 0, 'Color', color_Blue, 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 3
quiver(O4x, O4y, RBO4_2x, RBO4_2y, 0, 'Color', color_Brown, 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 4
axis equal; grid on;
title(['Case 2: Solution 2 (Crossed), \theta_4 = ' num2str(q4_global_deg)]);
xlabel('x'); ylabel('y');


% Display Results
disp('--- Results ---');
disp(['Case 1: Theta 2 = ' num2str(q2_global_1) ' deg, Theta 3 = ' num2str(q3_global_1) ' deg']);
disp(['Case 2: Theta 2 = ' num2str(q2_global_2) ' deg, Theta 3 = ' num2str(q3_global_2) ' deg']);
