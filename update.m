clear all
close all
clc


% 1. PARAMETERS & INPUT
L1 = 0.210; % Ground (Pink)
L2 = 0.118; % Crank (Cyan) - Output
L3 = 0.210; % Coupler (Blue)
L4 = 0.118; % Rocker (Brown) - Input

d = L1;
a = L2;
b = L3;
c = L4;

theta4_deg = 102.05; 
offset_deg = 0.81;
offset = deg2rad(offset_deg);


% 2. CALCULATION (Inverted Local Frame)


q_in = deg2rad(theta4_deg); % Local Input (relative to O4->O2)

% Constants for Inverted Calculation (Link 4 is Crank, Link 2 is Rocker)
% a_calc = c (L4), c_calc = a (L2)
K1 = d/c;
K2 = d/a;
K3 = (c^2 - b^2 + a^2 + d^2)/(2*c*a);
K4 = d/b;
K5 = (a^2 - d^2 - c^2 - b^2)/(2*c*b);

% Coefficients
A = cos(q_in) - K1 - K2*cos(q_in) + K3;
B = -2*sin(q_in);
C = K1 - (K2+1)*cos(q_in) + K3;

D_val = cos(q_in) - K1 + K4*cos(q_in) + K5;
E_val = -2*sin(q_in);
F_val = K1 + (K4-1)*cos(q_in) + K5;

% Solve Quadratic Roots
det_AC = sqrt(B^2 - 4*A*C);
det_DF = sqrt(E_val^2 - 4*D_val*F_val);

% --- Calculate Local Angles ---
% Case 1: Open (-sqrt)
q2_loc_1 = 2*atan2((-B - det_AC), (2*A));
q3_loc_1 = 2*atan2((-E_val - det_DF), (2*D_val));

% Case 2: Crossed (+sqrt)
q2_loc_2 = 2*atan2((-B + det_AC), (2*A));
q3_loc_2 = 2*atan2((-E_val + det_DF), (2*D_val));


% 3. TRANSFORM TO GLOBAL (Rotate +180)


% Input (Brown)
T4_Global = q_in + pi + offset;

% Case 1 (Open)
T2_Open = q2_loc_1 + pi + offset;
T3_Open = q3_loc_1 + pi + offset;

% Case 2 (Crossed)
T2_Cross = q2_loc_2 + pi + offset;
T3_Cross = q3_loc_2 + pi + offset;


% 4. VECTOR CONSTRUCTION & PLOTTING

% Global Rotation
Rot = exp(1i * offset);

% Ground (Pink) - Fixed at O2-O4
% Note: In Global, Ground is vector from O2(0,0) to O4
R1 = d * exp(1i * offset); 

% Function to plot
figure(1); clf;

% --- SUBPLOT 1: Case 1 (Open) ---์
R2_Op = a * exp(1i * T2_Open);       % Cyan (Link 2)
R4_Op = c * exp(1i * T4_Global);     % Brown (Link 4) from O4
% Link 3 
vec_B_Op = R1 + R4_Op; %  Link 4 (จุด B)
vec_A_Op = R2_Op;      % Link 2 (จุด A)
R3_Op = vec_B_Op - vec_A_Op;

subplot(1,2,1); hold on; grid on; axis equal;
title('Case 1: Open');
% Plot
plot([0 real(R1)], [0 imag(R1)], 'm-', 'LineWidth', 2); % Pink (Ground)
quiver(0, 0, real(R2_Op), imag(R2_Op), 0, 'c', 'LineWidth', 2, 'MaxHeadSize',0.5); % Cyan
quiver(real(R1), imag(R1), real(R4_Op), imag(R4_Op), 0, 'Color',[0.6 0.3 0], 'LineWidth', 2, 'MaxHeadSize',0.5); % Brown
quiver(real(R2_Op), imag(R2_Op), real(R3_Op), imag(R3_Op), 0, 'b', 'LineWidth', 2, 'MaxHeadSize',0.5); % Blue

% --- SUBPLOT 2: Case 2 (Crossed) ---์
R2_Cr = a * exp(1i * T2_Cross);       % Cyan (Link 2)
R4_Cr = c * exp(1i * T4_Global);      % Brown (Link 4)
vec_B_Cr = R1 + R4_Cr;
vec_A_Cr = R2_Cr;
R3_Cr = vec_B_Cr - vec_A_Cr;

subplot(1,2,2); hold on; grid on; axis equal;
title('Case 2: Crossed');
% Plot
plot([0 real(R1)], [0 imag(R1)], 'm-', 'LineWidth', 2); % Pink (Ground)
quiver(0, 0, real(R2_Cr), imag(R2_Cr), 0, 'c', 'LineWidth', 2, 'MaxHeadSize',0.5); % Cyan
quiver(real(R1), imag(R1), real(R4_Cr), imag(R4_Cr), 0, 'Color',[0.6 0.3 0], 'LineWidth', 2, 'MaxHeadSize',0.5); % Brown
quiver(real(R2_Cr), imag(R2_Cr), real(R3_Cr), imag(R3_Cr), 0, 'b', 'LineWidth', 2, 'MaxHeadSize',0.5); % Blue

% Display Check
disp('Case 2: Crossed');
disp(['Cyan Angle : ', num2str(rad2deg(T2_Cross))]);
disp(['Brown Angle : ', num2str(rad2deg(T4_Global))]);
