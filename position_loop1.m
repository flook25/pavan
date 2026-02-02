clear all
close all
clc

% ==========================================
% 1. PARAMETERS
% ==========================================
L1 = 0.210; % Ground (Pink) - d
L2 = 0.118; % Crank (Cyan) - a
L3 = 0.210; % Coupler (Blue) - b
L4 = 0.118; % Rocker (Brown) - c (Input)

% Assign Canonical Names
d = L1;
a = L2;
b = L3;
c = L4;

% Input Parameters
theta4_deg = 102.05; 
offset_deg = 0.81;

% Convert to radians
offset = deg2rad(offset_deg);
q_in_local = deg2rad(theta4_deg) - offset; 

% ==========================================
% 2. CALCULATION (Inverted Method)
% ==========================================
% Link 4 is Input -> Treat 'c' as Crank, 'a' as Follower
K1 = d/c;
K2 = d/a;
K3 = (c^2 - b^2 + a^2 + d^2)/(2*c*a);
K4 = d/b;
K5 = (a^2 - d^2 - c^2 - b^2)/(2*c*b);

% Coefficients for Theta 2 (Output)
A = cos(q_in_local) - K1 - K2*cos(q_in_local) + K3;
B = -2*sin(q_in_local);
C = K1 - (K2+1)*cos(q_in_local) + K3;

% Coefficients for Theta 3 (Coupler)
D_coeff = cos(q_in_local) - K1 + K4*cos(q_in_local) + K5;
E_coeff = -2*sin(q_in_local);
F_coeff = K1 + (K4-1)*cos(q_in_local) + K5;

% --- SOLVING QUADRATIC ---
det_2 = sqrt(B^2 - 4*A*C);
det_3 = sqrt(E_coeff^2 - 4*D_coeff*F_coeff);

% Calculate Roots
% Note: Using geometric consistency to pair theta2 and theta3
root1_q2 = 2*atan2((-B - det_2), (2*A)); 
root2_q2 = 2*atan2((-B + det_2), (2*A)); 

root1_q3 = 2*atan2((-E_coeff - det_3), (2*D_coeff)); 
root2_q3 = 2*atan2((-E_coeff + det_3), (2*D_coeff));

% --- ASSIGNMENT (SWAPPED BASED ON FEEDBACK) ---
% สลับค่าตามที่คุณแจ้งว่า "ค่าสลับกัน"
% เดิม: Open=root1, Crossed=root2
% ใหม่: Open=root2, Crossed=root1

% Case 1: Open
Theta2_Open_Local  = root2_q2; 
Theta3_Open_Local  = root2_q3;

% Case 2: Crossed
Theta2_Cross_Local = root1_q2;
Theta3_Cross_Local = root1_q3;

% ==========================================
% 3. VECTOR CONSTRUCTION & PLOTTING
% ==========================================
Rot = exp(1j * offset); 
Theta4_Global = q_in_local + offset;

% --- CASE 1: OPEN ---
T2_Open = Theta2_Open_Local + offset;
T3_Open = Theta3_Open_Local + offset;

% Vectors Case 1 (Standard Vector Loop: R2+R3 = R1+R4)
R1_Op = d * exp(1j * offset);            % Ground
R2_Op = a * exp(1j * T2_Open);           % Link 2 (Cyan)
R4_Op = c * exp(1j * Theta4_Global);     % Link 4 (Brown)
% Calculate R3 to ensure closure
vec_B_Op = R1_Op + R4_Op; 
vec_A_Op = R2_Op;
R3_Op = vec_B_Op - vec_A_Op;             % Link 3 (Blue)

% --- CASE 2: CROSSED ---
T2_Cr = Theta2_Cross_Local + offset;
T3_Cr = Theta3_Cross_Local + offset;

% Vectors Case 2
R1_Cr = d * exp(1j * offset);
R2_Cr = a * exp(1j * T2_Cr);             % Link 2 (Cyan)
R4_Cr = c * exp(1j * Theta4_Global);     % Link 4 (Brown)
vec_B_Cr = R1_Cr + R4_Cr;
vec_A_Cr = R2_Cr;
R3_Cr = vec_B_Cr - vec_A_Cr;             % Link 3 (Blue)

% --- PLOT ---
figure(1); clf;

% Subplot 1: Open
subplot(1,2,1); hold on; grid on; axis equal;
title('Case 1: Open Circuit');
quiver(0,0, real(R1_Op), imag(R1_Op), 0, 'm', 'LineWidth', 2, 'MaxHeadSize',0.5); % Ground
quiver(0,0, real(R2_Op), imag(R2_Op), 0, 'c', 'LineWidth', 2, 'MaxHeadSize',0.5); % Link 2 (Cyan)
quiver(real(R1_Op), imag(R1_Op), real(R4_Op), imag(R4_Op), 0, 'Color',[0.6 0.3 0], 'LineWidth', 2, 'MaxHeadSize',0.5); % Link 4
quiver(real(R2_Op), imag(R2_Op), real(R3_Op), imag(R3_Op), 0, 'b', 'LineWidth', 2, 'MaxHeadSize',0.5); % Link 3

% Subplot 2: Crossed
subplot(1,2,2); hold on; grid on; axis equal;
title('Case 2: Crossed Circuit');
quiver(0,0, real(R1_Cr), imag(R1_Cr), 0, 'm', 'LineWidth', 2, 'MaxHeadSize',0.5); % Ground
quiver(0,0, real(R2_Cr), imag(R2_Cr), 0, 'c', 'LineWidth', 2, 'MaxHeadSize',0.5); % Link 2 (Cyan)
quiver(real(R1_Cr), imag(R1_Cr), real(R4_Cr), imag(R4_Cr), 0, 'Color',[0.6 0.3 0], 'LineWidth', 2, 'MaxHeadSize',0.5); % Link 4
quiver(real(R2_Cr), imag(R2_Cr), real(R3_Cr), imag(R3_Cr), 0, 'b', 'LineWidth', 2, 'MaxHeadSize',0.5); % Link 3

% --- DISPLAY VALUES ---
disp('========================================');
disp(['LOOP 1 RESULTS (Input Theta 4 = ' num2str(theta4_deg) ' deg)']);
disp('Note: Values swapped per user feedback.');
disp('========================================');
disp('--- CASE 1: OPEN ---');
disp(['Theta 2 (Cyan) : ', num2str(rad2deg(T2_Open))]);
disp(['Theta 3 (Blue) : ', num2str(rad2deg(angle(R3_Op)))]);
disp(' ');
disp('--- CASE 2: CROSSED ---');
disp(['Theta 2 (Cyan) : ', num2str(rad2deg(T2_Cr))]);
disp(['Theta 3 (Blue) : ', num2str(rad2deg(angle(R3_Cr)))]);
disp('========================================');
