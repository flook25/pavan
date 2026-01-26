clear all
close all
clc

% 1. Parameter Definitions
L1 = 210; % Ground (Pink)
L2 = 118; % Link 2 (Light Blue)
L3 = 210; % Link 3 (Blue)
L4 = 118; % Link 4 (Brown)

a = L2;
b = L3;
c = L4;
d = L1;

% Ground angle
theta1_deg = 0.81;
theta1 = deg2rad(theta1_deg);

% Input: Theta 4 (Global)
q4_global_deg = 102.5;
q4_global = deg2rad(q4_global_deg);

% Local Theta 4
q4 = q4_global - theta1;

% ---------------------------------------------------------
% STEP 1: Find Theta 2 given Theta 4 (Inverse Kinematics)
% ---------------------------------------------------------
% K Constants (Swapped a and c for Inverse)
K1_inv = d/c; 
K2_inv = d/a; 
K3_inv = (c^2 - b^2 + a^2 + d^2)/(2*c*a); 

% Coefficients A, B, C for finding q2
A = cos(q4) - K1_inv - K2_inv*cos(q4) + K3_inv;
B = -2*sin(q4);
C = K1_inv - (K2_inv + 1)*cos(q4) + K3_inv;

% Solve for q2
disc = B^2 - 4*A*C;

% เราคำนวณทั้ง 2 คำตอบก่อน
q2_option_A = 2*atan((-B - sqrt(disc))/(2*A)); % ปกติคือค่าที่ชี้ขึ้น (Parallel)
q2_option_B = 2*atan((-B + sqrt(disc))/(2*A)); % ปกติคือค่าที่ชี้ลง (Crossed)

% *** จัดเรียง Case ตามคำขอ ***
% Case 2: สีฟ้าต้องอยู่บน (Above Pink) -> ใช้ Option A
q2_local_2 = q2_option_A;

% Case 1: อีกคำตอบหนึ่ง (ชี้ลง) -> ใช้ Option B
q2_local_1 = q2_option_B;


% ---------------------------------------------------------
% STEP 2: Find Theta 3 for EACH Case
% ---------------------------------------------------------
K1 = d/a;
K4 = d/b;
K5 = (c^2 - d^2 - a^2 - b^2)/(2*a*b);

% --- Calculate q3 for Case 1 (Down) ---
D1 = cos(q2_local_1) - K1 + K4*cos(q2_local_1) + K5;
E1 = -2*sin(q2_local_1);
F1 = K1 + (K4 - 1)*cos(q2_local_1) + K5;
% เลือกเครื่องหมายให้ปิดลูปแบบ Crossed
q3_local_1 = 2*atan((-E1 + sqrt(E1^2 - 4*D1*F1))/(2*D1));

% --- Calculate q3 for Case 2 (Up / Above Pink) ---
D2 = cos(q2_local_2) - K1 + K4*cos(q2_local_2) + K5;
E2 = -2*sin(q2_local_2);
F2 = K1 + (K4 - 1)*cos(q2_local_2) + K5;
% เลือกเครื่องหมายให้ปิดลูปแบบ Parallel
q3_local_2 = 2*atan((-E2 - sqrt(E2^2 - 4*D2*F2))/(2*D2));


% Calculate Global Angles
q2_global_1 = rad2deg(q2_local_1 + theta1);
q3_global_1 = rad2deg(q3_local_1 + theta1);

q2_global_2 = rad2deg(q2_local_2 + theta1);
q3_global_2 = rad2deg(q3_local_2 + theta1);


% ---------------------------------------------------------
% PLOTTING PREPARATION
% ---------------------------------------------------------
% Colors
color_Pink = [1, 0.07, 0.57];
color_LBlue = [0.2, 0.8, 1];
color_Blue = [0, 0, 1];
color_Brown = [0.6, 0.4, 0.2];

% Ground (Same for both)
RO4O2 = d*exp(j*theta1); 
O4x = real(RO4O2); O4y = imag(RO4O2);

% --- Case 1 Vectors (Down) ---
RA1 = a*exp(j*(q2_local_1 + theta1));
RBA1 = b*exp(j*(q3_local_1 + theta1));
Ax1 = real(RA1); Ay1 = imag(RA1);
Bx1 = real(RA1 + RBA1); By1 = imag(RA1 + RBA1);

% --- Case 2 Vectors (Up / Above Pink) ---
RA2 = a*exp(j*(q2_local_2 + theta1));
RBA2 = b*exp(j*(q3_local_2 + theta1));
Ax2 = real(RA2); Ay2 = imag(RA2);
Bx2 = real(RA2 + RBA2); By2 = imag(RA2 + RBA2);


% ---------------------------------------------------------
% PLOTTING
% ---------------------------------------------------------

% --- Figure 1: Case 1 (Down/Crossed) ---
figure(1);
hold on;
quiver(0, 0, O4x, O4y, 0, 'Color', color_Pink, 'LineWidth', 4, 'MaxHeadSize', 0.5); % Ground
quiver(0, 0, Ax1, Ay1, 0, 'Color', color_LBlue, 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 2
quiver(Ax1, Ay1, Bx1-Ax1, By1-Ay1, 0, 'Color', color_Blue, 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 3
quiver(O4x, O4y, Bx1-O4x, By1-O4y, 0, 'Color', color_Brown, 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 4
axis equal; grid on;
title(['Case 1: Down/Crossed, \theta_4 = ' num2str(q4_global_deg)]);
xlabel('x'); ylabel('y');


% --- Figure 2: Case 2 (Up/Above Pink) ---
figure(2);
hold on;
quiver(0, 0, O4x, O4y, 0, 'Color', color_Pink, 'LineWidth', 4, 'MaxHeadSize', 0.5); % Ground
quiver(0, 0, Ax2, Ay2, 0, 'Color', color_LBlue, 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 2 (อยู่บน)
quiver(Ax2, Ay2, Bx2-Ax2, By2-Ay2, 0, 'Color', color_Blue, 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 3
quiver(O4x, O4y, Bx2-O4x, By2-O4y, 0, 'Color', color_Brown, 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 4
axis equal; grid on;
title(['Case 2: Up/Above Pink (Parallel), \theta_4 = ' num2str(q4_global_deg)]);
xlabel('x'); ylabel('y');


% Display Results
fprintf('--- Results ---\n');
fprintf('Case 1 (Down): Theta 2 = %.4f deg, Theta 3 = %.4f deg\n', q2_global_1, q3_global_1);
fprintf('Case 2 (Up):   Theta 2 = %.4f deg, Theta 3 = %.4f deg\n', q2_global_2, q3_global_2);
