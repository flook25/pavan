clear all
close all
clc

% 1. Parameter Definitions
L1 = 210; % Length of Link1 d (Ground) - Pink
L2 = 118; % Length of Link2 a (Input/Crank) - Light Blue
L3 = 210; % Length of Link3 b (Coupler) - Blue
L4 = 118; % Length of Link4 c (Output/Rocker) - Brown

% Mapping to standard variables
d = L1;
a = L2;
b = L3;
c = L4;

% Ground angle (Lifted up by 0.81 degrees)
% แก้ไข: แปลง 0.81 องศา เป็น radian ทันที
theta1_deg = 0.81;
theta1 = deg2rad(theta1_deg); 

% Input: Theta 4 (Global)
q4_global_deg = 102.5;
q4_global = deg2rad(q4_global_deg);

% Convert to Local Frame (Relative to Ground Link)
q4 = q4_global - theta1;

% ---------------------------------------------------------
% STEP 1: Find Theta 2 given Theta 4
% ใช้ Concept เดิม: สลับบทบาท a กับ c เพื่อหา q2 จาก q4
% ---------------------------------------------------------

% Inverse K Constants
K1_inv = d/c; 
K2_inv = d/a; 
K3_inv = (c^2 - b^2 + a^2 + d^2)/(2*c*a); 

% Coefficients A, B, C for finding q2
A_inv = cos(q4) - K1_inv - K2_inv*cos(q4) + K3_inv;
B_inv = -2*sin(q4);
C_inv = K1_inv - (K2_inv + 1)*cos(q4) + K3_inv;

% Solve for q2 using 2*atan formula
disc = B_inv^2 - 4*A_inv*C_inv;

% คำนวณทั้ง 2 กรณี (Open และ Crossed)
q2_sol1 = 2*atan((-B_inv - sqrt(disc))/(2*A_inv));
q2_sol2 = 2*atan((-B_inv + sqrt(disc))/(2*A_inv));

% ---------------------------------------------------------
% STEP 2: Find Theta 3 given Theta 2
% ใช้สูตร Forward Kinematics ปกติ (หา q3 จาก q2)
% ---------------------------------------------------------
K1 = d/a;
K4 = d/b;
K5 = (c^2 - d^2 - a^2 - b^2)/(2*a*b);

% --- Case 1 Calculation ---
D1 = cos(q2_sol1) - K1 + K4*cos(q2_sol1) + K5;
E1 = -2*sin(q2_sol1);
F1 = K1 + (K4 - 1)*cos(q2_sol1) + K5;
% เลือกเครื่องหมายให้สอดคล้อง (Open มักใช้ -sqrt)
q3_sol1 = 2*atan((-E1 - sqrt(E1^2 - 4*D1*F1))/(2*D1));

% --- Case 2 Calculation ---
D2 = cos(q2_sol2) - K1 + K4*cos(q2_sol2) + K5;
E2 = -2*sin(q2_sol2);
F2 = K1 + (K4 - 1)*cos(q2_sol2) + K5;
% เลือกเครื่องหมายให้สอดคล้อง (Crossed มักใช้ +sqrt)
q3_sol2 = 2*atan((-E2 + sqrt(E2^2 - 4*D2*F2))/(2*D2));

% ---------------------------------------------------------
% PLOTTING (แยก 2 Figure ตามคำขอ)
% ---------------------------------------------------------

% Ground Vector (Lifted 0.81 deg)
RO4O2 = d*exp(j*theta1); 
RO4O2x = real(RO4O2); RO4O2y = imag(RO4O2);

% Colors
color_Pink = [1, 0.07, 0.57];  % L1 Pink
color_LBlue = [0.2, 0.8, 1];   % L2 Light Blue
color_Blue = [0, 0, 1];        % L3 Blue
color_Brown = [0.6, 0.4, 0.2]; % L4 Brown

% --- Figure 1: Open Circuit (Parallel) ---
% กรณีนี้คือ Parallelogram (q2 = q4) ลิ้งค์ฟ้าจะชี้ขึ้น
figure(1);
% คำนวณเวกเตอร์
RA1 = a*exp(j*(q2_sol1 + theta1));
RBA1 = b*exp(j*(q3_sol1 + theta1));
RBO4_1 = c*exp(j*(q4 + theta1));

RA1x = real(RA1); RA1y = imag(RA1);
RB1x = real(RA1 + RBA1); RB1y = imag(RA1 + RBA1);

hold on;
quiver(0,0, RO4O2x, RO4O2y, 0, 'Color', color_Pink, 'LineWidth', 4, 'MaxHeadSize', 0.5); % Ground
quiver(0,0, RA1x, RA1y, 0, 'Color', color_LBlue, 'LineWidth', 3, 'MaxHeadSize', 0.5);   % Link 2 (LBlue)
quiver(RA1x, RA1y, RB1x-RA1x, RB1y-RA1y, 0, 'Color', color_Blue, 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 3 (Blue)
quiver(RO4O2x, RO4O2y, RB1x-RO4O2x, RB1y-RO4O2y, 0, 'Color', color_Brown, 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 4 (Brown)
axis equal; grid on;
title(['Case 1: Open Circuit (Parallel)']);
xlabel('x'); ylabel('y');

% Print Results Case 1
fprintf('--- Case 1: Open Circuit ---\n');
fprintf('Theta 2 = %.4f deg\n', rad2deg(q2_sol1 + theta1));
fprintf('Theta 3 = %.4f deg\n', rad2deg(q3_sol1 + theta1));


% --- Figure 2: Crossed Circuit (Anti-Parallel) ---
% กรณีนี้ลิ้งค์จะไขว้กัน
figure(2);
% คำนวณเวกเตอร์
RA2 = a*exp(j*(q2_sol2 + theta1));
RBA2 = b*exp(j*(q3_sol2 + theta1));
RBO4_2 = c*exp(j*(q4 + theta1));

RA2x = real(RA2); RA2y = imag(RA2);
RB2x = real(RA2 + RBA2); RB2y = imag(RA2 + RBA2);

hold on;
quiver(0,0, RO4O2x, RO4O2y, 0, 'Color', color_Pink, 'LineWidth', 4, 'MaxHeadSize', 0.5); % Ground
quiver(0,0, RA2x, RA2y, 0, 'Color', color_LBlue, 'LineWidth', 3, 'MaxHeadSize', 0.5);   % Link 2 (LBlue)
quiver(RA2x, RA2y, RB2x-RA2x, RB2y-RA2y, 0, 'Color', color_Blue, 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 3 (Blue)
quiver(RO4O2x, RO4O2y, RB2x-RO4O2x, RB2y-RO4O2y, 0, 'Color', color_Brown, 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 4 (Brown)
axis equal; grid on;
title(['Case 2: Crossed Circuit']);
xlabel('x'); ylabel('y');

% Print Results Case 2
fprintf('\n--- Case 2: Crossed Circuit ---\n');
fprintf('Theta 2 = %.4f deg\n', rad2deg(q2_sol2 + theta1));
fprintf('Theta 3 = %.4f deg\n', rad2deg(q3_sol2 + theta1));
