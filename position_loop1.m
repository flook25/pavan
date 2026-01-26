clear all
close all
clc

% ==========================================
% 1. PARAMETERS & INPUT
% ==========================================
L1 = 0.210; % Ground (Pink) - d
L2 = 0.118; % Crank (Cyan) - a
L3 = 0.210; % Coupler (Blue) - b
L4 = 0.118; % Rocker (Brown) - c

% Assign variables
d = L1;
a = L2;
b = L3;
c = L4;

% Ground Offset (Global rotation)
theta1_deg = 0.81;
theta1 = deg2rad(theta1_deg);

% --- INPUT: Link 4 (Brown) ---
% โจทย์กำหนด Link 4 เป็น Input
q4_global_deg = -102.5; 
% แปลงเป็น Local frame (เทียบกับแนวแกนของ Link 1)
q4 = deg2rad(q4_global_deg) - theta1; 

% ==========================================
% 2. CALCULATION (Find q2 and q3 given q4)
% ==========================================

% --- STEP 1: หา q2 (Link 2) จาก q4 ---
% ในกรณีนี้ Input คือ q4 (Link c) และ Output คือ q2 (Link a)
% เราจะใช้สมการรูปแบบเดียวกับ Norton แต่สลับค่าคงที่ K เพื่อแก้หา q2
% เปรียบเสมือนการมองย้อนกลับ (Inverse)

% นิยาม K สำหรับการหา q2 จาก q4 (Swap a and c conceptually)
K1_inv = d/c; 
K2_inv = d/a;
K3_inv = (c^2 - b^2 + a^2 + d^2)/(2*c*a); 

% คำนวณ A, B, C โดยใช้ q4 เป็นตัวแปรต้น
A = cos(q4) - K1_inv - K2_inv*cos(q4) + K3_inv;
B = -2*sin(q4);
C = K1_inv - (K2_inv+1)*cos(q4) + K3_inv;

% แก้สมการหา q2 (2 คำตอบ: Crossed และ Open)
% ใช้สูตร 2*atan ตาม Format
det_val = B^2 - 4*A*C;
if det_val < 0
    error('No solution: Linkage cannot define valid geometry.');
end

% q2_sol1 และ q2_sol2 (Local angles)
q2_sol1 = 2*atan((-B + sqrt(det_val))/(2*A)); % Solution 1
q2_sol2 = 2*atan((-B - sqrt(det_val))/(2*A)); % Solution 2

% --- STEP 2: หา q3 (Link 3) จาก q2 ที่ได้ ---
% เมื่อรู้ q2 แล้ว เราสามารถหา q3 ได้โดยใช้ชุด K มาตรฐาน
K1 = d/a;
K4 = d/b;
K5 = (c^2 - d^2 - a^2 - b^2)/(2*a*b);

% ต้องคำนวณแยก 2 กรณีสำหรับ q2 แต่ละค่า
% Case 1: ใช้ q2_sol1
D1 = cos(q2_sol1) - K1 + K4*cos(q2_sol1) + K5;
E1 = -2*sin(q2_sol1);
F1 = K1 + (K4-1)*cos(q2_sol1) + K5;
% หา q3 สำหรับ Case 1 (เลือก sign ให้สอดคล้องกับ vector loop)
q3_sol1 = 2*atan((-E1 - sqrt(E1^2 - 4*D1*F1))/(2*D1)); 

% Case 2: ใช้ q2_sol2
D2 = cos(q2_sol2) - K1 + K4*cos(q2_sol2) + K5;
E2 = -2*sin(q2_sol2);
F2 = K1 + (K4-1)*cos(q2_sol2) + K5;
% หา q3 สำหรับ Case 2
q3_sol2 = 2*atan((-E2 + sqrt(E2^2 - 4*D2*F2))/(2*D2));


% ==========================================
% 3. PREPARE VECTORS FOR PLOTTING
% ==========================================

% เลือก Case ที่ต้องการ Plot (เลือก Case 2 ซึ่งมักเป็น Open Config ในบริบทนี้)
q2_select = q2_sol2;
q3_select = q3_sol2;

% แปลงกลับเป็น Global Angle เพื่อการพล็อตที่ถูกต้อง
q2_plot_global = q2_select + theta1;
q3_plot_global = q3_select + theta1;
q4_plot_global = q4 + theta1; % หรือเท่ากับ deg2rad(q4_global_deg)

% คำนวณเวกเตอร์ (ตาม Format ที่ให้มา)
% หมายเหตุ: การคำนวณเวกเตอร์ใช้มุม Global เพื่อให้ทิศทางถูกต้องบนกราฟ
RA = a*exp(1j*q2_plot_global);      % Vector Link 2 (Red)
RBA2 = b*exp(1j*q3_plot_global);    % Vector Link 3 (Blue)

% คำนวณตำแหน่งจุด B (Position Vector)
RB2 = RA + RBA2; 

% แยก Component (Real/Imag)
RAx = real(RA);
RAy = imag(RA);

RBA2x = real(RBA2);
RBA2y = imag(RBA2);

RB2x = real(RB2);
RB2y = imag(RB2);

% คำนวณ Link 4 และ Ground เพื่อวาดให้ครบวง
RO4O2 = d*exp(1j*theta1);           % Vector Ground (จาก Origin ไป O4)
RBO42 = c*exp(1j*q4_plot_global);   % Vector Link 4 (จาก O4 ไป B)

RO4O2x = real(RO4O2);
RO4O2y = imag(RO4O2);

RBO42x = real(RBO42);
RBO42y = imag(RBO42);


% ==========================================
% 4. DISPLAY & PLOT
% ==========================================
fprintf('--- Results (Input Theta4 = %.2f deg) ---\n', q4_global_deg);
fprintf('Theta 2 (Global): %.4f deg\n', rad2deg(q2_plot_global));
fprintf('Theta 3 (Global): %.4f deg\n', rad2deg(q3_plot_global));

figure;
hold on;
axis equal;
grid on;

% 1. Plot Link 2 (RA) - สีแดง (Red)
quiver(0, 0, RAx, RAy, 0, 'red', 'MaxHeadSize', 0.5, 'LineWidth', 2);

% 2. Plot Link 3 (RBA) - สีน้ำเงิน (Blue) เริ่มที่ปลาย RA
quiver(RAx, RAy, RBA2x, RBA2y, 0, 'blue', 'MaxHeadSize', 0.5, 'LineWidth', 2);

% 3. Plot Position Vector B (RB2) - สีเขียว (Green) จากจุด Origin
quiver(0, 0, RB2x, RB2y, 0, 'green', 'MaxHeadSize', 0.5, 'LineWidth', 2);

% 4. Plot Ground (RO4O2) - สีดำ (Black) จาก Origin ไป O4
quiver(0, 0, RO4O2x, RO4O2y, 0, 'black', 'MaxHeadSize', 0.5, 'LineWidth', 2);

% 5. Plot Link 4 (RBO4) - สีดำ (Black) จาก O4 ไป B
quiver(RO4O2x, RO4O2y, RBO42x, RBO42y, 0, 'black', 'MaxHeadSize', 0.5, 'LineWidth', 2);

xlabel('X Position (m)');
ylabel('Y Position (m)');
title(['4-Bar Linkage Analysis (Input \theta_4 = ', num2str(q4_global_deg), '^\circ)']);

% เพิ่ม Text ระบุจุด
text(0, 0, ' O_2');
text(RAx, RAy, ' A');
text(RB2x, RB2y, ' B');
text(RO4O2x, RO4O2y, ' O_4');
