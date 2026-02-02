clear all
close all
clc

% ==========================================
% 1. PARAMETERS
% ==========================================
L1 = 0.210; % Ground (Pink) - d
L2 = 0.118; % Crank (Cyan) - a
L3 = 0.210; % Coupler (Blue) - b
L4 = 0.118; % Rocker (Brown) - c (Input here)

% กำหนดตัวแปรสำหรับคำนวณ
d = L1;
a = L2; 
b = L3;
c = L4;

% Input Parameters
theta4_deg = 102.05; 
offset_deg = 0.81;

% Convert to radians
offset = deg2rad(offset_deg);
q4_local = deg2rad(theta4_deg) - offset;

% ==========================================
% 2. CALCULATION (Inverted with Frame Correction)
% ==========================================
% *** หัวใจสำคัญ: หมุน Input ไป -180 องศา เพื่อเข้าสู่ Inverted Frame (O4 มองไป O2) ***
q_in_calc = q4_local - pi; 

% สลับตัวแปร: ให้ c (Link 4) เป็น Input, a (Link 2) เป็น Output ที่จะหา
d_sim = d;
a_sim = c; % Link 4 ทำหน้าที่เป็น Crank
b_sim = b;
c_sim = a; % Link 2 ทำหน้าที่เป็น Rocker

% Freudenstein Constants (Inverted)
K1 = d_sim/a_sim;
K2 = d_sim/c_sim;
K3 = (a_sim^2 - b_sim^2 + c_sim^2 + d_sim^2)/(2*a_sim*c_sim);
K4 = d_sim/b_sim;
K5 = (c_sim^2 - d_sim^2 - a_sim^2 - b_sim^2)/(2*a_sim*b_sim);

% Coefficients
A = cos(q_in_calc) - K1 - K2*cos(q_in_calc) + K3;
B = -2*sin(q_in_calc);
C = K1 - (K2+1)*cos(q_in_calc) + K3;

D_val = cos(q_in_calc) - K1 + K4*cos(q_in_calc) + K5;
E_val = -2*sin(q_in_calc);
F_val = K1 + (K4-1)*cos(q_in_calc) + K5;

% Solve Quadratic
det_AC = sqrt(B^2 - 4*A*C);
det_DF = sqrt(E_val^2 - 4*D_val*F_val);

% --- หาคำตอบใน Inverted Frame ---
% สูตรมาตรฐาน: (-B - sqrt) คือ Open, (-B + sqrt) คือ Crossed
q2_inv_1 = 2*atan2((-B - det_AC), (2*A));
q3_inv_1 = 2*atan2((-E_val - det_DF), (2*D_val));

q2_inv_2 = 2*atan2((-B + det_AC), (2*A));
q3_inv_2 = 2*atan2((-E_val + det_DF), (2*D_val));

% ==========================================
% 3. TRANSFORM BACK TO GLOBAL (หมุนกลับ)
% ==========================================
% *** หัวใจสำคัญ: บวก 180 องศา กลับคืน เพื่อให้ทิศทางถูกต้องในโลกจริง ***
Theta2_Open_Local  = q2_inv_1 + pi;
Theta3_Open_Local  = q3_inv_1 + pi;

Theta2_Cross_Local = q2_inv_2 + pi;
Theta3_Cross_Local = q3_inv_2 + pi;

% บวก Offset ของพื้นเอียง
T2_Open = Theta2_Open_Local + offset;
T3_Open = Theta3_Open_Local + offset;

T2_Cross = Theta2_Cross_Local + offset;
T3_Cross = Theta3_Cross_Local + offset;

Theta4_Global = q4_local + offset;

% ==========================================
% 4. PLOTTING (VECTOR LOOP)
% ==========================================
% Global Rotation Matrix
Rot = exp(1i * offset);

% Ground (Pink)
R1 = d * exp(1i * offset);

% --- สร้างเวกเตอร์ Case 1 (Open) ---
R2_Op = a * exp(1i * T2_Open);           % Link 2 (Cyan) - เริ่มที่ O2 (0,0)
R4_Op = c * exp(1i * Theta4_Global);     % Link 4 (Brown) - สัมพัทธ์กับ O4
% Link 3 (Blue) ต้องเชื่อมปลาย R2 ไปหาปลาย R4
% ตำแหน่งปลาย R4 (Point B) = R1 + R4_Op
% ตำแหน่งปลาย R2 (Point A) = R2_Op
% เวกเตอร์ R3 = B - A
vec_B_Op = R1 + R4_Op;
vec_A_Op = R2_Op;
R3_Op = vec_B_Op - vec_A_Op;

% --- สร้างเวกเตอร์ Case 2 (Crossed) ---
R2_Cr = a * exp(1i * T2_Cross);
R4_Cr = c * exp(1i * Theta4_Global);
vec_B_Cr = R1 + R4_Cr;
vec_A_Cr = R2_Cr;
R3_Cr = vec_B_Cr - vec_A_Cr;

% --- PLOT ---
figure(1); clf;

% Subplot 1: Case 1 (Open)
subplot(1,2,1); hold on; grid on; axis equal;
title('Case 1: Open Circuit');
% Ground
plot([0 real(R1)], [0 imag(R1)], 'm-', 'LineWidth', 2);
% Link 2 (Cyan)
quiver(0, 0, real(R2_Op), imag(R2_Op), 0, 'c', 'LineWidth', 2, 'MaxHeadSize', 0.5);
% Link 4 (Brown) เริ่มที่ O4
quiver(real(R1), imag(R1), real(R4_Op), imag(R4_Op), 0, 'Color', [0.6 0.3 0], 'LineWidth', 2, 'MaxHeadSize', 0.5);
% Link 3 (Blue)
quiver(real(R2_Op), imag(R2_Op), real(R3_Op), imag(R3_Op), 0, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);

% Subplot 2: Case 2 (Crossed)
subplot(1,2,2); hold on; grid on; axis equal;
title('Case 2: Crossed Circuit');
% Ground
plot([0 real(R1)], [0 imag(R1)], 'm-', 'LineWidth', 2);
% Link 2 (Cyan)
quiver(0, 0, real(R2_Cr), imag(R2_Cr), 0, 'c', 'LineWidth', 2, 'MaxHeadSize', 0.5);
% Link 4 (Brown) เริ่มที่ O4
quiver(real(R1), imag(R1), real(R4_Cr), imag(R4_Cr), 0, 'Color', [0.6 0.3 0], 'LineWidth', 2, 'MaxHeadSize', 0.5);
% Link 3 (Blue)
quiver(real(R2_Cr), imag(R2_Cr), real(R3_Cr), imag(R3_Cr), 0, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);

% --- DISPLAY VALUES ---
disp('======================================');
disp(['Loop 1 Analysis (Input Theta4 = ' num2str(theta4_deg) ' deg)']);
disp('======================================');
disp('--- CASE 1 (OPEN) ---');
disp(['  Link 2 (Cyan):  ', num2str(rad2deg(T2_Open))]);
disp(['  Link 3 (Blue):  ', num2str(rad2deg(angle(R3_Op)))]);
disp(' ');
disp('--- CASE 2 (CROSSED) ---');
disp(['  Link 2 (Cyan):  ', num2str(rad2deg(T2_Cross))]);
disp(['  Link 3 (Blue):  ', num2str(rad2deg(angle(R3_Cr)))]);
