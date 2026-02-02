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

% Assign Canonical Names
d = L1;
a = L2;
b = L3;
c = L4;

% Input Parameters
theta4_deg = 102.05; 
offset_deg = 0.81;

% Convert to radians & Local Frame
offset = deg2rad(offset_deg);
q4_local = deg2rad(theta4_deg) - offset;

% ==========================================
% 2. CALCULATION (APPLYING YOUR EQUATION)
% ==========================================

% --- กำหนดตัวแปรสำหรับ Inverted Loop ---
% เราหา Theta 2 (a) โดยรู้ Theta 4 (c)
% ดังนั้นในสูตร: a_calc คือ L4, c_calc คือ L2
a_calc = c; 
b_calc = b; 
c_calc = a; 
d_calc = d;

% คำนวณ K1-K5 (ใช้ความยาวที่สลับแล้ว)
K1 = d_calc/a_calc;
K2 = d_calc/c_calc;
K3 = (a_calc^2 - b_calc^2 + c_calc^2 + d_calc^2)/(2*a_calc*c_calc);
K4 = d_calc/b_calc;
K5 = (c_calc^2 - d_calc^2 - a_calc^2 - b_calc^2)/(2*a_calc*b_calc);

% --- INPUT (เลียนแบบสมการ Loop 3: q_in = q_prev + pi) ---
q_in = q4_local + pi; 

% --- CASE 1: OPEN (ใช้สูตร -sqrt ตามมาตรฐาน Open ของ Loop 1) ---
A1 = cos(q_in) - K1 - K2*cos(q_in) + K3;
B1 = -2*sin(q_in);
C1 = K1 - (K2+1)*cos(q_in) + K3;

D1 = cos(q_in) - K1 + K4*cos(q_in) + K5;
E1 = -2*sin(q_in);
F1 = K1 + (K4-1)*cos(q_in) + K5;

% Solve (ผลลัพธ์ที่ได้คือมุมในเฟรมที่หมุนไป 180)
q2_calc_1 = 2*atan2((-B1 - sqrt(B1^2 - 4*A1*C1)), (2*A1));
q3_calc_1 = 2*atan2((-E1 - sqrt(E1^2 - 4*D1*F1)), (2*D1));

% --- CASE 2: CROSSED (ใช้สูตร +sqrt) ---
% ค่า A, B, C, D, E, F เหมือนเดิม ใช้ซ้ำได้เลย
q2_calc_2 = 2*atan2((-B1 + sqrt(B1^2 - 4*A1*C1)), (2*A1));
q3_calc_2 = 2*atan2((-E1 + sqrt(E1^2 - 4*D1*F1)), (2*D1));

% --- TRANSFORM BACK (หมุนกลับ 180 องศา) ---
% เพื่อให้ได้ค่า Theta จริงใน Global Frame
Theta2_Open_Local  = q2_calc_1 + pi;
Theta3_Open_Local  = q3_calc_1 + pi;

Theta2_Cross_Local = q2_calc_2 + pi;
Theta3_Cross_Local = q3_calc_2 + pi;

% Add Offset
T2_Open = Theta2_Open_Local + offset;
T3_Open = Theta3_Open_Local + offset;

T2_Cross = Theta2_Cross_Local + offset;
T3_Cross = Theta3_Cross_Local + offset;

Theta4_Global = q4_local + offset;

% ==========================================
% 3. PLOTTING
% ==========================================
Rot = exp(1j * offset);

% Ground
R1 = d * exp(1j * offset);

% --- Case 1 Vectors ---
R2_Op = a * exp(1j * T2_Open);
R4_Op = c * exp(1j * Theta4_Global);
% Loop Closure: R2 + R3 = R1 + R4
R3_Op = (R1 + R4_Op) - R2_Op; 

% --- Case 2 Vectors ---
R2_Cr = a * exp(1j * T2_Cross);
R4_Cr = c * exp(1j * Theta4_Global);
R3_Cr = (R1 + R4_Cr) - R2_Cr;

figure(1); clf;

% Subplot 1: Case 1
subplot(1,2,1); hold on; grid on; axis equal;
title('Case 1: Open');
quiver(0,0, real(R1), imag(R1), 0, 'm', 'LineWidth', 2, 'MaxHeadSize',0.5);
quiver(0,0, real(R2_Op), imag(R2_Op), 0, 'c', 'LineWidth', 2, 'MaxHeadSize',0.5);
quiver(real(R1), imag(R1), real(R4_Op), imag(R4_Op), 0, 'Color',[0.6 0.3 0], 'LineWidth', 2, 'MaxHeadSize',0.5);
quiver(real(R2_Op), imag(R2_Op), real(R3_Op), imag(R3_Op), 0, 'b', 'LineWidth', 2, 'MaxHeadSize',0.5);

% Subplot 2: Case 2
subplot(1,2,2); hold on; grid on; axis equal;
title('Case 2: Crossed');
quiver(0,0, real(R1), imag(R1), 0, 'm', 'LineWidth', 2, 'MaxHeadSize',0.5);
quiver(0,0, real(R2_Cr), imag(R2_Cr), 0, 'c', 'LineWidth', 2, 'MaxHeadSize',0.5);
quiver(real(R1), imag(R1), real(R4_Cr), imag(R4_Cr), 0, 'Color',[0.6 0.3 0], 'LineWidth', 2, 'MaxHeadSize',0.5);
quiver(real(R2_Cr), imag(R2_Cr), real(R3_Cr), imag(R3_Cr), 0, 'b', 'LineWidth', 2, 'MaxHeadSize',0.5);

% Display
disp('--- RESULTS ---');
disp(['Theta 4 Input: ', num2str(theta4_deg)]);
disp(['Theta 2 Open : ', num2str(rad2deg(T2_Open))]);
disp(['Theta 2 Cross: ', num2str(rad2deg(T2_Cross))]);
