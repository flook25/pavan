clear all
close all
clc

% ==========================================
% 1. PARAMETERS & INPUT
% ==========================================
L1 = 0.210; % Ground (Pink) - d
L2 = 0.118; % Crank (Cyan) - Link 2 - a
L3 = 0.210; % Coupler (Blue) - Link 3 - b
L4 = 0.118; % Rocker (Brown) - Link 4 - c

a = L2;
b = L3;
c = L4;
d = L1;

% Ground Offset Angle
theta1_deg = 0.81;
theta1 = deg2rad(theta1_deg);

% --- INPUT: Link 4 (Brown) ---
% โจทย์กำหนด q4 (global) มาให้
q4_global_deg = -102.5; 
q4 = deg2rad(q4_global_deg) - theta1; % Convert to Local Angle (relative to Link 1)

% ==========================================
% 2. CALCULATION (Find q2 given q4)
% ==========================================

% Norton Constants
K1 = d/a;
K2 = d/c;
K3 = (a^2 - b^2 + c^2 + d^2)/(2*a*c);

% เนื่องจาก Input คือ q4 และต้องการหา q2
% เราจึงต้องสลับบทบาทของตัวแปรในสมการ A, B, C
% (เปรียบเทียบกับ Standard Form ที่หา q4 จาก q2)
% โดยใช้ Logic: K1<->K2 และ q2<->q4

A = cos(q4) - K2 - K1*cos(q4) + K3;
B = -2*sin(q4);
C = K2 - (K1+1)*cos(q4) + K3;

% Solve for q2 (Link 2 angle) - 2 Solutions
% Case 1:
q2_sol1 = 2*atan((-B - sqrt(B^2-4*A*C))/(2*A)); 
% Case 2:
q2_sol2 = 2*atan((-B + sqrt(B^2-4*A*C))/(2*A));

% Convert to Degree (Local)
q2_sol1d = rad2deg(q2_sol1);
q2_sol2d = rad2deg(q2_sol2);

% Calculate q3 (Link 3 angle) from Vector Loop
% Loop: R2 + R3 - R4 - R1 = 0  => R3 = R1 + R4 - R2
% R1 = d, R4 = c*e^jq4, R2 = a*e^jq2
% เราใช้ atan2 หรือ angle function เพื่อหา q3 โดยตรงจาก Vector Sum

% Solution Set 1 (คู่กับ q2_sol1)
R3_vec1 = d + c*exp(1j*q4) - a*exp(1j*q2_sol1);
q3_sol1 = angle(R3_vec1);

% Solution Set 2 (คู่กับ q2_sol2)
R3_vec2 = d + c*exp(1j*q4) - a*exp(1j*q2_sol2);
q3_sol2 = angle(R3_vec2);

q3_sol1d = rad2deg(q3_sol1);
q3_sol2d = rad2deg(q3_sol2);

% ==========================================
% 3. VECTOR CONSTRUCTION FOR PLOTTING
% ==========================================
% เลือกใช้ Solution Set 2 (Open Config) หรือ Set 1 ตามต้องการ
% ในที่นี้สมมติเลือก Set 2 เพื่อพล็อต
q2_plot = q2_sol2; 
q3_plot = q3_sol2;

% Offset กลับเป็น Global เพื่อการพล็อตที่ถูกต้อง
q2_global = q2_plot + theta1;
q3_global = q3_plot + theta1;
q4_real_global = q4 + theta1; 
q1_global = theta1;

% สร้าง Vector (ใช้ Global Angle เพื่อพล็อตลงกราฟจริง)
RO2 = 0; % จุดเริ่มต้น
RO4 = d*exp(1j*q1_global); % Ground Vector

RA  = a*exp(1j*q2_global);      % Link 2 vector
RB_from_A = b*exp(1j*q3_global); % Link 3 vector
RB_from_O4 = c*exp(1j*q4_real_global); % Link 4 vector

% Absolute Positions
Pos_A = RO2 + RA;
Pos_B = RO2 + RA + RB_from_A;
Pos_O4 = RO4;

% แยก Component สำหรับ Quiver
RAx = real(RA);      RAy = imag(RA);
RBAx = real(RB_from_A); RBAy = imag(RB_from_A);
RO4x = real(RO4);    RO4y = imag(RO4);
RBO4x = real(RB_from_O4); RBO4y = imag(RB_from_O4);

% ==========================================
% 4. PLOT
% ==========================================
figure;
hold on; axis equal; grid on;

% Plot Ground (O2 -> O4)
quiver(0, 0, RO4x, RO4y, 0, 'k', 'LineWidth', 2, 'MaxHeadSize', 0.5); 

% Plot Link 2 (Crank) - O2 -> A
quiver(0, 0, RAx, RAy, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);

% Plot Link 3 (Coupler) - A -> B
quiver(RAx, RAy, RBAx, RBAy, 0, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);

% Plot Link 4 (Rocker/Input) - O4 -> B
% Note: Quiver start at O4, vector is RBO4
quiver(RO4x, RO4y, RBO4x, RBO4y, 0, 'color', [0.6 0.3 0], 'LineWidth', 2, 'MaxHeadSize', 0.5);

title(['4-Bar Linkage Position (Input \theta_4 = ' num2str(q4_global_deg) '^\circ)']);
xlabel('X'); ylabel('Y');

% แสดงผลลัพธ์ใน Command Window
fprintf('--- Results ---\n');
fprintf('Input Theta4 (Global): %.4f deg\n', q4_global_deg);
fprintf('Calculated Theta2 (Global): %.4f deg\n', rad2deg(q2_global));
fprintf('Calculated Theta3 (Global): %.4f deg\n', rad2deg(q3_global));
