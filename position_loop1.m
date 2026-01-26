clear all
close all
clc

% ==========================================
% 1. PARAMETERS
% ==========================================
L1 = 0.210; % Ground (Pink)
L2 = 0.118; % Crank (Cyan)
L3 = 0.210; % Coupler (Blue)
L4 = 0.118; % Rocker (Brown)

d = L1;
a = L2;
b = L3;
c = L4;

% Ground Offset
theta1_deg = 0.81;
theta1 = deg2rad(theta1_deg);

% ==========================================
% 2. INPUT: Link 4 (Brown)
% ==========================================
% กำหนดให้ Link 4 ชี้ลงล่าง (ตามที่คุณต้องการคืออยู่ใต้เส้นชมพู)
q4_global_deg = -102.5; 
q4 = deg2rad(q4_global_deg) - theta1; % Local Angle

% ==========================================
% 3. CALCULATION (หา q2 จาก q4)
% ==========================================
% K Constants (Swapped a and c for Inverse Kinematics)
K1_inv = d/c; 
K2_inv = d/a; 
K3_inv = (c^2 - b^2 + a^2 + d^2)/(2*c*a); 

% Coefficients A, B, C
A = cos(q4) - K1_inv - K2_inv*cos(q4) + K3_inv;
B = -2*sin(q4);
C = K1_inv - (K2_inv + 1)*cos(q4) + K3_inv;

disc = B^2 - 4*A*C;

% คำนวณ 2 คำตอบที่เป็นไปได้ทางคณิตศาสตร์
root1 = 2*atan((-B - sqrt(disc))/(2*A));
root2 = 2*atan((-B + sqrt(disc))/(2*A));

% --- AUTO SORTING LOGIC (ระบบคัดแยกอัตโนมัติ) ---
% เช็คว่าคำตอบไหนพา Link 2 (สีฟ้า) ขึ้น หรือ ลง
y_check_1 = a * sin(root1 + theta1);
y_check_2 = a * sin(root2 + theta1);

if y_check_1 < 0
    % ถ้า root1 ชี้ลง -> ให้เป็น Case 1 (Down)
    q2_case_down = root1;
    q2_case_up   = root2;
else
    % ถ้า root1 ชี้ขึ้น -> ให้เป็น Case 2 (Up)
    q2_case_down = root2;
    q2_case_up   = root1;
end

% ==========================================
% 4. FIND THETA 3 (Vector Method)
% ==========================================
% วิธีนี้ชัวร์ที่สุด: หา Vector Link 3 จาก (Ground + Link4) - Link2
% Vector R1 (Ground) + R4 (Brown)
Vec_R1_R4 = d*exp(j*0) + c*exp(j*q4); 

% --- CASE 1: Blue Down (สีฟ้าอยู่ล่าง) ---
Vec_R2_Down = a*exp(j*q2_case_down); 
Vec_R3_For_Down = Vec_R1_R4 - Vec_R2_Down; % Vector Link 3
q3_case_down = angle(Vec_R3_For_Down); % ดึงมุมออกมา

% --- CASE 2: Blue Up (สีฟ้าอยู่บน - ตามที่คุณต้องการใน Case 2) ---
Vec_R2_Up = a*exp(j*q2_case_up);
Vec_R3_For_Up = Vec_R1_R4 - Vec_R2_Up; % Vector Link 3
q3_case_up = angle(Vec_R3_For_Up); % ดึงมุมออกมา

% แปลงเป็น Global Degree
q2_glob_down = rad2deg(q2_case_down + theta1);
q3_glob_down = rad2deg(q3_case_down + theta1);

q2_glob_up   = rad2deg(q2_case_up + theta1);
q3_glob_up   = rad2deg(q3_case_up + theta1);


% ==========================================
% 5. PLOTTING
% ==========================================
% Colors
color_Pink = [1, 0.07, 0.57];
color_Cyan = [0, 1, 1];
color_Blue = [0, 0, 1];
color_Brown = [0.6, 0.4, 0.2];

RO4O2 = d*exp(j*theta1); 
O4x = real(RO4O2); O4y = imag(RO4O2);

% --- Vector Set 1 (Down) ---
RA_D = a*exp(j*(q2_case_down + theta1));
RBA_D = b*exp(j*(q3_case_down + theta1));
RBO4_D = c*exp(j*(q4 + theta1));

Ax_D = real(RA_D); Ay_D = imag(RA_D);
Bx_D = real(RA_D + RBA_D); By_D = imag(RA_D + RBA_D);
O4B_D_x = real(RBO4_D); O4B_D_y = imag(RBO4_D); % เช็คปลายลิ้งค์ 4

% --- Vector Set 2 (Up) ---
RA_U = a*exp(j*(q2_case_up + theta1));
RBA_U = b*exp(j*(q3_case_up + theta1));
RBO4_U = c*exp(j*(q4 + theta1));

Ax_U = real(RA_U); Ay_U = imag(RA_U);
Bx_U = real(RA_U + RBA_U); By_U = imag(RA_U + RBA_U);
O4B_U_x = real(RBO4_U); O4B_U_y = imag(RBO4_U); 

% --- FIGURE 1: Case 1 (สีฟ้าอยู่ล่าง / น้ำตาลอยู่ล่าง) ---
figure(1); hold on;
% Ground
quiver(0, 0, O4x, O4y, 0, 'Color', color_Pink, 'LineWidth', 4, 'MaxHeadSize', 0.5);
% Link 2 (Cyan)
quiver(0, 0, Ax_D, Ay_D, 0, 'Color', color_Cyan, 'LineWidth', 3, 'MaxHeadSize', 0.5);
% Link 3 (Blue)
quiver(Ax_D, Ay_D, Bx_D-Ax_D, By_D-Ay_D, 0, 'Color', color_Blue, 'LineWidth', 3, 'MaxHeadSize', 0.5);
% Link 4 (Brown)
quiver(O4x, O4y, O4B_D_x, O4B_D_y, 0, 'Color', color_Brown, 'LineWidth', 3, 'MaxHeadSize', 0.5);
axis equal; grid on;
title(['Case 1: Cyan DOWN, Brown DOWN (\theta_4 = ' num2str(q4_global_deg) ')']);
xlabel('x'); ylabel('y');


% --- FIGURE 2: Case 2 (สีฟ้าอยู่บน / น้ำตาลอยู่ล่าง) ---
figure(2); hold on;
% Ground
quiver(0, 0, O4x, O4y, 0, 'Color', color_Pink, 'LineWidth', 4, 'MaxHeadSize', 0.5);
% Link 2 (Cyan) - อันนี้คืออันที่อยู่บน
quiver(0, 0, Ax_U, Ay_U, 0, 'Color', color_Cyan, 'LineWidth', 3, 'MaxHeadSize', 0.5);
% Link 3 (Blue)
quiver(Ax_U, Ay_U, Bx_U-Ax_U, By_U-Ay_U, 0, 'Color', color_Blue, 'LineWidth', 3, 'MaxHeadSize', 0.5);
% Link 4 (Brown)
quiver(O4x, O4y, O4B_U_x, O4B_U_y, 0, 'Color', color_Brown, 'LineWidth', 3, 'MaxHeadSize', 0.5);
axis equal; grid on;
title(['Case 2: Cyan UP, Brown DOWN (\theta_4 = ' num2str(q4_global_deg) ')']);
xlabel('x'); ylabel('y');

% Display
fprintf('--- Results ---\n');
fprintf('Case 1 (Down): Theta 2 = %.4f, Theta 3 = %.4f\n', q2_glob_down, q3_glob_down);
fprintf('Case 2 (Up)  : Theta 2 = %.4f, Theta 3 = %.4f\n', q2_glob_up, q3_glob_up);
