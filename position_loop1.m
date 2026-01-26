clear all
close all
clc

% ==========================================
% 1. PARAMETERS & INPUT
% ==========================================
L1 = 0.210; % Ground (Pink)
L2 = 0.118; % Crank (Cyan) - Link 2
L3 = 0.210; % Coupler (Blue) - Link 3
L4 = 0.118; % Rocker (Brown) - Link 4

d = L1;
a = L2;
b = L3;
c = L4;

% Ground Offset
theta1_deg = 0.81;
theta1 = deg2rad(theta1_deg);

% --- INPUT: Link 4 (Brown) ---
% กำหนดให้ Brown อยู่ "ล่าง" (ตามที่คุณต้องการ)
q4_global_deg = -102.5; 
q4 = deg2rad(q4_global_deg) - theta1; % Local Angle

% ==========================================
% 2. FIND THETA 2 (Inverse Kinematics)
% ==========================================
% K Constants (Swapped 'a' and 'c' because q4 is input)
K1_inv = d/c; 
K2_inv = d/a; 
K3_inv = (c^2 - b^2 + a^2 + d^2)/(2*c*a); 

% Coefficients A, B, C
A = cos(q4) - K1_inv - K2_inv*cos(q4) + K3_inv;
B = -2*sin(q4);
C = K1_inv - (K2_inv + 1)*cos(q4) + K3_inv;

disc = B^2 - 4*A*C;

% คำนวณ 2 คำตอบของ q2
% Sol 1: ปกติจะเป็น Open/Parallel
q2_sol_A = 2*atan((-B - sqrt(disc))/(2*A));
% Sol 2: ปกติจะเป็น Crossed
q2_sol_B = 2*atan((-B + sqrt(disc))/(2*A));

% ==========================================
% 3. SORTING CASES (แยก Case บน/ล่าง)
% ==========================================
% เช็คว่าคำตอบไหนทำให้ Link 2 (สีฟ้า) ชี้ขึ้น หรือ ชี้ลง
y_check_A = a * sin(q2_sol_A);
y_check_B = a * sin(q2_sol_B);

if y_check_A > y_check_B
    % ถ้า A อยู่สูงกว่า -> A คือ Case Up
    q2_case_up   = q2_sol_A;
    q2_case_down = q2_sol_B;
else
    % ถ้า B อยู่สูงกว่า -> B คือ Case Up
    q2_case_up   = q2_sol_B;
    q2_case_down = q2_sol_A;
end

% ==========================================
% 4. FIND THETA 3 (Vector Method - ชัวร์ที่สุด)
% ==========================================
% หลักการ: วงรอบ R2 + R3 = R1 + R4
% ดังนั้น R3 = R1 + R4 - R2
% เราจะใช้ atan2 หาดมุมของ Vector R3 โดยตรง

% --- คำนวณ Case 1: Cyan DOWN (สีฟ้าลง) ---
Vec_Ground = d;
Vec_Brown  = c * exp(j*q4);
Vec_Cyan_D = a * exp(j*q2_case_down);

Vec_Blue_D = Vec_Ground + Vec_Brown - Vec_Cyan_D; % Vector ของ Link 3
q3_case_down = angle(Vec_Blue_D); % ดึงค่ามุมออกมาเลย (วิธีนี้ปิดลูปแน่นอน)

% --- คำนวณ Case 2: Cyan UP (สีฟ้าขึ้น) ---
Vec_Cyan_U = a * exp(j*q2_case_up);

Vec_Blue_U = Vec_Ground + Vec_Brown - Vec_Cyan_U; % Vector ของ Link 3
q3_case_up = angle(Vec_Blue_U);   % ดึงค่ามุมออกมาเลย


% แปลงเป็น Global Degrees
q2_glob_down = rad2deg(q2_case_down + theta1);
q3_glob_down = rad2deg(q3_case_down + theta1);

q2_glob_up   = rad2deg(q2_case_up + theta1);
q3_glob_up   = rad2deg(q3_case_up + theta1);


% ==========================================
% 5. PLOTTING
% ==========================================
color_Pink = [1, 0.07, 0.57];
color_Cyan = [0, 1, 1];
color_Blue = [0, 0, 1];
color_Brown = [0.6, 0.4, 0.2];

RO4O2 = d*exp(j*theta1); 
O4x = real(RO4O2); O4y = imag(RO4O2);

% --- สร้าง Vector สำหรับ Plot ---
% Case Down
RA_D = a*exp(j*(q2_case_down + theta1));
RBA_D = b*exp(j*(q3_case_down + theta1));
RBO4_D = c*exp(j*(q4 + theta1));

Ax_D = real(RA_D); Ay_D = imag(RA_D);
Bx_D = real(RA_D + RBA_D); By_D = imag(RA_D + RBA_D);
O4B_D_x = real(RBO4_D); O4B_D_y = imag(RBO4_D);

% Case Up
RA_U = a*exp(j*(q2_case_up + theta1));
RBA_U = b*exp(j*(q3_case_up + theta1));
RBO4_U = c*exp(j*(q4 + theta1));

Ax_U = real(RA_U); Ay_U = imag(RA_U);
Bx_U = real(RA_U + RBA_U); By_U = imag(RA_U + RBA_U);
O4B_U_x = real(RBO4_U); O4B_U_y = imag(RBO4_U);


% --- FIGURE 1: Cyan DOWN (Brown Down) ---
figure(1); hold on;
% Ground
quiver(0, 0, O4x, O4y, 0, 'Color', color_Pink, 'LineWidth', 4, 'MaxHeadSize', 0.5);
% Link 2 (Cyan)
quiver(0, 0, Ax_D, Ay_D, 0, 'Color', color_Cyan, 'LineWidth', 3, 'MaxHeadSize', 0.5);
% Link 3 (Blue) - ต่อจาก A ไป B
quiver(Ax_D, Ay_D, Bx_D-Ax_D, By_D-Ay_D, 0, 'Color', color_Blue, 'LineWidth', 3, 'MaxHeadSize', 0.5);
% Link 4 (Brown) - ต่อจาก O4 ไป B
quiver(O4x, O4y, O4B_D_x, O4B_D_y, 0, 'Color', color_Brown, 'LineWidth', 3, 'MaxHeadSize', 0.5);
axis equal; grid on;
title(['Case 1: Cyan DOWN, Brown DOWN (\theta_4 = ' num2str(q4_global_deg) ')']);
xlabel('x'); ylabel('y');


% --- FIGURE 2: Cyan UP (Brown Down) ---
figure(2); hold on;
% Ground
quiver(0, 0, O4x, O4y, 0, 'Color', color_Pink, 'LineWidth', 4, 'MaxHeadSize', 0.5);
% Link 2 (Cyan) - อันนี้คืออันที่อยู่บน
quiver(0, 0, Ax_U, Ay_U, 0, 'Color', color_Cyan, 'LineWidth', 3, 'MaxHeadSize', 0.5);
% Link 3 (Blue) - ต่อจาก A ไป B
quiver(Ax_U, Ay_U, Bx_U-Ax_U, By_U-Ay_U, 0, 'Color', color_Blue, 'LineWidth', 3, 'MaxHeadSize', 0.5);
% Link 4 (Brown) - ต่อจาก O4 ไป B
quiver(O4x, O4y, O4B_U_x, O4B_U_y, 0, 'Color', color_Brown, 'LineWidth', 3, 'MaxHeadSize', 0.5);
axis equal; grid on;
title(['Case 2: Cyan UP, Brown DOWN (\theta_4 = ' num2str(q4_global_deg) ')']);
xlabel('x'); ylabel('y');

% Display Results
fprintf('--- Results ---\n');
fprintf('Case 1 (Down): Theta 2 = %.4f deg, Theta 3 = %.4f deg\n', q2_glob_down, q3_glob_down);
fprintf('Case 2 (Up)  : Theta 2 = %.4f deg, Theta 3 = %.4f deg\n', q2_glob_up, q3_glob_up);
