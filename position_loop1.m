clear all
close all
clc

% ==========================================
% 1. PARAMETERS & INPUT
% ==========================================
L1 = 0.210; % Ground (Pink)
L2 = 0.118; % Crank (Cyan)
L3 = 0.210; % Coupler (Blue)
L4 = 0.118; % Rocker (Brown) - Input

d = L1;
a = L2;
b = L3;
c = L4;

theta4_deg = 102.05; 
offset_deg = 0.81;

offset = deg2rad(offset_deg);
% Global Input Angle for Link 4 (Brown)
q4_global = deg2rad(theta4_deg); 
% Note: Link 4 vector starts at O4, so angle is relative to Ground at O4

% ==========================================
% 2. CALCULATION (Geometric/Vector Method)
% ==========================================
% วิธีนี้แม่นยำที่สุดสำหรับแก้ปัญหา "ทิศทางผิด"
% Step 1: หาจุดปลายของ Link 4 (จุด B) และจุด O2, O4
O2 = 0 + 0i;
O4 = d * exp(1i * offset);

% จุด B (ปลาย Link 4) อ้างอิงจาก O4
R4_vec = c * exp(1i * (q4_global)); % มุม Global
Point_B = O4 + R4_vec;

% Step 2: หาจุด A (ปลาย Link 2)
% จุด A ต้องอยู่ห่างจาก O2 เป็นระยะ 'a' (Link 2)
% และต้องอยู่ห่างจาก B เป็นระยะ 'b' (Link 3)
% นี่คือปัญหาหาจุดตัดวงกลม (Intersection of two circles)

dist_O2_B = abs(Point_B - O2); % ระยะห่างจาก O2 ถึง B
angle_O2_B = angle(Point_B - O2); % มุมของเส้นตรง O2-B

% กฎของโคไซน์ (Law of Cosines) บนสามเหลี่ยม O2-A-B
% a^2 + dist^2 - b^2 = 2 * a * dist * cos(alpha)
cos_alpha = (a^2 + dist_O2_B^2 - b^2) / (2 * a * dist_O2_B);

% ตรวจสอบว่าเป็นไปได้หรือไม่ (Real Mechanism Check)
if abs(cos_alpha) > 1
    disp('Error: Mechanism cannot be assembled with these lengths/angles.');
    return;
end

alpha = acos(cos_alpha);

% คำตอบมี 2 กรณี (Open และ Crossed)
% Case 1: Theta 2 (Open) -> ลบมุม alpha (ตาม convention ของลูปนี้)
theta2_sol1 = angle_O2_B - alpha; 

% Case 2: Theta 2 (Crossed) -> บวกมุม alpha
theta2_sol2 = angle_O2_B + alpha;

% คำนวณ Theta 3 ตาม Vector Loop
% R3 = B - A
Point_A1 = a * exp(1i * theta2_sol1);
R3_vec1 = Point_B - Point_A1;
theta3_sol1 = angle(R3_vec1);

Point_A2 = a * exp(1i * theta2_sol2);
R3_vec2 = Point_B - Point_A2;
theta3_sol2 = angle(R3_vec2);

% --- กำหนดค่า Output (สลับตามความเหมาะสมของรูปกราฟ) ---
% ปกติ Sol1 จะเป็น Open สำหรับ Configuration นี้
T2_Open = theta2_sol1;
T3_Open = theta3_sol1;

T2_Cross = theta2_sol2;
T3_Cross = theta3_sol2;

% ==========================================
% 3. PLOTTING
% ==========================================
% Vector Definitions for Plotting

% Ground
R_Ground = O4; % Vector from O2 to O4

% --- Case 1: Open ---
R2_Op = a * exp(1i * T2_Open);
R4_Op = R4_vec; % จาก O4
R3_Op = Point_B - R2_Op;

% --- Case 2: Crossed ---
R2_Cr = a * exp(1i * T2_Cross);
R4_Cr = R4_vec;
R3_Cr = Point_B - R2_Cr;

figure(1); clf;

% Subplot 1: Open Case
subplot(1,2,1); hold on; grid on; axis equal;
title('Case 1: Open (Parallelogram)');
% Plot Ground (Pink)
plot([0 real(O4)], [0 imag(O4)], 'm-', 'LineWidth', 2);
% Plot Link 2 (Cyan)
plot([0 real(R2_Op)], [0 imag(R2_Op)], 'c-', 'LineWidth', 2);
% Plot Link 4 (Brown) - Start at O4
plot([real(O4) real(O4)+real(R4_Op)], [imag(O4) imag(O4)+imag(R4_Op)], 'Color', [0.6 0.3 0], 'LineWidth', 2);
% Plot Link 3 (Blue) - Connect A to B
plot([real(R2_Op) real(Point_B)], [imag(R2_Op) imag(Point_B)], 'b-', 'LineWidth', 2);

% Subplot 2: Crossed Case
subplot(1,2,2); hold on; grid on; axis equal;
title('Case 2: Crossed');
% Plot Ground (Pink)
plot([0 real(O4)], [0 imag(O4)], 'm-', 'LineWidth', 2);
% Plot Link 2 (Cyan)
plot([0 real(R2_Cr)], [0 imag(R2_Cr)], 'c-', 'LineWidth', 2);
% Plot Link 4 (Brown)
plot([real(O4) real(O4)+real(R4_Cr)], [imag(O4) imag(O4)+imag(R4_Cr)], 'Color', [0.6 0.3 0], 'LineWidth', 2);
% Plot Link 3 (Blue)
plot([real(R2_Cr) real(Point_B)], [imag(R2_Cr) imag(Point_B)], 'b-', 'LineWidth', 2);

% --- DISPLAY RESULTS ---
disp('======================================');
disp(['Loop 1 Analysis (Input Theta4 = ' num2str(theta4_deg) ' deg)']);
disp('======================================');
disp('--- CASE 1 (OPEN) ---');
disp(['  Link 2 (Cyan):  ', num2str(rad2deg(T2_Open) )]); % Global Angle
disp(['  Link 3 (Blue):  ', num2str(rad2deg(T3_Open) )]);
disp(' ');
disp('--- CASE 2 (CROSSED) ---');
disp(['  Link 2 (Cyan):  ', num2str(rad2deg(T2_Cross) )]);
disp(['  Link 3 (Blue):  ', num2str(rad2deg(T3_Cross) )]);
