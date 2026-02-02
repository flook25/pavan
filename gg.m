clear all
close all
clc

% ==========================================
% 1. PARAMETERS & INPUT
% ==========================================
L1 = 0.210; % Ground (Pink)
L2 = 0.180; % Crank (Green) - Output
L3 = 0.180; % Coupler (Yellow)
L4 = 0.118; % Rocker (Grey) - Input

d = L1;
a = L2;
b = L3;
c = L4;

theta_grey_deg = 206.3841; 
offset_deg = 0.81;

offset = deg2rad(offset_deg);
q4_global = deg2rad(theta_grey_deg);
q4_local = q4_global - offset;

% ==========================================
% 2. CALCULATION (Same Correct Logic)
% ==========================================
% Step 1: Inverted Frame
q_in_calc = q4_local - pi;

% Step 2: Constants (Input=c, Output=a)
K1 = d/c;
K2 = d/a;
K3 = (c^2 - b^2 + a^2 + d^2)/(2*c*a);
K4 = d/b;
K5 = (c^2 - d^2 - a^2 - b^2)/(2*c*b);

% Coefficients
A = cos(q_in_calc) - K1 - K2*cos(q_in_calc) + K3;
B = -2*sin(q_in_calc);
C = K1 - (K2+1)*cos(q_in_calc) + K3;

D_val = cos(q_in_calc) - K1 + K4*cos(q_in_calc) + K5;
E_val = -2*sin(q_in_calc);
F_val = K1 + (K4-1)*cos(q_in_calc) + K5;

% Roots
det_AC = sqrt(B^2 - 4*A*C);
det_DF = sqrt(E_val^2 - 4*D_val*F_val);

% Case 1
q2_loc_1 = 2*atan2((-B - det_AC), (2*A));
q3_loc_1 = 2*atan2((-E_val - det_DF), (2*D_val));

% Case 2
q2_loc_2 = 2*atan2((-B + det_AC), (2*A));
q3_loc_2 = 2*atan2((-E_val + det_DF), (2*D_val));

% Step 3: Global Angles (Correct Values)
T2_Open = q2_loc_1 + pi + offset;
T3_Open = q3_loc_1 + pi + offset;

T2_Cross = q2_loc_2 + pi + offset;
T3_Cross = q3_loc_2 + pi + offset;

T4_Global = q4_global;

% ==========================================
% 3. DISPLAY RESULTS (Keep Values Same)
% ==========================================
disp('======================================');
disp(['LOOP 3 RESULTS (Input Grey = ' num2str(theta_grey_deg) ')']);
disp('======================================');
disp('--- CASE 1 (Open) ---');
disp(['  Green  (Theta 2): ', num2str(rad2deg(T2_Open))]);
disp(['  Yellow (Theta 3): ', num2str(rad2deg(T3_Open))]);
disp(' ');
disp('--- CASE 2 (Crossed) ---');
disp(['  Green  (Theta 2): ', num2str(rad2deg(T2_Cross))]);
disp(['  Yellow (Theta 3): ', num2str(rad2deg(T3_Cross))]);

% ==========================================
% 4. PLOTTING (SWAPPED VIEW)
% ==========================================
% Trick: เราจะพล็อตโดยให้ O4 เป็นจุด (0,0) และหมุนมุมมอง 180 องศา
% เพื่อให้รูป "สลับจากซ้ายมาขวา" (Mirror/Flip View)
% โดยการลบ pi ออกจากมุมทั้งหมดในการ Plot

Rot_View = -pi; % หมุนภาพ 180 องศา

% จุด O4 อยู่ที่ (0,0)
O4_Plot = 0; 
% จุด O2 อยู่ที่ระยะ d (หมุนมุม offset + view)
O2_Plot = d * exp(1j * (offset + Rot_View));

% สร้างเวกเตอร์สำหรับ Plot (Apply Rot_View)
% Input Link (Grey) ที่ O4
Vec_Grey_Plot = c * exp(1j * (T4_Global + Rot_View));

% --- Case 1 Vectors ---
% Output Link (Green) ที่ O2
Vec_Green_Op_Plot = a * exp(1j * (T2_Open + Rot_View));
% หาจุดปลาย
Point_B_Op = O4_Plot + Vec_Grey_Plot;     % ปลาย Grey
Point_A_Op = O2_Plot + Vec_Green_Op_Plot; % ปลาย Green
% Yellow
Vec_Yellow_Op_Plot = Point_B_Op - Point_A_Op;

% --- Case 2 Vectors ---
Vec_Green_Cr_Plot = a * exp(1j * (T2_Cross + Rot_View));
Point_B_Cr = O4_Plot + Vec_Grey_Plot;
Point_A_Cr = O2_Plot + Vec_Green_Cr_Plot;
Vec_Yellow_Cr_Plot = Point_B_Cr - Point_A_Cr;


figure(3); clf;

% Subplot 1: Open
subplot(1,2,1); hold on; grid on; axis equal;
title('Loop 3: Case 1 (Swapped View)');
% Ground (O4 -> O2)
plot([real(O4_Plot) real(O2_Plot)], [imag(O4_Plot) imag(O2_Plot)], 'm-', 'LineWidth', 2); 
% Grey (Input at O4)
quiver(real(O4_Plot), imag(O4_Plot), real(Vec_Grey_Plot), imag(Vec_Grey_Plot), 0, 'Color', [0.5 0.5 0.5], 'LineWidth', 2, 'MaxHeadSize',0.5); 
% Green (Output at O2)
quiver(real(O2_Plot), imag(O2_Plot), real(Vec_Green_Op_Plot), imag(Vec_Green_Op_Plot), 0, 'g', 'LineWidth', 2, 'MaxHeadSize',0.5); 
% Yellow
plot([real(Point_A_Op) real(Point_B_Op)], [imag(Point_A_Op) imag(Point_B_Op)], 'y-', 'LineWidth', 2);

% Subplot 2: Crossed
subplot(1,2,2); hold on; grid on; axis equal;
title('Loop 3: Case 2 (Swapped View)');
% Ground
plot([real(O4_Plot) real(O2_Plot)], [imag(O4_Plot) imag(O2_Plot)], 'm-', 'LineWidth', 2); 
% Grey
quiver(real(O4_Plot), imag(O4_Plot), real(Vec_Grey_Plot), imag(Vec_Grey_Plot), 0, 'Color', [0.5 0.5 0.5], 'LineWidth', 2, 'MaxHeadSize',0.5); 
% Green
quiver(real(O2_Plot), imag(O2_Plot), real(Vec_Green_Cr_Plot), imag(Vec_Green_Cr_Plot), 0, 'g', 'LineWidth', 2, 'MaxHeadSize',0.5); 
% Yellow
plot([real(Point_A_Cr) real(Point_B_Cr)], [imag(Point_A_Cr) imag(Point_B_Cr)], 'y-', 'LineWidth', 2);
