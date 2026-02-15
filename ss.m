clear all
close all
clc


% 1. GLOBAL PARAMETERS & INPUTS

offset_deg = 0.81;
offset = deg2rad(offset_deg);
d_global = 0.210; % Shared Ground (Pink)

% INPUTS 
w_brown = 4.5;      % Velocity (rad/sec) of Input Link (Brown)
alpha_brown = 3.5;    % Acceleration (rad/sec^2) of Input Link (Brown)

% --- POINT P PARAMETERS ---
p_dist = 0.120;     % Distance 120 mm on Blue Link (Coupler)


% 2. POSITION ANALYSIS................................................


% LOOP 1: 
L1 = 0.210; L2 = 0.118; L3 = 0.210; L4 = 0.118;
d = L1; a = L2; b = L3; c = L4;

theta4_deg = 102.05; 
q_in_L1 = deg2rad(theta4_deg); 

K1=d/c; K2=d/a; K3=(c^2-b^2+a^2+d^2)/(2*c*a); K4=d/b; K5=(a^2-d^2-c^2-b^2)/(2*c*b);

A = cos(q_in_L1) - K1 - K2*cos(q_in_L1) + K3;
B = -2*sin(q_in_L1);
C = K1 - (K2+1)*cos(q_in_L1) + K3;
D_val = cos(q_in_L1) - K1 + K4*cos(q_in_L1) + K5;
E_val = -2*sin(q_in_L1);
F_val = K1 + (K4-1)*cos(q_in_L1) + K5;

det_AC = sqrt(max(0, B^2 - 4*A*C));
det_DF = sqrt(max(0, E_val^2 - 4*D_val*F_val));

% Loop 1: Case 1 (Open)
q2_loc_1 = 2*atan2((-B - det_AC), (2*A));
q3_loc_1 = 2*atan2((-E_val - det_DF), (2*D_val));
T2_Open_L1 = q2_loc_1 + pi + offset; 
T3_Open_L1 = q3_loc_1 + pi + offset; 
T4_Global_L1 = q_in_L1 + pi + offset; 

% Loop 1: Case 2 (Crossed)
q2_loc_2 = 2*atan2((-B + det_AC), (2*A));
q3_loc_2 = 2*atan2((-E_val + det_DF), (2*D_val));
T2_Cross_L1 = q2_loc_2 + pi + offset; 
T3_Cross_L1 = q3_loc_2 + pi + offset; 

% LOOP 2: CALCULATION (Input: Cyan L2 from Loop 1 + 180) 
L1 = 0.210; L2 = 0.118; L3 = 0.210; L4 = 0.118;
d = L1; a = L2; b = L3; c = L4;

theta2_global_Open = rad2deg(T2_Open_L1) + 180;
q_in_Open = deg2rad(theta2_global_Open) - offset;

theta2_global_Cross = rad2deg(T2_Cross_L1) + 180;
q_in_Cross = deg2rad(theta2_global_Cross) - offset;

K1=d/a; K2=d/c; K3=(a^2-b^2+c^2+d^2)/(2*a*c); K4=d/b; K5=(c^2-d^2-a^2-b^2)/(2*a*b);

% Loop 2: Case 1 (Open)
A=cos(q_in_Open)-K1-K2*cos(q_in_Open)+K3;
B=-2*sin(q_in_Open);
C=K1-(K2+1)*cos(q_in_Open)+K3;
D_val=cos(q_in_Open)-K1+K4*cos(q_in_Open)+K5;
E_val=-2*sin(q_in_Open);
F_val=K1+(K4-1)*cos(q_in_Open)+K5;
det_AC_2 = sqrt(max(0, B^2 - 4*A*C));
det_DF_2 = sqrt(max(0, E_val^2 - 4*D_val*F_val));

q4_loc_1 = 2*atan2((-B - det_AC_2), (2*A));
q3_loc_1 = 2*atan2((-E_val - det_DF_2), (2*D_val));
T4_Open_L2 = q4_loc_1 + offset; 
T3_Open_L2 = q3_loc_1 + offset; 

% Loop 2: Case 2 (Crossed)
A=cos(q_in_Cross)-K1-K2*cos(q_in_Cross)+K3;
B=-2*sin(q_in_Cross); C=K1-(K2+1)*cos(q_in_Cross)+K3;
D_val=cos(q_in_Cross)-K1+K4*cos(q_in_Cross)+K5;
E_val=-2*sin(q_in_Cross); 
F_val=K1+(K4-1)*cos(q_in_Cross)+K5;
det_AC_2 = sqrt(max(0, B^2 - 4*A*C));
det_DF_2 = sqrt(max(0, E_val^2 - 4*D_val*F_val));

q4_loc_2 = 2*atan2((-B + det_AC_2), (2*A));
q3_loc_2 = 2*atan2((-E_val + det_DF_2), (2*D_val));
T4_Cross_L2 = q4_loc_2 + offset; 
T3_Cross_L2 = q3_loc_2 + offset; 

% LOOP 3: CALCULATION (Inverted Input: Grey L4 from Loop 2)
L1 = 0.210; L2 = 0.180; L3 = 0.180; L4 = 0.118;
d = L1; a = L2; b = L3; c = L4;

K1=d/c; K2=d/a; K3=(c^2-b^2+a^2+d^2)/(2*c*a); K4=d/b; K5=(c^2-d^2-a^2-b^2)/(2*c*b);

% Loop 3: Case 1 (Open)
q4_local_Open = T4_Open_L2 - offset;
q_in_calc_Open = q4_local_Open - pi; 
A=cos(q_in_calc_Open)-K1-K2*cos(q_in_calc_Open)+K3; 
B=-2*sin(q_in_calc_Open);
C=K1-(K2+1)*cos(q_in_calc_Open)+K3;
det_AC_3 = sqrt(max(0, B^2 - 4*A*C)); 
q2_loc_SWAP = 2*atan2((-B + det_AC_3), (2*A));
R_Coupler_Vec = d + a*exp(1i*q2_loc_SWAP) - c*exp(1i*q_in_calc_Open);
q3_loc_SWAP = angle(R_Coupler_Vec);
T2_Open_L3 = q2_loc_SWAP + pi + offset; 
T3_Open_L3 = q3_loc_SWAP + pi + offset; 

% Loop 3: Case 2 (Crossed)
q4_local_Cross = T4_Cross_L2 - offset;
q_in_calc_Cross = q4_local_Cross - pi; 
A=cos(q_in_calc_Cross)-K1-K2*cos(q_in_calc_Cross)+K3;
B=-2*sin(q_in_calc_Cross); 
C=K1-(K2+1)*cos(q_in_calc_Cross)+K3;
det_AC_3 = sqrt(max(0, B^2 - 4*A*C));
q2_loc_SWAP2 = 2*atan2((-B - det_AC_3), (2*A));
R_Coupler_Vec = d + a*exp(1i*q2_loc_SWAP2) - c*exp(1i*q_in_calc_Cross);
q3_loc_SWAP2 = angle(R_Coupler_Vec);
T2_Cross_L3 = q2_loc_SWAP2 + pi + offset; 
T3_Cross_L3 = q3_loc_SWAP2 + pi + offset; 


% 3. VELOCITY ANALYSIS.........................................


% --- SYSTEM CROSSED ---
% Loop 1 (Crossed): Inverted
L2=0.118; L3=0.210; L4=0.118;
a=L2; b=L3; c=L4;
t2=T2_Cross_L1;
t3=T3_Cross_L1; 
t4=T4_Global_L1;
w4_L1 = w_brown;
w_Cyan_Cross = (c*w4_L1/a)*(sin(t4-t3)/sin(t2-t3));
w_Blue_Cross = (c*w4_L1/b)*(sin(t2-t4)/sin(t2-t3));

% Loop 2 (Crossed): Standard
L2=0.118; L3=0.210; L4=0.118;
a=L2; b=L3; c=L4;
t2=T2_Cross_L1+pi;
t3=T3_Cross_L2;
t4=T4_Cross_L2;
w2_L2 = w_Cyan_Cross;
w_Red_Cross  = (a*w2_L2/b)*(sin(t4-t2)/sin(t3-t4));
w_Grey_Cross = (a*w2_L2/c)*(sin(t2-t3)/sin(t4-t3));

% Loop 3 (Crossed): Inverted
L2=0.180; L3=0.180; L4=0.118; 
a=L2; b=L3; c=L4;
t2=T2_Cross_L3;
t3=T3_Cross_L3;
t4=T4_Cross_L2;
w4_L3 = w_Grey_Cross;
w_Green_Cross  = (c*w4_L3/a)*(sin(t4-t3)/sin(t2-t3));
w_Yellow_Cross = (c*w4_L3/b)*(sin(t2-t4)/sin(t2-t3));

% --- SYSTEM OPEN ---
% Loop 1 (Open): Inverted
L2=0.118; L3=0.210; L4=0.118;
a=L2; b=L3; c=L4;
t2=T2_Open_L1;
t3=T3_Open_L1; 
t4=T4_Global_L1;
w4_L1 = w_brown;
w_Cyan_Open = (c*w4_L1/a)*(sin(t4-t3)/sin(t2-t3));
w_Blue_Open = (c*w4_L1/b)*(sin(t2-t4)/sin(t2-t3));

% Loop 2 (Open): Standard
L2=0.118; L3=0.210; L4=0.118; 
a=L2; b=L3; c=L4;
t2=T2_Open_L1+pi; 
t3=T3_Open_L2; 
t4=T4_Open_L2;
w2_L2 = w_Cyan_Open;
w_Red_Open  = (a*w2_L2/b)*(sin(t4-t2)/sin(t3-t4));
w_Grey_Open = (a*w2_L2/c)*(sin(t2-t3)/sin(t4-t3));

% Loop 3 (Open): Inverted
L2=0.180; L3=0.180; L4=0.118;
a=L2; b=L3; c=L4;
t2=T2_Open_L3; 
t3=T3_Open_L3; 
t4=T4_Open_L2;
w4_L3 = w_Grey_Open;
w_Green_Open  = (c*w4_L3/a)*(sin(t4-t3)/sin(t2-t3));
w_Yellow_Open = (c*w4_L3/b)*(sin(t2-t4)/sin(t2-t3));



% 4. ACCELERATION ANALYSIS........................................

% --- SYSTEM CROSSED ---

% Loop 1 (Crossed) - INVERTED LOGIC
L_In=0.118; L_Cou=0.210; L_Out=0.118; 
a=L_In; b=L_Cou; c=L_Out;
q2=T4_Global_L1; q3=T3_Cross_L1; q4=T2_Cross_L1;
w2=w_brown; w3=w_Blue_Cross; w4=w_Cyan_Cross;
alp2=alpha_brown;

A_val = c*sin(q4); B_val = b*sin(q3);
C_val = a*alp2*sin(q2) + a*w2^2*cos(q2) + b*w3^2*cos(q3) - c*w4^2*cos(q4);
D_val = c*cos(q4); E_val = b*cos(q3);
F_val = a*alp2*cos(q2) - a*w2^2*sin(q2) - b*w3^2*sin(q3) + c*w4^2*sin(q4);

alpha_Cyan_Cross = (C_val*E_val - B_val*F_val)/(A_val*E_val - B_val*D_val); 
alpha_Blue_Cross = (C_val*D_val - A_val*F_val)/(A_val*E_val - B_val*D_val); 

% Loop 2 (Crossed) - STANDARD LOGIC
L_In=0.118; L_Cou=0.210; L_Out=0.118; 
a=L_In; b=L_Cou; c=L_Out;
q2=T2_Cross_L1+pi; q3=T3_Cross_L2; q4=T4_Cross_L2;
w2=w_Cyan_Cross; w3=w_Red_Cross; w4=w_Grey_Cross;
alp2=alpha_Cyan_Cross;

A_val = c*sin(q4); B_val = b*sin(q3);
C_val = a*alp2*sin(q2) + a*w2^2*cos(q2) + b*w3^2*cos(q3) - c*w4^2*cos(q4);
D_val = c*cos(q4); E_val = b*cos(q3);
F_val = a*alp2*cos(q2) - a*w2^2*sin(q2) - b*w3^2*sin(q3) + c*w4^2*sin(q4);
alpha_Grey_Cross = (C_val*E_val - B_val*F_val)/(A_val*E_val - B_val*D_val); 
alpha_Red_Cross  = (C_val*D_val - A_val*F_val)/(A_val*E_val - B_val*D_val); 

% Loop 3 (Crossed) - INVERTED LOGIC
L_In=0.118; L_Cou=0.180; L_Out=0.180; 
a=L_In; b=L_Cou; c=L_Out;
q2=T4_Cross_L2; q3=T3_Cross_L3; q4=T2_Cross_L3;
w2=w_Grey_Cross; w3=w_Yellow_Cross; w4=w_Green_Cross;
alp2=alpha_Grey_Cross;

A_val = c*sin(q4); B_val = b*sin(q3);
C_val = a*alp2*sin(q2) + a*w2^2*cos(q2) + b*w3^2*cos(q3) - c*w4^2*cos(q4);
D_val = c*cos(q4); E_val = b*cos(q3);
F_val = a*alp2*cos(q2) - a*w2^2*sin(q2) - b*w3^2*sin(q3) + c*w4^2*sin(q4);
alpha_Green_Cross = (C_val*E_val - B_val*F_val)/(A_val*E_val - B_val*D_val); 
alpha_Yellow_Cross = (C_val*D_val - A_val*F_val)/(A_val*E_val - B_val*D_val); 

% --- SYSTEM OPEN ---

% Loop 1 (Open) - INVERTED LOGIC
L_In=0.118; L_Cou=0.210; L_Out=0.118;
a=L_In; b=L_Cou; c=L_Out;
q2=T4_Global_L1; q3=T3_Open_L1; q4=T2_Open_L1;
w2=w_brown; w3=w_Blue_Open; w4=w_Cyan_Open;
alp2=alpha_brown;

A_val = c*sin(q4); B_val = b*sin(q3);
C_val = a*alp2*sin(q2) + a*w2^2*cos(q2) + b*w3^2*cos(q3) - c*w4^2*cos(q4);
D_val = c*cos(q4); E_val = b*cos(q3);
F_val = a*alp2*cos(q2) - a*w2^2*sin(q2) - b*w3^2*sin(q3) + c*w4^2*sin(q4);
alpha_Cyan_Open = (C_val*E_val - B_val*F_val)/(A_val*E_val - B_val*D_val);
alpha_Blue_Open = (C_val*D_val - A_val*F_val)/(A_val*E_val - B_val*D_val);

% Loop 2 (Open) - STANDARD LOGIC
L_In=0.118; L_Cou=0.210; L_Out=0.118;
a=L_In; b=L_Cou; c=L_Out;
q2=T2_Open_L1+pi; q3=T3_Open_L2; q4=T4_Open_L2;
w2=w_Cyan_Open; w3=w_Red_Open; w4=w_Grey_Open;
alp2=alpha_Cyan_Open;

A_val = c*sin(q4); B_val = b*sin(q3);
C_val = a*alp2*sin(q2) + a*w2^2*cos(q2) + b*w3^2*cos(q3) - c*w4^2*cos(q4);
D_val = c*cos(q4); E_val = b*cos(q3);
F_val = a*alp2*cos(q2) - a*w2^2*sin(q2) - b*w3^2*sin(q3) + c*w4^2*sin(q4);
alpha_Grey_Open = (C_val*E_val - B_val*F_val)/(A_val*E_val - B_val*D_val);
alpha_Red_Open  = (C_val*D_val - A_val*F_val)/(A_val*E_val - B_val*D_val);

% Loop 3 (Open) - INVERTED LOGIC
L_In=0.118; L_Cou=0.180; L_Out=0.180;
a=L_In; b=L_Cou; c=L_Out;
q2=T4_Open_L2; q3=T3_Open_L3; q4=T2_Open_L3;
w2=w_Grey_Open; w3=w_Yellow_Open; w4=w_Green_Open;
alp2=alpha_Grey_Open;

A_val = c*sin(q4); B_val = b*sin(q3);
C_val = a*alp2*sin(q2) + a*w2^2*cos(q2) + b*w3^2*cos(q3) - c*w4^2*cos(q4);
D_val = c*cos(q4); E_val = b*cos(q3);
F_val = a*alp2*cos(q2) - a*w2^2*sin(q2) - b*w3^2*sin(q3) + c*w4^2*sin(q4);
alpha_Green_Open = (C_val*E_val - B_val*F_val)/(A_val*E_val - B_val*D_val);
alpha_Yellow_Open = (C_val*D_val - A_val*F_val)/(A_val*E_val - B_val*D_val);


% 5. PLOTTING & POINT P ACCELERATION (VISUAL-BASED)........................

figure(1); clf;
AccScale = 200; 
Rot = exp(1i * offset);
R1 = d_global * exp(1i * offset); % Shared Ground


% PLOT 1: SYSTEM CROSSED

subplot(1,2,1); hold on; grid on; axis equal;
title('System Configuration 2 (Crossed)');
plot([0 real(R1)], [0 imag(R1)], 'm-', 'LineWidth', 2); % Ground

% DRAW LOOP 1 VECTORS
L2=0.118; L4=0.118;
V_Cyan_L1 = L2 * exp(1i * T2_Cross_L1);
V_Brown_L1 = L4 * exp(1i * T4_Global_L1);
% Calculate Blue Vector based on Visual drawing logic to prevent 180 flip error
V_Blue_L1 = (R1 + V_Brown_L1) - V_Cyan_L1; 

quiver(0, 0, real(V_Cyan_L1), imag(V_Cyan_L1), 0, 'c', 'LineWidth', 2, 'MaxHeadSize',0.5); 
quiver(real(R1), imag(R1), real(V_Brown_L1), imag(V_Brown_L1), 0, 'Color',[0.6 0.3 0], 'LineWidth', 2, 'MaxHeadSize',0.5); 
quiver(real(V_Cyan_L1), imag(V_Cyan_L1), real(V_Blue_L1), imag(V_Blue_L1), 0, 'b', 'LineWidth', 2, 'MaxHeadSize',0.5); 

% CALCULATE POINT P ACCELERATION 
% 1. Get the exact angle of the blue link as drawn
theta_Blue_Visual = angle(V_Blue_L1);

% 2. Get Acceleration of the Base of Blue Link (Tip of Cyan)
%    A_Cyan_Tip = alpha * j * r - w^2 * r
A_Base_Cyan = L2*alpha_Cyan_Cross*1i*exp(1i*T2_Cross_L1) - L2*w_Cyan_Cross^2*exp(1i*T2_Cross_L1);

% 3. Calculate Relative Acceleration of P
%    Using visual angle ensures direction is correct
A_P_Normal = -p_dist * w_Blue_Cross^2 * exp(1i*theta_Blue_Visual);
A_P_Tang   = p_dist * alpha_Blue_Cross * 1i * exp(1i*theta_Blue_Visual);

% 4. Total Linear Acceleration of P
Acc_P_Total_Cross = A_Base_Cyan + A_P_Normal + A_P_Tang;

% PLOT POINT P 
Pos_P_Cross = V_Cyan_L1 + p_dist * exp(1i * theta_Blue_Visual);
plot(real(Pos_P_Cross), imag(Pos_P_Cross), 'bo', 'MarkerFaceColor', 'b'); 
text(real(Pos_P_Cross), imag(Pos_P_Cross)+0.02, 'P');
quiver(real(Pos_P_Cross), imag(Pos_P_Cross), ...
       real(Acc_P_Total_Cross)/AccScale, imag(Acc_P_Total_Cross)/AccScale, ...
       0, 'k', 'LineWidth', 2, 'MaxHeadSize', 0.5);

% Draw Acceleration Vectors for joints (Optional/Existing)
AC_Brown = L4*alpha_brown*1i*exp(1i*T4_Global_L1) - L4*w_brown^2*exp(1i*T4_Global_L1);
quiver(real(R1), imag(R1), real(AC_Brown)/AccScale, imag(AC_Brown)/AccScale, 0, 'k', 'LineWidth', 1);
quiver(0, 0, real(A_Base_Cyan)/AccScale, imag(A_Base_Cyan)/AccScale, 0, 'k', 'LineWidth', 1);

% Draw Other Loops (Visual Only)
V_Cyan_L2 = L2 * exp(1i * deg2rad(rad2deg(T2_Cross_L1) + 180)); 
V_Grey_L2 = L4 * exp(1i * T4_Cross_L2);
V_Red_L2 = (R1 + V_Grey_L2) - V_Cyan_L2;
quiver(0, 0, real(V_Cyan_L2), imag(V_Cyan_L2), 0, 'c', 'LineWidth', 2, 'MaxHeadSize',0.5); 
quiver(real(R1), imag(R1), real(V_Grey_L2), imag(V_Grey_L2), 0, 'Color', [0.5 0.5 0.5], 'LineWidth', 2, 'MaxHeadSize',0.5); 
quiver(real(V_Cyan_L2), imag(V_Cyan_L2), real(V_Red_L2), imag(V_Red_L2), 0, 'r', 'LineWidth', 2, 'MaxHeadSize',0.5); 

L2=0.180; L4=0.118;
V_Grey_sys = L4 * exp(1i * T4_Cross_L2); 
V_Green_sys = L2 * exp(1i * T2_Cross_L3); 
V_Yellow_sys = (R1 + V_Grey_sys) - V_Green_sys;
quiver(0, 0, real(V_Green_sys), imag(V_Green_sys), 0, 'g', 'LineWidth', 2, 'MaxHeadSize',0.5); 
quiver(real(V_Green_sys), imag(V_Green_sys), real(V_Yellow_sys), imag(V_Yellow_sys), 0, 'y', 'LineWidth', 2, 'MaxHeadSize',0.5); 
AC_Green = L2*alpha_Green_Cross*1i*exp(1i*T2_Cross_L3) - L2*w_Green_Cross^2*exp(1i*T2_Cross_L3);
quiver(0, 0, real(AC_Green)/AccScale, imag(AC_Green)/AccScale, 0, 'k', 'LineWidth', 1);


% PLOT 2: SYSTEM OPEN

subplot(1,2,2); hold on; grid on; axis equal;
title('System Configuration 1 (Open)');
plot([0 real(R1)], [0 imag(R1)], 'm-', 'LineWidth', 2); % Ground

%  DRAW LOOP 1 VECTORS
L2=0.118; L4=0.118;
V_Cyan_L1 = L2 * exp(1i * T2_Open_L1);
V_Brown_L1 = L4 * exp(1i * T4_Global_L1);
V_Blue_L1 = (R1 + V_Brown_L1) - V_Cyan_L1;

quiver(0, 0, real(V_Cyan_L1), imag(V_Cyan_L1), 0, 'c', 'LineWidth', 2, 'MaxHeadSize',0.5); 
quiver(real(R1), imag(R1), real(V_Brown_L1), imag(V_Brown_L1), 0, 'Color',[0.6 0.3 0], 'LineWidth', 2, 'MaxHeadSize',0.5); 
quiver(real(V_Cyan_L1), imag(V_Cyan_L1), real(V_Blue_L1), imag(V_Blue_L1), 0, 'b', 'LineWidth', 2, 'MaxHeadSize',0.5); 

% CALCULATE POINT P ACCELERATION (VISUAL-BASED)
theta_Blue_Visual_Open = angle(V_Blue_L1);

A_Base_Cyan = L2*alpha_Cyan_Open*1i*exp(1i*T2_Open_L1) - L2*w_Cyan_Open^2*exp(1i*T2_Open_L1);

A_P_Normal = -p_dist * w_Blue_Open^2 * exp(1i*theta_Blue_Visual_Open);
A_P_Tang   = p_dist * alpha_Blue_Open * 1i * exp(1i*theta_Blue_Visual_Open);

Acc_P_Total_Open = A_Base_Cyan + A_P_Normal + A_P_Tang;

% PLOT POINT P 
Pos_P_Open = V_Cyan_L1 + p_dist * exp(1i * theta_Blue_Visual_Open);
plot(real(Pos_P_Open), imag(Pos_P_Open), 'bo', 'MarkerFaceColor', 'b'); 
text(real(Pos_P_Open), imag(Pos_P_Open)+0.02, 'P');
quiver(real(Pos_P_Open), imag(Pos_P_Open), ...
       real(Acc_P_Total_Open)/AccScale, imag(Acc_P_Total_Open)/AccScale, ...
       0, 'k', 'LineWidth', 2, 'MaxHeadSize', 0.5);

% Draw Other Loops
V_Cyan_L2 = L2 * exp(1i * deg2rad(rad2deg(T2_Open_L1) + 180)); 
V_Grey_L2 = L4 * exp(1i * T4_Open_L2);
V_Red_L2 = (R1 + V_Grey_L2) - V_Cyan_L2;
quiver(0, 0, real(V_Cyan_L2), imag(V_Cyan_L2), 0, 'c', 'LineWidth', 2, 'MaxHeadSize',0.5); 
quiver(real(R1), imag(R1), real(V_Grey_L2), imag(V_Grey_L2), 0, 'Color', [0.5 0.5 0.5], 'LineWidth', 2, 'MaxHeadSize',0.5); 
quiver(real(V_Cyan_L2), imag(V_Cyan_L2), real(V_Red_L2), imag(V_Red_L2), 0, 'r', 'LineWidth', 2, 'MaxHeadSize',0.5); 

L2=0.180; L4=0.118;
V_Grey_sys = L4 * exp(1i * T4_Open_L2);
V_Green_sys = L2 * exp(1i * T2_Open_L3);
V_Yellow_sys = (R1 + V_Grey_sys) - V_Green_sys;
quiver(0, 0, real(V_Green_sys), imag(V_Green_sys), 0, 'g', 'LineWidth', 2, 'MaxHeadSize',0.5); 
quiver(real(V_Green_sys), imag(V_Green_sys), real(V_Yellow_sys), imag(V_Yellow_sys), 0, 'y', 'LineWidth', 2, 'MaxHeadSize',0.5);

AC_Green = L2*alpha_Green_Open*1i*exp(1i*T2_Open_L3) - L2*w_Green_Open^2*exp(1i*T2_Open_L3);
quiver(0, 0, real(AC_Green)/AccScale, imag(AC_Green)/AccScale, 0, 'k', 'LineWidth', 1);


%  DISPLAY RESULTS............................................................


disp(['FULL MECHANISM RESULTS (Input Theta4 = ' num2str(theta4_deg) ')']);
disp(['Input Velocity w_brown = ' num2str(w_brown) ' rad/s']);

disp('--- CROSSED CIRCUIT (Position & Velocity & Acceleration) ---');
disp('  L1 Blue  (In) :');
disp(['     Pos = ' num2str(rad2deg(T4_Global_L1), '%.2f') ' deg']);
disp(['     Vel = ' num2str(w_brown, '%.3f') ' rad/s']);
disp(['     Acc = ' num2str(alpha_brown, '%.3f') ' rad/s^2']);

disp('  L1 Cyan   (Out):');
disp(['     Pos = ' num2str(rad2deg(T2_Cross_L1), '%.2f') ' deg']);
disp(['     Vel = ' num2str(w_Cyan_Cross, '%.3f') ' rad/s']);
disp(['     Acc = ' num2str(alpha_Cyan_Cross, '%.3f') ' rad/s^2']);

disp('  ----------------');
disp('  L2 Grey   (Out):');
disp(['     Pos = ' num2str(rad2deg(T4_Cross_L2), '%.2f') ' deg']);
disp(['     Vel = ' num2str(w_Grey_Cross, '%.3f') ' rad/s']);
disp(['     Acc = ' num2str(alpha_Grey_Cross, '%.3f') ' rad/s^2']);

disp('  L2 Red    (Cou):');
disp(['     Pos = ' num2str(rad2deg(T3_Cross_L2), '%.2f') ' deg']);
disp(['     Vel = ' num2str(w_Red_Cross, '%.3f') ' rad/s']);
disp(['     Acc = ' num2str(alpha_Red_Cross, '%.3f') ' rad/s^2']);

disp('  ----------------');
disp('  L3 Green  (Out):');
disp(['     Pos = ' num2str(rad2deg(T2_Cross_L3), '%.2f') ' deg']);
disp(['     Vel = ' num2str(w_Green_Cross, '%.3f') ' rad/s']);
disp(['     Acc = ' num2str(alpha_Green_Cross, '%.3f') ' rad/s^2']);

disp('  L3 Yellow (Cou):');
disp(['     Pos = ' num2str(rad2deg(T3_Cross_L3), '%.2f') ' deg']);
disp(['     Vel = ' num2str(w_Yellow_Cross, '%.3f') ' rad/s']);
disp(['     Acc = ' num2str(alpha_Yellow_Cross, '%.3f') ' rad/s^2']);

disp('  ----------------');
disp('  POINT P (Blue Link 112mm):');
disp(['     Acc Magnitude = ' num2str(abs(Acc_P_Total_Cross), '%.3f') ' m/s^2']);
disp(['     Acc Angle     = ' num2str(rad2deg(angle(Acc_P_Total_Cross)), '%.2f') ' deg']);

disp(' ');
disp('--- OPEN CIRCUIT (Position & Velocity & Acceleration) ---');
disp('  L1 Blue (In) :');
disp(['     Pos = ' num2str(rad2deg(T4_Global_L1), '%.2f') ' deg']);
disp(['     Vel = ' num2str(w_brown, '%.3f') ' rad/s']);
disp(['     Acc = ' num2str(alpha_brown, '%.3f') ' rad/s^2']);

disp('  L1 Cyan   (Out):');
disp(['     Pos = ' num2str(rad2deg(T2_Open_L1), '%.2f') ' deg']);
disp(['     Vel = ' num2str(w_Cyan_Open, '%.3f') ' rad/s']);
disp(['     Acc = ' num2str(alpha_Cyan_Open, '%.3f') ' rad/s^2']);

disp('  ----------------');
disp('  L2 Grey   (Out):');
disp(['     Pos = ' num2str(rad2deg(T4_Open_L2), '%.2f') ' deg']);
disp(['     Vel = ' num2str(w_Grey_Open, '%.3f') ' rad/s']);
disp(['     Acc = ' num2str(alpha_Grey_Open, '%.3f') ' rad/s^2']);

disp('  L2 Red    (Cou):');
disp(['     Pos = ' num2str(rad2deg(T3_Open_L2), '%.2f') ' deg']);
disp(['     Vel = ' num2str(w_Red_Open, '%.3f') ' rad/s']);
disp(['     Acc = ' num2str(alpha_Red_Open, '%.3f') ' rad/s^2']);

disp('  ----------------');
disp('  L3 Green  (Out):');
disp(['     Pos = ' num2str(rad2deg(T2_Open_L3), '%.2f') ' deg']);
disp(['     Vel = ' num2str(w_Green_Open, '%.3f') ' rad/s']);
disp(['     Acc = ' num2str(alpha_Green_Open, '%.3f') ' rad/s^2']);

disp('  L3 Yellow (Cou):');
disp(['     Pos = ' num2str(rad2deg(T3_Open_L3), '%.2f') ' deg']);
disp(['     Vel = ' num2str(w_Yellow_Open, '%.3f') ' rad/s']);
disp(['     Acc = ' num2str(alpha_Yellow_Open, '%.3f') ' rad/s^2']);

disp('  ----------------');
disp('  POINT P (Blue Link 112mm):');
disp(['     Acc Magnitude = ' num2str(abs(Acc_P_Total_Open), '%.3f') ' m/s^2']);
disp(['     Acc Angle     = ' num2str(rad2deg(angle(Acc_P_Total_Open)), '%.2f') ' deg']);
