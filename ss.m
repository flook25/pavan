clear all
close all
clc

% --- 1. GLOBAL PARAMETERS ---
offset_deg = 0.81;
offset = deg2rad(offset_deg);
d_global = 0.210; % Shared Ground

% Input Parameters (Brown Link / Link 4 of Loop 1)
w_input = 4.5;       % rad/sec
alpha_input = 3.5;   % rad/sec^2
theta_input_deg = 102.05;
q_in_L1 = deg2rad(theta_input_deg);

% Point P
p_dist = 0.120;


% --- 2. POSITION ANALYSIS ---

% =========================================================================
% LOOP 1 ANALYSIS (Input: Brown)
% =========================================================================
% Logic: Inverted (Input is Link c/L4)
L1 = 0.210; L2 = 0.118; L3 = 0.210; L4 = 0.118;
d=L1; a=L2; b=L3; c=L4; 

q_input_1 = q_in_L1; 

% Position
K1=d/c; K2=d/a; K3=(c^2-b^2+a^2+d^2)/(2*c*a); 
K4=d/b; K5=(a^2-d^2-c^2-b^2)/(2*c*b);

A = cos(q_input_1) - K1 - K2*cos(q_input_1) + K3;
B = -2*sin(q_input_1);
C = K1 - (K2+1)*cos(q_input_1) + K3;
D = cos(q_input_1) - K1 + K4*cos(q_input_1) + K5;
E = -2*sin(q_input_1);
F = K1 + (K4-1)*cos(q_input_1) + K5;

det_AC = sqrt(B^2 - 4*A*C);
det_DF = sqrt(E^2 - 4*D*F);

% Case Crossed (Plus)
q2_L1_Cross = 2*atan2((-B + det_AC), (2*A));
q3_L1_Cross = 2*atan2((-E + det_DF), (2*D));
T2_Cross_L1 = q2_L1_Cross + pi + offset; % Cyan
T3_Cross_L1 = q3_L1_Cross + pi + offset; % Blue

% Case Open (Minus)
q2_L1_Open  = 2*atan2((-B - det_AC), (2*A));
q3_L1_Open  = 2*atan2((-E - det_DF), (2*D));
T2_Open_L1 = q2_L1_Open + pi + offset; 
T3_Open_L1 = q3_L1_Open + pi + offset; 

T4_Global   = q_input_1 + pi + offset;   % Brown (Common)


% =========================================================================
% LOOP 2 ANALYSIS (Input: Cyan from Loop 1)
% =========================================================================
L1 = 0.210; L2 = 0.118; L3 = 0.210; L4 = 0.118;
d=L1; a=L2; b=L3; c=L4;
K1=d/a; K2=d/c; K3=(a^2-b^2+c^2+d^2)/(2*a*c); K4=d/b; K5=(c^2-d^2-a^2-b^2)/(2*a*b);

% --- Path 1: Crossed Input ---
q_in_2C = T2_Cross_L1 + pi - offset; 
A = cos(q_in_2C)-K1-K2*cos(q_in_2C)+K3; B = -2*sin(q_in_2C); 
C = K1-(K2+1)*cos(q_in_2C)+K3; D = cos(q_in_2C)-K1+K4*cos(q_in_2C)+K5; 
E = -2*sin(q_in_2C); F = K1+(K4-1)*cos(q_in_2C)+K5;
det_AC = sqrt(B^2 - 4*A*C); det_DF = sqrt(E^2 - 4*D*F);

q4_L2_Cross = 2*atan2((-B + det_AC), (2*A)); 
q3_L2_Cross = 2*atan2((-E + det_DF), (2*D));
T4_Cross_L2 = q4_L2_Cross + offset; % Grey
T3_Cross_L2 = q3_L2_Cross + offset; % Red

% --- Path 2: Open Input ---
q_in_2O = T2_Open_L1 + pi - offset; 
A = cos(q_in_2O)-K1-K2*cos(q_in_2O)+K3; B = -2*sin(q_in_2O); 
C = K1-(K2+1)*cos(q_in_2O)+K3; D = cos(q_in_2O)-K1+K4*cos(q_in_2O)+K5; 
E = -2*sin(q_in_2O); F = K1+(K4-1)*cos(q_in_2O)+K5;
det_AC = sqrt(B^2 - 4*A*C); det_DF = sqrt(E^2 - 4*D*F);

q4_L2_Open = 2*atan2((-B - det_AC), (2*A)); 
q3_L2_Open = 2*atan2((-E - det_DF), (2*D));
T4_Open_L2 = q4_L2_Open + offset; 
T3_Open_L2 = q3_L2_Open + offset;


% =========================================================================
% LOOP 3 ANALYSIS (Input: Grey from Loop 2)
% =========================================================================
L1 = 0.210; L2 = 0.180; L3 = 0.180; L4 = 0.118;
d=L1; a=L2; b=L3; c=L4;
K1=d/c; K2=d/a; K3=(c^2-b^2+a^2+d^2)/(2*c*a);

% --- Path 1: Crossed Input ---
q_in_3C = T4_Cross_L2 - offset - pi;
A = cos(q_in_3C)-K1-K2*cos(q_in_3C)+K3; B = -2*sin(q_in_3C); C = K1-(K2+1)*cos(q_in_3C)+K3;
det_AC = sqrt(B^2 - 4*A*C);
q2_L3_Cross = 2*atan2((-B - det_AC), (2*A));
T2_Cross_L3 = q2_L3_Cross + pi + offset; % Green
T3_Cross_L3 = angle(d + a*exp(j*q2_L3_Cross) - c*exp(j*q_in_3C)) + pi + offset; % Yellow

% --- Path 2: Open Input ---
q_in_3O = T4_Open_L2 - offset - pi;
A = cos(q_in_3O)-K1-K2*cos(q_in_3O)+K3; B = -2*sin(q_in_3O); C = K1-(K2+1)*cos(q_in_3O)+K3;
det_AC = sqrt(B^2 - 4*A*C);
q2_L3_Open = 2*atan2((-B + det_AC), (2*A)); 
T2_Open_L3 = q2_L3_Open + pi + offset; 
T3_Open_L3 = angle(d + a*exp(j*q2_L3_Open) - c*exp(j*q_in_3O)) + pi + offset; 


% --- 3. VELOCITY & ACCELERATION ANALYSIS ---

% Loop 1 Setup
L2=0.118; L3=0.210; L4=0.118; a=L2; b=L3; c=L4;

% --- CROSSED CASE ---
% Velocity L1
t2=T2_Cross_L1; t3=T3_Cross_L1; t4=T4_Global;
w4 = w_input; alp4 = alpha_input;
w_Cyan_C = (c*w4/a)*(sin(t4-t3)/sin(t2-t3));
w_Blue_C = (c*w4/b)*(sin(t2-t4)/sin(t2-t3));

% Acceleration L1
A_val=c*sin(t2); B_val=b*sin(t3);
C_val=a*alp4*sin(t4) + a*w4^2*cos(t4) + b*w_Blue_C^2*cos(t3) - c*w_Cyan_C^2*cos(t2);
D_val=c*cos(t2); E_val=b*cos(t3);
F_val=a*alp4*cos(t4) - a*w4^2*sin(t4) - b*w_Blue_C^2*sin(t3) + c*w_Cyan_C^2*sin(t2);
alp_Cyan_C = (C_val*E_val - B_val*F_val)/(A_val*E_val - B_val*D_val);
alp_Blue_C = (C_val*D_val - A_val*F_val)/(A_val*E_val - B_val*D_val);

% Velocity L2 (Standard Logic)
L2=0.118; L3=0.210; L4=0.118; a=L2; b=L3; c=L4;
t2=T2_Cross_L1+pi; t3=T3_Cross_L2; t4=T4_Cross_L2;
w2=w_Cyan_C; alp2=alp_Cyan_C;
w_Red_C = (a*w2/b)*(sin(t4-t2)/sin(t3-t4));
w_Grey_C = (a*w2/c)*(sin(t2-t3)/sin(t4-t3));

% Acceleration L2
A_val=c*sin(t4); B_val=b*sin(t3);
C_val=a*alp2*sin(t2) + a*w2^2*cos(t2) + b*w_Red_C^2*cos(t3) - c*w_Grey_C^2*cos(t4);
D_val=c*cos(t4); E_val=b*cos(t3);
F_val=a*alp2*cos(t2) - a*w2^2*sin(t2) - b*w_Red_C^2*sin(t3) + c*w_Grey_C^2*sin(t4);
alp_Grey_C = (C_val*E_val - B_val*F_val)/(A_val*E_val - B_val*D_val);
alp_Red_C = (C_val*D_val - A_val*F_val)/(A_val*E_val - B_val*D_val);

% Velocity L3 (Inverted Logic)
L2=0.180; L3=0.180; L4=0.118; a=L2; b=L3; c=L4;
t2=T2_Cross_L3; t3=T3_Cross_L3; t4=T4_Cross_L2;
w4=w_Grey_C; alp4=alp_Grey_C;
w_Green_C = (c*w4/a)*(sin(t4-t3)/sin(t2-t3));
w_Yellow_C = (c*w4/b)*(sin(t2-t4)/sin(t2-t3));

% Acceleration L3
A_val=c*sin(t2); B_val=b*sin(t3);
C_val=a*alp4*sin(t4) + a*w4^2*cos(t4) + b*w_Yellow_C^2*cos(t3) - c*w_Green_C^2*cos(t2);
D_val=c*cos(t2); E_val=b*cos(t3);
F_val=a*alp4*cos(t4) - a*w4^2*sin(t4) - b*w_Yellow_C^2*sin(t3) + c*w_Green_C^2*sin(t2);
alp_Green_C = (C_val*E_val - B_val*F_val)/(A_val*E_val - B_val*D_val);
alp_Yellow_C = (C_val*D_val - A_val*F_val)/(A_val*E_val - B_val*D_val);


% --- OPEN CASE ---
% Velocity L1
L2=0.118; L3=0.210; L4=0.118; a=L2; b=L3; c=L4;
t2=T2_Open_L1; t3=T3_Open_L1; t4=T4_Global;
w4 = w_input; alp4 = alpha_input;
w_Cyan_O = (c*w4/a)*(sin(t4-t3)/sin(t2-t3));
w_Blue_O = (c*w4/b)*(sin(t2-t4)/sin(t2-t3));

% Acceleration L1
A_val=c*sin(t2); B_val=b*sin(t3);
C_val=a*alp4*sin(t4) + a*w4^2*cos(t4) + b*w_Blue_O^2*cos(t3) - c*w_Cyan_O^2*cos(t2);
D_val=c*cos(t2); E_val=b*cos(t3);
F_val=a*alp4*cos(t4) - a*w4^2*sin(t4) - b*w_Blue_O^2*sin(t3) + c*w_Cyan_O^2*sin(t2);
alp_Cyan_O = (C_val*E_val - B_val*F_val)/(A_val*E_val - B_val*D_val);
alp_Blue_O = (C_val*D_val - A_val*F_val)/(A_val*E_val - B_val*D_val);

% Velocity L2
L2=0.118; L3=0.210; L4=0.118; a=L2; b=L3; c=L4;
t2=T2_Open_L1+pi; t3=T3_Open_L2; t4=T4_Open_L2;
w2=w_Cyan_O; alp2=alp_Cyan_O;
w_Red_O = (a*w2/b)*(sin(t4-t2)/sin(t3-t4));
w_Grey_O = (a*w2/c)*(sin(t2-t3)/sin(t4-t3));

% Acceleration L2
A_val=c*sin(t4); B_val=b*sin(t3);
C_val=a*alp2*sin(t2) + a*w2^2*cos(t2) + b*w_Red_O^2*cos(t3) - c*w_Grey_O^2*cos(t4);
D_val=c*cos(t4); E_val=b*cos(t3);
F_val=a*alp2*cos(t2) - a*w2^2*sin(t2) - b*w_Red_O^2*sin(t3) + c*w_Grey_O^2*sin(t4);
alp_Grey_O = (C_val*E_val - B_val*F_val)/(A_val*E_val - B_val*D_val);
alp_Red_O = (C_val*D_val - A_val*F_val)/(A_val*E_val - B_val*D_val);

% Velocity L3
L2=0.180; L3=0.180; L4=0.118; a=L2; b=L3; c=L4;
t2=T2_Open_L3; t3=T3_Open_L3; t4=T4_Open_L2;
w4=w_Grey_O; alp4=alp_Grey_O;
w_Green_O = (c*w4/a)*(sin(t4-t3)/sin(t2-t3));
w_Yellow_O = (c*w4/b)*(sin(t2-t4)/sin(t2-t3));

% Acceleration L3
A_val=c*sin(t2); B_val=b*sin(t3);
C_val=a*alp4*sin(t4) + a*w4^2*cos(t4) + b*w_Yellow_O^2*cos(t3) - c*w_Green_O^2*cos(t2);
D_val=c*cos(t2); E_val=b*cos(t3);
F_val=a*alp4*cos(t4) - a*w4^2*sin(t4) - b*w_Yellow_O^2*sin(t3) + c*w_Green_O^2*sin(t2);
alp_Green_O = (C_val*E_val - B_val*F_val)/(A_val*E_val - B_val*D_val);
alp_Yellow_O = (C_val*D_val - A_val*F_val)/(A_val*E_val - B_val*D_val);


% --- 4. PREPARE VECTORS & PLOT ---

RG = d_global * exp(j*offset);

% VECTORS: CROSSED CASE
R_Br_C = 0.118*exp(j*T4_Global);
R_Cy_C = 0.118*exp(j*T2_Cross_L1);
R_Bl_C = (RG + R_Br_C) - R_Cy_C;
R_Gr_C = 0.118*exp(j*T4_Cross_L2);
R_Gn_C = 0.180*exp(j*T2_Cross_L3);
R_Yl_C = (RG + R_Gr_C) - R_Gn_C;
R_Rd_C = (RG + R_Gr_C) - (0.118*exp(j*(T2_Cross_L1+pi)));
theta_P_C = angle(R_Bl_C);
R_P_C = R_Cy_C + p_dist * exp(j*theta_P_C);

AC_Base_C = 0.118*alp_Cyan_C*j*exp(j*T2_Cross_L1) - 0.118*w_Cyan_C^2*exp(j*T2_Cross_L1);
AC_P_N_C = -p_dist * w_Blue_C^2 * exp(j*theta_P_C);
AC_P_T_C = p_dist * alp_Blue_C * j * exp(j*theta_P_C);
AC_Total_C = AC_Base_C + AC_P_N_C + AC_P_T_C;

% VECTORS: OPEN CASE
R_Cy_O = 0.118*exp(j*T2_Open_L1);
R_Bl_O = (RG + R_Br_C) - R_Cy_O; 
R_Gr_O = 0.118*exp(j*T4_Open_L2);
R_Gn_O = 0.180*exp(j*T2_Open_L3);
R_Yl_O = (RG + R_Gr_O) - R_Gn_O;
R_Rd_O = (RG + R_Gr_O) - (0.118*exp(j*(T2_Open_L1+pi)));
theta_P_O = angle(R_Bl_O);
R_P_O = R_Cy_O + p_dist * exp(j*theta_P_O);

AC_Base_O = 0.118*alp_Cyan_O*j*exp(j*T2_Open_L1) - 0.118*w_Cyan_O^2*exp(j*T2_Open_L1);
AC_P_N_O = -p_dist * w_Blue_O^2 * exp(j*theta_P_O);
AC_P_T_O = p_dist * alp_Blue_O * j * exp(j*theta_P_O);
AC_Total_O = AC_Base_O + AC_P_N_O + AC_P_T_O;


% --- PLOTTING ---
figure(1);
RGx = real(RG); RGy = imag(RG);

% SUBPLOT 1: CROSSED
subplot(1,2,1); hold on; axis equal; grid on; title('System Configuration 2 (Crossed)');
quiver(0,0,RGx,RGy,0,'m','LineWidth',2,'MaxHeadSize',0); % Ground
quiver(RGx,RGy,real(R_Br_C),imag(R_Br_C),0,'Color',[0.6 0.3 0],'LineWidth',2,'MaxHeadSize',0.5); % Brown
quiver(0,0,real(R_Cy_C),imag(R_Cy_C),0,'c','LineWidth',2,'MaxHeadSize',0.5); % Cyan
quiver(real(R_Cy_C),imag(R_Cy_C),real(R_Bl_C),imag(R_Bl_C),0,'b','LineWidth',2,'MaxHeadSize',0.5); % Blue
quiver(RGx,RGy,real(R_Gr_C),imag(R_Gr_C),0,'Color',[0.5 0.5 0.5],'LineWidth',2,'MaxHeadSize',0.5); % Grey
quiver(real(R_Cy_C),imag(R_Cy_C),real(R_Rd_C),imag(R_Rd_C),0,'r','LineWidth',2,'MaxHeadSize',0.5); % Red
quiver(0,0,real(R_Gn_C),imag(R_Gn_C),0,'g','LineWidth',2,'MaxHeadSize',0.5); % Green
quiver(real(R_Gn_C),imag(R_Gn_C),real(R_Yl_C),imag(R_Yl_C),0,'y','LineWidth',2,'MaxHeadSize',0.5); % Yellow
quiver(real(R_P_C),imag(R_P_C),real(AC_Total_C)/200,imag(AC_Total_C)/200,0,'k','LineWidth',2,'MaxHeadSize',0.5);
plot(real(R_P_C), imag(R_P_C), 'bo', 'MarkerFaceColor', 'b');
text(real(R_P_C), imag(R_P_C)+0.02, 'P');

% SUBPLOT 2: OPEN
subplot(1,2,2); hold on; axis equal; grid on; title('System Configuration 1 (Open)');
quiver(0,0,RGx,RGy,0,'m','LineWidth',2,'MaxHeadSize',0); 
quiver(RGx,RGy,real(R_Br_C),imag(R_Br_C),0,'Color',[0.6 0.3 0],'LineWidth',2,'MaxHeadSize',0.5); 
quiver(0,0,real(R_Cy_O),imag(R_Cy_O),0,'c','LineWidth',2,'MaxHeadSize',0.5); 
quiver(real(R_Cy_O),imag(R_Cy_O),real(R_Bl_O),imag(R_Bl_O),0,'b','LineWidth',2,'MaxHeadSize',0.5); 
quiver(RGx,RGy,real(R_Gr_O),imag(R_Gr_O),0,'Color',[0.5 0.5 0.5],'LineWidth',2,'MaxHeadSize',0.5); 
quiver(real(R_Cy_O),imag(R_Cy_O),real(R_Rd_O),imag(R_Rd_O),0,'r','LineWidth',2,'MaxHeadSize',0.5); 
quiver(0,0,real(R_Gn_O),imag(R_Gn_O),0,'g','LineWidth',2,'MaxHeadSize',0.5); 
quiver(real(R_Gn_O),imag(R_Gn_O),real(R_Yl_O),imag(R_Yl_O),0,'y','LineWidth',2,'MaxHeadSize',0.5); 
quiver(real(R_P_O),imag(R_P_O),real(AC_Total_O)/200,imag(AC_Total_O)/200,0,'k','LineWidth',2,'MaxHeadSize',0.5);
plot(real(R_P_O), imag(R_P_O), 'bo', 'MarkerFaceColor', 'b');
text(real(R_P_O), imag(R_P_O)+0.02, 'P');


% --- 5. RESULTS DISPLAY ---

% Linear velocities of Link Tips (Joints)
V_Cy_C = abs(w_Cyan_C * 0.118);
V_Gr_C = abs(w_Grey_C * 0.118);
V_Gn_C = abs(w_Green_C * 0.180);

V_Cy_O = abs(w_Cyan_O * 0.118);
V_Gr_O = abs(w_Grey_O * 0.118);
V_Gn_O = abs(w_Green_O * 0.180);



disp(' ');
disp('--- 1. CROSSED CIRCUIT ---');
disp('-----------------------------------------');
disp('LOOP 1:');
disp(['  Brown (Input):  Angle = ' num2str(theta_input_deg, '%.2f') ' deg,  Omega = ' num2str(w_input, '%.2f') ' rad/s']);
disp(['  Cyan (Link 1):  Angle = ' num2str(rad2deg(T2_Cross_L1),'%.2f') ' deg,  Omega = ' num2str(w_Cyan_C,'%.2f') ' rad/s,  V_Tip = ' num2str(V_Cy_C,'%.3f') ' m/s']);
disp(['  Blue (Coupler): Angle = ' num2str(rad2deg(T3_Cross_L1),'%.2f') ' deg,  Omega = ' num2str(w_Blue_C,'%.2f') ' rad/s']);
disp('LOOP 2:');
disp(['  Grey (Link 2):  Angle = ' num2str(rad2deg(T4_Cross_L2),'%.2f') ' deg,  Omega = ' num2str(w_Grey_C,'%.2f') ' rad/s,  V_Tip = ' num2str(V_Gr_C,'%.3f') ' m/s']);
disp(['  Red (Coupler):  Angle = ' num2str(rad2deg(T3_Cross_L2),'%.2f') ' deg,  Omega = ' num2str(w_Red_C,'%.2f') ' rad/s']);
disp('LOOP 3:');
disp(['  Green (Link 3): Angle = ' num2str(rad2deg(T2_Cross_L3),'%.2f') ' deg,  Omega = ' num2str(w_Green_C,'%.2f') ' rad/s,  V_Tip = ' num2str(V_Gn_C,'%.3f') ' m/s']);
disp(['  Yellow (Coup):  Angle = ' num2str(rad2deg(T3_Cross_L3),'%.2f') ' deg,  Omega = ' num2str(w_Yellow_C,'%.2f') ' rad/s']);
disp('POINT P:');
disp(['  Acceleration:   Mag = ' num2str(abs(AC_Total_C),'%.3f') ' m/s^2,  Angle = ' num2str(rad2deg(angle(AC_Total_C)),'%.2f') ' deg']);


disp(' ');
disp('--- 2. OPEN CIRCUIT ---');
disp('-----------------------------------------');
disp('LOOP 1:');
disp(['  Brown (Input):  Angle = ' num2str(theta_input_deg,'%.2f') ' deg,  Omega = ' num2str(w_input,'%.2f') ' rad/s']);
disp(['  Cyan (Link 1):  Angle = ' num2str(rad2deg(T2_Open_L1),'%.2f') ' deg,  Omega = ' num2str(w_Cyan_O,'%.2f') ' rad/s,  V_Tip = ' num2str(V_Cy_O,'%.3f') ' m/s']);
disp(['  Blue (Coupler): Angle = ' num2str(rad2deg(T3_Open_L1),'%.2f') ' deg,  Omega = ' num2str(w_Blue_O,'%.2f') ' rad/s']);
disp('LOOP 2:');
disp(['  Grey (Link 2):  Angle = ' num2str(rad2deg(T4_Open_L2),'%.2f') ' deg,  Omega = ' num2str(w_Grey_O,'%.2f') ' rad/s,  V_Tip = ' num2str(V_Gr_O,'%.3f') ' m/s']);
disp(['  Red (Coupler):  Angle = ' num2str(rad2deg(T3_Open_L2),'%.2f') ' deg,  Omega = ' num2str(w_Red_O,'%.2f') ' rad/s']);
disp('LOOP 3:');
disp(['  Green (Link 3): Angle = ' num2str(rad2deg(T2_Open_L3),'%.2f') ' deg,  Omega = ' num2str(w_Green_O,'%.2f') ' rad/s,  V_Tip = ' num2str(V_Gn_O,'%.3f') ' m/s']);
disp(['  Yellow (Coup):  Angle = ' num2str(rad2deg(T3_Open_L3),'%.2f') ' deg,  Omega = ' num2str(w_Yellow_O,'%.2f') ' rad/s']);
disp('POINT P:');
disp(['  Acceleration:   Mag = ' num2str(abs(AC_Total_O),'%.3f') ' m/s^2,  Angle = ' num2str(rad2deg(angle(AC_Total_O)),'%.2f') ' deg']);
