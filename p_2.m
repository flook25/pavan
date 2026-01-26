clear all
close all
clc

% ==========================================
% 1. PARAMETERS & CONSTANTS
% ==========================================
L1 = 0.210; % Ground (Pink) - d
L2 = 0.118; % Cyan/Light Blue - a (Connected from Loop 1)
L3 = 0.210; % Red - b
L4 = 0.118; % Grey - c

% Canonical names
d = L1;
a = L2;
b = L3;
c = L4;

% Ground Offset
theta1_deg = 0.81;
theta1 = deg2rad(theta1_deg);

% ==========================================
% 2. INPUTS (From Loop 1 Results)
% ==========================================
% Input values for Theta 2 in Global Degrees
input_theta2_deg = [-102.5000, 39.8576];

% Convert to Local Inputs (relative to theta1)
input_theta2_local = deg2rad(input_theta2_deg) - theta1;

% ==========================================
% 3. CALCULATION & PLOTTING LOOP
% ==========================================
% Calculate K constants (Standard Norton for Theta2 Input)
K1 = d/a;
K2 = d/c;
K3 = (a^2 - b^2 + c^2 + d^2)/(2*a*c);

% Colors
color_ground = [1, 0.4, 0.7]; % Pink
color_L2 = [0, 1, 1];         % Cyan
color_L3 = [1, 0, 0];         % Red
color_L4 = [0.5, 0.5, 0.5];   % Grey

for k = 1:length(input_theta2_local)
    q2 = input_theta2_local(k);
    q2_global_deg = input_theta2_deg(k);
    
    % --- Calculate A, B, C (Standard Form for Theta2 Input) ---
    A = cos(q2) - K1 - K2*cos(q2) + K3;
    B = -2*sin(q2);
    C = K1 - (K2+1)*cos(q2) + K3;
    
    det_val = B^2 - 4*A*C;
    
    if det_val < 0
        fprintf('Input %.4f deg: No solution (Mechanism breaks)\n', q2_global_deg);
        continue;
    end
    
    % --- Solve for Theta 4 (2 Solutions: Open/Crossed) ---
    % Solution 1
    q4_1 = 2*atan((-B + sqrt(det_val))/(2*A));
    % Solution 2
    q4_2 = 2*atan((-B - sqrt(det_val))/(2*A));
    
    % --- Solve for Theta 3 ---
    % Vector Loop: R2 + R3 - R4 - R1 = 0  => R3 = R1 + R4 - R2
    % (Note: R1 is d, R2 is a*exp(jq2), R4 is c*exp(jq4))
    
    % Case 1
    Vec3_1 = c*exp(1j*q4_1) + d - a*exp(1j*q2);
    q3_1 = angle(Vec3_1);
    
    % Case 2
    Vec3_2 = c*exp(1j*q4_2) + d - a*exp(1j*q2);
    q3_2 = angle(Vec3_2);
    
    % --- Global Conversion ---
    q2g = q2 + theta1;
    q3_1g = q3_1 + theta1; q4_1g = q4_1 + theta1;
    q3_2g = q3_2 + theta1; q4_2g = q4_2 + theta1;
    
    % --- Display Results ---
    fprintf('========================================\n');
    fprintf('INPUT SET %d: Theta 2 = %.4f deg\n', k, q2_global_deg);
    fprintf('  [Config 1] Theta3: %.4f, Theta4: %.4f\n', rad2deg(q3_1g), rad2deg(q4_1g));
    fprintf('  [Config 2] Theta3: %.4f, Theta4: %.4f\n', rad2deg(q3_2g), rad2deg(q4_2g));
    
    % --- PLOTTING ---
    figure('Name', sprintf('Loop 2 Analysis: Input %.2f', q2_global_deg));
    
    % Vectors
    RO2 = 0; 
    RO4 = d*exp(1j*theta1);
    RA = a*exp(1j*q2g); % Link 2 (Input)
    
    % Subplot 1: Config 1
    subplot(1, 2, 1); hold on; axis equal; grid on;
    RBA_1 = b*exp(1j*q3_1g);
    RB_1 = RA + RBA_1;
    RBO4_1 = c*exp(1j*q4_1g);
    
    % Components
    O2x=0; O2y=0; O4x=real(RO4); O4y=imag(RO4);
    Ax=real(RA); Ay=imag(RA);
    Bx_1=real(RB_1); By_1=imag(RB_1);
    
    quiver(O2x, O2y, O4x-O2x, O4y-O2y, 0, 'Color', color_ground, 'LineWidth', 2, 'MaxHeadSize', 0.5); % Ground
    quiver(O2x, O2y, Ax-O2x, Ay-O2y, 0, 'Color', color_L2, 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 2
    quiver(Ax, Ay, Bx_1-Ax, By_1-Ay, 0, 'Color', color_L3, 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 3
    quiver(O4x, O4y, Bx_1-O4x, By_1-O4y, 0, 'Color', color_L4, 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 4
    title(sprintf('Config 1\n\\theta_3=%.1f, \\theta_4=%.1f', rad2deg(q3_1g), rad2deg(q4_1g)));
    
    % Subplot 2: Config 2
    subplot(1, 2, 2); hold on; axis equal; grid on;
    RBA_2 = b*exp(1j*q3_2g);
    RB_2 = RA + RBA_2;
    RBO4_2 = c*exp(1j*q4_2g);
    
    Bx_2=real(RB_2); By_2=imag(RB_2);
    
    quiver(O2x, O2y, O4x-O2x, O4y-O2y, 0, 'Color', color_ground, 'LineWidth', 2, 'MaxHeadSize', 0.5); % Ground
    quiver(O2x, O2y, Ax-O2x, Ay-O2y, 0, 'Color', color_L2, 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 2
    quiver(Ax, Ay, Bx_2-Ax, By_2-Ay, 0, 'Color', color_L3, 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 3
    quiver(O4x, O4y, Bx_2-O4x, By_2-O4y, 0, 'Color', color_L4, 'LineWidth', 3, 'MaxHeadSize', 0.5); % Link 4
    title(sprintf('Config 2\n\\theta_3=%.1f, \\theta_4=%.1f', rad2deg(q3_2g), rad2deg(q4_2g)));
    
end
