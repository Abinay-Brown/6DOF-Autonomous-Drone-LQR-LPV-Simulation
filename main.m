%% Drone Simulation
clear; clc;
%% Time Stamps
t_end = 50;
h = 0.05;
t = 0:h:t_end;
sol = zeros(length(t), 15);

%% Parameters
m = 0.698;
Ixx = 0.0034;
Iyy = 0.0034;
Izz = 0.006;
J = 1.302*10^(-6);
Ct = 7.6184*(10^(-8))*(60/(2*pi))^2;
Ctheta = 2.6839*(10^(-9))*(60/(2*pi))^2;
l = 0.171;
g = 9.81;
%% Initial Conditions
state0 = [0, 0, 0,  0, 0, 0,  0, 0, 0,  0, 0, 0,  0, 0, 0];
sol(1,:) = state0;

U1=m*g; U2=0; U3=0; U4=0; % Initial Inputs

Omega = Inp2Omega(U1, U2, U3, U4, Ct, Ctheta, l);

phi_d = 0;theta_d = 0; psi_d = 0; % Initial Euler Angles

% Initial parameters
params = [m,Ixx,Iyy,Izz,J,Omega,U1,U2,U3,U4,phi_d,theta_d,psi_d];

%% Position Controller gains
[k1x, k2x] = poleplace(-1, -1);
[k1y, k2y] = poleplace(-1, -1);
[k1z, k2z] = poleplace(-1, -1);


%% Record Results
pos = zeros(length(t), 3);
vel = zeros(length(t), 3);
phi_ref = zeros(length(t), 1);
theta_ref = zeros(length(t), 1);

%% Reference Trajectory
[Xr,Yr,Zr,Xdr,Ydr,Zdr,Xddr,Yddr,Zddr, psi_ref] = refTraj3(t);

%% Solver
j = 0;
for i = 1:length(t)-1
    % Using Runge Kutta to propagate
    sol(i+1,:) = RK8(@diffEqn, t(i), sol(i, :), h, params);

    % Unpack integrated states
    % Euler Angles
    phi = sol(i+1, 1);
    theta = sol(i+1, 2);
    psi = sol(i+1, 3);
    
    % Body Angular rates
    p = sol(i+1, 4);
    q = sol(i+1, 5);
    r = sol(i+1, 6);
    
    % body frame velocity
    u = sol(i+1, 7);
    v = sol(i+1, 8);
    w = sol(i+1, 9);

    % Inertial Position
    X = sol(i+1, 10);
    Y = sol(i+1, 11);
    Z = sol(i+1, 12);

    % integrated errors
    err_phi = sol(i+1, 13);
    err_theta = sol(i+1, 14);
    err_psi = sol(i+1, 15);

    % Euler rates
    T_mat = T_rot(phi, theta);
    dots = T_mat*[p,q,r]';
    phi_dot = dots(1);
    theta_dot = dots(2);
    psi_dot = dots(3);

    % Inertial Velocity
    R_mat = rot(phi, theta, psi);
    vels = R_mat*[u, v, w]';
    X_dot = vels(1);
    Y_dot = vels(2);
    Z_dot = vels(3);
   
    %% Attitude Controller
    % Linearized Atittude dynamics
    A = [0, 1, 0, 0, 0, 0;...
       0, 0, 0, (J*Omega/Ixx), 0, ((Iyy-Izz)*theta_dot/Ixx);...
       0, 0, 0, 1, 0, 0;...
       0, (-J*Omega/Iyy), 0, 0, 0, ((Izz-Ixx)*phi_dot/Iyy);...
       0, 0, 0, 0, 0, 1;...
       0, 0, 0, ((Ixx-Iyy)*phi_dot/Izz), 0, 0];
    B = [0, 0, 0;
         1/Ixx, 0, 0;
         0, 0, 0;
         0, 1/Iyy, 0;
         0, 0, 0;
         0, 0, 1/Izz];
    C = [1, 0, 0, 0, 0, 0;...
         0, 0, 1, 0, 0, 0;...
         0, 0, 0, 0, 1, 0];

    % Augmented matrices for Integral action setpoint control
    Ahat = [A, zeros(6, 3);...
           -C, zeros(3, 3)];
    Bhat = [B; 0, 0, 0; 0, 0, 0; 0, 0, 0];
    
    % Attitude Performance Weight matrices
    Q = diag([1, 1, 1, 1, 1, 1, 600, 600, 600]);
    R = diag([1, 1, 1]);
    
    % Updated LQR gains
    khat = lqrd(Ahat, Bhat, Q, R, h);
    
    
    state_aug = [phi, phi_dot, theta, theta_dot, psi, psi_dot, err_phi, err_theta, err_psi];
    
    % Calculate updated drone inputs
    inp = -khat*state_aug';
    U2 = inp(1);
    U3 = inp(2);
    U4 = inp(3);
    
    
    % Inertial Frame Position and Velocity
    
    %% Position Controller
    if j == 4
        
        j = 0;
        ex = Xr(i+1) - X;
        ey = Yr(i+1) - Y;
        ez = Zr(i+1) - Z;
        
        ex_d = Xdr(i+1) - X_dot;
        ey_d = Ydr(i+1) - Y_dot;
        ez_d = Zdr(i+1) - Z_dot;
        
        ex_dd = (k1x*ex) + (k2x*ex_d);
        ey_dd = (k1y*ey) + (k2y*ey_d);
        ez_dd = (k1z*ez) + (k2z*ez_d);
        
        x_dd = Xddr(i+1) - ex_dd;
        y_dd = Yddr(i+1) - ey_dd;
        z_dd = Zddr(i+1) - ez_dd;
       
%         
        psi_d = psi_ref(i+1);
        a=x_dd/(z_dd+g);
        b=y_dd/(z_dd+g);
        c=cos(psi_d);
        d=sin(psi_d);
       
        tan_theta=a*c+b*d;
        
        theta_d=atan(tan_theta);
        

        if psi_d >=0
            psi_sing=psi_d-floor(abs(psi_d)/(2*pi))*2*pi;
        else
            psi_sing=psi_d+floor(abs(psi_d)/(2*pi))*2*pi;
        end
        if ((abs(psi_sing)<pi/4 | abs(psi_sing)>7*pi/4) | (abs(psi_sing)>3*pi/4 & abs(psi_sing)<5*pi/4))
            tan_phi=cos(theta_d)*(tan(theta_d)*d-b)/c;
        else
            tan_phi=cos(theta_d)*(a-tan(theta_d)*c)/d;
        end
        phi_d = atan(tan_phi);
        
        U1 = (z_dd+g)*m/(cos(phi_d)*cos(theta_d));
      
    end

    %% Update Parameters for next integration step
    
    Omega = Inp2Omega(U1, U2, U3, U4, Ct, Ctheta, l);
    params(6) = Omega;
    params(7) = U1;
    params(8) = U2;
    params(9) = U3;
    params(10) = U4;
    params(11) = phi_d;
    params(12) = theta_d;
    params(13) = psi_d;
    
    % Record results
    phi_ref(i+1) = phi_d;
    theta_ref(i) = theta_d;
    psi_ref(i) = psi_d;
    pos(i+1, :) = [X, Y, Z];
    vel(i+1, :) = [X_dot, Y_dot, Z_dot];
    
    % update Position control loop iteration
    j = j +1;
end
%% Trajectory Plot
plot3(pos(:,1), pos(:,2), pos(:,3))
axis equal;
hold on;
plot3(Xr(:), Yr(:), Zr(:))
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
title('Trajectory Result')
legend('Drone trajectory', 'Reference Trajectory')
grid on;

%% Position Plot
figure;
subplot(3, 1, 1);
plot(t, pos(:,1));
hold on;
plot(t, Xr)
xlabel('time (sec)')
ylabel('X (m)')
legend('X', 'X_r_e_f')
grid on;

subplot(3, 1, 2);
plot(t, pos(:,2));
hold on;
plot(t, Yr)
xlabel('time (sec)')
ylabel('Y (m)')
legend('Y', 'Y_r_e_f')
grid on;

subplot(3, 1, 3);
plot(t, pos(:,3));
hold on;
plot(t, Zr)
xlabel('time (sec)')
ylabel('Z (m)')
legend('Z', 'Z_r_e_f')
grid on;
%% Euler Angle Plots
figure;
subplot(3, 1, 1);
plot(t, sol(:,1)*(180/pi));
hold on;
plot(t, phi_ref*(180/pi))
xlabel('time (sec)')
ylabel('\phi (deg)')
legend('\phi', '\phi_r_e_f')
grid on;

subplot(3, 1, 2);
plot(t, sol(:,2)*(180/pi));
hold on;
plot(t, theta_ref*(180/pi))
xlabel('time (sec)')
ylabel('\theta (deg)')
legend('\theta', '\theta_r_e_f')
grid on;

subplot(3, 1, 3);
plot(t, sol(:,3)*(180/pi));
hold on;
plot(t, psi_ref*(180/pi))
xlabel('time (sec)')
ylabel('\psi (deg)')
legend('\psi', '\psi_r_e_f')
grid on;

phi_val = sol(:, 1);
theta_val = sol(:, 2);
psi_val = sol(:,3);
save('Simulation.mat', "pos", "phi_val", "theta_val", "psi_val",...
    "phi_ref", "theta_ref", "psi_ref", "Xr", "Yr", "Zr", "l");