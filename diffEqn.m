function out = diffEqn(t, state, params)
% unpack params
m = params(1);
Ixx = params(2);
Iyy = params(3);
Izz = params(4);
J = params(5);
Omega = params(6);
U1 = params(7);
U2 = params(8);
U3 = params(9);
U4 = params(10);
phi_d = params(11);
theta_d = params(12);
psi_d = params(13);

% unpack states
% Euler Angles
phi = state(1);
theta = state(2);
psi = state(3);

% Body Angular rates
p = state(4);
q = state(5);
r = state(6);

% Body Frame Velocity
u = state(7);
v = state(8);
w = state(9);

% Inertial Frame Position
X = state(10);
Y = state(11);
Z = state(12);

% Error States
err_phi = state(13);
err_theta = state(14);
err_psi = state(15);

% Newton-Euler Equations
p_dot = (((Iyy-Izz)/Ixx)*q*r) + (J*q*Omega/Ixx) + (U2/Ixx);
q_dot = (((Izz-Ixx)/Iyy)*p*r) - (J*p*Omega/Iyy) + (U3/Iyy);
r_dot = (((Ixx-Iyy)/Izz)*p*q) + (U4/Izz);


T_mat = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);...
    0, cos(phi), -sin(phi);...
    0, (sin(phi)/cos(theta)), (cos(phi)/cos(theta))];

thetas = T_mat*[p, q, r]';

phi_dot = thetas(1);
theta_dot = thetas(2);
psi_dot = thetas(3);

g = 9.81;
u_dot = (v*r) - (w*q) + (g*sin(theta));
v_dot = (w*p) - (u*r) - (g*cos(theta)*sin(phi));
w_dot = (u*q) - (v*p) - (g*cos(theta)*cos(phi)) + (U1/m);

R_mat = rot(phi, theta, psi);
Vels = R_mat*[u, v, w]';

U = Vels(1);
V = Vels(2);
W = Vels(3);

% Error States
err_phi_dot = phi_d - phi;
err_theta_dot = theta_d - theta;
err_psi_dot = psi_d - psi;

% output states

% Euler Rates
out(1) = phi_dot;
out(2) = theta_dot;
out(3) = psi_dot;

% Body angular velocity rates
out(4) = p_dot;
out(5) = q_dot;
out(6) = r_dot;

% body frame acceleration
out(7) = u_dot;
out(8) = v_dot;
out(9) = w_dot;

% Inertial Frame Velocity
out(10) = U;
out(11) = V;
out(12) = W;

% Error State derivative
out(13) = err_phi_dot;
out(14) = err_theta_dot;
out(15) = err_psi_dot;
end