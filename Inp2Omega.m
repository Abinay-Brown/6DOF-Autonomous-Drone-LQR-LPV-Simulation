function Omega = Inp2Omega(U1, U2, U3, U4, Ct, Ctheta, l)
mat = [0.25, 0, -0.5, -0.25;...
    0.25, 0.5, 0, 0.25;...
    0.25, 0, 0.5, -0.25;...
    0.25, -0.5, 0, 0.25];
    Omegas = mat*[U1/Ct, U2/Ct/l, U3/Ct/l, U4/Ctheta]';
    Omega1 = sqrt(Omegas(1));
    Omega2 = sqrt(Omegas(2));
    Omega3 = sqrt(Omegas(3));
    Omega4 = sqrt(Omegas(4));
    Omega = Omega1-Omega2+Omega3-Omega4;
    if (Omegas(1)<0 | Omegas(2)<0 | Omegas(3)<0 | Omegas(4)<0)
        error('Error! Aggressive Maneuver Soln not Found')
    end
%     disp(Omegas)
end