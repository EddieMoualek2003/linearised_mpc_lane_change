function rho = closedLoopRho(A, B, Kx, Ku)
% Car:        x_next = A*x + B*u
% Controller: u      = Kx*x + Ku*u_prev
% Together:   [x_next; u] = Acl * [x; u_prev]
Acl = [A + B*Kx,  B*Ku;
    Kx,        Ku  ];
rho = max(abs(eig(Acl)));
end