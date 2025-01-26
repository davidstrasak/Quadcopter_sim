function dxdt = QuadcopterDynamics(t,x,T,M1,M2,M3,g,m,Ix,Iy,Iz)
% arguments
%     t
%     x
%     T
%     M1
%     M2
%     M3
%     g
%     m
%     Ix
%     Iy
%     Iz
% end


% x1 = poloha x
% x2 = rychlost x
% x3 = poloha y
% x4 = rychlost y
% x5 = poloha z
% x6 = rychlost z
% x7 = phi
% x8 = rychlost phi
% x9 = theta
% x10 = rychlost theta
% x11 = psi
% x12 = rychlost psi

dxdt = [
    x(2);
    (T/m) * (cos(x(7)) * sin(x(9)) * cos(x(11)) + sin(x(7)) * sin(x(11)));
    x(4);
    (T/m) * (cos(x(7)) * sin(x(9)) * sin(x(11)) - sin(x(7)) * cos(x(11)));
    x(6);
    (T/m) * (cos(x(7)) * cos(x(9))) + g;
    x(8);
    M1/Ix;
    x(10);
    M2/Iy;
    x(12);
    M3/Iz
    ];

% B = [
%     0;
%     T;
%     0;
%     T;
%     0;
%     T + g/T;
%     0;
%     M1;
%     0;
%     M2;
%     0;
%     M3;
%     ];

% C = eye(12);
% 
% D = zeros(1,12);


end

