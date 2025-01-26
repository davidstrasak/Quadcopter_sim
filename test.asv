clear,close,clc

m = 1.3;                 % [kg]
Ix =  0.023;  % [kg m^2]
Iy =  0.023;  % [kg m^2]
Iz =  0.047;  % [kg m^2]
g = 9.81

C = eye(12);


% Define symbolic variables for the control inputs (T, M1, M2, M3)
syms T M1 M2 M3 x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12

% Define the state vector
x = [x1; x2; x3; x4; x5; x6; x7; x8; x9; x10; x11; x12];

% Define the control input vector (T, M1, M2, M3)
u = [T; M1; M2; M3];

% Define the system dynamics (as in your example)
dxdt = [
    x(2);  % x1_dot = x2 (velocity in x direction)
    (T/m) * (cos(x(7)) * sin(x(9)) * cos(x(11)) + sin(x(7)) * sin(x(11)));  % x2_dot (acceleration in x)
    x(4);  % x3_dot = x4 (velocity in y direction)
    (T/m) * (cos(x(7)) * sin(x(9)) * sin(x(11)) - sin(x(7)) * cos(x(11)));  % x4_dot (acceleration in y)
    x(6);  % x5_dot = x6 (velocity in z direction)
    (T/m) * (cos(x(7)) * cos(x(9))) + g;  % x6_dot (acceleration in z)
    x(8);  % x7_dot = x8 (angular velocity in roll)
    M1/Ix;  % x8_dot = M1/Ix (angular acceleration in roll)
    x(10);  % x9_dot = x10 (angular velocity in pitch)
    M2/Iy;  % x10_dot = M2/Iy (angular acceleration in pitch)
    x(12);  % x11_dot = x12 (angular velocity in yaw)
    M3/Iz   % x12_dot = M3/Iz (angular acceleration in yaw)
];

% Now compute the Jacobian matrices A and B

% Jacobian of dxdt w.r.t. state vector x (12x12)
A = jacobian(dxdt, x);

% Jacobian of dxdt w.r.t. input vector u (12x4)
B = jacobian(dxdt, u);

% Define equilibrium values for the states (position = 0, velocities = 0, etc.)
equilibrium_values = zeros(12, 1);  % Hover condition where all states are 0

% Define input values at hover: thrust = mg, M1 = M2 = M3 = 0
equilibrium_inputs = [m*g; 0; 0; 0];  % T = mg, M1 = M2 = M3 = 0

% Substitute the equilibrium values into A and B to get the linearized matrices
A_sub = subs(A, [x; u], [equilibrium_values; equilibrium_inputs]);
B_sub = subs(B, [x; u], [equilibrium_values; equilibrium_inputs]);

A = vpa(A_sub)
B = vpa(B_sub)

A = double(A)
B = double(B)

miK = [-2+2.4i, -2-2.4i,-2+2.4i, -2-2.4i,-2+1i, -2-1i,-2+1i, -2-1i,-3+1i, -3-1i,-3+1i, -3-1i]; %Poly k matici K
miKe = [-8, -8, -8, -8,-8, -8, -8, -8,-8, -8, -8, -8]; %Poly k matici Ke

K = place(A,B,miK)
Ke = place(A', C', miKe')'

At = A - Ke*C - B*K;
Bt = Ke;
Ct = K;
Dt = 0;

Regulator = ss(At,Bt,Ct,Dt)

