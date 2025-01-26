function Regulator = CreateFullStateObserverRegulator(args)
arguments
    args.system
end

[A,B,C,D] = ssdata(args.system)

miK = [-2+2.4i, -2-2.4i]; %Poly k matici K
miKe = [-8, -8]; %Poly k matici Ke

K = acker(A,B,miK)
Ke = acker(A', C', miKe')'

At = A - Ke*C - B*K;
Bt = Ke;
Ct = K;
Dt = 0;

Regulator = ss(At,Bt,Ct,Dt)
end