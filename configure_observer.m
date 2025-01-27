function stateSpace = configure_observer(axisObj)
%Function that configures a full state observer based on A,B,C and D
%matrixes and outputs the regulator that it creates
    arguments
        axisObj (1,1) struct
    end

    A = axisObj.A;
    B = axisObj.B;
    C = axisObj.C;
    D = axisObj.D;
    poles = axisObj.poles;

    polesKe = [-8, -8];  % Much faster observer

    K = place(A,B,poles);
    Ke = place(A', C', polesKe')';

    K=10*K;

    At = A - Ke*C - B*K;
    Bt = Ke;
    Ct = K;
    Dt = 0;

    stateSpace = ss(At, Bt, Ct, Dt);
end