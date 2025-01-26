function stateSpace = configure_observer(axisObj)
%% original
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
   %% alternative
   %  arguments
   %      axisObj (1,1) struct
   %  end
   % 
   %  % Extract system matrices
   %  A = axisObj.A;
   %  B = axisObj.B;
   %  C = axisObj.C(1,:);  % Force SISO configuration
   %  poles = axisObj.poles;
   % 
   %  % Design full-state observer parameters
   %  obs_poles = [-8, -8];  % Observer poles (must be different from regulator poles)
   % 
   %  % Calculate regulator and observer gains
   %  K = acker(A, B, poles);
   %  Ke = acker(A', C', obs_poles)';
   % 
   %  % Construct observer-based controller state-space
   %  At = A - Ke*C - B*K;
   %  Bt = [Ke B];  % Inputs: [y u]
   %  Ct = -K;
   %  Dt = [0 0];   % Direct feedthrough terms
   % 
   %  % Create 1x1 state-space system (input: y, output: u)
   %  stateSpace = ss(At, Bt(:,1), Ct, Dt(1));
end