clc, clear, clf, close, format compact;

%% Instructions
% At these times, the quadcopter must pass the appropriate waypoint
timeForWaypointPassage = [110,220,330,440,550]; % [s]

% Waypoints - these points must be flown by quadcopter
wayPoints = [0 0 -6;        % [X, Y, Z] - waypoint in [m]
             1 1 -6;
             2 1 -7;
             3 0 -5;
             1 1 -6];
% Position tolerance
positionTolerance = 0.1;    % [m]

% Simulation parameters
deltaT = 0.01;              % [s]
% deltaT = 0.1;              % [s]
simulationTime = max(timeForWaypointPassage) + 20; % [s]

% Constants
% Radians to degree
RadianToDegree = 180/pi; 
% Degree to radians
DegreeToRadian = pi/180;

% Quadcopter parameters
Mass = 1.3;                 % [kg]
ArmLenght = 0.27;           % [m]
XMomentOfInertia =  0.023;  % [kg m^2]
YMomentOfInertia =  0.023;  % [kg m^2]
ZMomentOfInertia =  0.047;  % [kg m^2]

% Initial state of quadcopter
quadcopterInitState.BodyXYZPosition.X = 0;  % [m]
quadcopterInitState.BodyXYZPosition.Y = 0;  % [m]
quadcopterInitState.BodyXYZPosition.Z = -6; % [m]           
quadcopterInitState.BodyXYZVelocity.X = 0;            
quadcopterInitState.BodyXYZVelocity.Y = 0;            
quadcopterInitState.BodyXYZVelocity.Z = 0;
quadcopterInitState.BodyEulerAngle.Phi = 0;
quadcopterInitState.BodyEulerAngle.Theta = 0;
quadcopterInitState.BodyEulerAngle.Psi = 0;
quadcopterInitState.BodyAngularRate.dPhi = 0;
quadcopterInitState.BodyAngularRate.dTheta = 0;
quadcopterInitState.BodyAngularRate.dPsi = 0;

% Control variables - total thrust and moments on each control axis
quadcopterInitControlInputs = [0, 0, 0, 0]';     % (T, M1, M2, M3)
                           
% Initiate quadcopter
quadcopter = Quadcopter(Mass, ...               
               XMomentOfInertia, ...
               YMomentOfInertia, ...
               ZMomentOfInertia, ...
               quadcopterInitState, ...
               quadcopterInitControlInputs,...
               deltaT);

%% Init the rest of variables
% Get gravity constant from quadcopter model
grav = quadcopter.g;

%% Vertical dynamics
% State space model for altitude control
% States: [position; velocity]
% A matrix represents natural system dynamics
vertDyn.A = [0 1; 0 0];
% B matrix includes gravity compensation for hover
vertDyn.B = [0; grav];
% Full state feedback
vertDyn.C = eye(2);
vertDyn.D = [0; 0];

% Complex poles for better damping characteristics
% Imaginary parts create oscillatory response with controlled overshoot
vertDyn.poles = [-4+2j, -4-2j];

altitudeController = configure_observer(vertDyn);

%% Longitudinal dynamics (X-axis motion)
% Similar structure to vertical dynamics but without gravity term
% This controller generates desired longitudinal acceleration which is
% passed on to the pitch control
longDyn.A = [0 1; 0 0];
longDyn.B = [0; 1];
longDyn.C = eye(2);
longDyn.D = [0; 0];

% Slightly less aggressive poles than vertical control
longDyn.poles = [-4+1j, -4-1j];

forwardController = configure_observer(longDyn);

%% Lateral dynamics (Y-axis motion)
% Identical to longitudinal dynamics due to quadcopter symmetry
% This controller generates desired lateral acceleration which is passed on
% to roll control
latDyn.A = [0 1; 0 0];
latDyn.B = [0; 1];
latDyn.C = eye(2);
latDyn.D = [0; 0];

latDyn.poles = [-4+1j,-4-1j];

sideController = configure_observer(latDyn);

%% Roll dynamics
% Angular motion control for roll axis
% States: [angle; angular_rate]
rollDyn.A = [0 1; 0 0];
rollDyn.B = [0; 1];
rollDyn.C = eye(2);
rollDyn.D = [0; 0];

rollDyn.poles = [-4+1j,-4-1j];

rollController = configure_observer(rollDyn);

%% Pitch dynamics
% Similar to roll due to quadcopter symmetry
pitchDyn.A = [0 1; 0 0];
pitchDyn.B = [0; 1];
pitchDyn.C = eye(2);
pitchDyn.D = [0; 0];

pitchDyn.poles = [-4+1j,-4-1j];

pitchController = configure_observer(pitchDyn);

%% Simulation variables
% Waypoint tracking variables
currentTarget = 1;
targetPoint = wayPoints(currentTarget,:);

% Safety limits for attitude control
angleLimit = 0.01;    % Maximum allowed tilt angle (radians)

% Setup visualization
figure(WindowState="maximized")
hold on
grid on
speedUp = 50;  % Simulation speedup factor

pause(2)

%% Main control loop
for timeStep = 0 : deltaT : simulationTime
    %% Update states of simulation
    % Check if it's time to switch to next waypoint based on predefined timing
    if (timeStep < timeForWaypointPassage(end) && timeStep > timeForWaypointPassage(currentTarget))
        currentTarget = currentTarget + 1;
        targetPoint = wayPoints(currentTarget,:);
    end    

    % Get latest state from quadcopter dynamics simulation
    quadcopter.UpdateState();
    currentState = quadcopter.GetState();
    
    %% Altitude control
    % Extract current vertical states
    altitudeCurrent = currentState.BodyXYZPosition.Z;
    verticalSpeed = currentState.BodyXYZVelocity.Z;
    
    % Calculate required vertical velocity using time-to-go
    % This creates a velocity profile that gets drone to target by specified time
    if (timeForWaypointPassage(currentTarget) - timeStep) > 1
        targetVertSpeed = (currentState.BodyXYZPosition.Z - targetPoint(3))/...
                         (timeForWaypointPassage(currentTarget)-timeStep);
    else
        targetVertSpeed = 0;  % So we are not trying to divide by 0
    end
    
    % Setup control input for altitude controller
    timeVec = linspace(0, deltaT, 3);   
    % Replicate commands for continuous control simulation
    refAlt = [targetVertSpeed, targetVertSpeed, targetVertSpeed];
    measAlt = [verticalSpeed, verticalSpeed, verticalSpeed];
    inputAlt = [refAlt; measAlt];
    
    % Generate thrust correction from controller
    altitudeCommand = lsim(altitudeController, inputAlt, timeVec);
    
    % Total thrust = hover thrust (mg) + correction
    thrustCommand = Mass * grav + altitudeCommand(end);
    quadcopter.TotalThrustControlAction(thrustCommand);

    %% Forward control (X-axis)
    % Similar structure to altitude control but for horizontal motion
    forwardSpeed = currentState.BodyXYZVelocity.X;
    forwardPos = currentState.BodyXYZPosition.X;
    
    % Time-based velocity profile for X direction
    if (timeForWaypointPassage(currentTarget) - timeStep) > 1
        targetForwardSpeed = (currentState.BodyXYZPosition.X - targetPoint(1))/...
                            (timeForWaypointPassage(currentTarget)-timeStep);
    else
        targetForwardSpeed = 0; % So we are not trying to divide by 0
    end
    
    timeVec = linspace(0, deltaT, 3);
    refFwd = [targetForwardSpeed, targetForwardSpeed, targetForwardSpeed];
    measFwd = [forwardSpeed, forwardSpeed, forwardSpeed];
    inputFwd = [refFwd; measFwd];
    
    forwardCommand = lsim(forwardController, inputFwd, timeVec);

    %% Pitch control
    % Cascade control: output of position controller becomes pitch reference
    pitchAngle = currentState.BodyEulerAngle.Theta;
    pitchRate = currentState.BodyAngularRate.dTheta;

    % Convert linear acceleration command to pitch angle
    % Using small angle approximation: ax = g*theta
    targetPitch = (Mass * forwardCommand(end)) / thrustCommand;
    targetPitch = saturate(targetPitch, angleLimit);  % Safety limit

    % Calculate required pitch rate for smooth transition
    targetPitchRate = (pitchAngle - targetPitch)/0.5;  % 0.5s time constant
    timeVec = linspace(0, deltaT, 3);

    refPitch = [targetPitchRate, targetPitchRate, targetPitchRate];
    measPitch = [pitchRate, pitchRate, pitchRate];
    inputPitch = [refPitch; measPitch];

    pitchCommand = -lsim(pitchController, inputPitch, timeVec);

    % Convert to physical moment using inertia
    pitchMoment = pitchCommand(end) * quadcopter.physicalParameters.I(1,1);

    %% Lateral control (Y-axis)
    % Mirror of forward control but for sideways motion
    lateralSpeed = currentState.BodyXYZVelocity.Y;
    lateralPos = currentState.BodyXYZPosition.Y;
    
    if (timeForWaypointPassage(currentTarget) - timeStep) > 1
        targetLateralSpeed = (currentState.BodyXYZPosition.Y - targetPoint(2))/...
                            (timeForWaypointPassage(currentTarget)-timeStep);
    else
        targetLateralSpeed = 0; % So we are not trying to divide by 0
    end
    
    timeVec = linspace(0, deltaT, 3);
    refLat = [targetLateralSpeed, targetLateralSpeed, targetLateralSpeed];
    measLat = [lateralSpeed, lateralSpeed, lateralSpeed];
    inputLat = [refLat; measLat];
    
    lateralCommand = lsim(sideController, inputLat, timeVec);

    %% Roll control
    % Cascade control: lateral position controller output becomes roll reference
    rollAngle = currentState.BodyEulerAngle.Phi;
    rollRate = currentState.BodyAngularRate.dPhi;
    
    % Convert lateral acceleration command to roll angle
    % Negative sign due to coordinate system convention
    targetRoll = -(Mass * lateralCommand(end)) / thrustCommand;
    targetRoll = saturate(targetRoll, angleLimit);
    
    targetRollRate = (rollAngle - targetRoll)/0.5;
    timeVec = linspace(0, deltaT, 3);
    
    refRoll = [targetRollRate, targetRollRate, targetRollRate];
    measRoll = [rollRate, rollRate, rollRate];
    inputRoll = [refRoll; measRoll];
    
    rollCommand = -lsim(rollController, inputRoll, timeVec);
    
    rollMoment = rollCommand(end) * quadcopter.physicalParameters.I(1,1);

    % Apply computed control moments
    quadcopter.AttitudeControlAction(rollMoment, pitchMoment, 0);

    %% Visualization
    vizInput = struct();
    vizInput.timeStep = timeStep;
    vizInput.deltaT = deltaT;
    vizInput.speedUp = speedUp;
    vizInput.currentState = currentState;
    vizInput.wayPoints = wayPoints;
    vizInput.targetPoint = targetPoint;
    vizInput.positionTolerance = positionTolerance;
    vizInput.forwardPos = forwardPos;
    vizInput.lateralPos = lateralPos;
    vizInput.altitudeCurrent = altitudeCurrent;
    visualize(vizInput);

    %% Safety checks
    
    % Crash Check
    if (currentState.BodyXYZPosition.Z >= 0)
        msgbox('Quadcopter Crashed!', 'Error', 'error');
        break;
    end

    % Waypoint check
    if (~CheckWayPointTrack(...
                currentState.BodyXYZPosition,...
                timeStep * deltaT,...
                timeForWaypointPassage,...
                wayPoints,...
                positionTolerance))
        msgbox('Quadcopter did not passed waypoint', 'Error', 'error');
        break;
    end
end
