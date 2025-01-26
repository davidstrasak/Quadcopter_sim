clc, clear, clf, close, format compact;

%% Instructions
% At these times, the quadcopter must pass the appropriate waypoint
timeForWaypointPasage = [110,220,330,440,550]; % [s]

% Waypoints - these points must be flown by quadcopter
wayPoints = [0 0 -6;        % [X, Y, Z] - waypoint in [m]
             1 1 -6;
             2 1 -7;
             3 0 -5;
             1 1 -6];
% Position tolerance
positionTolerance = 0.1;    % [m]

% Simulation parameters
% deltaT = 0.01;              % [s]
deltaT = 0.1;              % [s]
simulationTime = max(timeForWaypointPasage) + 20; % [s]

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

g = quadcopter.g;

% Init the Z axis observer (altitude estimation)
zAxis.A = [0 1; 0 0];
zAxis.B = [0; g];
zAxis.C = eye(2);
zAxis.D = [0;0];
% Desired poles for the alitutde estimation
zAxis.poles = [-2+0.5j, -2-0.5j];
Zregulator = configure_observer(zAxis);

% Init the X axis observer (lateral position estimation)
xAxis.A = [0 1; 0 0];
xAxis.B = [0; 1];
xAxis.C = eye(2);
xAxis.D = [0;0];
% Desired poles for the lateral position estimation
xAxis.poles = [-2+0.5j, -2-0.5j];
Xregulator = configure_observer(xAxis);

% Init the Y axis observer (longitudinal position estimation)
yAxis.A = [0 1; 0 0];
yAxis.B = [0; 1];
yAxis.C = eye(2);
yAxis.D = [0;0];
% Desired poles for the longitudinal position estimation
yAxis.poles = [-2+0.5j, -2-0.5j];
Yregulator = configure_observer(yAxis);

% Init the Phi axis observer (roll angle estimation)
phiAxis.A = [0 1; 0 0];
phiAxis.B = [0; 1];
phiAxis.C = eye(2);
phiAxis.D = [0;0];
% Desired poles for the roll angle estimation
phiAxis.poles = [-2+0.5j, -2-0.5j];
Phiregulator = configure_observer(phiAxis);

% Init the Theta axis observer (pitch angle estimation)
thetaAxis.A = [0 1; 0 0];
thetaAxis.B = [0; 1];
thetaAxis.C = eye(2);
thetaAxis.D = [0;0];
% Desired poles for the pitch angle estimation
thetaAxis.poles = [-2+0.5j, -2-0.5j];
Thetaregulator = configure_observer(thetaAxis);

% Simulation Configuration
targetWaypointIndex = 1; % Active navigation waypoint
targetWaypoint = wayPoints(targetWaypointIndex,:);

% Maximum angle configuration so we are in small angles
maximumAngle = 0.001;

% Init the visualisation
figure(WindowState="maximized")
hold on
grid on
speedUp = 5;

% Set the time in which we will be passing the waypoints
timeToReachWaypoints = timeForWaypointPasage;

pause(2)

%% Simulation
for currentTime = 1 : deltaT : simulationTime 
    %% Updates

    % Update the navigation
    if (currentTime < timeToReachWaypoints(end) && currentTime > timeToReachWaypoints(targetWaypointIndex))
        targetWaypointIndex = targetWaypointIndex + 1;
        targetWaypoint = wayPoints(targetWaypointIndex,:);
    end    

    % Update state of quadcopter
    quadcopter.UpdateState();
    % Get actual state of quadcopter
    quadcopterActualState = quadcopter.GetState();

    %% Regulator
    
    % Z axis control (altitude)

    if (timeToReachWaypoints(targetWaypointIndex) - currentTime) > 1
        targetVerticalVelocity = (quadcopterActualState.BodyXYZPosition.Z - targetWaypoint(3))/...
                        (timeToReachWaypoints(targetWaypointIndex)-currentTime)
    else
        targetVerticalVelocity = 0;
    end
    
    velocityError = quadcopterActualState.BodyXYZVelocity.Z - targetVerticalVelocity;
    if abs(velocityError) < 0.5
        t = linspace(0, deltaT, 3);
        u = linspace(quadcopterActualState.BodyXYZVelocity.Z, targetVerticalVelocity, 3);
    else
        t = [0, deltaT];
        u = [quadcopterActualState.BodyXYZVelocity.Z, targetVerticalVelocity];
    end

    verticalResponse = lsim(Zregulator, u, t);
    thrust = (Mass * g) - verticalResponse(end);


    % X axis control (lateral position)

    if (timeToReachWaypoints(targetWaypointIndex) - currentTime) > 1
        targetLatVelocity = (quadcopterActualState.BodyXYZPosition.Y - targetWaypoint(2))/...
                           (timeToReachWaypoints(targetWaypointIndex)-currentTime);
    else
        targetLatVelocity = 0;
    end
    
    if (abs(quadcopterActualState.BodyXYZVelocity.Y) - abs(targetLatVelocity)) < 0.2
        t = linspace(0, deltaT, 3);
        u = linspace(quadcopterActualState.BodyXYZVelocity.Y, targetLatVelocity, 3);
    else
        t = [0, deltaT];
        u = [quadcopterActualState.BodyXYZVelocity.Y, targetLatVelocity];
    end

    latResponse = lsim(Xregulator, u, t);
    rollReference = (Mass * latResponse(end)) / thrust;
    rollReference = saturate_value(rollReference, maximumAngle);

    rollRateReference = (quadcopterActualState.BodyEulerAngle.Phi - rollReference)/0.05;
        
    t = linspace(0, deltaT, 3);
    u = linspace(quadcopterActualState.BodyAngularRate.dPhi, rollRateReference, 3);
    rollResponse = lsim(Phiregulator, u, t);
    momentY = rollResponse(end) * quadcopter.physicalParameters.I(5);

    if currentTime == 111
        do = 5;
    end

    % Y axis control (longitudinal position)

    if (timeToReachWaypoints(targetWaypointIndex) - currentTime) > 1
        targetLonVelocity = (quadcopterActualState.BodyXYZPosition.X - targetWaypoint(1))/...
                           (timeToReachWaypoints(targetWaypointIndex) - currentTime);
    else
        targetLonVelocity = 0;
    end
    
    if (abs(quadcopterActualState.BodyXYZVelocity.X) - abs(targetLonVelocity)) < 0.2
        t = linspace(0, deltaT, 3);
        u = linspace(quadcopterActualState.BodyXYZVelocity.X, targetLonVelocity, 3);
    else
        t = [0, deltaT];
        u = [quadcopterActualState.BodyXYZVelocity.X, targetLonVelocity];
    end

    lonResponse = lsim(Yregulator, u, t);
    pitchReference = (Mass * - lonResponse(end)) / thrust;
    pitchReference = saturate_value(pitchReference, maximumAngle);

    pitchRateReference = (quadcopterActualState.BodyEulerAngle.Theta - pitchReference)/0.05;
        
    t = linspace(0, deltaT, 3);
    u = linspace(quadcopterActualState.BodyAngularRate.dTheta, pitchRateReference, 3);
    pitchResponse = lsim(Thetaregulator, u, t);
    momentX = pitchResponse(end) * quadcopter.physicalParameters.I(1);


    %% Regulator control actions
    quadcopter.TotalThrustControlAction(thrust);
    quadcopter.AttitudeControlAction(momentX, momentY, 0);

    %% Vizualizace
    if mod(currentTime,deltaT*speedUp) == 0 % the multiplicator of deltaT speeds up the simulation this many times
        hold off
        plot3(quadcopterActualState.BodyXYZPosition.X,quadcopterActualState.BodyXYZPosition.Y,quadcopterActualState.BodyXYZPosition.Z,'xb')
        hold on
        [X,Y] = meshgrid(-0.5:0.1:4,-0.5:0.1:1.5);
        Z = zeros(size(X));
        plot3(X,Y,Z,'k.')
        plot3(wayPoints(:,1),wayPoints(:,2),wayPoints(:,3),'r-')
        xlabel('x')
        ylabel('y')
        zlabel('z')
        pause(0.00001)
        disp('Current time is:')
        disp(currentTime)
        clear X Y Z
    end

    
    % Crash check
    if (quadcopterActualState.BodyXYZPosition.Z >= 0)
        msgbox('Quadcopter Crashed!', 'Error', 'error');
        break;
    end

    % Waypoint check
    if (CheckWayPointTrack(...
                quadcopterActualState.BodyXYZPosition,...
                currentTime * deltaT,...
                timeForWaypointPasage,...
                wayPoints,...
                positionTolerance))
        msgbox('Quadcopter did not passed waypoint', 'Error', 'error');
        break;
    end
end