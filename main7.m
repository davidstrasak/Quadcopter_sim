clc;
clear;
clf;
close all;
format compact

addpath('C:\Users\student\Desktop\Quadcopter_sim')

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

% Gravitational acceleration
g = 9.81;

% Simulation parameters
deltaT = 0.01;              % [s]
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

%Inicializace vizualizace
figure(WindowState="maximized")
hold on
grid on

% Simulation
for i = 1 : deltaT : simulationTime

    if i == 1 
        quadcopterActualState = quadcopterInitState
    end

    t = i;
    state = [quadcopterActualState.BodyXYZPosition.X;
        quadcopterActualState.BodyXYZVelocity.X;
        quadcopterActualState.BodyXYZPosition.Y;
        quadcopterActualState.BodyXYZVelocity.Y;
        quadcopterActualState.BodyXYZPosition.Z;
        quadcopterActualState.BodyXYZVelocity.Z;
        quadcopterActualState.BodyEulerAngle.Phi;
        quadcopterActualState.BodyAngularRate.dPhi;
        quadcopterActualState.BodyEulerAngle.Theta;
        quadcopterActualState.BodyAngularRate.dTheta;
        quadcopterActualState.BodyEulerAngle.Psi;
        quadcopterActualState.BodyAngularRate.dPsi;
];
    T = 100;
    M1 = 0;
    M2 = 0;
    M3 = 0;
    g = 9.81;
    m = quadcopter.physicalParameters.Mass;
    Ix = quadcopter.physicalParameters.I(1,1);
    Iy = quadcopter.physicalParameters.I(2,2);
    Iz = quadcopter.physicalParameters.I(3,3);

    targetWaypoint = wayPoints(2,:);

    positionError = targetWaypoint - [state(1), state(3), state(5)];

    [~, x] = ode45(@(t,x) QuadcopterDynamics(t,x,T,M1,M2,M3,g,m,Ix,Iy,Iz), [t t+deltaT], state);
    
    



    % Pregulator.X = wayPoints(2,1) - quadcopterActualState.BodyXYZPosition.X;
    % Pregulator.Y = wayPoints(2,2) - quadcopterActualState.BodyXYZPosition.Y;
    % Pregulator.Z = wayPoints(2,3) - quadcopterActualState.BodyXYZPosition.Z;
    % 
    % Pregulator.X = Pregulator.X * 10;
    % Pregulator.Y = Pregulator.Y * 10;
    % Pregulator.Z = Pregulator.Z * 10;
    % 
    % Pregulator.thrust = (Pregulator.X^2 + Pregulator.Y^2 + Pregulator.Z^2)^(1/2);

    % Pregulator
        
    % if isnan(Pregulator.thrust)
    %     msgbox('Quadcopter was removed from existence', 'Error', 'error');
    %     break
    % end
    
    % Action for total thrust
    quadcopter.TotalThrustControlAction(neededThrustInZ);
    % Action for attitude
    % prvni ovlada smer Y, druhy ovlada smer X, treti ovlada smer Z
    % quadcopter.AttitudeControlAction(Pregulator.Y, Pregulator.X, 0);
    quadcopter.AttitudeControlAction(0, 0, 0);
   
    % Update state of quadcopter
    quadcopter.UpdateState();

    % Get actual state of quadcopter
    quadcopterActualState = quadcopter.GetState();

    %Vizualizace
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
    pause(deltaT)

    
    % Crash check
    if (quadcopterActualState.BodyXYZPosition.Z >= 0)
        msgbox('Quadcopter Crashed!', 'Error', 'error');
        break;
    end

    % Waypoint check
    if (CheckWayPointTrack(...
                quadcopterActualState.BodyXYZPosition,...
                i * deltaT,...
                timeForWaypointPasage,...
                wayPoints,...
                positionTolerance))
        msgbox('Quadcopter did not passed waypoint', 'Error', 'error');
        break;
    end
end