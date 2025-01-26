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

%% Z axis SS

zAxis.A = [0 1; 0 0];
zAxis.B = [0; g];
zAxis.C = eye(2);
zAxis.D = [0; 0];

zAxis.poles = [complex(-4,1), complex(-4,-1)];

mysysZ = configure_observer(zAxis);

%% X axis

xAxis.A = [0 1; 0 0];
xAxis.B = [0; 1];
xAxis.C = eye(2);
xAxis.D = [0; 0];

xAxis.poles = [complex(-4,1), complex(-4,-1)];

mysysX = configure_observer(xAxis);

%% Y axis

yAxis.A = [0 1; 0 0];
yAxis.B = [0; 1];
yAxis.C = eye(2);
yAxis.D = [0; 0];

yAxis.poles = [complex(-4,1), complex(-4,-1)];

mysysY = configure_observer(yAxis);

%% Phi

phiAxis.A = [0 1; 0 0];
phiAxis.B = [0; 1];
phiAxis.C = eye(2);
phiAxis.D = [0; 0];

phiAxis.poles = [complex(-4,1), complex(-4,-1)];

mysysPhi = configure_observer(phiAxis);

%% Theta

thetaAxis.A = [0 1; 0 0];
thetaAxis.B = [0; 1];
thetaAxis.C = eye(2);
thetaAxis.D = [0; 0];

thetaAxis.poles = [complex(-4, 1), complex(-4, -1)];

mysysTheta = configure_observer(thetaAxis);

%% Simulation variables

% starting waypoint
whatWaypoint = 1;

% set current waypoint 
currentWaypoint = wayPoints(whatWaypoint,:);

% data storage variables
tracking_FPS = 4;       % 4 Hz, that is 0.25 s (1/FPS)
y = 1;                  % index for data storage 

% saturation limit
limitAngle = 0.01;
maxAngRate = limitAngle/0.05; % Match 0.05s time constant from your code

% Init the visualization
figure(WindowState="maximized")
hold on
grid on
speedUp = 100;

pause(2)
%% Simulation
for i = 0 : deltaT : simulationTime
    %% Update states of simulation

    % Change waypoint once time passes
    if (i < timeForWaypointPasage(end) && i > timeForWaypointPasage(whatWaypoint))
        whatWaypoint = whatWaypoint + 1;
        currentWaypoint = wayPoints(whatWaypoint,:);
    end    

    % Update state of quadcopter
    quadcopter.UpdateState();

    % Get actual state of quadcopter
    quadcopterActualState = quadcopter.GetState();
    
        %% Z
% souhlas
    if (timeForWaypointPasage(whatWaypoint) - i) > 1
        requiredVelZ = (quadcopterActualState.BodyXYZPosition.Z - currentWaypoint(3)) / (timeForWaypointPasage(whatWaypoint) - i);
    else
        requiredVelZ = 0;
    end


    % if ( abs(quadcopterActualState.BodyXYZVelocity.Z) - abs(requiredVelZ) ) < 0.5
    % t = linspace(0,deltaT, 3);
    % u = repmat(linspace(quadcopterActualState.BodyXYZVelocity.Z, requiredVelZ,3),2,1);
    % else
        t = [0, deltaT];
        u = repmat([quadcopterActualState.BodyXYZVelocity.Z, requiredVelZ],2,1);
    % end

    out = lsim(mysysZ, u, t);

    % out(end+1) = 0;

    Thrust = (Mass * g) + out(1) + out(2);

    quadcopter.TotalThrustControlAction(Thrust);

    if i == 50 || quadcopterActualState.BodyXYZVelocity.X ~= 0
        hello = 1;
    end

    %% X
% souhlas
    if (timeForWaypointPasage(whatWaypoint) - i) > 1
        requiredVelX = (quadcopterActualState.BodyXYZPosition.X - currentWaypoint(1))/(timeForWaypointPasage(whatWaypoint)-i);
    else
        requiredVelX = 0;
    end
    
    if ( abs(quadcopterActualState.BodyXYZVelocity.X) - abs(requiredVelX) ) < 0.2
        t = linspace(0,deltaT, 3);
        u = repmat(linspace(quadcopterActualState.BodyXYZVelocity.X, requiredVelX, 3),2,1);
    else
        t = [0, deltaT];
        u = repmat([quadcopterActualState.BodyXYZVelocity.X, requiredVelX],2,1);
    end

    

    out = lsim(mysysX, u, t);
    out(end + 1) = 0;

    requiredAngTheta = (Mass * (- out(1) - out(2) - out(3))) / Thrust;
    requiredAngTheta = saturate_value(requiredAngTheta, limitAngle);

    requiredAngRateTheta = (quadcopterActualState.BodyEulerAngle.Theta - requiredAngTheta)/0.05;
    requiredAngRateTheta = saturate_value(requiredAngRateTheta,maxAngRate);
        
    t = linspace(0,deltaT, 3);
    u = repmat(linspace(quadcopterActualState.BodyAngularRate.dTheta, requiredAngRateTheta, 3),2,1);

    out = lsim(mysysTheta, u, t);
    out(end+1) = 0;

    MomentY = (out(1) + out(2) + out(3)) * quadcopter.physicalParameters.I(1);
    MomentY = saturate_value(MomentY,0.001);

    %% Y

    if (timeForWaypointPasage(whatWaypoint) - i) > 1
        requiredVelY = (quadcopterActualState.BodyXYZPosition.Y - currentWaypoint(2))/(timeForWaypointPasage(whatWaypoint)-i);
    else
        requiredVelY = 0;
    end
    
    if ( abs(quadcopterActualState.BodyXYZVelocity.Y) - abs(requiredVelY) ) < 0.2
        t = linspace(0,deltaT, 3);
        u = repmat(linspace(quadcopterActualState.BodyXYZVelocity.Y, requiredVelY, 3),2,1);
    else
        t = [0, deltaT];
        u = repmat([quadcopterActualState.BodyXYZVelocity.Y, requiredVelY],2,1);
    end

    out = lsim(mysysY, u, t);
    out(end + 1) = 0;

    requiredAngPhi = (Mass * (out(1) + out(2) + out(3))) / Thrust;
    requiredAngPhi = saturate_value(requiredAngPhi, limitAngle);

    requiredAngRatePhi = (quadcopterActualState.BodyEulerAngle.Phi - requiredAngPhi)/0.05;
    requiredAngRatePhi = saturate_value(requiredAngRatePhi,maxAngRate);
        
    t = linspace(0,deltaT, 3);
    u = repmat(linspace(quadcopterActualState.BodyAngularRate.dPhi, requiredAngRatePhi, 3),2,1);

    out = lsim(mysysPhi, u, t);
    out(end+1) = 0;

    MomentX = (out(1) + out(2) + out(3)) * quadcopter.physicalParameters.I(5);
    MomentX = saturate_value(MomentX,0.001);


    quadcopter.AttitudeControlAction(MomentX, MomentY, 0);

    %% Visualization
    if mod(i,deltaT*speedUp) == 0 || i > 110% the multiplicator of deltaT speeds up the simulation this many times 
        hold off
        plot3(quadcopterActualState.BodyXYZPosition.X,quadcopterActualState.BodyXYZPosition.Y,quadcopterActualState.BodyXYZPosition.Z,'x',Color="#00FF00")
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
        disp(i)
        clear X Y Z
    end



    % %% Tracking - only for later use for plotting
    % 
    % % mod(i, 0.25) ensures that only data is recorded at 4 Hz, can be lower
    % if (mod(i, (1/tracking_FPS) ) == 0)
    %     trackDroneXYZPosition(1,y) = quadcopterActualState.BodyXYZPosition.X;
    %     trackDroneXYZPosition(2,y) = quadcopterActualState.BodyXYZPosition.Y;
    %     trackDroneXYZPosition(3,y) = quadcopterActualState.BodyXYZPosition.Z;
    % 
    %     trackDroneXYZVelocity(1,y) = quadcopterActualState.BodyXYZVelocity.X;
    %     trackDroneXYZVelocity(2,y) = quadcopterActualState.BodyXYZVelocity.Y;
    %     trackDroneXYZVelocity(3,y) = quadcopterActualState.BodyXYZVelocity.Z;
    % 
    %     trackDroneAngularRate(1,y) = quadcopterActualState.BodyAngularRate.dPhi;
    %     trackDroneAngularRate(2,y) = quadcopterActualState.BodyAngularRate.dTheta;
    %     trackDroneAngularRate(3,y) = quadcopterActualState.BodyAngularRate.dPsi;
    % 
    %     trackDroneXYZMoment(1,y) = MomentX;
    %     trackDroneXYZMoment(2,y) = MomentY;
    %     trackDroneXYZMoment(3,y) = 0;
    % 
    %     trackDroneAngle(1,y) = quadcopterActualState.BodyEulerAngle.Phi;
    %     trackDroneAngle(2,y) = quadcopterActualState.BodyEulerAngle.Theta;
    % 
    %     trackWaypoint(:,y) = currentWaypoint;
    % 
    %     trackThrust(y) = Thrust;
    %     trackTime(y) = i;
    %     y = y + 1;
    % end
    
    % Crash check
    if (quadcopterActualState.BodyXYZPosition.Z >= 0)
        msgbox('Quadcopter Crashed!', 'Error', 'error');
        break;
    end

    % % Waypoint check
    % if (~CheckWayPointTrack(...
    %             quadcopterActualState.BodyXYZPosition,...
    %             i,...
    %             timeForWaypointPasage,...
    %             currentWaypoint,...
    %             positionTolerance))
    %     msgbox('Quadcopter did not pass waypoint', 'Error', 'error')
    %     disp(i);
    %     disp(timeForWaypointPasage(whatWaypoint));
    %     disp(positionTolerance);
    %     disp(quadcopterActualState.BodyXYZPosition)
    %     break;
    % end
end

% plot_data(trackTime, trackThrust, ...
%     trackDroneXYZPosition, trackDroneXYZVelocity, trackDroneXYZMoment, ...
%     trackDroneAngle, trackDroneAngularRate, trackWaypoint)