function hasSuccesfullyPassed = CheckWayPointTrack(bodyXYZPosition, actualTime, timeForWaypointPasage, wayPoints, positionTolerance)
    % Determine if waypoint passage was successful within tolerance
    
    % Early exit if not at a waypoint time
    if ~ismember(actualTime, timeForWaypointPasage)
        hasSuccesfullyPassed = 1;
        return
    end

    % Calculate bounds for each axis
    tol = positionTolerance;
    target = wayPoints';
    
    % Create bounds matrix: [min, max] for each axis
    bounds = [target - tol, target + tol];
    
    % Check position against bounds
    success = true;
    for i = 1:3
        if ~(bodyXYZPosition.(char(96 + i)) >= bounds(i,1) && ...
            bodyXYZPosition.(char(96 + i)) <= bounds(i,2))
            success = false;
            break
        end
    end
    
    hasSuccesfullyPassed = success;
end
