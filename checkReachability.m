% Checking if robot can reach centroid of a detected block
function canReach = checkReachability(distance)

    canReach = false;
    
    % estimated radial distance from origin to pt
    if (distance > 23 && distance < 550) 
        canReach = true;
    end
    
end
