function guiString = createPnPData(tempJ,shape_color,X,Y,block_angleC)
    
    global camParam_Conv R_Conv t_Conv;
    
    % i) PICK COORDINATES
    dataVector(1,1:2) = pointsToWorld(camParam_Conv,...
    R_Conv,t_Conv,[515.0+X,4.50+Y]);                    
    % Swap X and Y (to match robot frame)
    dataVector(1,[1,2]) = dataVector(1,[2,1]); 

    % ---CHECK REACHABILITY (world coordinates)---
    robotReachDistance = sqrt(dataVector(1,1)^2 + (dataVector(1,2)^2));
    
    canReach = checkReachability(robotReachDistance);

    if (~canReach)
        disp('Not Reachable');
        % move conveyor back/forwards
        if (dataVector(2) < -150) %TODO: Improve this if possible
            moveConveyor(true,true); %direction = true (forward towards robot)
        elseif (dataVector(2) > -50)  
            moveConveyor(false,true); %direction = false (away from robot)
        end
    end

    % ii) PLACE COORDINATES                            
    dataVector(1,3) = shape_color(3,tempJ);
    dataVector(1,4) = shape_color(4,tempJ);

    % iii) ANGLE                               
    dataVector(1,5) = block_angleC - shape_color(5,tempJ);

    guiString = sendPnP(dataVector); %array-string HERE                  
    fprintf('Sent %s to GUI!\n',guiString);
end
