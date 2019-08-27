function [moveConveyorFlag, guiString, shape_color, conv_match_ctr,moveDirection] = Detection(conv_match_ctr, shape_color, min_conveyor,bdim)
    % CAKE - Computer Vision (Decoration)
    global detector_updated_FINAL;
    global camParam_Conv R_Conv t_Conv;
    global scanOnce;
    global cLabels cBboxes;
    global posMatchNum;
    global cImage;
    global Offline;
    
    correctColor = false;
    moveConveyorFlag = false;    
    failedDetect = 0;
    moveDirection = 1; %forward
    
    %Xi,Yi,Xf,Yf,Angle_delta
    dataVector = zeros(1,5);
    guiString = "";
    % for each frame at a time
    while (true)
        
        if (failedDetect > 3)
                moveConveyorFlag = true;  
                moveDirection = 1; %forward
                scanOnce = false;
                break;
        end
            
        if(~scanOnce)            

            tempJ = 0;
            ix = 0;

                %for conveyor camera (get one frame)
                if(Offline)
                    %cImage = imread('PnPTestC2.jpg');
                    cImage = MTRN4230_Image_Capture([],[]);

                    disp('Photo of Conveyor');
                else
                    cImage = MTRN4230_Image_Capture([],[]);
                    disp('Photo of Conveyor Taken');

                end
                cImage = imcrop(cImage,[515.0,4.50,676.00,720.00]);

                    %moveConveyor(true,true); 
                    disp('Moved Conveyor Once');
                    [cBboxes,~,cLabels] = detect(detector_updated_FINAL,cImage,'Threshold',0.20,...
                            'NumStrongestRegions',10);

                    scanOnce = true;
                    for posMatch = 1 : size(cBboxes,1)
                        matchCheck = uint8(cLabels);
                        if (ismember(matchCheck(posMatch),shape_color(1,:)) ~= 0)
                            posMatchNum = posMatchNum + 1;
                        end
                    end 
        end
        
        % Look for a shape_color pair in current frame @ conveyor                            
        anyShape = false;
        while (anyShape == false)
            for j = 1 : size(shape_color,2)
                [check,id] = shapeCheck(uint8(cLabels),shape_color(1,j));
                if (check == true && uint8(cLabels(id)) ~= 0)
                    anyShape = true;
                    tempID = id;
                    tempJ = j;
                    break;
                end
            end          
            break;
        end    

         % no shape found in current frame
         % if anyShape is still false after all labels
         if (anyShape == false)
             moveConveyorFlag = true;
             break; % activate conveyor to move to next set of blocks
         end 

        disp('Shape Found!');   
        
        % Found one of the potential matching shapes
        posMatchNum = posMatchNum - 1;
        pause(1.0);
        
        % Annotate Shape detection result
        imshow(cImage);
        hold on
        rectangle('Position',[cBboxes(tempID,1),cBboxes(tempID,2),cBboxes(tempID,3),cBboxes(tempID,4)],'EdgeColor'...
            ,'g','LineWidth',2); 

         fprintf('%Possible %s %s FOUND\n',whatColor(shape_color(2,tempJ)),cLabels(tempID));
        % 2. Check if the matched shape is in right color       
        % Create mask to find pixels with desired RGB ranges (binary mask) -
        % from customer image results
        [correctColor,tempX,tempY] = isShapeRightColor(shape_color,tempJ,cBboxes,cImage,tempID,min_conveyor);       

        if (correctColor == true)
            colFound = false;
            tempROI_imageC = cImage;
            
            %---increment number of successful block found---
            conv_match_ctr = conv_match_ctr + 1;
            %---increment number of successful block found---
            
            fprintf('%d: %s %s FOUND\n',conv_match_ctr,whatColor(shape_color(2,tempJ)),cLabels(tempID));                            
            cLabels(tempID) = '';

            % 3. Detected pose (match to customer's desired pose)

            angle_roiC = [tempX-bdim/2,tempY-bdim/2,bdim,bdim];
            aligned_blockC = imcrop(tempROI_imageC,angle_roiC); % CustomerImage remains as RGB for color detection
            block_angleC = 45.0; %checkBlockOrientation(aligned_blockC,2);

            % 4. Send Data to Robot Arm
            [guiString,reachIssue,convDirection] = createPnPData(tempJ,shape_color,tempX,tempY,block_angleC);    
            
            if (reachIssue)
                if (convDirection == 1)
                    moveDirection = 1; %forward
                else
                    moveDirection = 0; %back
                end
                moveConveyorFlag = true;                    
                break;
            end            

            % If a block and color is successfully found, remove this
            % from the array so the conveyor does not look for it again
            shape_color(1,tempJ) = -1; 
            shape_color(2,tempJ) = -1;
            check = false; % reset current T/F detection
            correctShape = false;
            correctColor = false; % reset flag
           
            % Keep current frame and do NOT move conveyor
            if (posMatchNum == 0)
               disp('Scan More Blocks, Move Conveyor!');
               scanOnce = false;
               moveConveyorFlag = true;
               moveDirection = 1; %forward
               pause(1.0);            
            else
                scanOnce = true;
                moveConveyorFlag = false;
            end                       
            break;

        else
            % Correct shape but wrong color
            % check for duplicates
            disp('KEEP FRAME');
            shapeID = shape_color(1,tempJ);
            checkDuplicates = find(shape_color(1,:) == shapeID);
            numDuplicates = size(checkDuplicates,2);
            for j = 1 : size(checkDuplicates,2) %eg: [3 4]
                ix = find(checkDuplicates(1,:) ~= tempJ);
                % if more than one of the shape, check color
                if (numDuplicates == 0)
                    moveConveyorFlag = true;
                    return;
                end
                if (numDuplicates > 0)
                    [correctColor,newX,newY] = isShapeRightColor(shape_color,checkDuplicates(ix),cBboxes,...
                        cImage,tempID,min_conveyor);       
                    if (correctColor)
                        
                        %---increment number of successful block found---
                        conv_match_ctr = conv_match_ctr + 1; 
                        %---increment number of successful block found---
                        
                        fprintf('%d: %s %s FOUND\n',conv_match_ctr,whatColor(shape_color(2,checkDuplicates(ix))),...
                            cLabels(tempID));                            

                        angle_roiC = [newX-bdim/2,newY-bdim/2,bdim,bdim];
                        tempROI_imageC = cImage;
                        aligned_blockC = imcrop(tempROI_imageC,angle_roiC); % CustomerImage remains as RGB for color detection
                        block_angleC = checkBlockOrientation(aligned_blockC,2);
                        
                        guiString = createPnPData(checkDuplicates(ix),shape_color,newX,newY,block_angleC);   
                         % If a block and color is successfully found, remove this
                        % from the array so the conveyor does not look for it again
                        shape_color(1,checkDuplicates(ix)) = -1; 
                        shape_color(2,checkDuplicates(ix)) = -1;
                        check = false; % reset current T/F detection
                        correctShape = false;
                        correctColor = false; % reset flag
                        cLabels(checkDuplicates(ix)) = '';                       
                        
                        % Keep current frame and do NOT move conveyor
                        if (posMatchNum == 0)
                           disp('Scan More Blocks, Move Conveyor!');
                           scanOnce = false;
                           moveConveyorFlag = true;
                           moveDirection = 1; %forward
                           pause(1.0);            
                        else
                            scanOnce = false;
                            moveConveyorFlag = true;
                        end                  
            
                        break;
                    end
                end            
            end
            % Increment faile detect counter
            % remove shape from labels
            failedDetect = failedDetect + 1;
            cLabels(tempID) = '';    
        end
    end        
end
