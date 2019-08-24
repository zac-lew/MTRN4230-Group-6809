function [guiString, shape_color, conv_match_ctr] = Detection(conv_match_ctr, shape_color, min_conveyor)
    % CAKE - Computer Vision (Decoration)
    global detector_updated_FINAL;
    global bdim;
    global useRobotCellCamera;
    global camParam_Conv R_Conv t_Conv;
    global scanOnce;
    global cLabels cBboxes;
    global posMatchNum;
    global cImage;
    
    correctColor = false;
    
    %Xi,Yi,Xf,Yf,Angle_delta
    dataVector = zeros(1,5);
    guiString = "";
    % for each frame at a time
    while (true)

        if (~scanOnce) % looking at current frame (no need to move conveyor along)

            %for conveyor camera (get one frame)
            if(useRobotCellCamera)
                cImage = MTRN4230_Image_Capture([],[]); 
                disp('Photo of Conveyor Taken');
            else
                cImage = imread('PnPTestC2.jpg');
                disp('Photo of Conveyor');
                %cImage = imread('.\YOLO_TEST\ConveyorImages\C12.jpg');
            end
            cImage = imcrop(cImage,[515.0,4.50,676.00,720.00]);

                moveConveyor(true,true); 
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
             moveConveyor(true,true);
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

            %angle_roiC = [tempX-bdim/2,tempY-bdim/2,bdim,bdim];
            %aligned_blockC = imcrop(tempROI_imageC,angle_roiC); % CustomerImage remains as RGB for color detection
            block_angleC = 45.0; %checkBlockOrientation(aligned_blockC);

            % 4. Send Data to Robot Arm
            guiString = createPnPData(tempJ,shape_color,tempX,tempY,block_angleC);    

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
               pause(1.0);            
            else
                scanOnce = true;
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
                if (numDuplicates > 0)
                    [correctColor,newX,newY] = isShapeRightColor(shape_color,checkDuplicates(ix),cBboxes,...
                        cImage,tempID,min_conveyor);       
                    if (correctColor)
                        
                        %---increment number of successful block found---
                        conv_match_ctr = conv_match_ctr + 1; 
                        %---increment number of successful block found---
                        
                        fprintf('%d: %s %s FOUND\n',conv_match_ctr,whatColor(shape_color(2,checkDuplicates(ix))),...
                            cLabels(tempID));                            

                        %angle_roiC = [newX-bdim/2,newY-bdim/2,bdim,bdim];
                        %tempROI_imageC = cImage;
                        %aligned_blockC = imcrop(tempROI_imageC,angle_roiC); % CustomerImage remains as RGB for color detection
                        block_angleC = 45.0; %checkBlockOrientation(aligned_blockC);
                        
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
                           pause(1.0);            
                        else
                            scanOnce = true;
                        end                  
            
                        break;
                    end
                end            
            %scanOnce = true;
            end
        end
    end        
end

%% ~~~~~~~FUNCTIONS~~~~~~
% Move Conveyor Forwards
function moveConveyor(direction,enable)
    
    global socket_1;    

    if (enable)
        if (direction) %direction = true (forward towards robot)
            %fwrite command for direction
            %fwrite(socket_1,'CON');
            pause(0.75);
            %fwrite(socket_1,'COF');
        else %direction = false (backward away from robot)
            %fwrite command for direction
            %fwrite(socket_1,'CON');
            pause(0.75);
            %fwrite(socket_1,'COF');
        end
    end
end

function block_angle = checkBlockOrientation(block_image)

    % find angle - Ben's code
    %     grey = im2double(rgb2gray(block_image));
    %     th = otsu(grey);
    %     grey_th = grey >= th;
    %     assumptions for blob detection:
    %     - area is between 1000-1500 pixels
    %     - the blob is black (class 0; white is class 1)
    %     blob detection. detect single square. Refine blob detection
    %     blobs = iblobs(grey_th, 'boundary', 'area', [500, 1500], 'class', 0, 'aspect', [0.8,1]);
    %     edges = blobs(1).edge; % assuming only one blob has been detected - needs fixing by tightening the blob detection criteria so this is true
    %     [leftmost_x_val, leftmost_pt_ind] = min(edges(1,:));
    %     [highest_sq_y_val, highest_pt_ind] = min(edges(2,:));
    %     del_y = edges(2, leftmost_pt_ind) - edges(2, highest_pt_ind);
    %     del_x = edges(1, highest_pt_ind) - edges(1, leftmost_pt_ind);
    %     block_angle = atand(del_y/del_x);
    
end
