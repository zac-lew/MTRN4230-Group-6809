% CAKE - Computer Vision (Decoration)

warning('off','all');
clc;
close all;

% Load .mat files
%load('FINAL_FRCNN_V4.mat');
%load('CalibConv.mat');
%load('CalibTable.mat');

%% 1. Set up TCP Connection

% The robot's IP address and listener port
robot_IP_address = '192.168.125.1'; % Real robot ip address
robot_port = 1025;
global socket;
%socket = openConnection(robot_IP_address,robot_port);

%% 2. Obtain Place Coordinates
useRobotCellCamera = false; % change if using robot cell camera

if (~useRobotCellCamera)
    disp('---USING ROBOT CELL CAMERA---');      
    customerImage = imread('.\YOLO_TEST\Test5.jpg'); 
else
    disp('---USING ROBOT CELL CAMERA---');      
    customerImage = MTRN4230_Image_Capture([]); %for robot cell
end

%[a,b] = imcrop(customerImage);
rectROI = [548.51,286.51,494.98,497.98];
ROI_image = imcrop(customerImage,rectROI); 

figure
imshow(ROI_image)
hold on

% Detect Quirkle Blocks using ML detector
ML_threshold = 0.15;
[bboxes,scores,labels] = detect(detector_updated_final,ROI_image,'Threshold',...
    ML_threshold,'NumStrongestRegions',25);

% Remove double detections from ML
sorted_detect = sort(scores,'descend');

for j = 1 : size(bboxes,1) 
    for check_ctr = 1 : size(bboxes,1)  
        D = sqrt((bboxes(j,1) - bboxes(check_ctr,1))^2 +...
            (bboxes(j,2) - bboxes(check_ctr,2))^2);
        if (D < 15 && D > 0)
            if scores(j) > scores(check_ctr)
                bboxes(check_ctr,:) = 0;                
            else
                bboxes(j,:) = 0;                
            end            
        end    
    end
end  

% Annotate BB around detected shapes
for j = 1 : size(bboxes,1)

    if (scores(j) < ML_threshold)
        continue;
    end
    
    if (bboxes(j,1) ~= 0)
        rectangle('Position',[bboxes(j,1),bboxes(j,2),bboxes(j,3),bboxes(j,4)],'EdgeColor'...
              ,'r','LineWidth',2); 
        ML_1 = sprintf('%0.3f',scores(j));
        ML_2 = sprintf('%s',labels(j));
        text(bboxes(j,1)-10,bboxes(j,2)-25,{[ML_1];[ML_2]},'FontSize',10,'Color','r','FontWeight','bold')
    end
    
end

% 3. HSV Color Filtering + Localisation

num_blocks = find(bboxes(:,1) ~= 0);
num_blocks = size(num_blocks,1);

% ROI on cropped region of 9x9 grid
hsv_path = rgb2hsv(ROI_image);

% array to store which color (R,G,B,Y)
color_array = zeros(1,4); 
color_hsv_low = zeros(1,3);
color_hsv_hi = zeros(1,3);

curr_filter_on = 1;
max_hsv = 4;
filter_counter = num_blocks;
block_counter = 0;

% "struct" to store x,y,z and hsv filter
cv_block_struct = zeros(4,num_blocks);
block_struct_row = 1;

    % Making HSV filtering dynamic and automatically iterate through all
    % 4 HSV filter ranges

for h = curr_filter_on:max_hsv % encoding RGBY as 1234

    [color_hsv_hi,color_hsv_low] = HSV_Iterator(h);

    % Create mask to find pixels with desired HSV ranges (binary mask) -
    % Current iterated HSV filter    
    mask_desired = (hsv_path(:,:,1) >= color_hsv_low(1)) & (hsv_path(:,:,1) <= color_hsv_hi(1)) & ...
            (hsv_path(:,:,2) >= color_hsv_low(2) ) & (hsv_path(:,:,2) <= color_hsv_hi(2)) & ...
            (hsv_path(:,:,3) >= color_hsv_low(3) ) & (hsv_path(:,:,3) <= color_hsv_hi(3));

    SE = strel('disk',2);
    mask_desired = imdilate(mask_desired,SE);

    stats = regionprops(mask_desired,'basic');
    centroids = cat(1,stats.Centroid);

    areas = cat(1,stats.Area); %(suitable area > 150)
    [sort_area_m,sorted_area_row] = sort(areas,'descend'); 
    min_block_size = 450;
    % Checking max number of items WITH a particular HSV filter
    for p = 1:filter_counter

      [color_row,~] = size(sort_area_m); %number of centroids per color
      % Error handling if no suitably sized object with color is present

        if (p > color_row)
            continue;
        elseif (block_counter > num_blocks)
            continue;
        elseif (color_row < num_blocks && sort_area_m(p,1) < min_block_size)
            continue;
        end                

        if (sort_area_m(p,1) >= min_block_size)
            hold on
            % Only plot a + if there is a suitable sized binary area           
            plot(centroids(sorted_area_row(p),1),centroids(sorted_area_row(p),2),'g+','LineWidth',2)
            cv_block_struct(1,block_struct_row) = centroids(sorted_area_row(p),1);
            cv_block_struct(2,block_struct_row) = centroids(sorted_area_row(p),2);
            cv_block_struct(3,block_struct_row) = 147.00;
            cv_block_struct(4,block_struct_row) = h; % which HSV filter was used
            block_counter = block_counter + 1;
            block_struct_row = block_struct_row + 1;
        else
            % not even one suitable block was found in particular color
            continue;
        end
    end 
end

% -----------ERROR CHECKING-----------
% Cleaning up double color localistaion errors AND outside ML results
for j = 1 : size(cv_block_struct,2) 
    for check_ctr = 1 : size(cv_block_struct,2)  
        D = sqrt((cv_block_struct(1,j) - cv_block_struct(1,check_ctr))^2 +...
            (cv_block_struct(2,j) - cv_block_struct(2,check_ctr))^2);
        if (D < 10 && D > 0)
            cv_block_struct(1,check_ctr) = 0;
            cv_block_struct(2,check_ctr) = 0;
        end    
    end
end            

image_place_data = cv_block_struct; % Image coordinate frame

pick_counter = 1;
j = 1;
missingCentroid = false;
shape_color = zeros(2,size(bboxes,1));
image_place_data = fix(image_place_data);

while (pick_counter <= size(bboxes,1) && missingCentroid == false)
    % each bounding box vector from FRCNN in turn
    ROI = bboxes(pick_counter,:); 
    while (j <= size(bboxes,1))
        % check if any of the color/centroids are in this current ROI 
        tf = isInROI(ROI,image_place_data(1,j),image_place_data(2,j));

        if (tf == true)            
            % Store which shape
            shape_color(1,pick_counter) = labels(pick_counter);
            % Store which color (matching)
            shape_color(2,pick_counter) = image_place_data(4,j);
            pick_counter = pick_counter + 1; % fill in shape_color array
            % after each successful match
            % Break out of loop
            plot(image_place_data(1,j),image_place_data(2,j),'bo','LineWidth',1.5);
            break;
        end

        % Error handle in case of missing centroid
        if (j == size(bboxes,1))
            % Can't find centroid

            if (pick_counter < size(bboxes,1))
                pick_counter = pick_counter + 1;
                break;
            else
                missingCentroid = true; % raise flag
                break;
            end                         
        end

        % if tf is false (the current centroid is not in current ROI
        j = j+1; % increment to next centroid location from CV
    end

    j = 1; % reset counter for the next centroid check
end

for r = 1 : size(shape_color,2)
    fprintf('%s %s\n',whatShape(shape_color(1,r)),whatColor(shape_color(2,r)));
end

% ---------------------Orientation of Blocks----------------------
% Image processing to determine orientation of blocks
% Call function for each detected block in turn

bdim = 50;
tempROI_image = ROI_image;
for k = 1: size(shape_color,2)

    hold on

    % avoid calculating angle if missing color/shape match up
    if (shape_color(1,k) == 0 || shape_color(2,k) == 0)
        continue;
    end

    % Make temp comparison image (for each block)    
    angle_roi = [image_place_data(1,k)-bdim/2,image_place_data(2,k)-bdim/2,bdim,bdim];
    aligned_block = imcrop(tempROI_image,angle_roi); % CustomerImage remains as RGB for color detection

    % Call function to detect orientation
    % block_angle = checkBlockOrientation(aligned_block);
    block_angle = 2.0;        
    image_place_data(5,k) = round(block_angle);
    tempROI_image = ROI_image;
end

hold off

% Convert image points to world coordinates (PLACE)            
world_place_data = zeros(4,size(image_place_data,1));

for bCount = 1:num_blocks

    world_place_data(1:2,bCount) = pointsToWorld(camParam_Table,...
        R_Table,t_Table,...
        [548.51+image_place_data(1,bCount),286.51+image_place_data(2,bCount)]);
end

disp('Obtained PLACE Coordinates');
%% 3. Obtain PICK Coordinates at Conveyor

correctShape = false;
conv_match_ctr = 1;
correctColor = false;
foundAllBlocks = false;
pick_array = zeros(4,size(world_place_data,2));
checkMatch = false;
frameScanOnce = false;
nextPulse = true;

cImage = imread('.\YOLO_TEST\ConveyorImages\Test5_C.jpg');
    cImage = imcrop(cImage,[515.0,4.50,676.00,720.00]);
    figure
    imshow(cImage);

while (foundAllBlocks ~= true)

    %Xi,Yi,Xf,Yf,Angle_delta
    dataVector = zeros(1,5);
    
    % Pulse conveyor along if no more potential blocks in 
    % current frame
    if (nextPulse)
        %fwrite(socket,'C03');
        pause(0.75);
        %fwrite(socket,'C04');
    end
    
    %for conveyor camera (get one frame)
    %cImage = MTRN4230_Image_Capture([],[]); 
    %cImage = imcrop(cImage,[515.0,4.50,676.00,720.00]);

    [cBboxes,~,cLabels] = detect(detector_updated_final,cImage,'Threshold',0.30,...
         'NumStrongestRegions',10);
        
        if (frameScanOnce == false)
            frameScanOnce = true;
            posMatchNum = 1;
            for posMatch = 1 : size(shape_color,2)
                [check,~] = shapeCheck(uint8(cLabels),shape_color(1,posMatch));
                if (check == true)
                    posMatchNum = posMatchNum + 1;
                end
            end
        end
        
        while (correctShape == false)        

            % Look for a shape_color pair in current frame @ conveyor                            
            anyShape = false;
            while (anyShape == false)
                for j = 1 : size(shape_color,2)
                    [check,id] = shapeCheck(uint8(cLabels),shape_color(1,j));
                    if (check == true)
                        anyShape = true;
                        posMatchNum = posMatchNum - 1;
                        break;
                    end
                end
                break;
            end                

            if (anyShape == true) %redundant check (keep if still using sequential)
                anyShape = false;    
                correctShape = true; % if current shape from shape_col match array is detected

                disp('Stop the Conveyor!');                   
                % Annotate Shape detection result
                %imshow(cImage);
                hold on
                rectangle('Position',[cBboxes(id,1),cBboxes(id,2),cBboxes(id,3),cBboxes(id,4)],'EdgeColor'...
                    ,'g','LineWidth',2); 

                % 2. Check if the matched shape is in right color
                tempCtr = 0;                 
                hsv_pathC = rgb2hsv(cImage);

                % Create mask to find pixels with desired HSV ranges (binary mask) -
                % from customer image results
                csv_encoding = shape_color(2,j);
                [color_hsv_hi,color_hsv_low] = HSV_IteratorC(csv_encoding);               

                mask_desiredC = (hsv_pathC(:,:,1) >= color_hsv_low(1)) & (hsv_pathC(:,:,1) <= color_hsv_hi(1)) & ...
                        (hsv_pathC(:,:,2) >= color_hsv_low(2) ) & (hsv_pathC(:,:,2) <= color_hsv_hi(2)) & ...
                        (hsv_pathC(:,:,3) >= color_hsv_low(3) ) & (hsv_pathC(:,:,3) <= color_hsv_hi(3));

                statsC = regionprops(mask_desiredC,'basic');
                Ccentroids = cat(1,statsC.Centroid);
                Careas = cat(1,statsC.Area); %(suitable area > 150)
                [~,sorted_area_rowC] = sort(Careas,'descend'); 

                if (size(sorted_area_rowC,1) < 2)
                    continue;
                end

                % check labels detected at conveyor    
                colFound = false;
                for ctr = 1 : size(shape_color,2)
                    checkMatch = isInROI(cBboxes(id,:),Ccentroids(sorted_area_rowC(ctr),1),...
                        Ccentroids(sorted_area_rowC(ctr),2));
                    if (checkMatch == true && colFound == false)
                        %which color was in BBox of correct shape
                        colFound = true; 
                        tempCtr = checkMatch; 
                        tempX = Ccentroids(sorted_area_rowC(ctr),1);
                        tempY = Ccentroids(sorted_area_rowC(ctr),2);
                    end
                end

                % scan over largest centroids in matching color
                if (tempCtr == true) 
                    % if yes -> correctColor = true;
                    correctColor = true;
                    disp('Correct Color AND Correct Shape!');
                    plot(tempX,tempY,'g*','LineWidth',2);
                    checkMatch = false; % reset
                    csv_encoding = 0; % reset csv encoding for next
                    % shape detection
                else
                    correctColor = false;
                    disp('Incorrect Color BUT Correct Shape!');
                end                                

                if (correctColor == true)
                    tempROI_imageC = cImage;
                    fprintf('%d: %s %s FOUND\n',conv_match_ctr,whatColor(shape_color(2,j)),cLabels(id));                            

                    % 3. Detected pose (match to customer's desired pose)

                    angle_roiC = [tempX-bdim/2,tempY-bdim/2,bdim,bdim];
                    aligned_blockC = imcrop(tempROI_imageC,angle_roiC); % CustomerImage remains as RGB for color detection
                    block_angleC = 45.0; %checkBlockOrientation(aligned_blockC);

                    % 4. Send Data to Robot Arm

                    % i) PICK COORDINATES
                    dataVector(1,1:2) = pointsToWorld(camParam_Conv,...
                    R_Conv,t_Conv,[515.0+tempX,4.50+tempY]);                    
                    % Swap X and Y (to match robot frame)
                    dataVector(1,[1,2]) = dataVector(1,[2,1]); 

                    % ---CHECK REACHABILITY (world coordinates)---
                    robotReach = sqrt(dataVector(1,1)^2 + (dataVector(1,2)^2));
                    if (robotReach > 23 && robotReach < 550)
                        %  do nothing
                    else
                        disp('Not Reachable');
                        break;
                    end
                    
                    % ii) PLACE COORDINATES                            
                    dataVector(1,3) = world_place_data(1,j);
                    dataVector(1,4) = world_place_data(2,j);

                    % iii) ANGLE                               
                    dataVector(1,5) = block_angleC - image_place_data(5,j);
                    
                    guiString = sendPnP(dataVector); %array-string HERE                  
                    fprintf('Sent %s to GUI!\n',guiString);

                    tempCtr = 0;
                    % to scan for overall
                    correctColor = false; % reset flag                        

                    % If all required blocks are found
                    missingBlocks = find(shape_color(1,:) ~= 0);
                    if (conv_match_ctr == missingBlocks)
                        foundAllBlocks = true;
                        fprintf('~~~ALL %d BLOCKS FOUND AND PLACED ON CAKE~~'...
                            ,conv_match_ctr);
                        % sendPnP sends '[0,0,0,0,0]'
                        doneAllBlocks = [0,0,0,0,0];
                        finishPnP = sendPnP(doneAllBlocks); %array-string HERE                  
                        fprintf('Sent %s to GUI!\n',finishPnP);
                        pause(2);
                        break;
                    end

                    % If a block and color is successfully found, remove this
                    % from the array so the conveyor does not look for it again
                    shape_color(1,j) = -1;                                
                    check = false; % reset current T/F detection
                    correctShape = false;
                    conv_match_ctr = conv_match_ctr + 1; % max number of blocks
                    %close;
                else

                    % If NOT, see if there were any other detected objects
                    % in same frame
                    disp('Checking Similar Shapes?');
                    shape_color(1,j) = -1;
                    correctShape = false;
                    
                    if (posMatchNum == 0)
                        nextPulse = false; % do not move conveyor if more shapes to check
                    end
                    
                    %close;
                end                    

            end                    
            break;
        end
    disp('NEXT'); %search next pulse/frame         
    
end

disp('DONE DEMO of DECORATION');

%% ---------------------FUNCTIONS----------------------

% Array to String for GUI
function blockInfo = sendPnP(dataV)
    global socket;

    %Make into string '[Xi,Yi,Xf,Yf,A]'
    %32.9119  532.4741  376.4049  141.8782   43.0000

    startBracket = '[';
    endBracket = ']';
    dataV =  fix(dataV);
    stringBlock = string(dataV);    
    blockInfo = join(stringBlock,",");
    blockInfo = strcat(startBracket,blockInfo,endBracket);
    
%     fwrite(socket, blockInfo); % 1x5 array for pick and place shape blocks
%     str = fgetl(socket);
%     fprintf(char(str));
%     disp("First send")
%     i = 0;
%     while (~strcmp(str,"ACK"))
%         i = i+1;
%         fprintf("Looking for ACK: %d", i);
%         fwrite(socket,blockInfo);
%         str = fgetl(socket);
%         fprintf("\n");
%         fprintf(char(str));
%     end
%     i = 0;
%     while (~strcmp(str,"DONE"))
%         fprintf("Looking for DONE: %d", i);
%         str = fgetl(socket);
%         fprintf(char(str));
%         fprintf("\n");
%     end
%     fprintf("\n");
%     fprintf("Next block \n");
end

% Check if any label from ML detector on live frame
% matches the designated customer shape/color     
function [shapeFound,shapeID] = shapeCheck(curr_Labels,curr_Shape)
%encoded shape ~ 1-6
    
    shapeFound = false;
    shapeID = 0;
    
    %eg: args are cLabels,shape_color(1,j)
    for k = 1 : size(curr_Labels,1)
        if (curr_Labels(k) == curr_Shape)
            shapeFound = true;
            shapeID = k;
        end
    end

end

% Encoding for color and shape
function shapeName = whatShape(match_shape)

    if (match_shape == 1)
        shapeName = 'Circle';
    elseif (match_shape == 2)
        shapeName = 'Clover';
    elseif (match_shape == 3)
        shapeName = 'CrissCross';
    elseif (match_shape == 4)
        shapeName = 'Diamond';
    elseif (match_shape == 5)
        shapeName = 'Square';
    elseif (match_shape == 6)
        shapeName = 'Starburst';
    else
        shapeName = '-';
    end
    
end

function colorName = whatColor(match_col)

    if (match_col == 1)
        colorName = 'Red';
    elseif (match_col == 2)
        colorName = 'Green';
    elseif (match_col == 3)
        colorName = 'Blue';
    elseif (match_col == 4)
        colorName = 'Yellow';
    else
        colorName = '-';
    end
    
end

function checkMatch = isInROI(ROI,x,y)
   
    checkMatch = false;

    %ROI is (x,y,x_length,y_length)
    if ((x > ROI(1) && x < ROI(1) + ROI(3)) && (y > ROI(2) && y < ROI(2) + ROI(4)))
        checkMatch = true;
    end
    
    % else checkMatch remains false

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

% HSV at Conveyor
function [color_hsv_hi,color_hsv_low] = HSV_IteratorC(counter)
    if (counter == 1)
        % Threshold for HSV - RED
        color_hsv_low = [0.90,0.405,0.100];
        color_hsv_hi = [0.980,1.00,0.900];
    end
    if (counter == 2)
        % Threshold for HSV - GREEN
        color_hsv_low = [0.180,0.300,0.010];
        color_hsv_hi = [0.430,0.700,0.620];
    end
    if (counter == 3)
        % Threshold for HSV - BLUE
        color_hsv_low = [0.542,0.290,0.00];
        color_hsv_hi = [0.701,1.00,0.807];        
    end
    if (counter == 4)
        % Threshold for HSV - YELLOW
        color_hsv_low = [0.10,0.500,0.152];
        color_hsv_hi = [0.230,0.800, 0.850];
    end
end

% HSV at Robot Cell
function [color_hsv_hi,color_hsv_low] = HSV_Iterator(counter)
    if (counter == 1)
        % Threshold for HSV - RED
        color_hsv_low = [0.750,0.275,0.20];
        color_hsv_hi = [1.00,0.800,0.800];
    end
    if (counter == 2)
        % Threshold for HSV - GREEN
        color_hsv_low = [0.180,0.200,0.084];
        color_hsv_hi = [0.330,0.700,0.600];
    end
    if (counter == 3)
        % Threshold for HSV - BLUE
        color_hsv_low = [0.542,0.290,0.20];
        color_hsv_hi = [0.801,1.00,0.807];        
    end
    if (counter == 4)
        % Threshold for HSV - YELLOW
        color_hsv_low = [0.10,0.165,0.152];
        color_hsv_hi = [0.200,0.600, 0.850];
    end
end

function robot_image = MTRN4230_Image_Capture (varargin)
%Table Camera (Robot Cell)
% {
   if nargin == 0 || nargin == 1
        fig1 =figure(1);
        axe1 = axes ();
        axe1.Parent = fig1;
        vid1 = videoinput('winvideo', 1, 'MJPG_1600x1200');
        video_resolution1 = vid1.VideoResolution;
        nbands1 = vid1.NumberOfBands;
        %img1 = imshow(zeros([video_resolution1(2), video_resolution1(1), nbands1]), 'Parent', axe1);
        src1 = getselectedsource(vid1);
        src1.ExposureMode = 'manual';
        src1.Exposure = -4;
        src1.Contrast = 21;
        src1.Brightness = 128;
        robot_image = getsnapshot(vid1);
    end
%}
% Conveyor Camera
% {
    if nargin == 0 || nargin == 2
        fig2 =figure(2);
        axe2 = axes ();
        axe2.Parent = fig2;
        vid2 = videoinput('winvideo', 2, 'MJPG_1600x1200');
        video_resolution2 = vid2.VideoResolution;
        nbands2 = vid2.NumberOfBands;
        src2 = getselectedsource(vid2);
        src2.ExposureMode = 'manual';    
        src2.Exposure = -4;        
        src2.Brightness = 165;
        src2.Contrast = 32;
        robot_image = getsnapshot(vid2);
    end

%}
end


% Open client_server
function socketInUse = openConnection(robot_IP_address,robot_port)
    
% Open a TCP connection to the robot.
    socket = tcpip(robot_IP_address, robot_port);
    set(socket, 'ReadAsyncMode', 'continuous');
    fopen(socket);

    % Check if the connection is valid.+6

    if(~isequal(get(socket, 'Status'), 'open'))
       warning(['Could not open TCP connection to ', robot_IP_address, ' on port ', robot_port]);
       return;
    end
    
    socketInUse = socket;
    
end
