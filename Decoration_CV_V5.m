% MTRN4230 Group Project

% ----------------ChangeLog---------------
% v1. 6/7/19 Initial creation with comments and brief summary. Image capture
% v2. 15/7/19 Improved image capture at robot cell
% v3. 18/7/19 Added in structure for YOLOv2 network
% v4. 21/7/19 Add in color detection code (initial)
% v5. 1/8/19 Added in camera calibration for robotcell and conveyor (TBD). 
% v6. 4/8/19 Finetuned YOLOv2 implementation
% v7. 7/8/19 Changed to Faster R-CNN. Training better + 2018b compatible
%            Added in conversion from image to world for PLACE coordinates
% v8/ 9/8/19 Initial testing of fully trained ML network
% ----------------ChangeLog---------------

% Computer Vision Engineer (Decoration)
% 1. Detect Quirkle blocks as they are supplied using the conveyor (ML)
% 2. Detect the customer's desired decorating pattern from robot cell camera (ML/CV)

% Shapes == (criss cross, clover, starburst, square, diamond, circle}
% Color == (red, blue, green, yellow)

%~~Decoration Computer Vision Pipeline~~

%% LOAD PRE_TRAINED DETECTOR

% When using pretrained FRCNN    
% Load detector into workspace (pretrained x 2)
%load('FINAL_FRCNN_V2.mat'); 
%load('info.mat')
%disp('Detector & Training Info Loaded!');  

%% 1. Obtain Customer Image @ Robot CEll

% Testing with customer's sample image:
customerImage = imread('.\YOLO_TEST\Test11.jpg');
warning('off','all');
close all

%(LATER ON USE ROBOT CELL CAMERA)
%MTRN4230_Image_Capture([]) %for robot cell

% Intial image processing before the FRCNN Detector

%[a,b] = imcrop(customerImage);
rectROI = [506.51,239.51,576.98,581.98];

customerImage = imsharpen(customerImage,'Radius',5);
% gray_customerImage = rgb2gray(customerImage);
% se = strel('disk',5);
% gray_customerImage = imbinarize(gray_customerImage,0.6);
% dilated_g_customerImage = imdilate(gray_customerImage,se);
% highlighted_blocks = imreconstruct(imcomplement(dilated_g_customerImage),imcomplement(gray_customerImage));
% highlighted_blocks_c = imcomplement(highlighted_blocks);
highlighted_blocks_c = imcrop(customerImage,rectROI);

ROI_image = imcrop(customerImage,rectROI); % CustomerImage remains as RGB for color detection
%ROI_image = customerImage; %duplication? Though need ROI ideally to avoid unnecessary block detection
ROI_image = imsharpen(ROI_image,'Radius',2);

figure
imshow(ROI_image)
hold on

disp('1. Customer Image Obtained')

%% 2. Detect Quirkle Blocks from Sample Image (Classification + Localisation)
%% ------------------------RUN ML MODEL ON CUSTOMER IMAGE--------------------

% Input image must be greater than [224 224]
% Run the Qwirkle detector on customer's image
[bboxes,scores,labels] = detect(detector_updated,highlighted_blocks_c,'Threshold',0.1,'NumStrongestRegions',25);

% Annotate BB detections in the image.

% Draw BB and Labels   
for j = 1 : size(bboxes,1)
    
    rectangle('Position',[bboxes(j,1),bboxes(j,2),bboxes(j,3),bboxes(j,4)],'EdgeColor'...
          ,'r','LineWidth',2); 
    ML_result = sprintf('%f, %s',scores(j),labels(j));
    %disp(ML_result); 
    text(bboxes(j,1)-10,bboxes(j,2)-15,ML_result,'FontSize',10,'Color','r','FontWeight','bold')
end

disp('2. DONE: Ran Faster RCNN Qwirkle Block Detector on Image')

% Now have bounding boxes, scores and labels

%% 3. Color Filtering + Localisation

% After ML detector is run:
% 1) Labels of which Shape that was detected
% 2) Approx Bounding box/centroid of each shape
% 3) Number of blocks detected (length of BB)

%close all

% ---------------------Detect Colors----------------------

%HSV
%hsv_path = rgb2hsv(customerImage);
num_blocks = size(scores,1);

%imshow(ROI_image); % ROI on cropped region of 9x9 grid (RGB Colorscape still)
hsv_path = rgb2hsv(ROI_image);

% -Initiate Threshold Iterator-
color_array = zeros(1,4); % array to store which color (R,G,B,Y)
color_hsv_low = zeros(1,3);
color_hsv_hi = zeros(1,3);

curr_filter_on = 1;
max_hsv = 4;
filter_counter = num_blocks;
block_counter = 0;

% "struct" to store x,y and hsv filter
cv_block_struct = zeros(3,num_blocks);
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

    % For each region area for a particular HSV filter,
    % check if suitable sized area.

    areas = cat(1,stats.Area); %(suitable area > 150)
    [sort_area_m,sorted_area_row] = sort(areas,'descend'); 
       
    % Checking max number of items WITH a particular HSV filter
    for p = 1:filter_counter
            
       % block_locations(1,k) = centroids(sorted_area_row(p),1);
       % block_locations(2,k) = centroids(sorted_area_row(p),1);   
       min_block_size = 300;
            
      [color_row,color_col] = size(sort_area_m); 
      % Error handling if no suitably sized object with color is present
        if (color_row < num_blocks)
            continue;
        end
            
        if (sort_area_m(p,1) >= min_block_size)
            hold on
            % Only plot a + if there is a suitable sized binary area
            plot(centroids(sorted_area_row(p),1),centroids(sorted_area_row(p),2),'g+','LineWidth',1.5)
            cv_block_struct(1,block_struct_row) = centroids(sorted_area_row(p),1);
            cv_block_struct(2,block_struct_row) = centroids(sorted_area_row(p),2);
            cv_block_struct(3,block_struct_row) = h; % which HSV filter was used
            block_counter = block_counter + 1;
            block_struct_row = block_struct_row + 1;
        else
            % not even one suitable block was found in particular color
            continue;
        end
    end 
end

 % debug show the results of computer vision color/pose detection
 image_place_data = cv_block_struct; % Image coordinate frame

% 3. Error checking;
% yolo_block_check = sprintf('Number of ML blocks detected: %f', size(scores,1));
% disp(yolo_block_check)
% block_check = sprintf('Number of blocks detected: %f', block_counter);
% disp(block_check)
% 
% if (block_counter ~= num_blocks)
%     disp('Error: Incorrect Number of Blocks!!')
% end

disp('3. DONE: Qwirkle Localisation and Color')

%% ---------------------Match Shape with Color----------------------

% Check which detected shape is in which color (compare centroids)
% cv_block_struct = x,y of centroid and which hsv filter it had used.
% analyse rows 1 (x) and rows 2 (y)
% bboxes is array of bounding boxes of detected shapes
% [x,y,x_length,y_length] where [x,y] = upper-left corner of BB

%for j = 1 : size(scores,1)
    
%end

% Display Results!
% eg: Green Clover, Red Starburst

disp('4. NOT DONE: Matched Shape and Color ')

%% ---------------------Orientation of Blocks----------------------
% Image processing to determine orientation of blocks
% Call function for each detected block in turn
% 100 x 100. make image around each centroid

surfMin = 130; % minimum size for SURF detector to work
bdim = surfMin;
offset = 10; % pixel boundary around a block
rect_length = 65;
%tempROI_image = zeros(size(ROI_image,1),size(ROI_image,2));
tempROI_image = ROI_image;

for k = 1: size(image_place_data,2)

    hold on
    
    % Make comparison image (for each block)
    surf_x = image_place_data(1,k);
    surf_y = image_place_data(2,k);
    
    % ROI centred on each detected block in turn
    surf_roi = [surf_x-bdim/2,surf_y-bdim/2,bdim,bdim];
    aligned_block = imcrop(tempROI_image,surf_roi); % CustomerImage remains as RGB for color detection
    
    % Convert to grayscale
    aligned_block = rgb2gray(aligned_block);
    block_angle = checkBlockOrientation(aligned_block);
    
    % Overlay 45 degrees or 0 degrees rectangle over each block
    % Visual on rotation angle of each block
    if (block_angle == 0)

        x = [surf_x-rect_length/2 surf_x-rect_length/2 surf_x+rect_length/2 surf_x+rect_length/2 surf_x-rect_length/2];
        y = [surf_y-rect_length/2 surf_y+rect_length/2 surf_y+rect_length/2 surf_y-rect_length/2 surf_y-rect_length/2];
        %plot(x,y,'b', 'LineWidth',2)

    else

        x = [surf_x surf_x-rect_length/2 surf_x surf_x+rect_length/2 surf_x];
        y = [surf_y-rect_length/2 surf_y surf_y+rect_length/2 surf_y surf_y-rect_length/2];
        %plot(x,y,'b', 'LineWidth',2)

    end

    tempROI_image = ROI_image;
end

hold off

disp('5. DONE: Block Orientation');

%% ---------------------Place Coordinates----------------------
% Applied to whole image struct (after it has been filled)

disp('DONE: ML and Computer Vision at Robot Cell');

load('calibrationSession.mat', 'calibrationSession');

% From Extrinsic function output
translationVector = [21.6771020996404,-377.712398323210,859.963449998696];

rotationMatrix = [-0.000532049447634045,0.999998919650671,-0.00137026306816968;...
                0.999994863561974,0.000527715353405883,-0.00316138674858810;...
                -0.00316066022432677,-0.00137193804397179,-0.999994063988857];

% Convert image points to world coordinates (from struct)
% cv_block_struct (rows 1 and 2)

for bCount = 1:num_blocks

    cv_block_struct(1:2,bCount) = pointsToWorld(calibrationSession.CameraParameters,rotationMatrix,translationVector,...
        [cv_block_struct(1,bCount),cv_block_struct(2,bCount)]);
end

% Array after conversion from image to world
world_place_data = cv_block_struct; % World coordinate frame

%disp('6. DONE: PLACE Coordinates');

%% Send PLACE Data to Robot Arm
% For each Block:
% 1) [X,Y] PLACE COORDINATES (WORLD frame)
% 2) Orientation (add on as 4th row to the struct_array)

%disp('7. DONE: Sent PLACE to Robot');

%% 4. Detect on Conveyor Belt (Real-time Object Detection!!)

% Change to conveyor camera
%MTRN4230_Image_Capture([],[]) %for conveyor camera
% Load camera calibration

% ML network - run conveyor until 1st desired bock. stop conveyor

%conveyorImage = imread('.\YOLO_TEST\C1.jpg');
%imshow(conveyorImage)

%disp('8. DONE: Detected Shapes on Conveyor');

%% 5. Send PICK Data to Robot Arm
% For each Block:
% [X,Y] PICK COORDINATES (WORLD frame)



%% FUNCTIONS

function block_angle = checkBlockOrientation(block_image)

    % SURF descriptors = encoder vector which shows regions where the image
    % is not affected by brightness, scale or rotation
    surf_points = detectSURFFeatures(block_image);    
    block_original = imread('block_0_pos.jpg'); % constant reference
    block_original = rgb2gray(block_original);

    % detect SURF Features (as many blobs as possible)
    ptsOriginal  = detectSURFFeatures(block_original,'MetricThreshold',25);
    ptsDistorted = detectSURFFeatures(block_image,'MetricThreshold',25);

    % Extract feature descriptors
    [featuresOriginal,  validPtsOriginal]  = extractFeatures(block_original, ptsOriginal);
    [featuresDistorted, validPtsDistorted] = extractFeatures(block_image, ptsDistorted);

    % Match features
    indexPairs = matchFeatures(featuresOriginal, featuresDistorted);
    matchedOriginal = validPtsOriginal(indexPairs(:,1));
    matchedDistorted = validPtsDistorted(indexPairs(:,2));

    % Calculate Orientation
    [tform,~,~] = estimateGeometricTransform(...
        matchedDistorted, matchedOriginal, 'similarity');

    Tinv  = tform.invert.T;
    ss = Tinv(2,1);
    sc = Tinv(1,1);
    block_angle = round(atan2(ss,sc)*180/pi);
    
    % Meaningful orientations to send to robot arm
    if ((block_angle) <= -90)
        block_angle = 45;
    else
        block_angle = 0;
    end                   
    
end

function [color_hsv_hi,color_hsv_low] = HSV_Iterator(counter)
    if (counter == 1)
        % Threshold for HSV - RED
        color_hsv_low = [0.850,0.275,0.20];
        color_hsv_hi = [1.00,0.800,0.800];
    end
    if (counter == 2)
        % Threshold for HSV - GREEN
        color_hsv_low = [0.255,0.050,0.084];
        color_hsv_hi = [0.430,1.00,0.700];
    end
    if (counter == 3)
        % Threshold for HSV - BLUE
        color_hsv_low = [0.542,0.290,0.20];
        color_hsv_hi = [0.801,1.00,0.807];        
    end
    if (counter == 4)
        % Threshold for HSV - YELLOW
        color_hsv_low = [0.10,0.235,0.452];
        color_hsv_hi = [0.250,0.950, 0.850];
    end
end

function capture_image (vid,name)
  snapshot = getsnapshot(vid);
  imwrite(snapshot, [name, datestr(datetime('now'),'_mm_dd_HH_MM_SS'), '.jpg']);
  disp([name ' Captured']);
end

function MTRN4230_Image_Capture (varargin)
    close all;
%Table Camera (Robot Cell)
% {
    if nargin == 0 || nargin == 1
        fig1 =figure(1);
        axe1 = axes ();
        axe1.Parent = fig1;
        vid1 = videoinput('winvideo', 1, 'MJPG_1280x720');
        video_resolution1 = vid1.VideoResolution;
        nbands1 = vid1.NumberOfBands;
        img1 = imshow(zeros([video_resolution1(2), video_resolution1(1), nbands1]), 'Parent', axe1);
        prev1 = preview(vid1,img1);
        src1 = getselectedsource(vid1);
        cam1_capture_func = @(~,~)capture_image(vid1,'QBlock_RC');
        prev1.ButtonDownFcn = cam1_capture_func;
        fig1.KeyPressFcn = cam1_capture_func;
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
        img2 = imshow(zeros([video_resolution2(2), video_resolution2(1), nbands2]), 'Parent', axe2);
        prev2 = preview(vid2,img2);
        src2 = getselectedsource(vid2);
        cam2_capture_func = @(~,~)capture_image(vid2,'QBlock_CON');
        fig2.KeyPressFcn = cam2_capture_func;
        prev2.ButtonDownFcn = cam2_capture_func;
    end
%}
end

