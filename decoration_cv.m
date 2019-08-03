% MTRN4230 Group Project

% ----------------ChangeLog---------------
% v1. 6/7/19 Initial creation with comments and brief summary. Image capture
% v2. 15/7/19 Improved image capture at robot cell
% v3. 18/7/19 Added in structure for YOLOv2 network
% v4. 21/7/19 Add in color detection code (initial)
% v5. 1/8/19 Added in camera calibration for robotcell and conveyor (TBD). 
% ----------------ChangeLog---------------

% Computer Vision Engineer (Decoration)
% 1. Detect Quirkle blocks as they are supplied using the conveyor (ML)
% 2. Detect the customer's desired decorating pattern from robot cell camera (ML/CV)

% Customer's IMAGE::
% Location = x,y for blocks? (target locations)
% Type of Block = Shape + Colour

% Shapes == (criss cross, clover, starburst, square, diamond, circle}
% Color == (red, blue, green, yellow)

%~~Decoration Computer Vision Pipeline~~

%% 1. Obtain Customer Image @ Robot CEll

% Testing with customer's sample image: (sample2.jpg)
customerImage = imread('sample1.jpg');
imshow(customerImage)

%(LATER ON USE ROBOT CELL CAMERA)
%MTRN4230_Image_Capture([]) %for robot cell

%% 2. Quirkle Blocks from Sample Image (Classification + Localisation)
% YOLOv2 Detection Network for Quirkle Blocks

doTraining = true;

%% ------------------------SET UP TRAINING and TEST SETS----------------

% Load Training Data (using ImageLabeller - ground truth)
%data = load('qLabels.mat');
%QuirkleDataset = data.gTruth; %(export to workspace directly?)

% Randomly split data into a training and test set.
% Set random seed to ensure example training reproducibility.
rng(2);
shuffledIndices = randperm(142);
idx = floor(0.6 * length(shuffledIndices));
trainingData = gTruth.LabelData(shuffledIndices(1:idx),:); %60
testData = gTruth.LabelData(shuffledIndices(idx+1:end),:); %40

%% ------------------------CREATE DETECTION NETWORK-------------------

% YOLOv2 = feature network (pre-trained CNN) & detection network (smaller
% CNN specific to YOLO)

% The image input size should be at least as big as the images in the training image set.
% Define the image input size.
imageSize = [1600 1200 3]; %might need to resize but are RBG

% Define the number of object classes to detect. (6 shapes == 6 classes)
numClasses = 6;

% Create rest of YOLOv2 Detection Network:

% Anchor Box dimensions (analyse more if have time/want to improve accuracy)

%testBB = gTruth.LabelData(1,2)
%tableBB = table2array(testBB)

anchorBoxes = [
    43 59
    65 50
    76 66
    120 98
];

% Load a pretrained ResNet-50. (transfer learning) - Feature Network
baseNetwork = resnet50;

% Specify the feature extraction layer. (Detection network is added after
% this layer)
featureLayer = 'activation_40_relu';

% Create the YOLO v2 object detection network. 
lgraph = yolov2Layers(imageSize,numClasses,anchorBoxes,baseNetwork,featureLayer);

% % Training YOLOv2
% 
%  % Configure the training options. 
%     %  * Lower the learning rate to 1e-3 to stabilize training. 
%     %  * Set CheckpointPath to save detector checkpoints to a temporary
%     %    location. If training is interrupted due to a system failure or
%     %    power outage, you can resume training from the saved checkpoint.
%     options = trainingOptions('sgdm', ...
%         'MiniBatchSize', 16, ....
%         'InitialLearnRate',1e-3, ...
%         'MaxEpochs',10,...
%         'CheckpointPath', tempdir, ...
%         'Shuffle','every-epoch');    
%     
% % Train YOLO v2 detector.
% [detector,info] = trainYOLOv2ObjectDetector(vehicleDataset,lgraph,options);

%% ------------------------TEST MODEL ON A TEST IMAGES--------------------

% Read a test image.
I = imread(testData.imageFilename{end});

% Run the detector.
[bboxes,scores] = detect(detector,I);

% Annotate detections in the image.
I = insertObjectAnnotation(I,'rectangle',bboxes,scores);
imshow(I)

%% ------------------------TEST MODEL ON TEST SET--------------------

% Create a table to hold the bounding boxes, scores, and labels output by
% the detector. 
numImages = height(testData);
results = table('Size',[numImages 3],...
    'VariableTypes',{'cell','cell','cell'},...
    'VariableNames',{'Boxes','Scores','Labels'});

% Run detector on each image in the test set and collect results.
for i = 1:numImages
    
    % Read the image.
    I = imread(testData.imageFilename{i});
    
    % Run the detector.
    [bboxes,scores,labels] = detect(detector,I);
   
    % Collect the results.
    results.Boxes{i} = bboxes;
    results.Scores{i} = scores;
    results.Labels{i} = labels;
end

% Extract expected bounding box locations from test data.
expectedResults = testData(:, 2:end);

% Evaluate the object detector using average precision metric.
[ap, recall, precision] = evaluateDetectionPrecision(results, expectedResults);


%% 3. Color Filtering + Localisation + Classification

% After ML detector is run:
%* Have ID'd which shapes are needed
%* Bounding box/centroid of each shape (hopefully)

% 1. Have centroid locations [x,y] of each block from Customer image
% 2. Detect Required number of objects??

block_locations = [1,2,3,4,5;1,2,3,4,5];

close all
% Testing with customer's sample image:
% customerImage = imread('sample2.jpg'); %5
% customerImage = imread('sample3.jpg'); %2
customerImage = imread('sample4.jpg'); %4
%  customerImage = imread('sample5.jpg'); %3
%  customerImage = imread('sample6.jpg'); %3
% customerImage = imread('sample7.jpg'); %3
num_blocks = 4; 

% Create ROI
%[a,b] = imcrop(customerImage);
rectROI = [542.51,303.51,515.98,444.98];
ROI_image = imcrop(customerImage,rectROI);

%HSV
%hsv_path = rgb2hsv(customerImage);
hsv_path = rgb2hsv(ROI_image);
hsv_path = imgaussfilt(hsv_path,0.5);

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

figure
%imshow(customerImage)
imshow(ROI_image)

    % Making HSV filtering dynamic and automatically iterate through all
    % 4 HSV filter ranges
    
 for h = curr_filter_on:max_hsv 
        
    [color_hsv_hi,color_hsv_low] = HSV_Iterator(h);
    
    % Create mask to find pixels with desired HSV ranges (binary mask) -
    % Current iterated HSV filter    
    mask_desired = (hsv_path(:,:,1) >= color_hsv_low(1)) & (hsv_path(:,:,1) <= color_hsv_hi(1)) & ...
            (hsv_path(:,:,2) >= color_hsv_low(2) ) & (hsv_path(:,:,2) <= color_hsv_hi(2)) & ...
            (hsv_path(:,:,3) >= color_hsv_low(3) ) & (hsv_path(:,:,3) <= color_hsv_hi(3));
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
       min_block_size = 80;
            
      [color_row,color_col] = size(sort_area_m); 
      % Error handling if no suitably sized object with color is present
        if (color_row < num_blocks)
            continue;
        end
            
        if (sort_area_m(p,1) >= min_block_size)
            hold on
            % Only plot a + if there is a suitable sized binary area
            plot(centroids(sorted_area_row(p),1),centroids(sorted_area_row(p),2),'g+','LineWidth',0.5)
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

% 3. Error checking;
yolo_block_check = sprintf('Number of ML blocks detected: %f', num_blocks);
disp(yolo_block_check)
block_check = sprintf('Number of blocks detected: %f', block_counter);
disp(block_check)

if (block_counter ~= num_blocks)
    disp('Error: Incorrect Number of Blocks!!')
end

% 4. Match detected shape with detected color

% Check which detected shape is in which color (compare centroids)
% block_locations = x,y of bb position and shape detected.
% cv_block_struct = x,y of centroid and which hsv filter it had used.

% 5. Orientation of Blocks

% Image processing to determine orientation of blocks
% SURF?

% 6. Confirm PLACE coordinates

%load('robotCellCalib.mat', 'robotCellSession');

% From Extrinsic function output
translationVector = [21.6771020996404,-377.712398323210,859.963449998696];

rotationMatrix = [-0.000532049447634045,0.999998919650671,-0.00137026306816968;...
                0.999994863561974,0.000527715353405883,-0.00316138674858810;...
                -0.00316066022432677,-0.00137193804397179,-0.999994063988857];

% Convert image points to world coordinates
% cv_block_struct (rows 1 and 2)

placeCount = 1;

%[x,y] = pointsToWorld(robotCellSession.CameraParameters,rotationMatrix,translationVector,(point);

% Finalized array of data (might actually make struct later)
cv_block_struct 

% 7. 
% Send commands to Robot Arm (through Ethernet)
% For each Block:
% 1) [X,Y] PLACE COORDINATES (WORLD frame)
% 2) Orientation

%% 5. Detect on Conveyor Belt (Object Detection)

% Change to conveyor camera
%MTRN4230_Image_Capture([],[]) %for conveyor camera
% Load camera calibration
load('conveyorCalib.mat', 'conveyorSession');

% YOLOv2 Network - run conveyor until 1st desired bock. stop conveyor

% Send commands to Robot Arm (through Ethernet)
% For each Block:
% [X,Y] PICK COORDINATES (WORLD frame)


%% FUNCTIONS

function [color_hsv_hi,color_hsv_low] = HSV_Iterator(counter)
    if (counter == 1)
        % Threshold for HSV - RED
        color_hsv_low = [0.825,0.275,0.20];
        color_hsv_hi = [1.00,1.00,1.00];
    end
    if (counter == 2)
        % Threshold for HSV - GREEN
        color_hsv_low = [0.219,0.266,0.084];
        color_hsv_hi = [0.421,1.00,0.378];
    end
    if (counter == 3)
        % Threshold for HSV - BLUE
        color_hsv_low = [0.542,0.290,0.20];
        color_hsv_hi = [0.801,1.00,0.807];        
    end
    if (counter == 4)
        % Threshold for HSV - YELLOW
        color_hsv_low = [0.122,0.335,0.552];
        color_hsv_hi = [0.222,0.936, 0.850];
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

