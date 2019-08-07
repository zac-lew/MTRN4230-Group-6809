% MTRN4230 Group Project

% ----------------ChangeLog---------------
% v1. 6/7/19 Initial creation with comments and brief summary. Image capture
% v2. 15/7/19 Improved image capture at robot cell
% v3. 18/7/19 Added in structure for YOLOv2 network
% v4. 21/7/19 Add in color detection code (initial)
% v5. 1/8/19 Added in camera calibration for robotcell and conveyor (TBD). 
% v6. 4/8/19 Finetuned YOLOv2 implementation
% v7. 7/8/19 Changed to Faster R-CNN. Training better + 2018b compatible
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
%% ------------------------SET UP TRAINING and TEST SETS----------------

% Load training data
load('trainingData.mat'); 

% Modify path to training dataset
% for Tdata_counter = 1: height(YtrainingData)
%     [filepath,name,ext] = fileparts(YtrainingData.imageFilename{Tdata_counter});
%     % NOTE: change the directory to the current working directory
%     new_path = fullfile();
%     YtrainingData.imageFilename{Tdata_counter} = new_path;
% end

% Set random seed to ensure example training reproducibility.
rng(9);

% Randomly split data into a training and test set.
shuffledIndices = randperm(height(YtrainingData));
idx = floor(0.70 * length(shuffledIndices) );
trainingData = YtrainingData(shuffledIndices(1:idx),:);
testData = YtrainingData(shuffledIndices(idx+1:end),:);

disp('YTraining Data loaded!');
disp('~Training and Test Datasets Created~');

% Display one of the sample training images/labelled ground Truth

I = imread(trainingData.imageFilename{end});

% Insert the ROI labels.
I = insertShape(I,'Rectangle',trainingData.Starburst{end});

% Resize and display image.
I = imresize(I,3);
imshow(I)

%% ------------------------CREATE DETECTION NETWORK-------------------

 % Configure the training options. 
    options = trainingOptions('sgdm', ...
        'MiniBatchSize', 1, ....
        'InitialLearnRate',1e-3, ...
        'MaxEpochs',1,...
        'Verbose',true,...
        'CheckpointPath', tempdir, ...
        'VerboseFrequency',5,....
        'Shuffle','every-epoch');    

% -------------Testing Faster-RCNN------------------
 [detector, info] = trainFasterRCNNObjectDetector(trainingData, 'resnet50', options, ...
        'NegativeOverlapRange', [0 0.3], ...
        'PositiveOverlapRange', [0.6 1]);

% Plot training accuracy / interation
% figure
% plot(info.TrainingLoss)
% grid on
% xlabel('Number of Iterations')
% ylabel('Training Loss for Each Iteration')


%% ------------------------TEST MODEL ON A TEST IMAGE--------------------

% NOTE: The minimum input image size must be equal to or greater
% than the input size in image input layer of the network.

% Load detector into workspace
%load('trained_FasterRCNN.mat')
%detector = trained_FasterRCNN.detector;

% Read a test image.
I = imread(testData.imageFilename{1});

% Run the detector.
[bboxes,scores,labels] = detect(detector,I,'Threshold',0.20)

% Annotate detections in the image.
if ~isempty(bboxes)
    I = insertObjectAnnotation(I,'rectangle',bboxes,scores,labels);
    imshow(I)
end

disp('Test Done')

%% ------------------------TEST MODEL WITH TEST SET--------------------

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


%% 3. Color Filtering + Localisation

% After ML detector is run:
% 1) Labels of which Shape that was detected
% 2) Approx Bounding box/centroid of each shape
% 3) Number of blocks detected?

close all
customerImage = imread('.\YOLO_TEST\test6.jpg');
num_blocks = 2; 

% Create ROI
%[a,b] = imcrop(customerImage);
rectROI = [542.51,303.51,515.98,444.98];
ROI_image = imcrop(customerImage,rectROI);

% Initial image processing (to remove noise + errant grid lines)


% ---------------------Detect Colors----------------------
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
       min_block_size = 130;
            
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
 
 % debug show the results of shape detection
 cv_block_struct

% 3. Error checking;
yolo_block_check = sprintf('Number of ML blocks detected: %f', num_blocks);
disp(yolo_block_check)
block_check = sprintf('Number of blocks detected: %f', block_counter);
disp(block_check)

if (block_counter ~= num_blocks)
    disp('Error: Incorrect Number of Blocks!!')
end

%% ---------------------Match Shape with Color----------------------

% Check which detected shape is in which color (compare centroids)
% block_locations = x,y of bb position and shape detected. bboxes
% cv_block_struct = x,y of centroid and which hsv filter it had used.



%% ---------------------Orientation of Blocks----------------------

% Image processing to determine orientation of blocks
% SURF?



%% ---------------------Place Coordinates----------------------

load('calibrationSession.mat', 'calibrationSession');

% From Extrinsic function output
translationVector = [21.6771020996404,-377.712398323210,859.963449998696];

rotationMatrix = [-0.000532049447634045,0.999998919650671,-0.00137026306816968;...
                0.999994863561974,0.000527715353405883,-0.00316138674858810;...
                -0.00316066022432677,-0.00137193804397179,-0.999994063988857];

% Convert image points to world coordinates (from struct)
% cv_block_struct (rows 1 and 2)

placeCount = 1;
% Finalized array of data (might actually make struct later)
cv_block_struct 

%[x,y] = pointsToWorld(calibrationSession.CameraParameters,rotationMatrix,translationVector,([x][y]);

%% Send commands to Robot Arm (through Ethernet)
% For each Block:
% 1) [X,Y] PLACE COORDINATES (WORLD frame)
% 2) Orientation

%% 4. Detect on Conveyor Belt (Object Detection)

% Change to conveyor camera
%MTRN4230_Image_Capture([],[]) %for conveyor camera
% Load camera calibration

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

