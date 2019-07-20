% MTRN4230 Group Project

% ----------------ChangeLog---------------
% v1. 6/7/19 Initial creation with comments and brief summary. Image capture
% v2. 15/7/19 Improved image capture at robot cell
% v3. 18/7/19 Added in structure for YOLOv2 network
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
customerImage = imread('sample1.png');
imshow(customerImage)

%(LATER ON USE ROBOT CELL CAMERA)
%MTRN4230_Image_Capture([]) %for robot cell

%% 2. Quirkle Blocks from Sample Image (Classification + Localisation)
% YOLOv2 Detection Network for Quirkle Blocks

doTraining = true;

% ------------------------SET UP TRAINING and TEST SETS----------------

% Load Training Data (using ImageLabeller - ground truth)
%data = load('quirkleGroundTruth.mat');
%QuirkleDataset = data.quirkleDataset;

% Show Sample Training Data Image

% Set random seed to ensure example training reproducibility.
rng(0);

% Randomly split data into a training and test set.
shuffledIndices = randperm(height(QuirkleDataset));
idx = floor(0.6 * length(shuffledIndices) );
trainingData = QuirkleDataset(shuffledIndices(1:idx),:); %60
testData = QuirkleDataset(shuffledIndices(idx+1:end),:); %40

% ------------------------CREATE DETECTION NETWORK-------------------

% YOLOv2 = feature network (pre-trained CNN) & detection network (smaller
% CNN specific to YOLO)

% The image input size should be at least as big as the images in the training image set.
% Define the image input size.
imageSize = [80 80 3]; %might need to resize but are RBG

% Define the number of object classes to detect. (6 shapes == 6 classes)
numClasses = width(QuirkleDataset)-1; %5

% Create rest of YOLOv2 Detection Network....

% ------------------------TEST MODEL ON TEST IMAGES--------------------

% Read a test image.
I = imread(testData.imageFilename{end});

% Run the detector.
[bboxes,scores] = detect(detector,I);

% Annotate detections in the image.
I = insertObjectAnnotation(I,'rectangle',bboxes,scores);
imshow(I)

% Test Detector with test set

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

% Testing with customer's sample image: (sample2.jpg)
customerImage = imread('sample1.png');
figure
imshow(customerImage)

% %[a,b] = imcrop(customerImage);
% rectROI = [311.51,173.51,621.98,462.98]; 
% ROI_image = imcrop(customerImage,rectROI);
% figure
% imshow(ROI_image)

% i. ---------------Find RED Object-------------

%HSV
hsv_path = rgb2hsv(customerImage);
hsv_path = imgaussfilt(hsv_path,0.5);

% Threshold for HSV - Green Square (END)
low_red = [0.825,0.475,0.2];
hi_red = [1.00,1.00,1.00];

% Create mask to find pixels with desired HSV ranges (binary mask)
mask_red = (hsv_path(:,:,1) >= low_red(1)) & (hsv_path(:,:,1) <= hi_red(1)) & ...
    (hsv_path(:,:,2) >= low_red(2) ) & (hsv_path(:,:,2) <= hi_red(2)) & ...
    (hsv_path(:,:,3) >= low_red(3) ) & (hsv_path(:,:,3) <= hi_red(3));

stats = regionprops(mask_red,'basic');
centroids = cat(1,stats.Centroid);
areas = cat(1,stats.Area);
[m_red,i_red] = max(areas(:,1)); %largest contour
start_circle_x = centroids(i_red,1);
start_circle_y = centroids(i_red,2);
hold on
plot(start_circle_x,start_circle_y,'bo','LineWidth',2)
hold off

%% 4. Orientation of Blocks

% Image processing to determine orientation of blocks

%% 5. Detect on Conveyor Belt (Object Detection)

% Change to conveyor camera
%MTRN4230_Image_Capture([],[]) %for conveyor camera

%% 6. Send commands to Robot Arm (through Ethernet)

% For each Block
% [X,Y]
% Angle

%% FUNCTIONS

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

