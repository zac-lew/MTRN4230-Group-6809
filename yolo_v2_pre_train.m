% YOLOv2 Example

% using pre-trained detector (for testing functionality)
doTraining = false;
if ~doTraining && ~exist('yolov2ResNet50VehicleExample.mat','file')
    % Download pretrained detector.
    disp('Downloading pretrained detector (98 MB)...');
    pretrainedURL = 'https://www.mathworks.com/supportfiles/vision/data/yolov2ResNet50VehicleExample.mat';
    websave('yolov2ResNet50VehicleExample.mat',pretrainedURL);
end

% Unzip vehicle dataset images.
unzip vehicleDatasetImages.zip

% Load vehicle dataset ground truth.
data = load('vehicleDatasetGroundTruth.mat');
vehicleDataset = data.vehicleDataset;

% Display first few rows of the data set.
vehicleDataset(1:4,:)

% Add the fullpath to the local vehicle data folder.
vehicleDataset.imageFilename = fullfile(pwd,vehicleDataset.imageFilename);

% Read one of the images.
I = imread(vehicleDataset.imageFilename{10});

% Insert the ROI labels.
I = insertShape(I,'Rectangle',vehicleDataset.vehicle{10});

% Resize and display image.
I = imresize(I,3);
imshow(I)

% Create YOLOv2 Detection Network

% Create test and training data sets

% Set random seed to ensure example training reproducibility.
rng(0);

% Randomly split data into a training and test set.
shuffledIndices = randperm(height(vehicleDataset));
idx = floor(0.6 * length(shuffledIndices) );
trainingData = vehicleDataset(shuffledIndices(1:idx),:);
testData = vehicleDataset(shuffledIndices(idx+1:end),:);

% YOLOv2 = feature network (pre-trained CNN) & detection network (smaller
% CNN specific to YOLO)

% The image input size should be at least as big as the images in the training image set.
% Define the image input size. 224 x 224 pixels RGB images
imageSize = [224 224 3];

% Define the number of object classes to detect.
numClasses = width(vehicleDataset)-1;

% Anchor boxe sizes
anchorBoxes = [
    43 59
    18 22
    23 29
    84 109
];

% Load a pretrained ResNet-50.
baseNetwork = resnet50;

% Specify the feature extraction layer.
featureLayer = 'activation_40_relu';

% Create the YOLO v2 object detection network. 
lgraph = yolov2Layers(imageSize,numClasses,anchorBoxes,baseNetwork,featureLayer);

% Example training parameters
if doTraining
    
    % Configure the training options. 
    %  * Lower the learning rate to 1e-3 to stabilize training. 
    %  * Set CheckpointPath to save detector checkpoints to a temporary
    %    location. If training is interrupted due to a system failure or
    %    power outage, you can resume training from the saved checkpoint.
    options = trainingOptions('sgdm', ...
        'MiniBatchSize', 16, ....
        'InitialLearnRate',1e-3, ...
        'MaxEpochs',10,...
        'CheckpointPath', tempdir, ...
        'Shuffle','every-epoch');    
    
    % Train YOLO v2 detector.
    [detector,info] = trainYOLOv2ObjectDetector(vehicleDataset,lgraph,options);
else
    % Load pretrained detector for the example.
    pretrained = load('yolov2ResNet50VehicleExample.mat');
    detector = pretrained.detector;
end

% Read a test image.
I = imread(testData.imageFilename{end});

% Run the detector.
[bboxes,scores] = detect(detector,I);

% Annotate detections in the image.
I = insertObjectAnnotation(I,'rectangle',bboxes,scores);
imshow(I)
