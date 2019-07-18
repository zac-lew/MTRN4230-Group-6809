% Getting familiar with how to do Deep Learning on MATLAB
% R-CNN Stop Sign Detector

% Load training data
load('rcnnStopSigns.mat', 'stopSigns', 'layers')

% Load data into path
imDir = fullfile(matlabroot, 'toolbox', 'vision', 'visiondata',...
  'stopSignImages');
addpath(imDir);

% Batch size of 32 to reduce computation size
options = trainingOptions('sgdm', ...
  'MiniBatchSize', 32, ...
  'InitialLearnRate', 1e-6, ...
  'MaxEpochs', 10);

%train
rcnn = trainRCNNObjectDetector(stopSigns, layers, options, 'NegativeOverlapRange', [0 0.3]);

img = imread('stop_test.jpg');

% bbox = bounding boxes, score = percentage of certainty re stop sign,
% label = classification
[bbox, score, label] = detect(rcnn, img, 'MiniBatchSize', 32);

%Display strongest detection result.

[score, idx] = max(score);

bbox = bbox(idx, :);
annotation = sprintf('%s: (Confidence = %f)', label(idx), score);

detectedImg = insertObjectAnnotation(img, 'rectangle', bbox, annotation);

figure
imshow(detectedImg)

