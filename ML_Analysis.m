% Load training data
%load('trainingData.mat');

% Set random seed to ensure example training reproducibility.
% rng(12);
% 
% % Randomly split data into a training and test set.
% shuffledIndices = randperm(height(YtrainingData));
% idx = floor(0.80 * length(shuffledIndices) );
% trainingData2 = YtrainingData(shuffledIndices(1:idx),:);
% testData = YtrainingData(shuffledIndices(idx+1:end),:);

%disp('YTraining Data loaded!');
%disp('~Training and Test Datasets #2 Created~');

    % Change filepath for imageFilenames (for stats on TestData)
    for Tdata_counter = 1 : height (testData)
        [filepath,name,ext] = fileparts(testData.imageFilename{Tdata_counter});
        new_path = fullfile('C:\Users\Jonathan\Documents\UNSW Engineering\2019\S2\MTRN4230\Group Project\YOLO_TRAIN\Train_800_600\',[name,'.jpg']);
        testData.imageFilename{Tdata_counter} = new_path;
    end
    
    disp("TestData Filepaths Corrected");

%% ------------------------TEST MODEL ON A TEST IMAGE--------------------
%----------------------------FOR EVALUATION----------------------------

% NOTE: The minimum input image size must be equal to or greater
% than the input size in image input layer of the network.

% Read a test image.
I = imread(testData.imageFilename{2});

% Run the detector. (detector with 2 rounds of training)
[bboxes,scores,labels] = detect(updated_detector,I,'Threshold',0.01);

% Annotate detections in the image.
if ~isempty(bboxes)
    I = insertShape(I,'Rectangle',bboxes);
    imshow(I)
end

disp('Test Done')

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
        [bboxes, scores, labels] = detect(updated_detector, I);
        
        % Collect the results.
        % Collect the results.
        results.Boxes{i} = bboxes;
        results.Scores{i} = scores;
        results.Labels{i} = labels;
    end

% Extract expected bounding box locations from test data.
expectedResults = testData(:, 2:end);

% Evaluate the object detector using Average Precision metric.
[ap, recall, precision] = evaluateDetectionPrecision(results, expectedResults);

% Plot precision/recall curve
figure
plot(recall,precision)
xlabel('Recall')
ylabel('Precision')
grid on
title(sprintf('Average Precision = %.2f', ap))

%% Training loss
% Plot training accuracy / interation
figure
plot(info(1).TrainingLoss)
hold on
plot(info(4).TrainingLoss)
grid on
legend('(1): Region Proposal Network','(2): Re-training Fast R-CNN using updated RPN')
xlabel('Number of Iterations')
ylabel('Training Loss for Each Iteration')
