% Testing out Simpler Deep Learning Network
% 4/8/19

% Load training set
digitDatasetPath = ('C:\Users\Jonathan\Documents\UNSW Engineering\2019\S2\MTRN4230\Group Project\CNN_TRAIN_50');
imds = imageDatastore(digitDatasetPath, ...
    'IncludeSubfolders',true,'LabelSource','foldernames');

% Show sample of training set
% figure;
% perm = randperm(500,20);
% for i = 1:20
%     subplot(4,5,i);
%     imshow(imds.Files{perm(i)});
% end

% Number of images per class
labelCount = countEachLabel(imds);

% Size of image
%img = readimage(imds,1);
%size(img)

% Split dataset into Training and Validation sets
numTrainFiles = 108;
[imdsTrain,imdsValidation] = splitEachLabel(imds,numTrainFiles,'randomize');

% Creating network
layers = [
    imageInputLayer([50 50 3])
    
    convolution2dLayer(3,8,'Padding','same')
    batchNormalizationLayer
    reluLayer
    
    maxPooling2dLayer(2,'Stride',2)
    
    convolution2dLayer(3,16,'Padding','same')
    batchNormalizationLayer
    reluLayer
    
    maxPooling2dLayer(2,'Stride',2)
    
    convolution2dLayer(3,32,'Padding','same')
    batchNormalizationLayer
    reluLayer
    
    fullyConnectedLayer(6)
    softmaxLayer
    classificationLayer
    ];

% Training options
options = trainingOptions('sgdm', ...
    'InitialLearnRate',0.001, ...
    'MaxEpochs',10, ...
    'Shuffle','every-epoch', ...
    'ValidationData',imdsValidation, ...
    'ValidationFrequency',30, ...
    'Verbose',false, ...
    'Plots','training-progress');

% Execute training of CNN
net = trainNetwork(imdsTrain,layers,options);

% Accuracy
% YPred = classify(net,imdsValidation);
% YValidation = imdsValidation.Labels;
% accuracy = sum(YPred == YValidation)/numel(YValidation)
