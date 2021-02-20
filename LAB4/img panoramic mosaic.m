clear all
load calibration
buildingDir = fullfile('D:\Thesis\EECE5554\eece5554_roboticssensing\LAB4\building');
buildingScene = imageDatastore(buildingDir);
I = readimage(buildingScene, 1);
I=undistortImage(I, cameraParams);
grayImage = rgb2gray(I);
% grayImage = im2bw(I);
grayImage=improcess(grayImage);
% points = detectSURFFeatures(grayImage);
montage(buildingScene)
[y,x,m] = harris(grayImage,3000,'thresh',0.1,'tile',[1 1],'disp','sigma',5,'hsize',45,'eig','fft');

points=[x,y];
[features, points] = extractFeatures(grayImage, points);
numImages = numel(buildingScene.Files);
tforms(numImages) = projective2d(eye(3));
imageSize = zeros(numImages,2);
% Iterate over remaining image pairs

for n = 2:numImages
    pointsPrevious = points;
    featuresPrevious = features;
    I = readimage(buildingScene, n);
%      I=undistortImage(I, cameraParams);
    grayImage = rgb2gray(I);    
%     grayImage = im2bw(I);
    grayImage=improcess(grayImage);
    imageSize(n,:) = size(grayImage);
%     points = detectSURFFeatures(grayImage);  
[y,x,m] = harris(grayImage,3000,'thresh',0.1,'tile',[1 1],'disp','sigma',5,'hsize',45,'eig','fft');
    points=[x,y];
    [features, points] = extractFeatures(grayImage, points);
    indexPairs = matchFeatures(features, featuresPrevious, 'Unique', true);
    matchedPoints = points(indexPairs(:,1), :);
    matchedPointsPrev = pointsPrevious(indexPairs(:,2), :);        
    tforms(n) = estimateGeometricTransform(matchedPoints, matchedPointsPrev,...
        'projective', 'Confidence', 99.9, 'MaxNumTrials', 2000);
    tforms(n).T = tforms(n).T * tforms(n-1).T;
    
end
    

for i = 1:numel(tforms)           
    [xlim(i,:), ylim(i,:)] = outputLimits(tforms(i), [1 imageSize(i,2)], [1 imageSize(i,1)]);    
end
avgXLim = mean(xlim, 2);
[~, idx] = sort(avgXLim);
centerIdx = floor((numel(tforms)+1)/2);
centerImageIdx = idx(centerIdx);
Tinv = invert(tforms(centerImageIdx));

for i = 1:numel(tforms)    
    tforms(i).T = tforms(i).T * Tinv.T;
end

for i = 1:numel(tforms)           
    [xlim(i,:), ylim(i,:)] = outputLimits(tforms(i), [1 imageSize(i,2)], [1 imageSize(i,1)]);
end

maxImageSize = max(imageSize);
xMin = min([1; xlim(:)]);
xMax = max([maxImageSize(2); xlim(:)]);
yMin = min([1; ylim(:)]);
yMax = max([maxImageSize(1); ylim(:)]);
width  = round(xMax - xMin);
height = round(yMax - yMin);
panorama = zeros([height width 3], 'like', I);
blender = vision.AlphaBlender('Operation', 'Binary mask', ...
    'MaskSource', 'Input port');  
xLimits = [xMin xMax];
yLimits = [yMin yMax];
panoramaView = imref2d([height width], xLimits, yLimits);
for i = 1:numImages
    
    I = readimage(buildingScene, i);   
    I=undistortImage(I, cameraParams);
    warpedImage = imwarp(I, tforms(i), 'OutputView', panoramaView);
    mask = imwarp(true(size(I,1),size(I,2)), tforms(i), 'OutputView', panoramaView);
    panorama = step(blender, panorama, warpedImage, mask);
end

figure
imshow(panorama)

function nm=rot90(m)
    nm=imrotate(m,90);
end

function J=improcess(I)
    grayImage=I;
    J=medfilt2(grayImage,[25,25]);
end