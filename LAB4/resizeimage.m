buildingDir = fullfile('F:\lab4\cali');
buildingScene = imageDatastore(buildingDir);
numImages = numel(buildingScene.Files);
for i =1:numImages
    I = readimage(buildingScene, i);
    imwrite(imresize(I,0.1),buildingScene.Files{i});
end