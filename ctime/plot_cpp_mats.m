numImages = 10;
numPoints = 200;
%import images from directory direc
% direc = 'C:\Users\dylan.koenig\Dropbox\Hongbin\roof_images\rectified\';
direc = 'C:\Users\dylan.koenig\Dropbox\datasets\zipline\';
direcC = 'C:\Users\dylan.koenig\Dropbox\astromats_three\';
% DL = dir([direc 'L_*.png']);
% DR = dir([direc 'R_*.png']);
DL = dir([direc 'left_rect_*.png']);
DR = dir([direc 'right_rect_*.png']);
%sizes for helicopter images 1
% imHeight = 1936;
% imWidth = 1456;
%sizes for helicopter images 2
imHeight = 1936;
imWidth = 1456;
leftImages = uint8(zeros(imHeight,imWidth,numImages));
rightImages = uint8(zeros(imHeight,imWidth,numImages));
for i=1:numImages;
%     leftImages(:,:,i) = rgb2gray(imread([direc DL(i).name]));
%     rightImages(:,:,i) = rgb2gray(imread([direc DR(i).name]));
    leftImages(:,:,i) = imread([direc DL(i).name]);
    rightImages(:,:,i) = imread([direc DR(i).name]);
end
display('finished loading images');
%%
%load correspondences
DCP = dir([direcC 'corrPrev*.txt']);
DCR = dir([direcC 'corrRight*.txt']);
DCN = dir([direcC 'corrNext*.txt']);
corrPrev = zeros(numPoints,2,numImages-1);
corrNext = zeros(numPoints,2,numImages-1);
corrRight = zeros(numPoints,2,numImages-1);
for i=1:numImages-1;
    corrPrev(:,:,i) = load([direcC DCP(i).name]);
    corrNext(:,:,i) = load([direcC DCN(i).name]);
    corrRight(:,:,i) = load([direcC DCR(i).name]);
end
display('finished loading correspondences');
%%
%save tri-image figs to show correspondences
fig = figure;
for i=1:numImages-1
    set(fig,'PaperPositionMode','auto');
    catImage = [leftImages(:,:,i+1) zeros(imHeight,imWidth); leftImages(:,:,i) rightImages(:,:,i)];
    imshow(catImage)
    hold on;
    %points for left frame at time n
    plot(corrNext(:,1,i),corrNext(:,2,i)+imHeight,'b.');
    %points for right frame at time n
    plot(corrRight(:,1,i)+imWidth,corrRight(:,2,i)+imHeight,'g.');
    %points for left frame at time n+1
    plot(corrPrev(:,1,i),corrPrev(:,2,i),'r.');
    hold off;
    print(fig,'-dpng','-r0', ['C:/Users/dylan.koenig/Documents/vision/gits/astrovision/ctime/results_16/' int2str(i) '.png']);
end
display('finished saving tri figures');
%%
%make tri-image video
makeVideo('C:/Users/dylan.koenig/Documents/vision/ncc testing/results_dual',numImages-1);
display('finished making video');