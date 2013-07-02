%given im1, im2, and feature points (sx1,sy1) in im1 compute ncc across a window in im2 to find
%correspondence points in im2

numImages = 5;
numPoints = 300;

%import images from directory direc
direc = 'C:\Users\render\Downloads\Wean_6_11_10-52-25\Wean_6_11_10-52-25\';
DL = dir([direc 'L*']);
DR = dir([direc 'R*']);
%sizes for helicopter images 1
% imHeight = 1936;
% imWidth = 1456;
%sizes for helicopter images 2
% imHeight = 1827;
% imWidth = 1306;
% lefts = uint8(zeros(imHeight,imWidth,numImages));
for i=1:numel(DL);
    lefts(:,:,i) = rgb2gray(imread([direc DL(i).name]));
    rights(:,:,i) = rgb2gray(imread([direc DR(i).name]));
end
imWidth = size(lefts(:,:,1),2);
display('finished loading images');
%%

% im1rows = zeros(numPoints,1,numImages);
% im1cols = im1rows;
% im2rows = im1rows;
% im2cols = im1rows;
numImages=5;
numPoints=300;
%contains all points for all images, including newly acquired ones if
%points went out of frame
%in the form: [imNcols,imNrows]
% points = zeros(numPoints,2,numImages);
%contains only points with correspondences (ie no newly acquired points)
%in the form: [imNcols,imNrows,imN+1cols,imN+1rows]
correspondences = zeros(numPoints,4,numImages);

display('beginning image processing');
for i=1:(numImages)
    display(['processing image pair ' num2str(i)]);
    currIm1 = lefts(:,:,i);
    currIm2 = rights(:,:,i);
    [x1,y1,v1] = harris(currIm1);
    [sx1,sy1,~] = suppress(x1,y1,v1);
    correspondences(:,1,i) = sx1;
    correspondences(:,2,i) = sy1;
    [~,~,correspondences(:,4,i),correspondences(:,3,i)] = ncc_pyramid_match(currIm1,currIm2,correspondences(:,2,i),correspondences(:,1,i));
end
display('finished processing images');
%%

%save dual-image figs to make video
% D = dir('C:/Users/render/Desktop/dylan/helicopter images/left*.png');
fig = figure;
for i=1:numImages-1
    set(fig,'PaperPositionMode','auto');
    catImage = [lefts(:,:,i) rights(:,:,i+1)];
    imshow(catImage)
    hold on;
    plot(correspondences(:,1,i),correspondences(:,2,i),'r.');
    plot(correspondences(:,3,i)+imWidth,correspondences(:,4,i),'r.');
    hold off;
    print(fig,'-dpng','-r0', ['C:/Users/render/Desktop/dylan/ncc testing/data/LR' int2str(i) '.png']);
end
display('finished saving figures');
%%
makeVideo('C:/Users/render/Desktop/dylan/ncc testing/results',numImages);
display('finished making video');