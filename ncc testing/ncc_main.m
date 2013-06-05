%given im1, im2, and feature points (sx1,sy1) in im1 compute ncc across a window in im2 to find
%correspondence points in im2

numImages = 40;
numPoints = 500;

% im1rows = zeros(numPoints,1,numImages);
% im1cols = im1rows;
% im2rows = im1rows;
% im2cols = im1rows;

points = zeros(numPoints,2,numImages);

% leftImages = zeros(size(left_0000,1),size(left_0000,2),numImages);
% leftImages(:,:,1) = rgb2gray(left_0000);
% leftImages(:,:,2) = rgb2gray(left_0001);
% leftImages(:,:,3) = rgb2gray(left_0002);
% leftImages(:,:,4) = rgb2gray(left_0003);
% leftImages(:,:,5) = rgb2gray(left_0004);
% leftImages(:,:,6) = rgb2gray(left_0005);
% leftImages(:,:,7) = rgb2gray(left_0006);
% leftImages(:,:,8) = rgb2gray(left_0007);
% leftImages(:,:,9) = rgb2gray(left_0008);
% leftImages(:,:,10) = rgb2gray(left_0009);
% leftImages(:,:,1) = rgb2gray(left_0010);
% leftImages(:,:,1) = rgb2gray(left_0020);
% leftImages(:,:,1) = rgb2gray(left_0030);
% leftImages(:,:,1) = rgb2gray(left_0040);
% leftImages(:,:,1) = rgb2gray(left_0050);
% leftImages(:,:,1) = rgb2gray(left_0060);
% leftImages(:,:,1) = rgb2gray(left_0070);
% leftImages(:,:,1) = rgb2gray(left_0080);
% leftImages(:,:,1) = rgb2gray(left_0090);

%unroll first image
display('setting up first image');
currIm2 = leftImages(:,:,1);
[x1, y1, v1] = harris(currIm2);
[points(:,1,1), points(:,2,1), ~] = suppress(x1,y1,v1);
%instantiate im2pts
display('beginning image processing');
for i=1:(numImages-1)
    display(['processing images ' num2str(i) ' and ' num2str(i+1)]);
    currIm1 = currIm2;
    currIm2 = leftImages(:,:,i+1);
    [points(:,2,i+1),points(:,1,i+1)] = ncc_match(currIm1,currIm2,points(:,2,i),points(:,1,i));
end
%%

%save figs to make video\
D = dir('C:/Users/render/Desktop/dylan/helicopter images');
for i=1:numImages
    fig = imshow(leftImages(:,:,i));
    hold on;
    plot(points(:,1,i),points(:,2,i),'r.');
    hold off;
    saveas(fig,['C:/Users/render/Desktop/dylan/ncc testing/results 6-5-13/' D(i).name]);
end
makeVideo('C:/Users/render/Desktop/dylan/ncc testing/results 6-5-13',40);
