%given im1, im2, and feature points (sx1,sy1) in im1 compute ncc across a window in im2 to find
%correspondence points in im2

numImages = 100;
numPoints = 300;

%import images from directory direc
direc = 'C:\Users\render\Desktop\dylan\parking_lot_images\';
DL = dir([direc 'left*']);
DR = dir([direc 'right*']);
%sizes for helicopter images 1
% imHeight = 1936;
% imWidth = 1456;
%sizes for helicopter images 2
imHeight = 1936;
imWidth = 1456;
leftImages = uint8(zeros(imHeight,imWidth,numImages));
rightImages = uint8(zeros(imHeight,imWidth,numImages));
for i=1:numImages;
    leftImages(:,:,i) = imread(['C:\Users\render\Desktop\dylan\parking_lot_images\' DL(i).name]);
    rightImages(:,:,i) = imread(['C:\Users\render\Desktop\dylan\parking_lot_images\' DR(i).name]);
end
display('finished loading images');
%%

% im1rows = zeros(numPoints,1,numImages);
% im1cols = im1rows;
% im2rows = im1rows;
% im2cols = im1rows;

%contains all points for all images, including newly acquired ones if
%points went out of frame
%in the form: [imNcols,imNrows]
% points = zeros(numPoints,2,numImages);
%contains only points with correspondences (ie no newly acquired points)
%in the form: [imNcols,imNrows,imN+1cols,imN+1rows]
correspondences = zeros(numPoints,4,numImages);

display('beginning image processing');
for i=1:(numImages-1)
    display(['processing images ' num2str(i) ' and ' num2str(i+1)]);
    currIm1 = leftImages(:,:,i);
    currIm2 = rightImages(:,:,i);
    [x1, y1, v1] = harris(currIm1);
    [correspondences(:,1,1), correspondences(:,2,1), ~] = suppress(x1,y1,v1);
%     correspondences(:,2,i) = points(:,2,i);
%     correspondences(:,1,i) = points(:,1,i);
    [~,~,correspondences(:,4,i),correspondences(:,3,i)] = ncc_pyramid_match(currIm1,currIm2,correspondences(:,2,i),correspondences(:,1,i));
end
display('finished processing images');
%%
%save dual-image figs to show correspondences
fig = figure;
for i=1:numImages-1
    set(fig,'PaperPositionMode','auto');
    catImage = [leftImages(:,:,i) leftImages(:,:,i+1)];
    imshow(catImage)
    hold on;
    plot(correspondences(:,1,i),correspondences(:,2,i),'r.');
    plot(correspondences(:,3,i)+imWidth,correspondences(:,4,i),'r.');
    hold off;
    print(fig,'-dpng','-r0', ['C:/Users/render/Desktop/dylan/ncc testing/results_dual/' int2str(i) '.png']);
end
display('finished saving dual figures');
%%

%save single-image figs to make video
% D = dir('C:/Users/render/Desktop/dylan/helicopter images/left*.png');
fig = figure;
for i=1:numImages
    set(fig,'PaperPositionMode','auto');
    imshow(leftImages(:,:,i));
    hold on;
%     plot(points(:,1,i),points(:,2,i),'r.'); %plot all points
    %plot correspondences
    if (i==1)
        plot(correspondences(:,1,i),correspondences(:,2,i),'r.');
    else
        plot(correspondences(:,3,i-1),correspondences(:,4,i-1),'r.');
    end
    hold off;
%     saveas(fig,['C:/Users/render/Desktop/dylan/ncc testing/results 6-5-13/' int2str(i)],'png');
    print(fig,'-dpng', '-r0', ['C:/Users/render/Desktop/dylan/ncc testing/results/' int2str(i) '.png']);
end
display('finished saving figures');
%%
%make single-image video
makeVideo('C:/Users/render/Desktop/dylan/ncc testing/results',numImages);
display('finished making video');
%%
%make dual-image video
makeVideo('C:/Users/render/Desktop/dylan/ncc testing/results_dual',numImages-1);
display('finished making video');