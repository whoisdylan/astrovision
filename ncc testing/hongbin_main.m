%given im1, im2, and feature points (sx1,sy1) in im1 compute ncc across a window in im2 to find
%correspondence points in im2

numImages = 40;
numPoints = 300;

%import images from directory direc
direc = 'C:\Users\render\Desktop\dylan\helicopter images 2\left*.tiff';
D = dir(direc);
%sizes for helicopter images 1
% imHeight = 1936;
% imWidth = 1456;
%sizes for helicopter images 2
imHeight = 1827;
imWidth = 1306;
leftImages = uint8(zeros(imHeight,imWidth,numImages));
for i=1:numImages;
    leftImages(:,:,i) = imread(['C:\Users\render\Desktop\dylan\helicopter images 2\' D(i).name]);
end
display('finished loading images');
%%

% im1rows = zeros(numPoints,1,numImages);
% im1cols = im1rows;
% im2rows = im1rows;
% im2cols = im1rows;
numImages=8;
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
    [~,~,correspondences(:,4,i),correspondences(:,3,i)] = ncc_match(currIm1,currIm2,correspondences(:,2,i),correspondences(:,1,i));
end
display('finished processing images');
%%
%save dual image figs to show correspondences

%%

%save figs to make video
% D = dir('C:/Users/render/Desktop/dylan/helicopter images/left*.png');
fig = figure;
for i=1:numImages
    set(fig,'PaperPositionMode','auto');
    imshow(lefts(:,:,i));
    hold on;
%     plot(points(:,1,i),points(:,2,i),'r.'); %plot all points
    %plot correspondences
    plot(correspondences(:,1,i),correspondences(:,2,i),'r.');
    for j=1:numPoints
        if (~isnan(correspondences(j,3,i)))
            plot([correspondences(j,1,i) correspondences(j,3,i)],[correspondences(j,2,i) correspondences(j,4,i)]);
        end
    end
    hold off;
%     saveas(fig,['C:/Users/render/Desktop/dylan/ncc testing/results 6-5-13/' int2str(i)],'png');
    print(fig,'-dpng', '-r0', ['C:/Users/render/Desktop/dylan/ncc testing/hongbin/LR' int2str(i) '.png']);
end
display('finished saving figures');
%%
makeVideo('C:/Users/render/Desktop/dylan/ncc testing/results',numImages);
display('finished making video');