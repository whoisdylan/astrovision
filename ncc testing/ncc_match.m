function [im2rows,im2cols,correspondenceRows,correspondenceCols] = ncc_match(im1,im2,im1rows,im1cols)
%takes in im1 im2 and im1 feature points, creates descriptors of im1pts and
%searches windows within im2 to find and return im2pts

descHalfSize = 16;
windowHalfSize = 32;
halfSize = descHalfSize + windowHalfSize;

imHeight = size(im1,1);
imWidth = size(im1,2);
numPts = size(im1rows,1);

im2rows = zeros(size(im1rows,1),1);
im2cols = zeros(size(im1cols,1),1);
correspondenceRows = zeros(size(im1rows,1),1);
correspondenceCols = zeros(size(im1cols,1),1);

%number of invalid points (ie outside of frame after offset)
invalidCount = 0;

for i=1:numPts
    
    %create descriptor from im1 and search window from im2
    currRow = im1rows(i);
    currCol = im1cols(i);
    currDesc = im1((currRow-descHalfSize):(currRow+descHalfSize-1),(currCol-descHalfSize):(currCol+descHalfSize-1));
    currWindow = im2((currRow-windowHalfSize):(currRow+windowHalfSize-1),(currCol-windowHalfSize):(currCol+windowHalfSize-1));
    
    %compute NCC
    xcc = normxcorr2(currDesc,currWindow);
%     [max_xcc, imax] = max(abs(xcc(:)));

    %calculate row (y) and col (x) offset relative to im1pt for feature point in im2
    [~, imax] = max(abs(xcc(:)));
    [ypeak, xpeak] = ind2sub(size(xcc),imax);
    rowOffset = ypeak-descHalfSize*3;
    colOffset = xpeak-descHalfSize*3;
    currIm2row = currRow + rowOffset;
    currIm2col = currCol + colOffset;
    
    correspondenceRows(i) = currIm2row;
    correspondenceCols(i) = currIm2col;
    %check if new point is outside of the tolerance frame
    if ((currIm2row < halfSize) || (currIm2col < halfSize) || (currIm2row > (imHeight - halfSize + 1)) || (currIm2col > (imWidth - halfSize + 1)))
        invalidCount = invalidCount + 1;
        
    else
        im2rows(i-invalidCount) = currIm2row;
        im2cols(i-invalidCount) = currIm2col;
    end
    %consider adding threshold so if no match exists, im2pt is set to null
end

%detect new features if necessary, two options...
%1: try to fill in empty spots with new features
%2: replace all features with 500 new, valid features

%option 2:
if (invalidCount ~= 0)
    display('acquiring new points');
    [x2, y2, v2] = harris(im2);
    [im2cols, im2rows, ~] = suppress(x2, y2, v2);
end 

end

