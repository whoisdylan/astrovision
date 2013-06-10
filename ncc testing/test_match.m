function [correspondenceRows,correspondenceCols] = test_match(im1,im2,sx1,sy1)
descHalfSize = 16;
windowHalfSize = 32;
halfSize = descHalfSize + windowHalfSize;

imHeight = size(im1,1);
imWidth = size(im1,2);
numPoints = size(sy1,1);

im2rows = zeros(size(sy1,1),1);
im2cols = zeros(size(sx1,1),1);
correspondenceRows = zeros(size(sy1,1),1);
correspondenceCols = zeros(size(sx1,1),1);

%number of invalid points (ie outside of frame after offset)
invalidCount = 0;

maxRowOffset = 0;
maxColOffset = 0;

for i=1:numPoints
    
    %create descriptor from im1 and search window from im2
    currRow = sy1(i);
    currCol = sx1(i);
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
    
    if (rowOffset > maxRowOffset)
        maxRowOffset = rowOffset;
    end
    if (colOffset > maxColOffset)
        maxColOffset = colOffset;
    end
    
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
end