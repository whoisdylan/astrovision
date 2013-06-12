function [im2rows,im2cols,correspondenceRows,correspondenceCols] = ncc_match(im1,im2,resizedIm1,resizedIm2,im1rows,im1cols)
%takes in im1 im2 and im1 feature points, creates descriptors of im1pts and
%searches windows within im2 to find and return im2pts

imScale = 3;
descHalfSize = 16*imScale;
windowHalfSize = 64*imScale;
halfSize = (descHalfSize + windowHalfSize)/imScale;
resizedHalfSize = descHalfSize + windowHalfSize;

imHeight = size(im1,1);
imWidth = size(im1,2);
numPoints = size(im1rows,1);

im2rows = zeros(size(im1rows,1),1);
im2cols = zeros(size(im1cols,1),1);
correspondenceRows = zeros(size(im1rows,1),1);
correspondenceCols = zeros(size(im1cols,1),1);

%number of invalid points (ie outside of frame after offset)
invalidCount = 0;

maxRowOffset = 0;
maxColOffset = 0;

for i=1:numPoints
    
    %create descriptor from im1 and search window from im2
    currRow = im1rows(i);
    currCol = im1cols(i);
%     currDesc = im1((currRow-descHalfSize):(currRow+descHalfSize-1),(currCol-descHalfSize):(currCol+descHalfSize-1));
%     currWindow = im2((currRow-windowHalfSize):(currRow+windowHalfSize-1),(currCol-windowHalfSize):(currCol+windowHalfSize-1));
    currDesc = resizedIm1((currRow*imScale-descHalfSize):(currRow*imScale+descHalfSize-1),(currCol*imScale-descHalfSize):(currCol*imScale+descHalfSize-1));
    currWindow = resizedIm2((currRow*imScale-windowHalfSize):(currRow*imScale+windowHalfSize-1),(currCol*imScale-windowHalfSize):(currCol*imScale+windowHalfSize-1));    

    %compute NCC
    xcc = normxcorr2(currDesc,currWindow);
%     [max_xcc, imax] = max(abs(xcc(:)));

    %calculate row (y) and col (x) offset relative to im1pt for feature point in im2
    [~, imax] = max(abs(xcc(:)));
    [ypeak, xpeak] = ind2sub(size(xcc),imax);
    rowOffset = (ypeak-resizedHalfSize)/imScale;
    colOffset = (xpeak-resizedHalfSize)/imScale;
    currIm2row = currRow + rowOffset;
    currIm2col = currCol + colOffset;
    
    if (rowOffset > maxRowOffset)
        maxRowOffset = round(rowOffset);
    end
    if (colOffset > maxColOffset)
        maxColOffset = round(colOffset);
    end
    
    correspondenceRows(i) = currIm2row;
    correspondenceCols(i) = currIm2col;
    %check if new point is outside of the tolerance frame
    if ((currIm2row < halfSize) || (currIm2col < halfSize) || (currIm2row > (imHeight - halfSize + 1)) || (currIm2col > (imWidth - halfSize + 1)))
        invalidCount = invalidCount + 1;
        
    else
        im2rows(i-invalidCount) = round(currIm2row);
        im2cols(i-invalidCount) = round(currIm2col);
    end
    %consider adding threshold so if no match exists, im2pt is set to null
end

%detect new features if necessary, two options...
%1: try to fill in empty spots with new features
%2: replace all features with 500 new, valid features

%option 1:
% if (invalidCount ~= 0)
%     newPoints = 0;
%     i = 0;
%     xDim = imWidth - halfSize;
%     yDim = imHeight - halfSize;
%     display('acquiring new points');
%     [x2, y2, v2] = harris(im2);
%     [sx2, sy2, sv2] = suppress(x2, y2, v2);
%     while (newPoints ~= invalidCount)\
%         currX = sx2(i);
%         currY = sy2(i);
%         if (maxRowOffset < 0)
%             if (currX < xDim && currX > (xDim-maxRowOffset))f
%             end
%         end
%         i = i + 1;

%%%try to find points intelligently
if (invalidCount ~= 0)
    display('acquiring new points');
    if (maxRowOffset < 0)
        yDim = imHeight - halfSize - maxRowOffset;
        top = false;
    else
        yDim = halfSize + maxRowOffset;
        top = true;
    end
    if (maxColOffset < 0)
        xDim = imWidth - halfSize - maxColOffset;
        left = false;
    else
        xDim = halfSize + maxColOffset;
        left = true;
    end
%     display([top,left]);
    [x2, y2, v2] = harrisRegion(im2, xDim, yDim, top, left);
    
    %%%otherwise look for any new features not in old list
    if (size(x2,1) < invalidCount)
        display('no new features, widening search region');
        [x3, y3, v3] = harris(im2);
        [sx2, sy2, ~] = suppress(x3, y3, v3);
%         suppressedPoints = [sy2,sx2];
        newPointsMask = ~ismember([sy2,sx2],[im2rows,im2cols],'rows');
        [~,~,newRows] = find(sy2.*newPointsMask);
        [~,~,newCols] = find(sx2.*newPointsMask);
    else
        [newCols, newRows, ~] = suppressRegion(x2, y2, v2, invalidCount);
    end
    im2rows((numPoints-invalidCount+1):end) = newRows(1:invalidCount);
    im2cols((numPoints-invalidCount+1):end) = newCols(1:invalidCount);
end

%option 2:
% if (invalidCount ~= 0)
%     display('acquiring new points');
%     [x2, y2, v2] = harris(im2);
%     [im2cols, im2rows, ~] = suppress(x2, y2, v2);
% end 

end

