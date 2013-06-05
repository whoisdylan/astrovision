function [im2rows,im2cols] = ncc_match(im1,im2,im1rows,im1cols)
%takes in im1 im2 and im1 feature points, creates descriptors of im1pts and
%searches windows within im2 to find and return im2pts

descHalfSize = 16;
windowHalfSize = 32;

im2rows = zeros(size(im1rows,1),1);
im2cols = zeros(size(im1cols,1),1);

for i=1:size(im1rows,1)
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
    im2rows(i) = currRow + rowOffset;
    im2cols(i) = currCol + colOffset;
    %consider adding threshold so if no match exists, im2pt is set to null
end



end

