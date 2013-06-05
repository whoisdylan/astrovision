%given im1, im2, and feature points (sx1,sy1) in im1 compute ncc across a window in im2 to find
%correspondence points in im2

descHalfSize = 16;
windowHalfSize = 32;
%create descriptor from im1 and search window from im2
desc = im1((sy1(1)-descHalfSize):(sy1(1)+descHalfSize-1),(sx1(1)-descHalfSize):(sx1(1)+descHalfSize-1));
window = im2((sy1(1)-windowHalfSize):(sy1(1)+windowHalfSize-1),(sx1(1)-windowHalfSize):(sx1(1)+windowHalfSize-1));


xcc = normxcorr2(desc,window);
[max_xcc, imax] = max(abs(xcc(:)));
[ypeak, xpeak] = ind2sub(size(xcc),imax);
%calculate row (y) and col (x) offset for feature point in im2
rowOffset = ypeak-descHalfSize*3;
colOffset = xpeak-descHalfSize*3;
