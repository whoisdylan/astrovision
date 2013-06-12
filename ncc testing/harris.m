function [x,y,v] = harris(im)
%finds harris corners within the given image, imrgb
%by Alyosha Efros

% im = im2double(rgb2gray(imrgb));
im = im2double(im);
g1 = fspecial('gaussian',9,1); %gaussian with sigma_d
g2 = fspecial('gaussian',11,1.5); %gaussian with sigma_i
img1 = conv2(im,g1,'same'); %blur image with sigma_d
Ix = conv2(img1,[-1 0 1],'same'); % take x derivative
Iy = conv2(img1,[-1;0;1],'same'); %take y derivative

%compute elements of the Harris matrix H
%%% we can use blur instead of the summing window
Ix2 = conv2(Ix.*Ix,g2,'same');
Iy2 = conv2(Iy.*Iy,g2,'same');
IxIy = conv2(Ix.*Iy,g2,'same');
R = (Ix2.*Iy2-IxIy.*IxIy)./(Ix2+Iy2+eps); %det(h)./(trace(h)+epsilon)

%don't want corners close to image border

% imScale = 3;
descHalfSize = 16;
windowHalfSize = 64;
halfSize = descHalfSize+windowHalfSize;

R([1:halfSize,end-(halfSize+1):end],:) = 0;
R(:,[1:halfSize,end-(halfSize+1):end]) = 0;

%non-maxima supression within 3x3 windows
suppressWindow = 16;
nonmax = inline('max(x)');
Rmax = colfilt(R,[suppressWindow suppressWindow],'sliding',nonmax); %find neighborhood max
Rnm = R.*(R==Rmax); %supress non-max

%extract all interest points
%v is corner strength value
[y,x,v] = find(Rnm);

%show them
% imagesc(im);
% colormap(gray);
% hold on
% plot(x,y,'r.');
% hold off;

end

