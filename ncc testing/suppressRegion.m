function [ suppX, suppY, suppV ] = suppressRegion(x,y,v)
%computes adaptive non-maximal suppression on given set of points
%x, y, v are column vectors.  v is corner strength

numPoints = 100;
dists = zeros(size(y,1),1);
pts = zeros(size(y,1),1);
suppY = zeros(numPoints,1);
suppX = zeros(numPoints,1);
suppV = zeros(numPoints,1);


allDists = pdist2([x,y],[x,y]);
for i=1:size(y,1)
    minDist = inf;
    minPt = 0;
    for j=1:size(y,1)
        if (i ~= j)
            if (v(i) < (v(j)*.9))
%                 currDist = sqrt( (x(j)-x(i))^2 + (y(j)-y(i))^2 );
                currDist = allDists(i,j);
                if (currDist < minDist)
                    minDist = currDist;
                    minPt = j;
                end
            end
        end
    end
    if (minPt == 0) %if i is stronger than all j*.9
        dists(i) = 0;
        pts(i) = i;
    else
        dists(i) = minDist;
        pts(i) = minPt;
    end
end

[~, indices] = sortrows(dists,-1); %sort descending (max first)

for k = 1:numPoints
    suppY(k) = y(pts(indices(k)));
    suppX(k) = x(pts(indices(k)));
    suppV(k) = v(pts(indices(k)));
end

end