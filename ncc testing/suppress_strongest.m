function [ suppX, suppY, suppV ] = suppress_strongest(x,y,v)
%computes adaptive non-maximal suppression on given set of points
%x, y, v are column vectors.  v is corner strength
%keeps best 20 corners no matter what

numPoints = 300;
dists = zeros(size(y,1),1);
% pts = zeros(size(y,1),1);
suppY = zeros(numPoints,1);
suppX = zeros(numPoints,1);
suppV = zeros(numPoints,1);
% v2 = v;

%take best 20 corners first
[sortedV,vIndex] = sort(v,'descend');
suppY(1:20) = y(vIndex(1:20));
suppX(1:20) = x(vIndex(1:20));
suppV(1:20) = sortedV(1:20);

%remove these values from suppression calculation
x2=x;y2=y;v2=v;
x(vIndex(1:20)) = [];
y(vIndex(1:20)) = [];
v(vIndex(1:20)) = [];

allDists = pdist2([x,y],[x2,y2]);
for i=1:size(y,1)
    minDist = inf;
    for j=1:size(y2,1)
%         if (ismember(i,pts) || ismember(j,pts)) break;
%         end
        if (i ~= j)
            if (v(i) < (v2(j)*.9))
%                 currDist = sqrt( (x(j)-x(i))^2 + (y(j)-y(i))^2 );
                currDist = allDists(i,j);
                if (currDist < minDist)
                    minDist = currDist;
                end
            end
        end
    end
    dists(i) = minDist;
%     pts(i) = i;
%         v2(minPt) = 0; %ensure it's not picked again
end

[~, indices] = sort(dists,'descend'); %sort descending (max first)

suppY(21:numPoints) = y(indices(1:numPoints-20));
suppX(21:numPoints) = x(indices(1:numPoints-20));
suppV(21:numPoints) = v(indices(1:numPoints-20));

% for k = 1:numPoints
%     suppY(k) = y(pts(indices(k)));
%     suppX(k) = x(pts(indices(k)));
%     suppV(k) = v(pts(indices(k)));
% end

end