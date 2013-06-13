function [ suppX, suppY, suppV ] = suppressRegion(x,y,v,numPoints)
%computes adaptive non-maximal suppression on given set of points
%x, y, v are column vectors.  v is corner strength

dists = zeros(size(y,1),1);
% pts = zeros(size(y,1),1);
% suppY = zeros(numPoints,1);
% suppX = zeros(numPoints,1);
% suppV = zeros(numPoints,1);
% v2 = v;

allDists = pdist2([x,y],[x,y]);
for i=1:size(y,1)
    minDist = inf;
    for j=1:size(y,1)
%         if (ismember(i,pts) || ismember(j,pts)) break;
%         end
        if (i ~= j)
            if (v(i) < (v(j)*.9))
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

suppY = y(indices(1:numPoints));
suppX = x(indices(1:numPoints));
suppV = v(indices(1:numPoints));

% for k = 1:numPoints
%     suppY(k) = y(pts(indices(k)));
%     suppX(k) = x(pts(indices(k)));
%     suppV(k) = v(pts(indices(k)));
% end

end