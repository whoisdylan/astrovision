function [ score ] = ncc( p1, p2 )

%Given two patches, ncc calculates the normalized cross-correlation score
%between patches 1 and 2.  
%Robust to changes in illumination

    
%Wikipedia has a good description of how to calculate normalized cross
%correlation at the following link:
%en.wikipedia.org/wiki/Cross-correlation#Normalized_cross-correlation

    [r1 c1 ch1] = size(p1);
    [r2 c2 ch2] = size(p2);

    if (r1 ~= r2 || c1 ~= c2 || ch1 > 1 || ch2 > 1)
        error();
    end
    
    %find the mean of both patches
    mean1 = mean(reshape(p1, r1*c1, 1));
    mean2 = mean(reshape(p2, r2*c2, 1));
    
    std1 = std(reshape(p1, r1*c1, 1));
    std2 = std(reshape(p2, r2*c2, 1));
    
    p1 = p1 - mean1;
    p2 = p2 - mean2;
    
    p = sum(sum(p1.*p2))/sqrt(sum(sum(p1.*p1))*sum(sum(p2.*p2)));
    
    score = p;
    %score = sum(sum(p));
    %score = score/(r1*c1);
end