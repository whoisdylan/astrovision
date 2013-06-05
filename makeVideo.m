function makeVideo(path_to_images, numimages)

    %set up video object
    writerObj = VideoWriter('results.avi');
    writerObj.FrameRate = 10;
%     writerObj.Quality = 100;
    open(writerObj);
    
    %write frames
    for frame = 1:numimages
        fname = sprintf('%s/%d.png',path_to_images,frame);
        currFrame = imread(fname);
        
%         %plot 3 ims side by side
%         currIm1 = ['left_' num2str(frame-1,'%.4d') '.png'];
%         currIm2 = ['left_' num2str(frame,'%.4d') '.png'];
%         im1 = imread(['helicopter images/' currIm1]);
%         im2 = imread(['helicopter images/' currIm2]);
%         [h1,w1] = size(im1);
%         catImage = uint8(zeros(h1,w1*3,3));
%         catImage(1:h1,1:w1,:) = im1;
%         catImage(1:h2,w1+1:w1*2,:) = currFrame;
%         catImage(1:h2,2*(w1+1):w1*3,:) = im2;
%         imshow(catImage); hold on;
%         %plot im1 and im2 feature points on im1 and im2
%         [x1,y1,v1] = harris(im1);
%         [x2,y2,v2] = harris(im2);
%         [sx1,sy1,sv1] = suppress(x1,y1,v1);
%         [sx2,sy2,sv1] = suppress(x2,y2,v2);
%         plot(sx1,sy1,'r.');
%         plot(sx2+w1*2,sy1,'b.');
%         hold off;
%         saveas(catImage,['harris consistency/frame_' num2str(imNum) '.png']);
        
        writeVideo(writerObj,currFrame);
    end
    
    close(writerObj);
end