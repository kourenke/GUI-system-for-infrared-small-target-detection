function [mask] = Threshold_segmentation(Out,k_Th)
    mask = Out;
    avg = mean(mean(mask));
    normal = std(std(mask));
    th = avg + k_Th*normal;
    [row,col] = size(mask);
    for i = 1:row
        for j = 1:col
            if mask(i,j)>th
               mask(i,j)=255;
            else
               mask(i,j)=0;
            end
        end
    end
    
end