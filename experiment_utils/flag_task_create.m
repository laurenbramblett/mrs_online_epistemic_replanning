[fImg, m, alphachannel] = imread('images/flag.png'); %Task image
flag_crs = cbrewer2('qual','Set1', max(3, numBots));

% fImg(fImg>0) = ;
fImg_colors = sum(fImg,3);
ind = find(fImg_colors>0);
fImg_robots = repmat({fImg},numBots + 1,1);
flag_crs = [flag_crs(1:numBots,:); 0.5 0.5 0.5];
for k = 1:numBots + 1
    for d = 1:3
        slice = fImg_robots{k}(:,:,d);
        slice(ind) = repelem(flag_crs(k,d),length(ind))*255;
        fImg_robots{k}(:,:,d) = slice;
    end
end

[no_fImg, no_m, no_alphachannel] = imread('images/noflag_edit.png'); %Task image
no_flag_crs = cbrewer2('qual','Set1',numBots);
% fImg(fImg>0) = ;
no_fImg_colors = sum(no_fImg,3);
no_ind = find(no_fImg_colors>0);
no_fImg_robots = repmat({no_fImg},numBots + 1,1);
no_flag_crs = [no_flag_crs(1:numBots,:); 0.5 0.5 0.5];
for k = 1:numBots + 1
    for d = 1:3
        no_slice = no_fImg_robots{k}(:,:,d);
        no_slice(no_ind) = repelem(no_flag_crs(k,d),length(no_ind))*255;
        no_fImg_robots{k}(:,:,d) = no_slice;
    end
end


% imshow(no_fImg_robots{1})
