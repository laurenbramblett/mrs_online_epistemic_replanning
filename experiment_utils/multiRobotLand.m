
landmsg = rosmessage(landpub);
for k = 1:size(LAND_HEIGHTS, 2)
  landmsg.Data(k) = LAND_HEIGHTS(k);  
end
landmsg.Data(end+1) = LAND_DURATION;
send(landpub, landmsg)
