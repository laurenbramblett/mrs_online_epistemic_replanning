function singleRobotLand(pub, idx, height, duration)

  landmsg = rosmessage(pub);

  landmsg.Data(1) = idx - 1; % Matlab arrays are 1 indexed, Python's are 0 indexed
  landmsg.Data(2) = height;
  landmsg.Data(3) = duration;
  fprintf("Single Robot Land: %0.1f, %0.1f, %0.1f\n", landmsg.Data(1), landmsg.Data(2), landmsg.Data(3));
  
  send(pub, landmsg)
end