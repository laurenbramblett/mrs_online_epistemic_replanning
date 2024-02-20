
tfmsg = rosmessage(tfpub);
for k = 1:size(TAKEOFF_HEIGHTS, 2)
  tfmsg.Data(k) = TAKEOFF_HEIGHTS(k);
end
tfmsg.Data(end+1) = TAKEOFF_DURATION;
send(tfpub, tfmsg)
