function [pos,ori] = receive_vicon(sub,names)

    for i = 1:length(names)
        pos_msg = sub(1,i).LatestMessage;
        pos(i,1) = pos_msg.Transform.Translation.X;
        pos(i,2) = pos_msg.Transform.Translation.Y;
        oriList = quat2eul([pos_msg.Transform.Rotation.W,...
                   pos_msg.Transform.Rotation.X,...
                   pos_msg.Transform.Rotation.Y,...
                   pos_msg.Transform.Rotation.Z]);
        ori(i) = oriList(1);
    end
end