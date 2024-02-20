clc; clear all; close all;
rng(3) %2,3 works
%Potential Field
dt = 0.01;
%% Ros stuff
rosshutdown
pause(1)
setenv('ROS_MASTER_URI','http://192.168.8.201:11311')
setenv('ROS_IP','192.168.8.201')
rosinit
% rosinit('http://192.168.8.201:11311')
node = ros.Node('/matlab_2','192.168.8.201');
pause(1);
names = ["cf2","cf3","cf4"]; agents = length(names);
global_pos = ["vicon/cf2/cf2","vicon/cf3/cf3","vicon/cf4/cf4"];
odom_topic = "odom";
cmd_vel_topic = "cmd_vel";
topic_name_rep = repmat(cmd_vel_topic,[agents,1]);
cmd_vel_names = compose("/%s/%s",[names',topic_name_rep]);

%Crazyflie topics
tfpub   = rospublisher("/takeoff_msg","std_msgs/Float32MultiArray","DataFormat","struct");
landpub = rospublisher("/land_msg","std_msgs/Float32MultiArray","DataFormat","struct");
goTopub = rospublisher("/goto_msg","std_msgs/Float32MultiArray","DataFormat","struct");
velpub  = rospublisher("/vel_msg","std_msgs/Float32MultiArray","DataFormat","struct");

tfmsg = rosmessage(tfpub);
for k = 1:agents
    tfmsg.Data(k) = 0.6;
end
% tfmsg.Data(3) = 0.6;
tfmsg.Data(end+1) = 3;
send(tfpub,tfmsg)

% keyboard;
% 


r = ros.Rate(node,1/dt);
% reset(r);


robotMsgs.pub = velpub;
robotMsgs.msg = rosmessage(velpub);
for i = 1:agents
    sub_pos(i) = rossubscriber(global_pos(i));
    pause(1)
end
pause(3);
receive(sub_pos(1), 10);  % Ensure that messages are flowing



maxV = 0.2; dt = 0.1; maxT = pi/4;
gif = "yes";

iters = 1000;
xVel = zeros(agents,1); yVel = zeros(agents,1);

[pos,ori] = receive_vicon(sub_pos,names);
pose = [pos,ori'];


goal = mean(pose(:,1:2),1);
state = [pos, xVel, yVel];

for i = 1:iters
    i
    [pos,ori] = receive_vicon(sub_pos,names);
    pose = [pos,ori'];

    state(:,1:2) = pose(:,1:2);
    lastPos = state(:,1:2);
    edges = gabriel_graph(lastPos);
    idxs = sub2ind([agents,agents],edges(:,1),edges(:,2));
    adj_mat = zeros(agents,agents);
    adj_mat(idxs) = 1; adj_mat = adj_mat + adj_mat';

    for k = 1:agents
        [forceX,forceY] = force(lastPos,goal,xVel(k),yVel(k),edges,k);
        dxdt = [xVel(k), yVel(k), forceX, forceY];
        state(k,:) = state(k,:) + dxdt*dt;
        xVel(k) = sign(state(k,3))*min(abs(state(k,3)),maxV); 
        yVel(k) = sign(state(k,4))*min(abs(state(k,4)),maxV);
        state(k,:) = [state(k,1:2), xVel(k), yVel(k)];
    end
    
    goal = mean(state(:,1:2),1);
    pos_t(i,:) = mean(state(:,1:2),1);        


    for k = 1:agents
        tmp = (k-1)*3;
        robotMsgs.msg.Data(tmp+1) = state(k,3);
        robotMsgs.msg.Data(tmp+2) = state(k,4);
        robotMsgs.msg.Data(tmp+3) = 0;
        % robotMsgs(1,k).msg.Data(4) = 2;      
    end
    send(robotMsgs.pub,robotMsgs.msg)

    % %plot
    % hold on
    % % h = imagesc(map); colormap(flipud(bone));
    % axis xy; axis equal
    % PlotAgents(state(:,1),state(:,2),adj_mat,0.05);
    % plot(pos_t(1:i,1),pos_t(1:i,2),'b');
    % plot(goal(1),goal(2),'gp');
    % xlim([-3,3]); ylim([-3,3])
    % drawnow;
    % title(sprintf("$t = %0.2f$s",i*dt),'interpreter','latex')
    % set(gcf,'color','w'); box on; 
    % hold off
    % F(i) = getframe(gcf);
    % cla;
    % if pdist2(goal,pos_t(end,:))<0.01
    %     break
    % end
    waitfor(r);
end

landmsg = rosmessage(landpub);
for k = 1:agents
    landmsg.Data(k) = 0.01;
end
% landmsg.Data(3) = 0.01;
landmsg.Data(end+1) = 3;
send(landpub,landmsg)

rosshutdown


% if gif == "yes"
%     makeVideo(F,'multiRobot_example');
% end


% function map = makeMap(obs)
%     mapSize = [100,100];
%     map = zeros(mapSize);
%     idx = sub2ind(mapSize,obs(:,2),obs(:,1));
%     map(idx) = 1;
% end
% 
% function obs = checkObs(start,goal,obs)
%     [~,s_idx] = ismember(start,obs,'rows');
%     [~,g_idx] = ismember(goal,obs,'rows');
%     if s_idx>0
%         obs(s_idx,:)=[];
%     elseif g_idx>0
%         obs(g_idx,:)=[];
%     end
% end

