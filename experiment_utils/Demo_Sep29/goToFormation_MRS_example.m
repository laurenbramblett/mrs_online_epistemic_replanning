clc; clear all; close all;
rng(3) %2,3 works
%Potential Field
dt = 0.05; cf_height = 1.2;
%% Ros stuff
rosshutdown
pause(1)
setenv('ROS_MASTER_URI','http://192.168.8.201:11311')
setenv('ROS_IP','192.168.8.201')
rosinit
% rosinit('http://192.168.8.201:11311')
node = ros.Node('/matlab_2','192.168.8.201');
pause(1);
names = ["cf2","cf4"]; agents = length(names);
for k = 1:agents
    global_pos(1,k) = strcat("vicon/",names(k),"/",names(k));
end

odom_topic = "odom";
cmd_vel_topic = "cmd_vel";
topic_name_rep = repmat(cmd_vel_topic,[agents,1]);
cmd_vel_names = compose("/%s/%s",[names',topic_name_rep]);

%Crazyflie topics
tfpub = rospublisher("/takeoff_msg","std_msgs/Float32MultiArray","DataFormat","struct");
landpub = rospublisher("/land_msg","std_msgs/Float32MultiArray","DataFormat","struct");
goTopub = rospublisher("/goto_msg","std_msgs/Float32MultiArray","DataFormat","struct");
velpub = rospublisher("/vel_msg","std_msgs/Float32MultiArray","DataFormat","struct");

tfmsg = rosmessage(tfpub);
for k = 1:agents
    tfmsg.Data(k) = cf_height;
end
% tfmsg.Data(3) = 0.6;
tfmsg.Data(end+1) = 3;
send(tfpub,tfmsg)

r = ros.Rate(node,1/dt);
% reset(r);

robotMsgs.pub = velpub;
robotMsgs.msg = rosmessage(velpub);

for i = 1:agents
    sub_pos(i) = rossubscriber(global_pos(i));
    pause(1)
end
helmet_sub = rossubscriber("vicon/helmet_3/helmet_3");
pause(3);
receive(sub_pos(1), 10);  % Ensure that messages are flowing

joy_sub = rossubscriber('/joy');
receive(joy_sub,10)
whichCrazyflie_control = 1;
state = zeros(agents,4);
[pos,ori] = receive_vicon(sub_pos,names);
pose = [pos,ori'];

state(:,1:2) = pose(:,1:2);
iters = 10000; xVel = zeros(agents,1); yVel = zeros(agents,1);
maxV = 0.2; rad = 0.6;
for i = 1:iters
    i
    [pos,ori] = receive_vicon(sub_pos,names);
    pose = [pos,ori'];
    [helmet_pos,~] = receive_vicon(helmet_sub,"helmet_3");

    state(:,1:2) = pose(:,1:2);
    lastPos = state(:,1:2);
    if i == 1
        goalAng(1) = 2*pi*rand()-pi;
        goal(1,:) = rad.*[cos(goalAng(1)),sin(goalAng(1))];
        angles = 2*pi/(agents);
        for j = 1:agents-1
            goalAng(j+1) = wrapToPi(goalAng(j) + angles);
            goal(j+1,:) = rad.*[cos(goalAng(j+1)),sin(goalAng(j+1))];
        end
    end

    for k = 1:agents
        joy_msg = joy_sub.LatestMessage;
        if joy_msg.Buttons(8) == 1 && k == whichCrazyflie_control %Right trigger
            xVel(k) = -sign(joy_msg.Axes(1))*min(abs(joy_msg.Axes(1)),maxV);
            yVel(k) = sign(joy_msg.Axes(2))*min(abs(joy_msg.Axes(2)),maxV);
            
        else
            [forceX,forceY] = force_helmet(lastPos,goal(k,:),xVel(k),yVel(k),k,helmet_pos);
            dxdt = [xVel(k), yVel(k), forceX, forceY];
            state(k,:) = state(k,:) + dxdt;
            xVel(k) = sign(forceX)*min(abs(state(k,3)),maxV); 
            yVel(k) = sign(forceY)*min(abs(state(k,4)),maxV);
            disp(xVel(k)); disp(yVel(k))
        end
    end

    pos_t(i,:) = mean(state(:,1:2),1);    

    if joy_msg.Buttons(4) == 1
        break
    end


    for k = 1:agents
        tmp = (k-1)*3;
        robotMsgs.msg.Data(tmp+1) = xVel(k)+1e-4;
        robotMsgs.msg.Data(tmp+2) = yVel(k)+1e-4;
        fprintf("xVel: %f",xVel)
        fprintf("yVel: %f",yVel)
        robotMsgs.msg.Data(tmp+3) = cf_height;
        % robotMsgs(1,k).msg.Data(4) = 2;      
    end
    send(robotMsgs.pub,robotMsgs.msg)

        
    waitfor(r);

end

landmsg = rosmessage(landpub);
for k = 1:agents
    landmsg.Data(k) = 0.01;
end
% landmsg.Data(3) = 0.01;
landmsg.Data(end+1) = 3;
send(landpub,landmsg)
pause(1)
rosshutdown
