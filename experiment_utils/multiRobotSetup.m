
% Application Parameters
ROS_MASTER_IP  = "192.168.8.202";  %192.168.8.201";
ROS_MASTER_URI =  sprintf('http://%s:11311', ROS_MASTER_IP);
ROBOT_NAMES    = ["cf2", "cf4"];
num_agents     = length(ROBOT_NAMES);

TAKEOFF_HEIGHTS  = 0.6*ones(1, num_agents);
TAKEOFF_DURATION = 3.5;
LAND_HEIGHTS     = 0.05*ones(1, num_agents);
LAND_DURATION    = 3.0;

maxT = pi/4;
maxV = 0.2; 
dt   = 0.1; 
gif  = "yes";
obs  = []; % Obstacles: Leave empty for now.

%% ROS setup stuff
rosshutdown
pause(1)

setenv('ROS_MASTER_URI', ROS_MASTER_URI);
setenv('ROS_IP',ROS_MASTER_IP)
rosinit

node = ros.Node('/matlab',ROS_MASTER_IP);
rate = ros.Rate(node,1/dt);
pause(1);

vicon_str_rep = repmat("vicon",[num_agents,1]);
global_pos    = compose("/%s/%s/%s",[vicon_str_rep, ROBOT_NAMES', ROBOT_NAMES']);

cmd_str_rep   = repmat("cmd_vel",[num_agents,1]);
cmd_vel_names = compose("/%s/%s",[ROBOT_NAMES', cmd_str_rep]);

%Crazyflie topics
tfpub   = rospublisher("/takeoff_msg", "std_msgs/Float32MultiArray", "DataFormat", "struct");
landpub = rospublisher("/land_msg", "std_msgs/Float32MultiArray", "DataFormat", "struct");
goTopub = rospublisher("/goto_msg", "std_msgs/Float32MultiArray", "DataFormat", "struct");
velpub  = rospublisher("/vel_msg", "std_msgs/Float32MultiArray", "DataFormat", "struct");
singlelandpub = rospublisher("/single_rbt_land", "std_msgs/Float32MultiArray", "DataFormat", "struct");


robotMsgs.pub = velpub;
robotMsgs.msg = rosmessage(velpub);

for k = 1:num_agents
  sub_pos(k) = rossubscriber(global_pos(k));
  pause(1)
end
pause(3);
receive(sub_pos(1), 10); % Ensure that messages are flowing

xVel = zeros(num_agents,1); 
yVel = zeros(num_agents,1);

[pos,ori] = receive_vicon(sub_pos, ROBOT_NAMES);
pose      = [pos,ori'];
goal      = mean(pose(:,1:2),1);
state     = [pos; xVel, yVel]';

% Tell python Crazyflie application which crazyflies will be used
% Python will automatically create the neccessary callbacks to 
% subscribe to vicon position topics - psherman

rbtpub  = rospublisher("/robot_list", "std_msgs/String", "DataFormat", "struct", "IsLatching", false);
pause(1)

rbt_lst_msg  = rosmessage(rbtpub);
rbt_list = '';
for k = 1:num_agents
  rbt_list = rbt_list + ROBOT_NAMES(k) + ',';
end
rbt_lst_msg.Data = convertStringsToChars(strip(rbt_list, 'right', ','));
send(rbtpub, rbt_lst_msg);
