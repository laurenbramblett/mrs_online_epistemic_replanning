
[pos,ori] = receive_vicon(sub_pos, ROBOT_NAMES);
pose = [pos,ori'];

state(:,1:2)  = pose(:,1:2);
lastPos       = state(:,1:2);
edges         = gabriel_graph(lastPos);
idxs          = sub2ind([num_agents,num_agents],edges(:,1),edges(:,2));
adj_mat       = zeros(num_agents,num_agents);
adj_mat(idxs) = 1; 
adj_mat = adj_mat + adj_mat';

for k = 1:num_agents
    conBots = find(bins(k)==bins); % Find which robots are connected
    task_check_mcts                % Check if tasks should be added or are complete
    replan_check_mcts              % If environment has changed, replan
    goal_check_mcts                % Select goal for each robot


    [forceX,forceY] = force(lastPos, robots(k).goalLoc, xVel(k), yVel(k), edges, k);
    dxdt       = [xVel(k), yVel(k), forceX, forceY];
    state(k,:) = state(k,:) + dxdt*dt;
    xVel(k)    = sign(state(k,3))*min(abs(state(k,3)),maxV); 
    yVel(k)    = sign(state(k,4))*min(abs(state(k,4)),maxV);
    state(k,:) = [state(k,1:2), xVel(k), yVel(k)];

    robots(k).pose = pose(k,:);
end

goal        = mean(state(:,1:2),1);
pos_t(i, :) = mean(state(:,1:2),1);        


for k = 1:num_agents
    tmp = (k-1)*3;
    robotMsgs.msg.Data(tmp+1) = state(k,3);
    robotMsgs.msg.Data(tmp+2) = state(k,4);
    robotMsgs.msg.Data(tmp+3) = 0;
    % robotMsgs(1,k).msg.Data(4) = 2;      
end
send(robotMsgs.pub,robotMsgs.msg)


waitfor(rate);
