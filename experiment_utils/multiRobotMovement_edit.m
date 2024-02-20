

[pos, ori] = receive_vicon(sub_pos, ROBOT_NAMES);
pose = [pos, ori']';


lastPos = state(:,1:2);
for k = 1:num_agents
    loc = state(1:2,k)';
    [forceX,forceY] = force(loc, obs, goal, xVel(k), yVel(k), lastPos(k~=1:end,:));
    dxdt = [xVel(k), yVel(k), forceX, forceY]';
    state(:,k) = state(:,k) + dxdt*dt;
    xVel(k) = sign(state(3,k))*min(abs(state(3,k)),maxV); 
    yVel(k) = sign(state(4,k))*min(abs(state(4,k)),maxV);
    state(:,k) = [state(1:2,k); xVel(k); yVel(k)];
    goal = state(1:2,:)';
end

pos_t(i,:) = mean(state(1:2,:),2)';
for k = 1:num_agents
    robotMsgs(1,k).msg.Data(1) = linV{k}(1);
    robotMsgs(1,k).msg.Data(2) = linV{k}(2);
    robotMsgs(1,k).msg.Data(3) = 0;
    robotMsgs(1,k).msg.Data(4) = 2;
    send(robotMsgs(1,k).pub,robotMsgs(1,k).msg)
end
        


% --------------------------------- %
% ---     Local Functions       --- %
% --------------------------------- %

function [forceX,forceY] = force(pos,obs,goal,xVel,yVel,otherPos)
    distToNeigh = 5;
    rho  = 7; 
    ko   = 4; 
    kg   = 1; 
    damp = 4; 
    kn   = 10; 
    
    %Obstacle force
    c_obs = find(pdist2(pos,obs)<rho);
    if ~isempty(c_obs)
        nearObs = obs(c_obs,:);
        dist = pdist2(pos,nearObs)';
        obsNorm = (nearObs-pos)./norm(nearObs-pos);
        oForceX = sum(ko.*(dist-rho).*obsNorm(:,1));
        oForceY = sum(ko.*(dist-rho).*obsNorm(:,2));
    else
        oForceX = 0;
        oForceY = 0;
    end

    %Goal Force
    goalDist = pdist2(pos,goal);
    goalNorm = (goal-pos)./goalDist;
    if goalDist<2; goalNorm = [0,0]; end
    gForceX = kg*goalDist*goalNorm(1);
    gForceY = kg*goalDist*goalNorm(2);
    
    %Neighbor Force
    nDist = pdist2(pos,otherPos)';
    nNorm = (otherPos-pos)./nDist;
    nForceX = sum(kn.*(nDist-distToNeigh).*nNorm(:,1));
    nForceY = sum(kn.*(nDist-distToNeigh).*nNorm(:,2));
    
    % Sum Total Forces
    forceX = nForceX + oForceX + gForceX - damp*xVel;
    forceY = nForceY + oForceY + gForceY - damp*yVel;
    forceX = sign(forceX)*min(abs(forceX),5);
    forceY = sign(forceY)*min(abs(forceY),5);    
end
