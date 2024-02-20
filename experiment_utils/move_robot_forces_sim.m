function [pose,force] = move_robot_forces_sim(robot,pose,map,goal,obsRho)
    numParticles = size(robot.particles,2);
    dt = robot.dt;
%     obsRho = obsDist; 
    max_v = robot.max_v; 
    obsIdx = find(robot.M0>0.5);
    [r_o,c_o] = ind2sub(map.size,obsIdx);
    obs = [c_o,r_o];
    %% Obs Force
    if ~isempty(obs)
        obsDist = pdist2(pose(1:2),obs)';
    else
        obsDist = [];
    end
    c_obs = obs(obsDist<obsRho,:); obsDist = obsDist(obsDist<obsRho,:);
    if ~isempty(c_obs)
        [minVal,minIdx] = min(obsDist);
        dirObs = c_obs(minIdx,:)-pose(1:2);
        if minVal>1.5
            obsSpring = -(dirObs)./minVal*(1/1.5-1/minVal)*(1/minVal.^2);
        else
            obsSpring = -sign(dirObs).*[1000,1000];
        end
    else
        obsSpring = [0,0]; 
    end
    %% Goal Force
    goalDist = pdist2(pose(1:2),goal);
    if goalDist>0
        goalNorm = (goal-pose(1:2))./goalDist;
    else
        goalNorm = [0,0];
    end
    goalSpring = goalDist*goalNorm;
    if sqrt(sum(goalSpring.^2))>max_v
        goalSpring = goalSpring*max_v/sqrt(sum(goalSpring.^2));
    end

    %% Other Robot Force
%     otherPose = zeros(size(robot.particles,1)-1,2);
%     count = 0;
%     for r = 1:size(robot.particles,1)
%         if r~=robot.ID
%             particleIdx = min(robot.particleGuess(r),numParticles);
%             otherPose(r+count,:) = robot.particles(r,particleIdx).pose(1:2);
%         else
%             count = count-1;
%         end
%     end
%     otherDist = pdist2(pose(1:2),otherPose)';
%     c_Other = otherPose(otherDist<obsRho,:); otherDist = otherDist(otherDist<obsRho,:);
%     if ~isempty(c_Other)
%         [minVal,minIdx] = min(otherDist);
%         otherSpring = (c_Other(minIdx,:)-pose(1:2))./minVal*(1/obsRho-1/minVal)*(1/minVal.^2);
%     else
%         otherSpring = [0,0];
%     end
%     
    %% Combined Force
    v = 1/dt*goalSpring; %+ 6*obsSpring;% + 6/dt*otherSpring;

    if sqrt(sum(v.^2)) > max_v 
        v = v*max_v/sqrt(sum(v.^2));
    end 
    
    if norm(v)<0.1
        v = [0,0];
    end
    force = v;
    % pose(3) = 0;
    theta_d = wrapToPi(atan2(v(2),v(1))-pose(3));
    % theta_d = sign(theta_d)*min(pi/8, abs(theta_d));
    % vel = cos(theta_d)*norm(v);
    vel = norm(v);
    pose(3) = wrapToPi(pose(3)+theta_d);
    pose(1) = pose(1) + (vel*cos(pose(3))+normrnd(0,sqrt(0)))*dt;
    pose(2) = pose(2) + (vel*sin(pose(3))+normrnd(0,sqrt(0)))*dt;
    
    
    % pose = pose(1:2);

end