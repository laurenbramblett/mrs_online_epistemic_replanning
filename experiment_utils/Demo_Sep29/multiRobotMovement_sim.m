clc; clear all; close all;
rng(3) %2,3 works

goal = [2,2];
maxV = 0.2; dt = 0.1; maxT = pi/4;
gif = "yes";

% map = mapClutter(10,{'Box','Circle'},'MapSize',[100,100],'MapResolution',1);
map = binaryOccupancyMap;
map = occupancyMatrix(map);
[o_r,o_c] = find(map == 1);
obs = [o_c,o_r];
% obs = checkObs(start,goal,obs);
map = makeMap(obs);
%Potential Field
start = [-2, -2; -1.9 -1.9; -2, -1.9];
agents = 3;
pos = start;


%Start
pos_t(1,:) = mean(start,1);
iters = 1000;
xVel = zeros(agents,1); yVel = zeros(agents,1);
state = [pos, xVel, yVel];

for i = 1:iters    
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
    
    
    pos_t(i,:) = mean(state(:,1:2),1);        
    
    
    %plot
    hold on
    % h = imagesc(map); colormap(flipud(bone));
    axis xy; axis equal
    PlotAgents(state(:,1),state(:,2),adj_mat,0.05)
    plot(pos_t(1:i,1),pos_t(1:i,2),'b')
    plot(goal(1),goal(2),'gp')
    xlim([-3,3]); ylim([-3,3])
    drawnow
    title(sprintf("$t = %0.2f$s",i*dt),'interpreter','latex')
    set(gcf,'color','w'); box on; 
    hold off
    F(i) = getframe(gcf);
    cla
    if pdist2(goal,pos_t(end,:))<0.01
        break
    end
end
% if gif == "yes"
%     makeVideo(F,'multiRobot_example');
% end

function [forceX,forceY] = force(pos,goal,xVel,yVel,edges,k)
    rho = 7; ko = 4; kg = 1; damp = 4; kn = 10; distToNeigh = 0.5;
    %Obs Force
    oForceX = 0; oForceY = 0;
    %Goal Force
    goalDist = vecnorm(pos(k,:)-goal,2,2);
    goalNorm = (goal-pos(k,:))./goalDist;
    if goalDist<0.3; goalNorm = [0,0]; end
    gForceX = kg*goalDist*goalNorm(1);
    gForceY = kg*goalDist*goalNorm(2);
    
    %Neighbor Force
    conBots = edges(edges(:,1)==k,2);
    conBots = [conBots edges(edges(:,2)==k,1)];
    otherPos = pos(conBots,:);
    nDist = vecnorm(pos(k,:)-otherPos,2,2);
    nNorm = (otherPos-pos(k,:))./nDist;
    nForceX = sum(kn.*(nDist-distToNeigh).*nNorm(:,1));
    nForceY = sum(kn.*(nDist-distToNeigh).*nNorm(:,2));
    
    forceX = nForceX + oForceX + gForceX - damp*xVel;
    forceY = nForceY + oForceY + gForceY - damp*yVel;
    forceX = sign(forceX)*min(abs(forceX),0.5);
    forceY = sign(forceY)*min(abs(forceY),0.5);
    
end

function map = makeMap(obs)
    mapSize = [100,100];
    map = zeros(mapSize);
    idx = sub2ind(mapSize,obs(:,2),obs(:,1));
    map(idx) = 1;
end

function obs = checkObs(start,goal,obs)
    [~,s_idx] = ismember(start,obs,'rows');
    [~,g_idx] = ismember(goal,obs,'rows');
    if s_idx>0
        obs(s_idx,:)=[];
    elseif g_idx>0
        obs(g_idx,:)=[];
    end
end


