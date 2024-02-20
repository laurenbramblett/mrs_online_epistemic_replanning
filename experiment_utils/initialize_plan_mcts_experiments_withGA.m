%Initialize all particles and robots
%Read in plan
inits = readmatrix(initConditionsTxt);
headIdx = find(all(inits == [-1,-1],2));
numBots = headIdx(1)-1; 
plan = readmatrix(allocationsTxt);

%%%%-----%%%%
%Tasks
depot = inits(end,:);
numTasks = diff(headIdx);
taskLoc = inits(headIdx(1)+1:headIdx(2)-1,:);
taskLoc = [taskLoc;inits(end,:)];

%%%Obs & Initial Map%%%
map.size = round([max(taskLoc(:,1))+5,max(taskLoc(:,2))+5]);
% map.size = [max(map.size),max(map.size)];
[Xg,Yg] = meshgrid(1:map.size(1),1:map.size(2));
map.gridPoints = [Xg(:),Yg(:)];
% [X,Y] = meshgrid(20:30,20:30);
xObs = []; %X(:); 
yObs = []; %Y(:);
bdObs = [(1:map.size(1))',repelem(1,map.size(1))';
         (1:map.size(1))',repelem(map.size(2),map.size(1))';
         repelem(1,map.size(2))',(1:map.size(2))';
         repelem(map.size(1),map.size(2))',(1:map.size(2))'];
map.obs = [xObs,yObs; bdObs];
mapP = map; 
M0 = ones(map.size)*0.5;

%%%Robot Inits%%%
initLoc = inits(1:headIdx(1)-1,:);
initPose = [initLoc,repelem(pi/2,size(initLoc,1))']';
dummy = struct("M0",[],"pose",[],"angles",[],"maxRange",[],"max_v",[]);
robots = repmat(dummy,numBots,1); 
% Don't let robots end up at the exact same spot
robot_depots = zeros(numBots, 2);
for k = 1:numBots
  robots(k).depot = depot;
  if real_robots
      if k == 1
        robots(k).depot(1) = depot(1) - 0.5;
      elseif k == 2
        robots(k).depot(1) = depot(1) + 0.5;
      elseif k == 3
        robots(k).depot(2) = depot(2) + 0.5;
      elseif k == 4
        robots(k).depot(2) = depot(2) - 0.5;
      end
  end

  robot_depots(k, :) = robots(k).depot;
end
globalPlan = [];
for k = 1:numBots
    robots(k).ID = k;
    robots(k).dt = dt;
    robots(k).M0 = M0(:);

    robots(k).pose     = initPose(:,k)';
    robots(k).pose_sim = initPose(:,k)';

    %Sensor parameters
    robots(k).angles = -pi:pi/16:pi; %initialize sensor parameters
    robots(k).maxRange = obsDist; %initialize sensor max range

    %Motion parameters
    robots(k).max_v = initRobVel;
    robots(k).numParticles = ones(1,numBots)*numParticles;
    robots(k).state = "Do Tasks";

    %Plan
    kplan = plan(k,:);
    assigned = find(kplan~=0);
    [~,order] = sort(kplan(kplan~=0));

    %task order: task id, order id, robot, done?
    robots(k).taskQueue = assigned(order(1:end-1));
    robots(k).originalPlan = robots(k).taskQueue;
    robots(k).globalTaskLocs = taskLoc;

    %plan order: location
    robots(k).plan = [taskLoc(robots(k).taskQueue,:),robots(k).taskQueue',...
                        zeros(length(robots(k).taskQueue),2),(1:length(robots(k).taskQueue))'];
    robots(k).reachedNewParticle = true;
    robots(k).goalLoc = [1e10,1e10];
    % robots(k).depot = depot;
    robots(k).particleFollow = 1;
    robots(k).particleGuess = ones(numBots,1);
    robots(k).particleLastUpdate = zeros(numBots,1);
    robots(k).robotLastUpdate = zeros(numBots,1);
    robots(k).connected_replan = 0;

    % robots(k).velocityGuess = ones(numBots,1)*initRobVel;
    robots(k).replan = 0;
    robots(k).commQueue = [];
    robots(k).rzQueue = [];
    robots(k).dfwm = [];
    robots(k).tasks_done = [];
    robots(k).deadBots = zeros(numBots,1);
    robots(k).root = {};
    robots(k).iamdifferent = false;
    robots(k).history = [];
    robots(k).at_final_pos = false; % Robot done with tasks AND at final depot position
    robots(k).commError = commError; %Buffer
    robots(k).planID = 1;
end

fn = fieldnames(robots(k))';

%Particle Inits
for k = 1:numBots
    for r = 1:numBots
        for p = 1:numParticles
            particles(k,r,p) = robots(r);
            particles(k,r,p).max_v = robots(r).max_v*(1-(p-1)*decay);
        end
    end
end

for k = 1:numBots
    for r = 1:numBots
        for p = 1:numParticles
            robots(k).particles(r,p) = particles(k,r,p);
        end
    end
end

% Find time windows based on plan
knapsack = {robots(:).taskQueue};
for i = 1:numBots
    knapsack{i} = knapsack{i}';
end
tw_poses = cell2mat({robots(:).pose}');
tw_poses = tw_poses(:,1:2);
vels = [robots(:).max_v];
[timeWindows] = findIntersections(knapsack,taskLoc,tw_poses,vels,robots(1).maxRange);
