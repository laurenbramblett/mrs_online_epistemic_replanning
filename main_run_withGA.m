clc; clear; close
%%%%-----%%%%
% Add paths
addpath(genpath("centralized_plan"),genpath("experiment_utils"))

run_exe = true;
run_exe_script % Run exe script if true

store_data  = false;
plot_data   = true;
real_robots = false;
plot_inline = false;

filename          = "example";                      % storage name
seedDir           = "centralized_plan/Release";
initConditionsTxt = sprintf("%s/instance_3.txt", seedDir, filename);  % Text file with initial centralized plan
allocationsTxt    = sprintf("%s/allocation2.txt", seedDir); %Output of GA with rendezvous (allocation1.txt is the output without rendezvous)

%Init conditions
initRobVel   = 5;
obsDist      = 4;
numParticles = 1;
decay        = 0.2;
dt           = 0.1;
commError    = 0.1;

initialize_plan_mcts_experiments_withGA

allPoses = reshape([robots(:).pose]', [], numBots)';
allPlans = cell2mat({robots(:).plan}');
flag_task_create


if real_robots
  multiRobotSetup;
  [pos,ori] = receive_vicon(sub_pos, ROBOT_NAMES);
  pose      = [pos,ori'];
  for k = 1:numBots
    robots(k).pose     = pose(k,:);
    robots(k).pose_sim = robots(k).pose;
  end
end

%Failure simulations
numIters    = 1500;
goalReached = 1;
whenFail    = 40;%randi([50 80]); %60
whoFail     = 2;%[1 4];%randi(numBots);
howFail     = 2;%randi([2 numParticles]);
numHistory  = 60;

if real_robots
  disp("Press button to begin Crazyflie takeoff...");
  pause;
  multiRobotTakeoff;
end

% disp("Press button to begin experiment loop...");
% pause;

plot_stuff        = {}; 
operBotsGlobal    = true(numBots,1); 
length_connected  = 0;
plannedForAtDepot = [];
hist_data         = [];
tasksComplete     = false;

i = 0; taskColorStore = zeros(size(taskLoc,1),1); connected_cycles = 0;
while ((all(pdist2(allPoses(:,1:2),depot)>goalReached) || ~isempty(allPlans)) && i<numIters) || ...
         (all([robots.at_final_pos] == true) || ~tasksComplete)
  
  i = i + 1;

  whichBotsOperGlobal = find(operBotsGlobal);
  allPoses  = reshape([robots(:).pose]',[],numBots)';
  allPlans  = cell2mat({robots(:).plan}');
  distMat   = squareform(pdist(allPoses(:,1:2)))<2*obsDist;
  bins      = conncomp(graph(distMat));
  connected = all(bins==1);

  connected_logic_all %If connected update poses and states of particles and robots

  %Update particle positions and plans  
  for k = 1:numBots
      robots = updateParticles(robots,map,robots(k).depot,obsDist,k,goalReached);
  end
  
  for k = 1:numBots
    %Fail - instigate a failure
    if sum(i == whenFail)>0
      if any(k==whoFail(i==whenFail))
        fprintf("Robot %d: Failed on loop %d\n", k, i);
        robots(k).particleFollow = howFail;
        robots(k).iamdifferent   = true;
        robots(k).max_v          = 0;
        operBotsGlobal(k)        = false;

        if real_robots
          singleRobotLand(singlelandpub, k, LAND_HEIGHTS(k), LAND_DURATION)
        end
      end
    end
  end

  if real_robots
    % Vicon will get state for all robots at once
    [pos,ori] = receive_vicon(sub_pos, ROBOT_NAMES);
    pose = [pos,ori'];
  end

  row_data  = [];
  for k = 1:numBots
    conBots = find(bins(k)==bins); % Find which robots are connected
    task_check_mcts_all     % Check if tasks should be added or are complete
    replan_check_mcts       % If environment has changed, replan
    goal_check_mcts         % Select goal for each robot

    if real_robots
      robots(k).pose = pose(k,:);
    end
    whichFollow = min(numParticles,robots(k).particleFollow);
    sim_pose = robots(k).particles(k,whichFollow).pose;
    [robots(k).pose_sim, cmd_vel_sim] = move_robot_forces_sim(robots(k), robots(k).pose, map, robots(k).goalLoc, obsDist);

    if real_robots
      [~, cmd_vel_rbt] = move_robot_forces_experiment(robots(k), map, robots(k).pose_sim, obsDist);
    else
      cmd_vel_rbt = [0,0];
      [robots(k).pose,~] = move_robot_forces(robots(k),map, robots(k).goalLoc,obsDist);
    end

    if real_robots
      tmp = (k-1)*3;
      robotMsgs.msg.Data(tmp+1) = cmd_vel_rbt(1);
      robotMsgs.msg.Data(tmp+2) = cmd_vel_rbt(2);
      robotMsgs.msg.Data(tmp+3) = 0;
    else
      % robots(k).pose = robots(k).pose_sim;
    end

    robots(k).history(end+1,:) = robots(k).pose;
    if size(robots(k).history,1)>numHistory
        robots(k).history(1,:) = [];
    end

    row_data = [row_data, robots(k).pose, robots(k).pose_sim, robots(k).goalLoc, cmd_vel_rbt, cmd_vel_sim];
  end

  

  hist_data(end+1, :) = row_data;

  if real_robots
    send(robotMsgs.pub, robotMsgs.msg)
    waitfor(rate);
  end

  if mod(i, 10) == 0
    fprintf("loop: %d\n", i);
  end

  robots_t{i} = robots; 
  % pose_sim_t{i} = pose_sim;
  if plot_inline
     plot_plan_inline
  end

  % If all tasks are complete. Mark robot at final position if it EVER gets
  % close to it's depot. Needed since when using real robots, crazyflies
  % tend to oscillate enough to go in and out of goalReached threshold
  % such that not all robots are close to depot at the same time
  tasksComplete = isempty(my_setdiff(1:size(taskLoc,1)-1,robots(whichBotsOperGlobal(1)).tasks_done));
  if tasksComplete
    for k = 1:numBots
      if (norm(robots(k).pose(1:2) - robots(k).depot) < goalReached) || (operBotsGlobal(k) == false)
        robots(k).at_final_pos = true;
      end
    end
  end

  if all([robots.at_final_pos] == true)
    break
  end
end

if real_robots
  multiRobotLand;
  rosshutdown;
end

if store_data
    testtime = datestr(now,'yy-mm-dd-HHMMSS');

    storefn = sprintf("data_storage/%s_%s.mat", filename, testtime);
    save(storefn,'-v7.3')

    if real_robots
      tbl = array2table(hist_data);
      for i = 1:numBots
        temp_k = 12*(i-1);
        tbl.Properties.VariableNames(temp_k+1)  = sprintf("robot%d_x", i);
        tbl.Properties.VariableNames(temp_k+2)  = sprintf("robot%d_y", i);
        tbl.Properties.VariableNames(temp_k+3)  = sprintf("robot%d_yaw", i);
        tbl.Properties.VariableNames(temp_k+4)  = sprintf("robot%d_sim_x", i);
        tbl.Properties.VariableNames(temp_k+5)  = sprintf("robot%d_sim_y", i);
        tbl.Properties.VariableNames(temp_k+6)  = sprintf("robot%d_sim_yaw", i);
        tbl.Properties.VariableNames(temp_k+7)  = sprintf("robot%d_goal_x", i);
        tbl.Properties.VariableNames(temp_k+8)  = sprintf("robot%d_goal_y", i);
        tbl.Properties.VariableNames(temp_k+9)  = sprintf("robot%d_cmd_vel_x", i);
        tbl.Properties.VariableNames(temp_k+10) = sprintf("robot%d_cmd_vel_y", i);
        tbl.Properties.VariableNames(temp_k+11) = sprintf("robot%d_cmd_sim_x", i);
        tbl.Properties.VariableNames(temp_k+12) = sprintf("robot%d_cmd_sim_y", i);
      end
      csv_file = sprintf("data/mcts_data_%s.csv", testtime);
      writetable(tbl, csv_file, 'Delimiter', ',', 'WriteMode', 'overwrite');
    end
end

cla;
plot_plan(robots_t,taskLoc,obsDist,plot_data,filename,fImg_robots,alphachannel,1,initPose)
% plot_plan_pretty_experiments(robots_t,taskLoc,obsDist,1,filename,...
    % fImg_robots,no_fImg_robots,alphachannel,no_alphachannel,1,initPose,taskColorStore,depot)


