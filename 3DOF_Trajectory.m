%% ---------- SETUP ----------
startup()
clc
% % Set up the arm as a group
family = '3dof';
names = {'Base', 'Shoulder', 'Elbow'};
group = HebiLookup.newGroupFromNames(family, names);

% If not connected to any physical modules
%group = HebiUtils.newImitationGroup(3)

%% ---------- KINEMATICS ----------
kin = HebiKinematics('3dofDescription.hrdf');


%% ---------- FEEDBACK ----------
fbk = group.getNextFeedback;

%% ---------- MOVEMENT ----------
numModules = group.getNumModules;
trajGen = HebiTrajectoryGenerator();
cmd = CommandStruct();
% cmd.position = zeros(1,numModules);
% cmd.velocity = zeros(1,numModules);
cmd.effort = [];
tic;

%% ------ Main ------
% targets = [0.22, 0.0, -0.10;
%            0.22, -0.43, -0.10;
%            0.22, 0.43, -0.10;
%            0.22, 0, -0.10];
targets = [0, 0, 1;
           0, -1, 0;
           1, 0, 0];
positions = getWaypoints(targets, fbk, kin);
disp(positions)
moveArm(positions, group, cmd, trajGen)

% send position/velocity command for 1 seconds
% while toc < 2
%   cmd.position = [0, 1.5, 0];
%   % cmd.velocity = [-0.4, -1, 0];
%   % cmd.velocity = [0.1, 0, 0];
%   % kin.getForwardKinematics
%   fbk = group.getNextFeedback();  % Use getNextFeedback() to                               % limit loop rate.
%   % disp(kin.getForwardKinematics('endeffector', fbk.position));
%   group.send(cmd);
%   % disp(fbk)
%   toc;
% end


%% ---------- Helper Functions ----------
function positions = getWaypoints(targets, fbk, kin)
    [rows, cols] = size(targets)
    positions = zeros(rows, cols)
    positions(1,:) = kin.getInverseKinematics('XYZ', targets(1,:),...
        'InitialPositions', fbk.position)
    for i = 2:rows
        disp(i)
        positions(i,:) = kin.getInverseKinematics( 'XYZ', targets(i,:),...
            'InitialPositions', positions(i,:));
    end
    disp(positions)
end


function [] = moveArm(positions, group, cmd, trajGen)
[rows, ~] = size(positions);
timeToMove = 5;
time = [0 timeToMove];
trajGen.setSpeedFactor(0.4);
for i = 2:rows
    start = positions(i-1, :);
    finish = positions(i, :);
    trajectory = trajGen.newJointMove(...
        [start; finish],'Time',time);
    HebiUtils.plotTrajectory(trajectory);
    fbk = group.getNextFeedback();
    t0 = fbk.time;
    t = 0;
    
    while t < trajectory.getDuration()
        fbk = group.getNextFeedback();
        t = fbk.time - t0;
        [cmd.position, cmd.velocity, ~] = trajectory.getState(t);
        group.send(cmd);
    end
end
end

function [] = moveArm2(positions, group, cmd, trajGen)
trajectory = trajGen.newJointMove(positions);

% Visualize points along the trajectory
% HebiUtils.plotTrajectory(trajectory);

% Execute trajectory open-loop in position and velocity
t0 = tic();
t = toc(t0);
while t < trajectory.getDuration()
    t = toc(t0);
    fbk = group.getNextFeedback();  % limits loop rate
    [cmd.position, cmd.velocity, ~] = trajectory.getState(t);
    group.send(cmd);
end
end



function [] = startup()
    % startup sets up libraries and should be started once on startup.
    currentDir = fileparts(mfilename('fullpath'));
    addpath(fullfile(currentDir , 'hebi'));
    hebi_load(); % explicitely pre-load library
end
