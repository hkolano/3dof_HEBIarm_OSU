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
% targets = [0.09, 0.0, -0.04;
%            0.09, -0.17, -0.04;
%            0.09, 0.17, -0.04;
%            0.09, 0, -0.04];
targets = [0, 0, 1;
           .2, .2, 0];
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
    [rows, cols] = size(targets);
    positions = zeros(rows+1, cols);
    positions(1,:) = fbk.position;
    disp(positions)
    for i = 2:rows
        positions(i,:) = kin.getInverseKinematics( 'XYZ', targets(i,:),...
            'InitialPositions', positions(i,:));
    end
    disp(positions)
end


function [] = moveArm(positions, group, cmd, trajGen)
[rows, ~] = size(positions);
timeToMove = 20;
time = [0 timeToMove];
% trajGen.setSpeedFactor(1);
for i = 2:rows
    start = positions(i-1, :);
    finish = positions(i, :);
    trajectory = trajGen.newJointMove(...
        [start; finish],'time', time);
    
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

function [] = startup()
    % startup sets up libraries and should be started once on startup.
    currentDir = fileparts(mfilename('fullpath'));
    addpath(fullfile(currentDir , 'hebi'));
    hebi_load(); % explicitely pre-load library
end