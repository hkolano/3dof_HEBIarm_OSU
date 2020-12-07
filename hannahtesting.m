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

cmd = CommandStruct();
% cmd.position = zeros(1,numModules);
cmd.velocity = zeros(1,numModules);
cmd.effort = [];
tic;

%% --------- Inverse Kinematics ----
% Inverse Kinematics
goals = [0, 0.3, 0.0;
        0, 0.5, 0.2;
        0.1, 0.6, 0.3];

    for i = 1:(length(goals(:,1)))
        initialJointAngs = fbk.position;
        waypoints = kin.getInverseKinematics( 'XYZ', goals,...
    'IntialPositions', initialJointAngs );
    end

% send position/velocity command for 10 seconds
while toc < 1
  % cmd.position = [0, 1.5, 0];
  cmd.velocity = [0, 0.5, 0];
  % kin.getForwardKinematics
  fbk = group.getNextFeedback();  % Use getNextFeedback() to                               % limit loop rate.
  % disp(kin.getForwardKinematics('endeffector', fbk.position));
  group.send(cmd);
  % disp(fbk)
  toc;
end


%% ---------- Helper Functions ----------
function [] = startup()
    % startup sets up libraries and should be started once on startup.
    currentDir = fileparts(mfilename('fullpath'));
    addpath(fullfile(currentDir , 'hebi'));
    hebi_load(); % explicitely pre-load library
end