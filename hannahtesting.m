%% ---------- SETUP ----------
startup()

% % Set up the arm as a group
% family = '3dof';
% names = {'Base1', 'Base2', 'Elbow'};
% group = HebiLookup.newGroupFromNames(family, names);

% If not connected to any physical modules
group = HebiUtils.newImitationGroup(3)

%% ---------- KINEMATICS ----------
kin = HebiKinematics('3dofDescription');


%% ---------- MOVEMENT ----------
numModules = group.getNumModules;

cmd = CommandStruct();
cmd.position = zeros(1,numModules);
cmd.velocity = zeros(1,numModules);
cmd.effort = [];
tic;

% send position/velocity command for 10 seconds
while toc < 10
  fbk = group.getNextFeedback();  % Use getNextFeedback() to                               % limit loop rate.
  group.send(cmd);
end


%% ---------- Helper Functions ----------
function [] = startup()
    % startup sets up libraries and should be started once on startup.
    currentDir = fileparts(mfilename('fullpath'));
    addpath(fullfile(currentDir , 'hebi'));
    hebi_load(); % explicitely pre-load library
end