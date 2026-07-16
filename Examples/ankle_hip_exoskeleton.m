% --------------------------------------------------------------------------
% ankle_hip_exoskeleton
%   Simulate walking with a hip and ankle exoskeleton that provides hip
%   extension and flexion and ankle plantarflexion torque in function of the progression within a stride.
%   Running this simulation requires the functions
%   desired_torque_generator, desired_torque_generator_modif, desired_torque_generator_hip_ext
%
%   References
%   [1] P. W. Franks et al., “Comparing optimized exoskeleton assistance of the hip, knee,
%   and ankle in single and multi-joint configurations,” Wearable Technologies,
%   vol. 2, Oct. 2021, doi: 10.1017/wtc.2021.14.
%   [2] J. Zhang et al., “Human-in-the-loop optimization of exoskeleton 
%   assistance during walking,” Science, vol. 356, pp. 1280–1283, Jun. 2017, 
%   doi: 10.1126/science.aal5054.
% 
%   See also desired_torque_generator_modif, desired_torue_generator_hip_ext
%
% Original author: Josée Mallah
% Original date: 17/March/2026
% --------------------------------------------------------------------------

clear
close all
clc
addpath('\\ifs.eng.cam.ac.uk\users\jm2508\casadi-3.6.7-windows64-matlab2018b')

[pathExDir,~,~] = fileparts(mfilename('fullpath'));
[pathRepo,~,~] = fileparts(pathExDir);
[pathRepoFolder,~,~] = fileparts(pathRepo);

addpath(fullfile(pathRepo,'DefaultSettings'))
addpath(pathRepo)

%% Initialize S

[S] = initializeSettings('DHondt_et_al_2024_3seg');

%% Settings

% name of the subject
%S.subject.name = 'DHondt_et_al_2024_3seg';
S.subject.name = 'SUBJ01_26';

% Subject body mass (kg)
bm = 63.6; % SUBJ01
% bm = 77.9; % SUBJ02
% bm = 50.1; % SUBJ05
% bm = 72.1; % SUBJ22
% bm = 61.3; % SUBJ28

% Walking speed
S.misc.forward_velocity = 1.09; % SUBJ01
% S.misc.forward_velocity = 0.95; % SUBJ02
% S.misc.forward_velocity = 0.88; % SUBJ05
%S.misc.forward_velocity = 1.19; % SUBJ08
%S.misc.forward_velocity = 1.015; % SUBJ19
% S.misc.forward_velocity = 1.04; % SUBJ22
% S.misc.forward_velocity = 1.17; % SUBJ25
% S.misc.forward_velocity = 1.06; % SUBJ28

% path to folder where you want to store the results of the OCP
S.misc.save_folder  = fullfile(pathExDir,'ExampleResults','AnkleHipExo', 'SUBJ01_26', 'PFExtFlex');  

% either choose "quasi-random" or give the path to a .mot file you want to use as initial guess
%S.solver.IG_selection = fullfile(S.misc.main_path,'OCP','IK_Guess_Full_GC.mot');
S.solver.IG_selection = fullfile(S.misc.main_path,'Subjects',S.subject.name,'SUBJ01_ik_deg_GC.mot');
S.solver.IG_selection_gaitCyclePercent = 100;

% give the path to the osim model of your subject
osim_path = fullfile(pathRepo,'Subjects',S.subject.name,[S.subject.name '.osim']);


%% Add hip exoskeleton

% select orthosis function
exo1.function_name = 'ankleHipFranks2021';

% set path to downloaded function - CHANGE THIS
exo1.dependencies_path1 = '\\ifs.eng.cam.ac.uk\users\jm2508\Downloads\aal5054_zhang_sm_data_s2';
exo1.dependencies_path2 = 'C:\Users\jm2508\Documents\PredSim';

% set parameters of assistance profile
exo1.peak_torque_pf = 0.764 * bm; % [Nm] mean
exo1.peak_time_pf = 54.6; % [%] mean
exo1.rise_time_pf = 27.8; % [%] mean
exo1.fall_time_pf = 16.5;  % [%] mean
exo1.peak_torque_ext = 0.415 * bm; % [Nm] mean
exo1.peak_time_ext = 28.4; % [%] mean
exo1.rise_time_ext = 18.8; % [%] mean
exo1.fall_time_ext = 30.5;  % [%] mean
exo1.peak_torque_flex = 0.314 * bm; % [Nm] mean
exo1.peak_time_flex = 83.9; % [%] mean
exo1.rise_time_flex = 31.8; % [%] mean
exo1.fall_time_flex = 23.1;  % [%] mean

exo1.plotAssistanceProfile = true;

% add orthosis on right side
exo1.left_right = 'r';
S.orthosis.settings{1} = exo1;

% add the same orthosis on left side
exo1.left_right = 'l';
S.orthosis.settings{2} = exo1;

%% Initial-contact constraint settings

% Number of complete mesh intervals constrained at the start
S.initialContact.nStartMeshes = 3;

% Number of complete mesh intervals constrained at the end
S.initialContact.nEndMeshes = 3;

% Minimum right vertical GRF during the first interval [N]
S.initialContact.minStartGRFy = 20;

% Maximum right vertical GRF during the last interval [N]
S.initialContact.maxEndGRFy = 10;

%% Run predictive simulations

[savename] = runPredSim(S, osim_path);

