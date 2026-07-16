% --------------------------------------------------------------------------
% knee_exoskeleton_flex
%   Simulate walking with a knee exoskeleton that provides a
%   flexion torque in function of the progression within a stride.
%   Running this simulation requires the function desired_torque_generator_modif,
%
%   References
%   [1] P. W. Franks et al., “Comparing optimized exoskeleton assistance of the hip, knee,
%   and ankle in single and multi-joint configurations,” Wearable Technologies,
%   vol. 2, Oct. 2021, doi: 10.1017/wtc.2021.14.
%   [2] J. Zhang et al., “Human-in-the-loop optimization of exoskeleton 
%   assistance during walking,” Science, vol. 356, pp. 1280–1283, Jun. 2017, 
%   doi: 10.1126/science.aal5054.
% 
%   See also desired_torque_generator_modif
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
S.subject.name = 'SUBJ28_26';

% Subject body mass (kg)
% bm = 77.9; % SUBJ02
bm = 61.3; % SUBJ28

% Walking speed
%S.misc.forward_velocity = 1.09; % SUBJ01
% S.misc.forward_velocity = 0.95; % SUBJ02
%S.misc.forward_velocity = 1.19; % SUBJ08
%S.misc.forward_velocity = 1.015; % SUBJ19
% S.misc.forward_velocity = 1.17; % SUBJ25
S.misc.forward_velocity = 1.06; % SUBJ28

% path to folder where you want to store the results of the OCP
S.misc.save_folder  = fullfile(pathExDir,'ExampleResults','KneeExo', 'SUBJ28_26', 'Flex');  

% either choose "quasi-random" or give the path to a .mot file you want to use as initial guess
%S.solver.IG_selection = fullfile(S.misc.main_path,'OCP','IK_Guess_Full_GC.mot');
S.solver.IG_selection = fullfile(S.misc.main_path,'Subjects',S.subject.name,'SUBJ28_ik_deg_GC.mot');
S.solver.IG_selection_gaitCyclePercent = 100;

% give the path to the osim model of your subject
osim_path = fullfile(pathRepo,'Subjects',S.subject.name,[S.subject.name '.osim']);


%% Add hip exoskeleton

% select orthosis function
exo1.function_name = 'kneeExoFlex';

% set path to downloaded function - CHANGE THIS
exo1.dependencies_path = 'C:\Users\jm2508\Documents\PredSim';

% set parameters of assistance profile
exo1.peak_torque = 0.279 * bm; % [Nm] mean
exo1.peak_time = 59; % [%] mean
exo1.rise_time = 21.4; % [%] mean
exo1.fall_time = 9.5;  % [%] mean

exo1.plotAssistanceProfile = true;

% add orthosis on right side
exo1.left_right = 'r';
S.orthosis.settings{1} = exo1;

% add the same orthosis on left side
exo1.left_right = 'l';
S.orthosis.settings{2} = exo1;

%% Run predictive simulations

[savename] = runPredSim(S, osim_path);

