function [exo] = hipExoFlex(init, settings_orthosis)
% --------------------------------------------------------------------------
% hipExoFlex
%   Hip flexion exoskeleton that applies a torque profile (torque in function of
%   stride) to the hip. 
%
%   References
%   [1] P. W. Franks et al., “Comparing optimized exoskeleton assistance of the hip, knee,
%   and ankle in single and multi-joint configurations,” Wearable Technologies,
%   vol. 2, Oct. 2021, doi: 10.1017/wtc.2021.14.
%   [2] J. Zhang et al., “Human-in-the-loop optimization of exoskeleton 
%   assistance during walking,” Science, vol. 356, pp. 1280–1283, Jun. 2017, 
%   doi: 10.1126/science.aal5054.
%
% INPUT:
%   - init -
%   * struct with information used to initialise the Orthosis object.
% 
%   - settings_orthosis -
%   * struct with information about this orthosis, containing the fields:
%       - function_name = hipExoFlex  i.e. name of this function   
%       - dependencies_path path to dependencies
%       - isFullGaitCycle   assistance profile for full stride when true,
%       half stride when false. Default is false.
%       - peak_torque:      peak torque in Nm
%       - peak_time:        timing of peak as % of stride
%       - rise_time:        rise time as % of stride
%       - fall_time:        fall time as % of stride
%   Values are set via S.orthosis.settings{i} in main.m, with i the index
%   of the orthosis.
%
%
% OUTPUT:
%   - exo -
%   * an object of the class Orthosis
% 
% Original author: Josée Mallah
% Original date: 16/March/2026
% --------------------------------------------------------------------------

% create Orthosis object
exo = Orthosis('exo',init,true);


% read settings that were passed from main.m
if isfield(settings_orthosis,'isFullGaitCycle')
    isFullGaitCycle = settings_orthosis.isFullGaitCycle;
else
    isFullGaitCycle = false;
end
exo_params(1) = settings_orthosis.peak_torque;
exo_params(2) = settings_orthosis.peak_time;
exo_params(3) = settings_orthosis.rise_time;
exo_params(4) = settings_orthosis.fall_time;
side = settings_orthosis.left_right; % 'l' for left or 'r' for right

% number of control intervals for simulation
N_control = exo.getNmesh(); 
% number of control intervals for full stride
if isFullGaitCycle
    N_stride = N_control; 
else
    N_stride = N_control*2;
end

% mesh points for control
mesh_control = (1:N_control);
% if left side, shift mesh by half a stride
if strcmp(side,'l')
    mesh_control = mesh_control + N_stride/2;
    mesh_control = mod(mesh_control-1,N_stride)+1;
end


% load function to calculate desired torque
tmp = pwd;
cd(settings_orthosis.dependencies_path);
fun = str2func('desired_torque_generator_modif');
cd(tmp);


% call function to calculate torque
T_hip = zeros(3,N_control);
for i=1:N_control
    T_hip(3,i) = fun(mesh_control(i)/N_stride, 1, exo_params);
end


% apply exo torque on pelvis and femur
exo.addBodyMoment(-T_hip, 'T_exo_pelvis','pelvis');
exo.addBodyMoment(T_hip, ['T_exo_thigh_',side],['femur_',side],'pelvis');


% plot figure if wanted
if isfield(settings_orthosis,'plotAssistanceProfile')
    if isa(settings_orthosis.plotAssistanceProfile,'matlab.ui.Figure')
        figure(settings_orthosis.plotAssistanceProfile)
        plotAssistanceProfile = true;
    elseif settings_orthosis.plotAssistanceProfile
        figure();
        plotAssistanceProfile = true;
    else
        plotAssistanceProfile = false;
    end

    if plotAssistanceProfile
        if strcmp(side,'l')
            legName = 'left';
        else
            legName = 'right';
        end
        hold on
        plot((1:N_control)/N_stride*100,T_hip(3,:),'DisplayName',legName)
        xlabel('Stride [%]')
        ylabel('Assistance [Nm]')
        title('hipExoFlex')
        legend('Location','best')

    end

end

end