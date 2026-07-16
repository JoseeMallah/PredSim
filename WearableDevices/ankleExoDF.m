function [exo] = ankleExoDF(init, settings_orthosis)
% --------------------------------------------------------------------------
% ankleExoDF
%   Ankle exoskeleton that applies a torque profile (torque in function of
%   stride) to the ankle. 
%
%   References
%   [1] J. Miguel-Fernandez et al., “Relationship Between Ankle Assistive
%   Torque and Biomechanical Gait Metrics in Individuals After Stroke,”
%   IEEE Robotics and Automation Letters, vol. 7, pp. 7574-7580, Jul. 2022, 
%   doi: 10.1109/LRA.2022.3183799.
%
% INPUT:
%   - init -
%   * struct with information used to initialise the Orthosis object.
% 
%   - settings_orthosis -
%   * struct with information about this orthosis, containing the fields:
%       - function_name = ankleExoDF  i.e. name of this function   
%       - dependencies_path path to dependencies
%       - isFullGaitCycle   assistance profile for full stride when true,
%       half stride when false. Default is false.
%       - peak_torque:      peak torque in Nm
%       - t1:               onset time as % of stride
%       - t2:               peak rise time as % of stride
%       - t3:               peak fall time as % of stride
%       - t4:               offset time as % of stride
%   Values are set via S.orthosis.settings{i} in main.m, with i the index
%   of the orthosis.
%
%
% OUTPUT:
%   - exo -
%   * an object of the class Orthosis
% 
% Original author: Josée Mallah
% Original date: 01/March/2025
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
exo_params(2) = settings_orthosis.t1;
exo_params(3) = settings_orthosis.t2;
exo_params(4) = settings_orthosis.t3;
exo_params(5) = settings_orthosis.t4;
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
fun = str2func('desired_df_torque');
cd(tmp);


% call function to calculate torque
T_ankle = zeros(3,N_control);
for i=1:N_control
    T_ankle(3,i) = fun(mesh_control(i)/N_stride, 1, exo_params);
end


% apply exo torque on tibia and calcn
exo.addBodyMoment(T_ankle, ['T_exo_shank_',side],['tibia_',side]);
exo.addBodyMoment(-T_ankle, ['T_exo_foot_',side],['calcn_',side],['tibia_',side]);


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
        plot((1:N_control)/N_stride*100,T_ankle(3,:),'DisplayName',legName)
        xlabel('Stride [%]')
        ylabel('Assistance [Nm]')
        title('ankleExoDf')
        legend('Location','best')

    end

end

end