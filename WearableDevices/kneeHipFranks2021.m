function [exo] = kneeHipFranks2021(init, settings_orthosis)
% --------------------------------------------------------------------------
% kneeHipFranks2021
%   Knee (flexion) and hip (extension/flexion) exoskeleton that applies
%   a torque profile (torque in function of stride) to the knee and hip. 
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
%       - function_name = hipExoExtFlex  i.e. name of this function   
%       - dependencies_path path to dependencies
%       - isFullGaitCycle   assistance profile for full stride when true,
%       half stride when false. Default is false.
%       - peak_torque_kf:      peak knee flexion torque in Nm
%       - peak_time_kf:        timing of peak as % of stride
%       - rise_time_kf:        rise time as % of stride
%       - fall_time_kf:        fall time as % of stride
%       - peak_torque_ext:      peak hip extension torque in Nm
%       - peak_time_ext:        peak offset time from 84% prev (-16%) to peak
%       - rise_time_ext:        rise time as % of stride
%       - fall_time_ext:        fall time as % of stride
%       - peak_torque_flex:      peak hip flexion torque in Nm
%       - peak_time_flex:        timing of peak as % of stride
%       - rise_time_flex:        rise time as % of stride
%       - fall_time_flex:        fall time as % of stride
%   Values are set via S.orthosis.settings{i} in main.m, with i the index
%   of the orthosis.
%
%
% OUTPUT:
%   - exo -
%   * an object of the class Orthosis
% 
% Original author: Josée Mallah
% Original date: 17/March/2026
% --------------------------------------------------------------------------

% create Orthosis object
exo = Orthosis('exo',init,true);


% read settings that were passed from main.m
if isfield(settings_orthosis,'isFullGaitCycle')
    isFullGaitCycle = settings_orthosis.isFullGaitCycle;
else
    isFullGaitCycle = false;
end
exo_params_kf(1) = settings_orthosis.peak_torque_kf;
exo_params_kf(2) = settings_orthosis.peak_time_kf;
exo_params_kf(3) = settings_orthosis.rise_time_kf;
exo_params_kf(4) = settings_orthosis.fall_time_kf;
exo_params_ext(1) = settings_orthosis.peak_torque_ext;
exo_params_ext(2) = settings_orthosis.peak_time_ext;
exo_params_ext(3) = settings_orthosis.rise_time_ext;
exo_params_ext(4) = settings_orthosis.fall_time_ext;
exo_params_flex(1) = settings_orthosis.peak_torque_flex;
exo_params_flex(2) = settings_orthosis.peak_time_flex - 16;
exo_params_flex(3) = settings_orthosis.rise_time_flex - 16;
exo_params_flex(4) = settings_orthosis.fall_time_flex - 16;
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
T_knee = zeros(3,N_control);
for i=1:N_control
    T_knee(3,i) = fun(mesh_control(i)/N_stride, 1, exo_params_kf);
end

% apply exo torque on femur and tibia
exo.addBodyMoment(T_knee, ['T_exo_thigh_',side],['femur_',side]);
exo.addBodyMoment(-T_knee, ['T_exo_shank_',side],['tibia_',side],['femur_',side]);


% load function to calculate desired Ext torque
tmp = pwd;
cd(settings_orthosis.dependencies_path);
fun = str2func('desired_torque_generator_hip_extension');
cd(tmp);


% call function to calculate torque
T_hip_ext = zeros(3,N_control);
for i=1:N_control
    T_hip_ext(3,i) = fun(mesh_control(i)/N_stride, 1, exo_params_ext);
end

% load function to calculate desired Flex torque
tmp = pwd;
cd(settings_orthosis.dependencies_path);
fun = str2func('desired_torque_generator_modif');
cd(tmp);


% call function to calculate torque
T_hip_flex = zeros(3,N_control);
for i=1:N_control
    T_hip_flex(3,i) = fun(mesh_control(i)/N_stride, 1, exo_params_flex);
end

% add Ext and Flex torques
T_hip = T_hip_ext - T_hip_flex;

% apply exo torque on pelvis and femur
exo.addBodyMoment(T_hip, 'T_exo_pelvis','pelvis');
exo.addBodyMoment(-T_hip, ['T_exo_thigh_',side],['femur_',side],'pelvis');


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
        title('hipExoExtFlex')
        legend('Location','best')
        figure
        plot((1:N_control)/N_stride*100,T_knee(3,:),'DisplayName',legName)
        xlabel('Stride [%]')
        ylabel('Assistance [Nm]')
        title('kneeExoFlex')
        legend('Location','best')

    end

end

end