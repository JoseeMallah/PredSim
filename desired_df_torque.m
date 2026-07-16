% function torque = desired_df_torque(time_within_current_stride, stride_period, HLCParams)
%     % Extract control parameters
%     peakTorque = HLCParams(1);
%     t1 = HLCParams(2);
%     t2 = HLCParams(3);
%     t3 = HLCParams(4);
%     t4 = HLCParams(5);
% 
%     % Convert percentages to absolute stride time
%     time_points = [0, t1, t2, t3, t4, 100] / 100 * stride_period;
%     value_points = [0, 0, peakTorque, peakTorque, 0, 0];
% 
%     % Compute torque using piecewise cubic spline (pchip)
%     torque = pchip(time_points, value_points, time_within_current_stride);
% 
%     % Apply the shift
%     torque = torque - peakTorque;
% end

function torque = desired_df_torque(time_within_current_stride, stride_period, HLCParams)
    % Extract control parameters
    peakTorque = HLCParams(1);
    t1 = HLCParams(2);
    t2 = HLCParams(3);
    t3 = HLCParams(4);
    t4 = HLCParams(5);

    % Convert percentages to absolute stride time
    time_points = [0, t1, t2, t3, t4, 100] / 100 * stride_period;
    value_points = [0, 0, -peakTorque, -peakTorque, 0, 0];

    % Compute torque using piecewise cubic spline (pchip)
    torque = pchip(time_points, value_points, time_within_current_stride);
end
