%% Desired Torque Curve Generation from Control Parameters

% INPUTS
% Continuous profile spanning 84% previous stride -> 40% current stride.
% peakTime: % of stride from 84% previous stride (-16%) to peak
% riseTime:   % of stride before peak when torque starts rising
% peakTorque:    peak torque amplitude
% fallTime:    end time in CURRENT stride percent (typically ~40)

%OUTPUTS
% torque: Desired torque at time_within_current_stride within the current
% stride.


function torque = desired_torque_generator_hip_extension(time_within_current_stride, stride_period, HLCParams)
% HLCParams = [peakOffsetPct, riseTimePct, peakTorque, fallEndPct]
%   peakOffsetPct: % from 84% prev (-16%) to peak
%   riseTimePct:   % before peak when rise starts
%   peakTorque:    peak amplitude
%   fallEndPct:    end in current stride percent (e.g. 40)

    peakOffsetPct = HLCParams(2);   % <-- careful: depends on your chosen order
    riseTimePct   = HLCParams(3);
    peakTorque    = HLCParams(1);
    fallEndPct    = HLCParams(4);

    start_pct = -16.0;                 % 84% previous stride

    % Map time -> wrapped phase percent
    pct = (time_within_current_stride / stride_period) * 100.0;
    if pct >= 84.0
        phase_pct = pct - 100.0;       % -16..0
    else
        phase_pct = pct;               % 0..84
    end

    % Anchors in percent coordinates
    peak_pct       = start_pct + peakOffsetPct;
    rise_start_pct = peak_pct - riseTimePct;
    fall_end_pct   = fallEndPct;

    % Clamp anchors
    rise_start_pct = max(rise_start_pct, start_pct);
    peak_pct       = min(max(peak_pct, rise_start_pct + 1e-6), fall_end_pct - 1e-6);

    % Window check in phase space
    if (phase_pct < rise_start_pct) || (phase_pct > fall_end_pct)
        torque = 0.0;
        return;
    end

    % Convert % -> seconds (can be negative)
    t_anchor  = [rise_start_pct, peak_pct, fall_end_pct] / 100.0 * stride_period;
    tau_anchor = [0.0, peakTorque, 0.0];
    t_eval = (phase_pct / 100.0) * stride_period;

    torque = mycubicspline(t_anchor, tau_anchor, 2, t_eval);

function u = mycubicspline(x, y, ip, t)
% x and y are the coordinates of the anchor nodes
% ip is the index of peak/valley point
n=length(x);

h = zeros(1,n-1);
delta = zeros(1,n-1);
for i=1:n-1,
    
    h(i)=x(i+1)-x(i); delta(i)= (y(i+1)-y(i))/h(i);
    
end

% Construct the Tridiagonal System

hp=h(2:n-1); hm=h(1:n-2);
deltap=delta(2:n-1); deltam=delta(1:n-2);
U=[1,hm]; L=[hp,1]; D=2*[1,hp+hm,1];
B=3*[delta(1), hp.*deltam+hm.*deltap,delta(n-1)];

dy = my_tridiag(L,D,U,B,ip);

% this is just the Cubic Hermite Graphing Code


if t>=x(n)
    u = 0;
elseif t<=x(1)
    u = 0;
else
    k = 1;
    for i=1:n-1,
        if t>=x(i) && t<x(i+1)
            k = i;
        end;
    end;
    
    
    a=delta(k);
    b=(a-dy(k))/h(k); c=(dy(k+1)-a)/h(k); d=(c-b)/h(k);
    u=y(k)+(t-x(k)).*(dy(k)+(t-x(k)).*(b+(t-x(k+1)).*d));
    
end
return;

function X = my_tridiag(L,D,U,B,ip)
% X = my_tridiag(L,D,U,B,ip)
% Solution of a tridiagonal linear system (LDU)X = B.
% It is assumed that D and B have dimension n,
% and that L and U have dimension n-1;
% L sub diagonal, input.
% D diagonal vector, input.
% U super diagonal, input.
% B right hand side vector, input.
% ip index of the peak point
% X solution vector, output.

n = length(B);

UU = [[zeros(n-1, 1) diag(U)]; zeros(1, n)];
LL = [zeros(1, n); [diag(L) zeros(n-1, 1)]];
DD = diag(D);
DD = UU+LL+DD;
B(ip) = 0;
DD(ip, :)  = zeros(size(DD(ip,:)));
DD(ip, ip) = 1;
B(1) = 0;
DD(1, :)  = zeros(size(DD(ip,:)));
DD(1, 1) = 1;


X= DD\B';
return;

