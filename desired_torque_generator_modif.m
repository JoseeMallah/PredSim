%% Desired Torque Curve Generation from Control Parameters

% INPUTS
% time_within_current_stride: Time elasped in the current the stride.
% stride_period: The total stride time estimated up to the previous
% stride. 
% HLCParams: The high level control parameters used to determine the
% assistance conditionm,i.e., [peak torque, peak time, rise time, drop time
% in [N m, % stride period, % stride period, % stride period]. 

%OUTPUTS
% torque: Desired torque at time_within_current_stride within the current
% stride.


% This function generates a desired torque curve which passes through [0 %
% stride, 0 N m], [(peakTime-riseTime)% 2 N m], [peakTime %, peak Torque
% Nm], [(peakTime + fallTime)% 2 N m] and [65% stride, 0 Nm] sequencially.
% Between [(peakTime-riseTime)% 2 N m] and [(peakTime + fallTime)% 2 N m],
% the curve is made of 2 cubic splines, which has zero first derivatives at
% [(peakTime-riseTime)% 2 N m] and [peakTime %, peak Torque % Nm] and zero
% second derivative at [(peakTime + fallTime)% 2 N m]. In other sectors,
% the curve is made of straight lines. 


% Copyright@Juanjuan Zhang, Steven H Collins 11/30/2016



function torque = desired_torque_generator(time_within_current_stride, stride_period, HLCParams)

prev_stride_time = stride_period;

peakTorque = HLCParams(1);
peakTime = HLCParams(2);
riseTime = HLCParams(3);
fallTime = HLCParams(4);



t_ankor = [peakTime-riseTime peakTime peakTime+fallTime]/100*stride_period;

tau_ankor = [0 peakTorque-2 0];

if (time_within_current_stride < t_ankor(1) || time_within_current_stride>t_ankor(end))
    torque = 0;
else
    torque  = mycubicspline(t_ankor, tau_ankor, 2, time_within_current_stride);
end;

tt2 =peakTime+fallTime;
if time_within_current_stride>(prev_stride_time*tt2/100)
    torque = (prev_stride_time*0.65-time_within_current_stride)/(stride_period*(0.65-tt2/100))*2;
end;
if time_within_current_stride > t_ankor(end)
    torque = 0;
end

function u = mycubicspline(x, y, ip, t)
% x and y are the coordinates of the ankor nodes
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

