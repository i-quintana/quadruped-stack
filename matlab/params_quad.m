function p = params_quad()
%PARAMS_QUAD  Single source of truth for the quadruped model (SI units).
%   Mini-Cheetah-class dimensions. Inertial values are reasonable placeholders,
%   to be validated against a Simscape model before they are used for dynamics.

p.g = 9.81;                         % m/s^2

% ---- Leg link geometry (m) ----
p.l_abad  = 0.062;                  % HAA axis -> thigh mount (lateral offset)
p.l_thigh = 0.209;                  % HFE (hip pitch) -> knee
p.l_calf  = 0.195;                  % knee -> foot

% ---- Leg identifiers & abduction sign (+1 left / -1 right, along body +y) ----
p.legs     = {'FL','FR','RL','RR'};
p.abadSign = struct('FL',+1,'FR',-1,'RL',+1,'RR',-1);

% ---- Leg product-of-exponentials description (space frame) ----
% Screw axes S = [v; w] (Corke twist ordering), columns = [HAA HFE KFE], with
% v = -w x r for any point r on the axis at the zero pose:
%   HAA: w = x, r = [0;0;0]              -> v = 0
%   HFE: w = y, r = [0; s*l_abad; 0]     -> v = 0   (r lies ALONG w)
%   KFE: w = y, r = [0; s*l_abad; -l_thigh] -> v = [l_thigh; 0; 0]
% The screws are identical for all four legs: the abduction offset is parallel
% to the HFE axis, so l_abad cancels out of every v and never appears here. The
% leg's handedness enters only through the home pose M below.
%            HAA HFE KFE
p.legScrew = [0   0   p.l_thigh;    % vx
              0   0   0;            % vy
              0   0   0;            % vz
              1   0   0;            % wx
              0   1   1;            % wy
              0   0   0];           % wz

% Home pose M: the foot frame at q = 0, one per leg (this is where s lives).
for iLeg = 1:numel(p.legs)
    sLeg = p.abadSign.(p.legs{iLeg});
    p.legHome.(p.legs{iLeg}) = ...
        [eye(3), [0; sLeg*p.l_abad; -(p.l_thigh + p.l_calf)]; 0 0 0 1];
end

% ---- Hip mount positions in body frame (m); trunk centre = origin, x fwd, y left, z up ----
lx = 0.19; ly = 0.049;
p.hip = struct( ...
    'FL',[ lx;  ly; 0], 'FR',[ lx; -ly; 0], ...
    'RL',[-lx;  ly; 0], 'RR',[-lx; -ly; 0]);

% ---- Joint limits (rad): rows = [q1 HAA; q2 HFE; q3 KFE] ----
p.qmin = [-0.90; -1.70; -2.60];
p.qmax = [ 0.90;  1.70; -0.20];
p.q_nominal = [0; 0.80; -1.60];     % nominal per-leg stance angles

% ---- Trunk inertial ----
p.trunk.mass = 9.0;                 % kg
p.trunk.com  = [0;0;0];             % m, body frame
p.trunk.I    = diag([0.07 0.26 0.24]);  % kg m^2 about CoM

% ---- Leg link inertial (placeholders) ----
p.link.abad  = struct('mass',0.54,'com',[0;0;0],            'I',diag([4e-4 4e-4 4e-4]));
p.link.thigh = struct('mass',0.63,'com',[0;0;-0.5*p.l_thigh],'I',diag([5.5e-3 5.5e-3 3e-4]));
p.link.calf  = struct('mass',0.10,'com',[0;0;-0.5*p.l_calf ],'I',diag([3.0e-3 3.0e-3 1e-4]));
end


