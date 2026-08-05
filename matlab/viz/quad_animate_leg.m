function quad_animate_leg(q, params, leg, ax)
%QUAD_ANIMATE_LEG  Draw one leg (HAA -> thigh -> calf -> foot) in the hip frame.
%   quad_animate_leg(q, params, leg[, ax])
%
%   Computes the joint positions directly from the kinematic chain, independently
%   of leg_fk, so the plot checks the derivation instead of re-plotting it.
%
%   Example:
%     p = params_quad(); figure; quad_animate_leg([0;0.8;-1.6], p, 'FL');

if nargin < 3 || isempty(leg), leg = 'FL'; end
if nargin < 4 || isempty(ax),  ax  = gca;  end

s  = params.abadSign.(leg);
Rx = @(a)[1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];
Ry = @(a)[cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)];
q1 = q(1); q2 = q(2); q3 = q(3);

p_haa  = [0;0;0];
p_hfe  = Rx(q1)*[0; s*params.l_abad; 0];
p_knee = p_hfe  + Rx(q1)*Ry(q2)*[0;0;-params.l_thigh];
p_foot = p_knee + Rx(q1)*Ry(q2)*Ry(q3)*[0;0;-params.l_calf];
P = [p_haa p_hfe p_knee p_foot];

cla(ax);
plot3(ax, P(1,:), P(2,:), P(3,:), '-o', 'LineWidth', 2, 'MarkerFaceColor', 'k');
hold(ax, 'on');
plot3(ax, p_foot(1), p_foot(2), p_foot(3), 'r.', 'MarkerSize', 22);
grid(ax, 'on'); axis(ax, 'equal');
xlabel(ax, 'x [m]'); ylabel(ax, 'y [m]'); zlabel(ax, 'z [m]');
title(ax, sprintf('Leg %s   q = [%.2f  %.2f  %.2f]', leg, q1, q2, q3));
view(ax, 135, 20);
end
