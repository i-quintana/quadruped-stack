function quad_animate(x_body, q, params, ax)
%QUAD_ANIMATE  Draw the whole quadruped (body + four legs) in the world frame.
%   quad_animate(x_body, q, params[, ax])
%
%   INPUTS
%     x_body  floating-base pose struct from quad_body_pose (.p, .R)
%     q       3x4 joint angles, one column per leg, ordered as params.legs
%     params  struct from params_quad()
%     ax      target axes (optional; defaults to gca)
%
%   Like quad_animate_leg, every joint position is built straight from the
%   kinematic chain and then mapped world <- body by x_body, independently of
%   the kinematics functions under test — so it is a visual cross-check, not a
%   re-plot of the same computation. Trunk is drawn as the quad through the four
%   hip mounts, with a body triad showing the orientation; feet are the red dots
%   on the ground.
%
%   Example (a body pitched 15 deg, standing on nominal legs):
%     p  = params_quad();
%     xb = quad_body_pose([0;0;0.30], [cos(pi/24);0;sin(pi/24);0]);
%     figure; quad_animate(xb, repmat(p.q_nominal,1,4), p);

if nargin < 4 || isempty(ax), ax = gca; end

Rx = @(a)[1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];
Ry = @(a)[cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)];
toWorld = @(pb) x_body.p + x_body.R * pb;          % body point -> world

cla(ax); hold(ax, 'on');

% ---- trunk: quad through the hip mounts (FL-FR-RR-RL-FL) + body triad ----
order = {'FL','FR','RR','RL','FL'};
hips  = zeros(3, numel(order));
for k = 1:numel(order), hips(:,k) = toWorld(params.hip.(order{k})); end
plot3(ax, hips(1,:), hips(2,:), hips(3,:), 'k-', 'LineWidth', 2);

o = toWorld([0;0;0]); L = 0.08;
ax_c = 'rgb';
for a = 1:3
    e = toWorld(L * (a==(1:3))');
    plot3(ax, [o(1) e(1)], [o(2) e(2)], [o(3) e(3)], ax_c(a), 'LineWidth', 1.5);
end

% ---- legs: HAA -> HFE -> knee -> foot, per leg ----
feet = zeros(3, numel(params.legs));
for i = 1:numel(params.legs)
    leg = params.legs{i};
    s   = params.abadSign.(leg);
    hip = params.hip.(leg);
    q1 = q(1,i); q2 = q(2,i); q3 = q(3,i);

    p_haa  = hip;
    p_hfe  = p_haa  + Rx(q1)*[0; s*params.l_abad; 0];
    p_knee = p_hfe  + Rx(q1)*Ry(q2)*[0;0;-params.l_thigh];
    p_foot = p_knee + Rx(q1)*Ry(q2)*Ry(q3)*[0;0;-params.l_calf];
    P = toWorld([p_haa p_hfe p_knee p_foot]);
    feet(:,i) = P(:,end);

    plot3(ax, P(1,:), P(2,:), P(3,:), '-o', 'LineWidth', 2, ...
        'MarkerSize', 4, 'MarkerFaceColor', 'k', 'Color', [0.15 0.15 0.15]);
end
plot3(ax, feet(1,:), feet(2,:), feet(3,:), 'r.', 'MarkerSize', 22);

% ---- ground plane at z = 0 + framing ----
allP = [hips, feet, o];
c = mean(allP(1:2,:), 2);  r = 0.45;
patch(ax, 'XData', c(1)+r*[-1 1 1 -1], 'YData', c(2)+r*[-1 -1 1 1], ...
      'ZData', [0 0 0 0], 'FaceColor', [0.9 0.9 0.9], 'FaceAlpha', 0.4, ...
      'EdgeColor', 'none');

grid(ax, 'on'); axis(ax, 'equal'); hold(ax, 'off');
xlabel(ax, 'x [m]'); ylabel(ax, 'y [m]'); zlabel(ax, 'z [m]');
title(ax, 'Quadruped (world frame)');
view(ax, 135, 20);
end
