function x_body = quad_body_pose(p, orient)
%QUAD_BODY_POSE  Build the floating-base pose struct used across the project.
%
%   x_body = QUAD_BODY_POSE()               identity: at the origin, axes aligned
%   x_body = QUAD_BODY_POSE(p)              translated, no rotation
%   x_body = QUAD_BODY_POSE(p, R)           R is 3x3 rotation world<-body
%   x_body = QUAD_BODY_POSE(p, quat)        quat is 4x1 unit quaternion [w x y z]
%
%   The floating-base pose is the body frame's placement in the world:
%     .p  3x1  body-frame origin in world                       (m)
%     .R  3x3  rotation world<-body; columns are the body x/y/z axes in world.
%
%   Orientation input is normalized to .R here so every downstream function
%   reads one representation and never has to worry about quaternion
%   conventions. The quaternion is scalar-first Hamilton (w first), the MATLAB /
%   Robotics System Toolbox convention, so a state estimator can carry [w x y z]
%   and drop it straight in here. Velocity fields (.v, .w) follow with the
%   dynamics.
%
%   Rotations are built with explicit arithmetic (no rotx/roty/rotz), so the
%   result does not depend on which toolbox's rotx/roty/rotz is first on the
%   path, or on whether that one takes degrees or radians.

if nargin < 1 || isempty(p)
    p = [0;0;0];
end
p = p(:);
assert(numel(p) == 3, 'quad_body_pose: p must be 3x1');

if nargin < 2 || isempty(orient)
    R = eye(3);
elseif isequal(size(orient), [3 3])
    R = orient;
    assert(norm(R'*R - eye(3), 'fro') < 1e-9 && abs(det(R) - 1) < 1e-9, ...
        'quad_body_pose: R is not a proper rotation (R''R = I, det = +1)');
elseif numel(orient) == 4
    R = quat2R(orient(:));
else
    error('quad_body_pose: orient must be a 3x3 rotation or a 4x1 quaternion');
end

x_body.p = p;
x_body.R = R;
end

function R = quat2R(quat)
%QUAT2R  Unit quaternion [w x y z] (Hamilton, world<-body) -> rotation matrix.
q = quat / norm(quat);
w = q(1); x = q(2); y = q(3); z = q(4);
R = [1-2*(y^2+z^2),   2*(x*y - w*z),   2*(x*z + w*y);
     2*(x*y + w*z),   1-2*(x^2+z^2),   2*(y*z - w*x);
     2*(x*z - w*y),   2*(y*z + w*x),   1-2*(x^2+y^2)];
end
