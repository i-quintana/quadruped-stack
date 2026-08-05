function [q, info] = leg_ik_rst(p_foot, params, leg, q_ref)
%LEG_IK_RST  Inverse kinematics of one leg via Robotics System Toolbox.
%
%   q         = LEG_IK_RST(p_foot, params, leg)
%   [q, info] = LEG_IK_RST(p_foot, params, leg, q_ref)
%
%   Toolbox counterpart of LEG_IK. Numerical (damped least squares) rather than
%   closed form: it iterates from q_ref, so the seed sets both the branch it
%   converges to and how long it takes. Weights [0 0 0 1 1 1] ask it to match
%   position only and ignore foot orientation, which is what a point foot wants.
%
%   INPUTS  as LEG_IK; q_ref defaults to params.q_nominal.
%
%   OUTPUTS
%     q     3x1 solution
%     info  .reachable  true if the solver met its tolerance
%           .poseError  residual position error (m)
%           .status     solver status string
%           .inLimits   whether q lies inside [params.qmin, params.qmax]
%
%   WHEN TO PREFER LEG_IK: for a 3-DOF leg the closed form is exact, branch-
%   controllable and roughly two orders of magnitude faster — which matters at
%   4 legs x 1 kHz. This version earns its keep as an independent check, and as
%   the pattern that still works when a chain has no closed form.
%
%   LIMITS: inverseKinematics does NOT enforce joint limits in R2022b (there is
%   no EnforceJointLimits property in this release), so a solution can land
%   outside them — hence info.inLimits. Use generalizedInverseKinematics with a
%   constraintJointBounds if you need them enforced.
%
%   See also LEG_IK, LEG_RBT, INVERSEKINEMATICS.

persistent solver solverKey

if nargin < 4 || isempty(q_ref)
    q_ref = params.q_nominal;
end

key = sprintf('%s|%.17g|%.17g|%.17g', leg, params.l_abad, params.l_thigh, params.l_calf);
if isempty(solverKey) || ~strcmp(solverKey, key)
    solver    = inverseKinematics('RigidBodyTree', leg_rbt(params, leg));
    solverKey = key;
end

[q, sol] = solver('foot', trvec2tform(p_foot(:)'), [0 0 0 1 1 1], q_ref(:));

info.reachable = strcmp(sol.Status, 'success');
info.poseError = sol.PoseErrorNorm;
info.status    = sol.Status;
info.inLimits  = all(q >= params.qmin - 1e-12) && all(q <= params.qmax + 1e-12);
end
