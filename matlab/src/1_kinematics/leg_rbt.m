function rbt = leg_rbt(params, leg)
%LEG_RBT  rigidBodyTree model of one 3-DOF leg (Robotics System Toolbox).
%
%   rbt = LEG_RBT(params, leg)
%
%   Shared model behind LEG_FK_RST / LEG_IK_RST / LEG_JACOBIAN_RST. Same chain
%   as LEG_FK, expressed as bodies and joints instead of screws:
%
%     base  --[HAA: revolute about x, at the origin      ]--> abad
%           --[HFE: revolute about y, at [0; s*l_abad; 0]]--> thigh
%           --[KFE: revolute about y, at [0; 0; -l_thigh]]--> calf
%           --[fixed,                  at [0; 0; -l_calf]]--> foot
%
%   Each joint's fixed transform is the offset from its PARENT body frame, so
%   the offsets are exactly the link vectors of the transform chain. The base
%   frame IS the hip frame, so getTransform(rbt, q, 'foot') lands in the same
%   frame LEG_FK returns.
%
%   This is the object the rest of the toolchain wants: importrobot() builds one
%   from URDF, Simscape Multibody and the RST dynamics/contact functions consume
%   one, and geometricJacobian() generalises to the floating-base contact
%   Jacobians used for force mapping.
%
%   Cached per leg, rebuilt if the link lengths change. rigidBodyTree is a
%   handle class — treat the returned model as read-only.
%
%   See also LEG_FK_RST, LEG_IK_RST, LEG_JACOBIAN_RST, IMPORTROBOT.

persistent cache cacheKey

key = sprintf('%.17g|%.17g|%.17g', params.l_abad, params.l_thigh, params.l_calf);
if isempty(cacheKey) || ~strcmp(cacheKey, key)
    cache    = struct();            % geometry changed — drop every cached leg
    cacheKey = key;
end
if isfield(cache, leg)
    rbt = cache.(leg);
    return
end

s   = params.abadSign.(leg);
rbt = rigidBodyTree('DataFormat', 'column');

abad  = rigidBody('abad');
abad.Joint = revolute('HAA', [1 0 0], [0 0 0], 1);
addBody(rbt, abad, 'base');

thigh = rigidBody('thigh');
thigh.Joint = revolute('HFE', [0 1 0], [0 s*params.l_abad 0], 2);
addBody(rbt, thigh, 'abad');

calf  = rigidBody('calf');
calf.Joint = revolute('KFE', [0 1 0], [0 0 -params.l_thigh], 3);
addBody(rbt, calf, 'thigh');

foot  = rigidBody('foot');
foot.Joint = rigidBodyJoint('foot_fixed', 'fixed');
setFixedTransform(foot.Joint, trvec2tform([0 0 -params.l_calf]));
addBody(rbt, foot, 'calf');

cache.(leg) = rbt;

    function jnt = revolute(name, axis, offset, idx)
        jnt = rigidBodyJoint(name, 'revolute');
        jnt.JointAxis      = axis;
        % HomePosition first: its default of 0 is outside the KFE limits, and
        % narrowing the limits around an illegal home warns and recentres it.
        jnt.HomePosition   = params.q_nominal(idx);
        jnt.PositionLimits = [params.qmin(idx), params.qmax(idx)];
        setFixedTransform(jnt, trvec2tform(offset));
    end
end
