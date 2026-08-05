function [q, info] = leg_ik(p_foot, params, leg, q_ref)
%LEG_IK  Inverse kinematics of one 3-DOF leg: foot position -> joint angles.
%
%   q         = LEG_IK(p_foot, params, leg)
%   [q, info] = LEG_IK(p_foot, params, leg, q_ref)
%
%   Closed-form inverse of LEG_FK: abduction from the (y,z) projection, then the
%   planar 2-link problem in the (x,R) plane via the law of cosines.
%
%   INPUTS
%     p_foot 3x1 foot position in the hip frame (same frame as LEG_FK output)
%     params struct from params_quad()
%     leg    one of 'FL','FR','RL','RR'
%     q_ref  3x1 seed, used ONLY to choose between the two abduction branches.
%            Defaults to params.q_nominal; in a control loop pass the previous
%            solution so the branch stays continuous.
%
%   OUTPUTS
%     q      3x1 [q1; q2; q3]; knee branch fixed to q3 < 0 (bending backward)
%     info   .reachable  false if p_foot is outside the workspace, in which case
%                        q is the nearest reachable pose (clamped, not an error)
%            .branch     'down' if the foot sits below the hip in the leg plane
%                        (z' <= 0), 'up' for the mirrored solution
%            .inLimits   whether q lies inside [params.qmin, params.qmax]
%
%   BRANCH FIDELITY
%     Even with the knee branch fixed, p_foot has TWO solutions. The leg-plane
%     coordinate z' only satisfies |z'| = R = sqrt(y^2 + z^2 - l_abad^2), and
%     both z' = -R (foot below the hip) and z' = +R (foot above it) reproduce
%     p_foot exactly, their q1 differing by 2*atan2(z', s*l_abad). The textbook
%     form takes R >= 0 and so always returns 'down' — which silently mirrors
%     the pose whenever the true one was 'up', and typically throws q1 outside
%     its joint limit. Both branches are legal on this robot: about 22% of the
%     joint box puts the foot above the hip (folded leg, swing, crouch). The
%     branch therefore cannot be recovered from p_foot alone, so we return the
%     in-limits branch nearest q_ref.
%
%   REACHABILITY
%     Unreachable targets are clamped, never thrown — a 1 kHz controller must
%     degrade rather than error. Two ways to miss: p_foot inside the cylinder of
%     radius l_abad about the HAA axis (the abduction circle has no solution),
%     or outside the 2-link annulus |l_thigh-l_calf| <= sqrt(x^2+R^2) <=
%     l_thigh+l_calf. Both set info.reachable = false.
%
%   See also LEG_FK, LEG_JACOBIAN, LEG_IK_RST.

if nargin < 4 || isempty(q_ref)
    q_ref = params.q_nominal;
end
q_ref = q_ref(:);

s  = params.abadSign.(leg);
la = params.l_abad;   lt = params.l_thigh;   lc = params.l_calf;
x  = p_foot(1);       y  = p_foot(2);        z  = p_foot(3);

%(y, z) plane:
%R is what i see from the thigh and calf links from the front. It is |z'|, the
%signed leg-plane coordinate z' being recovered per branch below.
d2 = y^2 + z^2;
R  = sqrt(max(d2 - la^2, 0));                 % max(): clamp inside the cylinder

%(x, R) plane:

%Law of cosines:
L2 = x^2 + R^2;                               % squared foot distance in the leg plane
c3 = (L2 - lt^2 - lc^2) / (2*lt*lc);
q3 = -acos(min(1, max(-1, c3)));              % negative branch: knee bends backward

%Both abduction branches, z' = -R (foot below the hip) and z' = +R (above it).
psi   = atan2(lc*sin(q3), lt + lc*cos(q3));
qDown = branchAt(-R);
qUp   = branchAt(+R);

%Keep the branch nearest the seed, with anything outside the joint limits
%pushed behind everything inside them. Ties fall to 'down', the stance branch.
OUT = 1e3;                                    % > any in-box joint distance
if norm(qUp - q_ref) + OUT*~inLimits(qUp) < norm(qDown - q_ref) + OUT*~inLimits(qDown)
    q = qUp;     info.branch = 'up';
else
    q = qDown;   info.branch = 'down';
end

%Tolerated so that a target sitting a micron inside a boundary — which is where
%FK output lands when the leg plane degenerates — still counts as reachable.
info.reachable = (d2 >= la^2 - 1e-12) && (abs(c3) <= 1 + 1e-12);
info.inLimits  = inLimits(q);

    function qb = branchAt(zp)
        %Solution for a given signed leg-plane coordinate zp = z'.
        qb = [atan2(s*la*z - zp*y, s*la*y + zp*z);
              atan2(-x, -zp) - psi;
              q3];
    end

    function tf = inLimits(qb)
        tf = all(qb >= params.qmin - 1e-12) && all(qb <= params.qmax + 1e-12);
    end
end
