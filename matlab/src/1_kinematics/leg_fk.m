function p_foot = leg_fk(q, params, leg)
%LEG_FK  Forward kinematics of one 3-DOF leg: joint angles -> foot position.
%
%   p_foot = LEG_FK(q, params, leg)
%
%   INPUTS
%     q      3x1 joint angles [q1; q2; q3] = [HAA; HFE; KFE]  (rad)
%     params struct from params_quad()
%     leg    one of 'FL','FR','RL','RR'
%
%   OUTPUT
%     p_foot 3x1 foot position in the HIP (leg-base) frame — aligned with the
%            body frame at q = 0 (x fwd, y left, z up), origin at the HAA axis.
%
%   METHOD — product of exponentials in the space frame (Lynch & Park, Ch. 4):
%
%     T(q) = e^[S1]q1 · e^[S2]q2 · e^[S3]q3 · M ,   p_foot = T(1:3,4)
%
%   The screw axes S = params.legScrew (6x3, Corke [v;w] ordering) are the same
%   for every leg; the home pose M = params.legHome.(leg) carries the abduction
%   sign. skewa() lifts a twist to its 4x4 se(3) matrix, expm() exponentiates it.
%
%   This is the same model as the transform chain
%     Rx(q1) · [0; s*l_abad; 0] · Ry(q2) · [0;0;-l_thigh] · Ry(q3) · [0;0;-l_calf]
%   with s = params.abadSign.(leg), just written in screw form.
%
%   Sanity anchor: q = [0;0;0]  ->  p_foot = [0; s*l_abad; -(l_thigh+l_calf)].
%
%   See also LEG_IK, LEG_JACOBIAN, LEG_FK_RST.

S = params.legScrew;                % 6x3 space screws, [v; w]
M = params.legHome.(leg);           % foot pose at q = 0

p_foot = expm(skewa(S(:,1))*q(1)) * expm(skewa(S(:,2))*q(2)) ...
       * expm(skewa(S(:,3))*q(3)) * M;
p_foot = p_foot(1:3, 4);
end
