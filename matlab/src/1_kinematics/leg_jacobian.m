function Jp = leg_jacobian(q, params, leg)
%LEG_JACOBIAN  3x3 analytic Jacobian of the foot position, J = d(p_foot)/d(q).
%
%   Jp = LEG_JACOBIAN(q, params, leg)
%
%   Column i is the partial derivative of the foot position (hip frame) with
%   respect to q(i). Derived analytically — verify_phase01 checks it against a
%   central finite difference of LEG_FK, so the two must stay independent.
%
%   METHOD — space Jacobian by adjoints, then convert to a point Jacobian.
%   Column i of the space Jacobian is the i-th screw transported by the motion
%   of the joints before it (Lynch & Park, Ch. 5):
%
%     Js1 = S1
%     Js2 = Ad_{e^[S1]q1} S2
%     Js3 = Ad_{e^[S1]q1 · e^[S2]q2} S3
%
%   computed here as the conjugation e^[S]q · [Si] · e^-[S]q, mapped back to a
%   twist with vexa(). The space twist gives the velocity of the body-fixed
%   point instantaneously at the origin, so the foot velocity is
%
%     p_dot = v_s + w_s × p_foot  =>  Jp = Js(1:3,:) - skew(p_foot)·Js(4:6,:)
%
%   with Corke's [v; w] twist ordering (rows 1:3 = v, rows 4:6 = w). Note this
%   is the OPPOSITE of the Robotics System Toolbox convention — see
%   LEG_JACOBIAN_RST, which returns the same 3x3 from [w; v] data.
%
%   See also LEG_FK, LEG_IK, LEG_JACOBIAN_RST.

S = params.legScrew;                % 6x3 space screws, [v; w]

J = zeros(6, 3);

J(:,1) = S(:,1);
J(:,2) = vexa(trexp(S(:,1), q(1))*skewa(S(:,2))*trexp(-S(:,1), q(1)));
J(:,3) = vexa(trexp(S(:,1), q(1))*trexp(S(:,2), q(2))*skewa(S(:,3))...
    *trexp(-S(:,2), q(2))*trexp(-S(:,1), q(1)));

Jp = J(1:3,:) - skew(leg_fk(q, params, leg)) * J(4:6,:);
end
