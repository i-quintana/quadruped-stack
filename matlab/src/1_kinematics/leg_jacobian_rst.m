function Jp = leg_jacobian_rst(q, params, leg)
%LEG_JACOBIAN_RST  3x3 foot-position Jacobian via Robotics System Toolbox.
%
%   Jp = LEG_JACOBIAN_RST(q, params, leg)
%
%   Toolbox counterpart of LEG_JACOBIAN. geometricJacobian returns the 6x3
%   Jacobian of the 'foot' frame relative to the tree base, and its linear rows
%   are already the velocity of the foot origin — so d(p_foot)/d(q) is a slice,
%   with no skew(p) correction needed (that correction exists in LEG_JACOBIAN
%   because a space twist describes the body-fixed point at the origin, not the
%   foot).
%
%   CONVENTION WARNING: RST orders the geometric Jacobian [omega; v], so the
%   linear part is rows 4:6. Corke's toolbox orders twists [v; w], so in
%   LEG_JACOBIAN the linear part is rows 1:3. Mixing the two silently produces
%   a Jacobian that looks plausible and is wrong; pick one ordering per project.
%
%   See also LEG_JACOBIAN, LEG_RBT, GEOMETRICJACOBIAN.

J  = geometricJacobian(leg_rbt(params, leg), q(:), 'foot');
Jp = J(4:6, :);
end
