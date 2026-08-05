function p_foot = leg_fk_rst(q, params, leg)
%LEG_FK_RST  Forward kinematics of one leg via Robotics System Toolbox.
%
%   p_foot = LEG_FK_RST(q, params, leg)
%
%   Toolbox counterpart of LEG_FK: identical inputs, identical output, one call
%   instead of a product of exponentials. Kept alongside the hand-written
%   version as a cross-check — verify_phase01 asserts the two agree.
%
%   getTransform returns the full 4x4 pose of 'foot' relative to the tree base
%   (= the hip frame), so the position is just its translation column. Pass a
%   third body name to express it in some other frame, e.g.
%   getTransform(rbt, q, 'foot', 'thigh').
%
%   See also LEG_FK, LEG_RBT, GETTRANSFORM.

T = getTransform(leg_rbt(params, leg), q(:), 'foot');
p_foot = T(1:3, 4);
end
