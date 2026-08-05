function verify_phase01()
%VERIFY_PHASE01  Acceptance tests for the leg kinematics (FK, IK, Jacobian).
%   Exits cleanly iff leg_fk / leg_ik / leg_jacobian are correct and mutually
%   consistent for all four legs, their Robotics System Toolbox counterparts
%   agree, and leg_ik honours branch selection, joint limits and reachability.
%   Run:
%     matlab -batch "startup; verify_phase01"

p = params_quad();
rng(0);
tolAnchor = 1e-12;  tolRT = 1e-9;  tolJac = 1e-6;
tolRST    = 1e-12;  tolBranch = 1e-9;  tolIkRST = 1e-6;
nRT = 200;  nJ = 25;  nRST = 10;

for i = 1:numel(p.legs)
    leg = p.legs{i};
    s   = p.abadSign.(leg);

    % (1) zero-pose anchor: leg straight down, laterally offset by s*l_abad
    p0     = leg_fk([0;0;0], p, leg);
    anchor = [0; s*p.l_abad; -(p.l_thigh + p.l_calf)];
    assert(norm(p0 - anchor) < tolAnchor, ...
        'FK zero-pose anchor wrong for %s: got [%.4g %.4g %.4g]', leg, p0(1),p0(2),p0(3));

    % (1b) second anchor with the knee bent — catches models that collapse the
    % thigh and calf into one link (q2+q3), which the zero pose cannot see
    p1  = leg_fk([0;0;-pi/2], p, leg);
    a1  = [p.l_calf; s*p.l_abad; -p.l_thigh];
    assert(norm(p1 - a1) < tolAnchor, ...
        'FK bent-knee anchor wrong for %s: got [%.4g %.4g %.4g]', leg, p1(1),p1(2),p1(3));

    % sampling box (stay off joint limits and near-singular full extension)
    lo = p.qmin + 0.15;
    hi = p.qmax - 0.15;

    % (2) FK -> IK -> FK round trip (compare foot position; branch-agnostic)
    for k = 1:nRT
        q   = lo + (hi - lo).*rand(3,1);
        pf  = leg_fk(q, p, leg);
        qi  = leg_ik(pf, p, leg);
        pf2 = leg_fk(qi, p, leg);
        assert(norm(pf - pf2) < tolRT, ...
            'IK round-trip failed for %s (err %.3e)', leg, norm(pf - pf2));
    end

    % (3) analytic Jacobian vs central finite difference of FK
    h = 1e-6;
    for k = 1:nJ
        q  = lo + (hi - lo).*rand(3,1);
        J  = leg_jacobian(q, p, leg);
        Jn = zeros(3,3);
        for j = 1:3
            dq = zeros(3,1); dq(j) = h;
            Jn(:,j) = (leg_fk(q+dq, p, leg) - leg_fk(q-dq, p, leg)) / (2*h);
        end
        e = max(abs(J(:) - Jn(:)));
        assert(e < tolJac, 'Jacobian mismatch for %s (max err %.3e)', leg, e);
    end

    % (4) IK branch fidelity: seeded with the pose that generated the target,
    % IK must return THAT pose and not its mirror image. Roughly a fifth of this
    % box puts the foot above the hip, where a plain R = sqrt(y^2+z^2-l_abad^2)
    % returns the other branch — same point, wrong joint angles.
    nUp = 0;
    for k = 1:nRT
        q = lo + (hi - lo).*rand(3,1);
        [qi, info] = leg_ik(leg_fk(q, p, leg), p, leg, q);
        assert(norm(qi - q) < tolBranch, ...
            'IK returned the wrong branch for %s (err %.3e, branch %s)', ...
            leg, norm(qi - q), info.branch);
        assert(info.reachable, 'IK flagged a reachable pose unreachable for %s', leg);
        nUp = nUp + strcmp(info.branch, 'up');
    end

    % (5) with the default seed the branch is free to differ, but the answer
    % must still be a real pose: inside the joint limits and on the target
    for k = 1:nRT
        q  = lo + (hi - lo).*rand(3,1);
        pf = leg_fk(q, p, leg);
        [qi, info] = leg_ik(pf, p, leg);
        assert(info.inLimits, ...
            'IK left the joint limits for %s: q = [%.4g %.4g %.4g]', leg, qi(1),qi(2),qi(3));
        assert(norm(leg_fk(qi, p, leg) - pf) < tolRT, ...
            'IK default-seed round-trip failed for %s', leg);
    end

    % (6) reachability: unreachable targets clamp and report, never throw
    far  = [0; s*p.l_abad; -1.5*(p.l_thigh + p.l_calf)];      % beyond full extension
    near = [0; 0; 0];                                          % inside the abad cylinder
    fold = [0; s*p.l_abad; -0.5*abs(p.l_thigh - p.l_calf)];    % inside the folded annulus
    for pf = [far, near, fold]
        [qi, info] = leg_ik(pf, p, leg);
        assert(~info.reachable, 'IK called an unreachable target reachable for %s', leg);
        assert(all(isfinite(qi)), 'IK returned non-finite q for %s', leg);
    end
    pf = leg_fk(p.q_nominal, p, leg);
    [~, info] = leg_ik(pf, p, leg);
    assert(info.reachable, 'IK called the nominal stance unreachable for %s', leg);

    % (7) Robotics System Toolbox counterparts must agree with the hand-written
    % versions — two independent formulations of the same model
    for k = 1:nJ
        q  = lo + (hi - lo).*rand(3,1);
        eF = norm(leg_fk(q, p, leg) - leg_fk_rst(q, p, leg));
        eJ = max(abs(leg_jacobian(q, p, leg) - leg_jacobian_rst(q, p, leg)), [], 'all');
        assert(eF < tolRST, 'leg_fk_rst disagrees with leg_fk for %s (%.3e)', leg, eF);
        assert(eJ < tolRST, 'leg_jacobian_rst disagrees with leg_jacobian for %s (%.3e)', leg, eJ);
    end

    % (8) numerical RST IK reaches the same targets as the closed form
    for k = 1:nRST
        q  = lo + (hi - lo).*rand(3,1);
        pf = leg_fk(q, p, leg);
        [qi, info] = leg_ik_rst(pf, p, leg, p.q_nominal);
        assert(info.reachable, 'leg_ik_rst failed to converge for %s (%s)', leg, info.status);
        assert(norm(leg_fk(qi, p, leg) - pf) < tolIkRST, ...
            'leg_ik_rst round-trip failed for %s (err %.3e)', leg, norm(leg_fk(qi,p,leg) - pf));
    end

    fprintf('  %s ok  (%d/%d sampled poses put the foot above the hip)\n', leg, nUp, nRT);
end

fprintf(['verify_phase01 PASSED  (4 legs: 2 anchors + %d round-trips + %d Jacobians\n' ...
         '                        + %d branch/limit checks + reachability + %d RST parity\n' ...
         '                        + %d RST IK solves, each)\n'], nRT, nJ, 2*nRT, nJ, nRST);
end
