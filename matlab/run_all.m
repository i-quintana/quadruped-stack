function run_all()
%RUN_ALL  Run every verify_phaseNN in order and print a PASS/FAIL summary.
%   Errors (nonzero exit) if any phase fails — usable in -batch for CI-style checks.

root  = fileparts(mfilename('fullpath'));
files = dir(fullfile(root, 'verify_phase*.m'));
[~, order] = sort({files.name});
files = files(order);

fprintf('\n==== run_all: %d phase(s) ====\n', numel(files));
nPass = 0;
for i = 1:numel(files)
    name = erase(files(i).name, '.m');
    try
        feval(name);
        fprintf('  [PASS] %s\n', name);
        nPass = nPass + 1;
    catch ME
        fprintf(2, '  [FAIL] %s -- %s\n', name, ME.message);
    end
end
fprintf('==== %d/%d passed ====\n\n', nPass, numel(files));

if nPass < numel(files)
    error('run_all: %d phase(s) failed.', numel(files) - nPass);
end
end
