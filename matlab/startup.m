% startup.m — add project paths. Run first each session, or via -batch "startup; ...".
projectRoot = fileparts(mfilename('fullpath'));
addpath(projectRoot);
addpath(genpath(fullfile(projectRoot, 'src')));
addpath(fullfile(projectRoot, 'viz'));
clear projectRoot;
