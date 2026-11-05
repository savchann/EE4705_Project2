
% Input:
%   nSamplePaths: number of sample paths
%   sigma: sample covariance matrix
%   theta: mean trajectory from last iteration
%
% Output:
%   theta_paths: sampled trajectories
%   em: sampled zero-mean Gaussian trajectory for each joint

function [theta_paths, em] = stompSamples(nSamplePaths, sigma, theta)

    % Sample theta (joint angles) trajectory
    [nJoints, nDiscretize] = size(theta);

    em = cell(1, nJoints);
    ek = cell(1, nSamplePaths);

    theta_paths = cell(1, nSamplePaths);
    mu = zeros(1, length(sigma));

    %% TODO: Complete code of independent sampling for each joint
    for m = 1:nJoints

        % Each joint is sampled independently.
        % Sample noise for the intermediate waypoints.
        em_interior = mvnrnd(mu, sigma, nSamplePaths);

        % The starting q0 and final qT are fixed,
        % so set the noise at both endpoints to zero.
        em{m} = [zeros(nSamplePaths,1), ...
                 em_interior, ...
                 zeros(nSamplePaths,1)];

    end

    %% Regroup it by samples
    emk = [em{:}];

    for k = 1:nSamplePaths
        ek{k} = reshape(emk(k,:), nDiscretize, nJoints)';
        theta_paths{k} = theta + ek{k};
    end

end
