
% dtheta: estimated gradient
% em: 1 by nJoints cell, each cell is
% nSamples by nDiscretize matrix

function dtheta = stompDTheta(trajProb, em)

    nJoints = length(em);
    nDiscretize = size(trajProb, 2);

    % Variable declaration
    dtheta = zeros(nJoints, nDiscretize);

    %% TODO: Iterate over all joints to compute dtheta
    for m = 1:nJoints

        % Multiply sampled noise by trajectory probabilities
        weightedNoise = trajProb .* em{m};

        % Sum across all sampled trajectories
        dtheta(m,:) = sum(weightedNoise, 1);

    end

end
