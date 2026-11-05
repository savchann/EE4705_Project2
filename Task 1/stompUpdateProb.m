
function trajProb = stompUpdateProb(Stheta)

    % Sensitivity parameter
    h = 10;

    [nPaths, nDiscretize] = size(Stheta);

    trajProb = zeros(nPaths, nDiscretize);

    for t = 1:nDiscretize

        costs = Stheta(:,t);

        minCost = min(costs);
        maxCost = max(costs);

        if abs(maxCost-minCost) < 1e-12

            % Identical costs: uniform probabilities
            trajProb(:,t) = 1/nPaths;

        else

            normalizedCost = ...
                (costs-minCost)/(maxCost-minCost);

            weights = exp(-h*normalizedCost);

            trajProb(:,t) = weights/sum(weights);

        end
    end
end
