
function T = ur5eFKinSpace(M, Slist, theta)
% Forward kinematics using Product of Exponentials
%
% Inputs:
% M      - 4x4 home transformation
% Slist  - 6xn screw axes
% theta  - nx1 joint angles (radians)
%
% Output:
% T      - 4x4 end-effector transformation

T = eye(4);

for i = 1:length(theta)

    % Extract screw axis
    S = Slist(:,i);

    omega = S(1:3);
    v = S(4:6);

    % Skew-symmetric matrix
    omega_hat = [ ...
        0        -omega(3)  omega(2);
        omega(3)  0        -omega(1);
        -omega(2)  omega(1)  0 ];

    % Twist matrix
    S_hat = [omega_hat, v;
        0 0 0 0];

    % Matrix exponential
    T = T * expm(S_hat * theta(i));

end

% Apply home transformation
T = T * M;

end
