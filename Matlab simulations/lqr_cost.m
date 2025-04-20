function J = lqr_cost(params, A, B, C, D, y, Ts)
    % Unpack parameters
    q = params(1:6);       % Q diagonal (6 states)
    r = params(7:8);       % R diagonal (2 inputs)

    % LQR weight matrices
    Q = diag(q);           % 6x6 state weighting
    R = diag(r);           % 2x2 control weighting
    [K, ~, ~] = dlqr(A, B, Q, R);

    % Kalman filter design (fixed for now)
    Qn = eye(6) * 0.01;    % 6x6 process noise covariance
    Rn = eye(2) * 0.1;     % 2x2 measurement noise covariance
    [L, ~, ~] = dlqe(A, eye(6), C, Qn, Rn); % 6x2 Kalman gain

    N = size(y,1);              % Number of time steps
    x_hat = zeros(6,1);         % State estimate (6x1)
    u_lqg = zeros(2,N);         % Control input history (2xN)
    x_hist = zeros(6,N);        % State estimate history (6xN)

    for k = 1:N
        u_lqg(:,k) = -K*x_hat;                            % LQR control
        x_hat = A*x_hat + B*u_lqg(:,k);                   % Predict next state
        x_hat = x_hat + L*(y(k,:)' - C*x_hat);            % Kalman update
        x_hist(:,k) = x_hat;                              % Log estimated state
    end

    % Compute cost function
    J = 0;
    for k = 1:N
        xk = x_hist(:,k);
        uk = u_lqg(:,k);
        J = J + xk'*Q*xk + uk'*R*uk;
    end
end
