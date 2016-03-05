function [x, y] = simulation(model_params,x,t,u,example)

% This function simulates a movement and measurement
A = model_params.A;
B = model_params.B;
C = model_params.C;
D = model_params.D;
R = model_params.R;
Q = model_params.Q;
n = model_params.n;
m = model_params.m;

if example == 1
    % Select a motion disturbance
    e = sqrt(R)*randn(1);
    % Update state
    x = A*x+ B*u + e;

    % Take measurement
    % Select a motion disturbance
    d = sqrt(Q.Q)*randn(1);
    % Determine measurement
    y = C*x + d;
end

if example == 2
    [RE, Re] = eig (R);
    [QRE, QRe] = eig (Q.QR);
    
    n = length(A(1,:));
    m = length(C(:,1));
    
    % Select a motion disturbance
    e = RE*sqrt(Re)*randn(n,1);
    % Update state
    x = A*x+ B*u + e;

    % Take measurement
    % Select a motion disturbance
    d = QRE*sqrt(QRe)*randn(m,1);
    % Determine measurement
    y = C*x + d;
end

if example == 3
    [RE, Re] = eig (R);
    [QpE, Qpe] = eig(Q.Qp);
    [QvE, Qve] = eig(Q.Qv);
    Qp = Q.Qp;
    Qv = Q.Qv;
    Cp = C.Cp;
    Cv = C.Cv;
    n = length(A(1,:));
    mp = length(Cp(:,1));
    mv = length(Cv(:,1));
    
    % Select a motion disturbance
    e = RE*sqrt(Re)*randn(n,1);
    % Update state
    x = A*x+ B*u + e;

    % Take measurement
    % Select a measurement disturbance and determine measurement
    if (mod(t,10)==0)
        d = QpE*sqrt(Qpe)*randn(mp,1);
        y = Cp*x + d;
    else
        d = QvE*sqrt(Qve)*randn(mv,1);
        y([2 4]) = Cv*x + d;
    end
end