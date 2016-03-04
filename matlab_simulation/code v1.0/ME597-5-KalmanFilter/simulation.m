function [x, y] = simulation(A,B,C,D,R,Q,x,t,u,example)

% This function simulates a movement and measurement

if example == 1
    % Select a motion disturbance
    e = sqrt(R)*randn(1);
    % Update state
    x = A*x+ B*u(t) + e;

    % Take measurement
    % Select a motion disturbance
    d = sqrt(Q)*randn(1);
    % Determine measurement
    y = C*x + d;
end

if example == 2
    [RE, Re] = eig (R);
    [QRE, QRe] = eig (Q);
    
    % Select a motion disturbance
    e = RE*sqrt(Re)*randn(n,1);
    % Update state
    x = Ad*x+ Bd*u + e;

    % Take measurement
    % Select a motion disturbance
    d = QRE*sqrt(QRe)*randn(m,1);
    % Determine measurement
    y = Cd*x + d;
end

if example == 3
    [RE, Re] = eig (R);
    [QpE, Qpe] = eig (Qp);
    Qp = Q.Qp;
    Qv = Q.Qv;
    Cp = C.Cp;
    Cv = C.Cv;
    mp = length(Cp(:,1));
    mv = length(Cv(:,1));
    
    % Select a motion disturbance
    e = RE*sqrt(Re)*randn(n,1);
    % Update state
    x = Ad*x+ Bd*u(:,t) + e;

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