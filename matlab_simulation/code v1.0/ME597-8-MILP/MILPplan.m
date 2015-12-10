%% MILP obstacle avoidance
% Uses lp_solve, an open source MILP solver
% Bundled with Windows 64 bit version
% Windows 32 bit and Linux versions 
% http://lpsolve.sourceforge.net/5.5/
clear; clc;
sum = 0;
runs= 1;
for iii=1:runs
%% Problem parameters
% Set up the time
T = 20;
dt = 0.9;

% Set up the vehicle
nX = 4; % Number of states
nU = 2; % Number of inputs
A = [ 1 dt 0 0; 0 1 0 0; 0 0 1 dt; 0 0 0 1]; % Dynamics
B = [0 0; dt 0; 0 0; 0 dt];
xMax = [22 6 20 6]; % State bounds
xMin = [0 -6 0 -6];
uMax = 0.3*[1 1]; % Input bounds
uMin = 0.3*[-1 -1];

% Set up the goals
x0 = [15.5 18.5];
xF = [6 1];

% Set up the obstacles
%rand('state', 5);
nO = 5; % number of obstacles
nE = 4; % number of edges per obstacle (not changeable).
minLen.a = 2; % Obstacle size bounds
maxLen.a = 3;
minLen.b = 3;
maxLen.b = 8;

obstBuffer = 0.5; % Buffer space around obstacles
maxCount = 1000; % Iterations to search for obstacle locations

% Find obstacles that fit
[aObsts,bObsts,obsPtsStore] = polygonal_world(xMin(1:2:3), xMax(1:2:3), minLen, maxLen, nO, x0, xF, obstBuffer, maxCount);

% Plot obstacles
figure(1); clf; hold on;
plotEnvironment(obsPtsStore,xMin(1:2:3), xMax(1:2:3), x0, xF);
drawnow();

%% Problem Variables

% Define each set of variables in the problem
x = zeros(nX*T,1);
u = zeros(nU*(T-1),1);
umag = zeros(nU*(T-1),1);
oBin = zeros(nO*nE*T,1);

% Define main optimization variables (not actually used)
X = [x;u;umag;oBin]; 
% Subsizes
NX = length(X);
NU = length(x);
NUM = NU+length(u);
NO = NUM + length(umag);
bigM = 10000;

% Define which variables are binary
xint = [NO+1:NX]; % Integer variable definition (every row listed is restricted to int)

%% Cost - penalize magnitude of control and time until finished
f = [x;u;ones(size(umag));oBin];

%% Constraints
% Initial position
A0 = [zeros(nX,NX)];
A0(1:nX,1:nX) = eye(nX);
B0 = [x0(1);0; x0(2);0];
e0 = zeros(nX,1);

% Final position
AF = [zeros(nX,NX)];
AF(1:nX,nX*(T-1)+1:nX*T) = eye(nX);
BF = [xF(1);0; xF(2);0];
eF = zeros(nX,1);

% Magnitude of inputs
AM = zeros(2*(T-1)*nU,NX);
AM(1:(T-1)*nU,NUM+1:NO) = eye((T-1)*nU); 
AM(1:(T-1)*nU,NU+1:NUM) = -eye((T-1)*nU); 
AM((T-1)*nU+1:2*(T-1)*nU,NUM+1:NO) = eye((T-1)*nU); 
AM((T-1)*nU+1:2*(T-1)*nU,NU+1:NUM) = eye((T-1)*nU); 
BM = zeros(2*(T-1)*nU,1);
eM = ones(length(BM),1);

% Dynamics
AD = [zeros(nX*(T-1),length(X))];
for t = 1:T-1
    AD(nX*(t-1)+1:nX*(t),nX*(t-1)+1:nX*t) = A;
    AD(nX*(t-1)+1:nX*(t),NU+nU*(t-1)+1:NU+nU*t) = B;
    AD(nX*(t-1)+1:nX*(t),nX*(t)+1:nX*(t+1)) = -eye(nX);
    BD(nX*(t-1)+1:nX*(t),1) = zeros(nX,1);
end
eD = zeros(size(BD));

%% Obstacles
% a[j,i,1]*x[1,t] + a[j,i,2]*x[2,t] - b[j,i] + 10000*obstBinary[t,j,i] >= 0;
AO1 = zeros(nO*nE*T,NX);
for t = 1:T
    for i=1:nO
        for j = 1:nE
            AO1((t-1)*nO*nE+(i-1)*nE+j,nX*(t-1)+1:2:nX*t) = squeeze(aObsts(i,j,:))';
            AO1((t-1)*nO*nE+(i-1)*nE+j,NO+nO*nE*(t-1)+nE*(i-1)+j) = 10000;
            BO1((t-1)*nO*nE+(i-1)*nE+j,1)=bObsts(i,j);
        end
    end
end
eO1 = ones(size(BO1));

%sum{i in 1..4} obstBinary[t,j,i] <= numEdges - 1;
AO2 = zeros(nO*T,NX);
for t = 1:T
    for i=1:nO
        AO2((t-1)*nO+i,NO + nO*nE*(t-1)+nE*(i-1)+1:NO + nO*nE*(t-1)+nE*i) = ones(1,nE);
        BO2((t-1)*nO+i,1) = nE-1;
    end
end
eO2 = -ones(size(BO1));

ACON = [ A0; AF; AM; AD; ];
BCON = [ B0; BF; BM; BD; ];
eCON = [ e0; eF; eM; eD; ];

ACON = [ A0; AF; AM; AD; AO1; AO2];
BCON = [ B0; BF; BM; BD; BO1; BO2];
eCON = [ e0; eF; eM; eD; eO1; eO2];
%% Bounds
vlb = [];
vub = [];
for t = 1:T
    vlb = [vlb, xMin]; % State
    vub = [vub, xMax];
end
for t = 1:T-1
    vlb = [vlb, uMin]; % Inputs
    vub = [vub, uMax];
end    
for t = 1:T-1
    vlb = [vlb, 2*uMin]; % Mag of inputs
    vub = [vub, 2*uMax];
end    
vlb = [vlb, zeros(size(oBin))']; % Obstacle and done binaries
vub = [vub, ones(size(oBin))'];

% Solve MILP
tic;
milp = lp_maker(-f,ACON,BCON,eCON,vlb,vub,xint);
%mxlpsolve('set_timeout',milp, 15)
sol = mxlpsolve('solve', milp);
obj = mxlpsolve('get_objective', milp);
X = mxlpsolve('get_variables', milp);
mxlpsolve('delete_lp', milp);
time(iii) = toc;

% Rename solution variables
x = X(1:nX:NU);
y = X(3:nX:NU);
vx = X(2:nX:NU);
vy = X(4:nX:NU);
ux = X(NU+1:nU:NUM);
uy = X(NU+2:nU:NUM);
umx = X(NUM+1:nU:NO);
umy = X(NUM+2:nU:NO);
oBin = X(NO+1:NX);

% Re-report status for debugging
%sol
%
figure(1);hold on;
plot(x,y,'bx-');
end
average = sum/runs