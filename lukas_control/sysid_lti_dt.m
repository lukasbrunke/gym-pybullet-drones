%% SystemID for simulator data
%% Read data

states = readmatrix('sysid_data/states.csv');
control_inputs = readmatrix('sysid_data/control_inputs.csv');
timestamps = readmatrix('sysid_data/timestamps.csv');

%% Set up data
state_dim = size(states, 1);
input_dim = size(control_inputs, 2);
dt = timestamps(2) - timestamps(1);
num_data = size(timestamps, 2);

% Set the initial state to zero
states(1, :) = states(1, :) - states(1, 1);
states(2, :) = states(2, :) - states(2, 1);

%% Provide discrete-time LTI State space system structure
output_dim = state_dim;
% set dummy matrices
A = eye(state_dim);
B = zeros(state_dim, input_dim); 
C = eye(state_dim);
D = zeros(output_dim, input_dim);
% initialize dummy state space system
m = idss(A, B, C, D, 'Ts', dt);

% Set constraints on the system matrices:
% - full state measurement from the simulator --> C is identity
% - D is zero
% - the first entry of B is zero, because inputs do not directly affect the
%   position
for i = 1 : state_dim
    for j = 1 : state_dim
        if i == j
            m.Structure.C.Value(i, j) = 1;
        else
            m.Structure.C.Value(i, j) = 0;
        end
        m.Structure.C.Free(i, j) = false;
    end
    for j = 1 : input_dim
        m.Structure.D.Free(i,j) = false;
    end
end

m.Structure.B.Value(1, 1) = 0;
m.Structure.B.Free(1,1) = false;

%% Identify discrete-time LTI system
id_data = iddata(states', control_inputs, dt);
m = ssest(id_data, m);
A = m.A
B = m.B
C = m.C;
writematrix(A, 'sysid_data/A.csv');
writematrix(B, 'sysid_data/B.csv');

% Check controllability and observability
rank(ctrb(A, B))
rank(obsv(A, C))

%% Simulate model using control input data
X_sim = zeros(state_dim, num_data);
X_sim(:, 1) = states(:, 1);

for i = 2 : num_data
    X_sim(:, i) = A * X_sim(:, i - 1) + B * control_inputs(i - 1, 1);
end

%% Compare simulator system and identified system
figure 
hold on
plot(timestamps, states(1, :), 'r')
plot(timestamps, X_sim(1, :), 'b')
hold off

figure 
hold on
plot(timestamps, states(2, :), 'r')
plot(timestamps, X_sim(2, :), 'b')
hold off

%% LQR controller

Q = eye(state_dim);
R = 0.1;

K = dlqr(A, B, Q, R);
K = - K
writematrix(K, 'sysid_data/K.csv');


%% Determine w
w = max(abs([states(1,:), states(2, :)] - [X_sim(1, :), X_sim(2, :)]));




