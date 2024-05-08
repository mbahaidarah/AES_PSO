%{
The following code is an implementation of Active Elastic Sheet (AES) model
and the control parameters of the model were optimised using Particle Swarm
Optimisation (PSO).
The control parameters are: alpha, beta

-------------------------------
author: Mazen Bahaidarah
Last Update: 07-12-2022
-------------------------------

%}
clear; clc; close all;

% The control parameters of the AES model
parameters = [0.18082, 0.81649, 1, 0.02];

alpha = parameters(1);
beta  = parameters(2);
k     = parameters(3);
V0    = parameters(4); 


sim_time = 300; % max simulation time

n_robots = 100; % swarm size
number_of_robots_each_column = 10; % number of each column in a square shape

noise_strength = 0; % noise value
L = 100; % Length of arena side
R = 7.1;  % Sensing radius for each robot to detect its neighbours


% Build the swarm in a square shape
n = number_of_robots_each_column - 1; % to locate (robot_each_col) robots at each column
len_square = 5; % the distance between the robots
for i = 1 : n_robots
    for j = 1 : n_robots
        quotient = fix((i - 1)/(n+1));  % 
        remainder = mod((i + n),(n+1));  % 
        pos_init(1,i) = len_square * quotient; 
        pos_init(2,i) = len_square * remainder;
    end
end


positions = pos_init + 30; % for new square robot positioning
pos_meas = positions;
ang = rand(1, n_robots) * 1.5 * pi;
pos_env = [positions; ang];


% Calculate the initial distance between robots before they move 
l_ij = dist(positions, 'euclidean');
[k1, k2] = find(l_ij > 0 & l_ij < R);

for step = 1:sim_time

    for i = 1:n_robots
        % Generate the noise
        d_r_n = unifrnd(0, 1 * noise_strength); % Sensing Noise. It was unifrnd(0, 1.5 * noise_strength)
        random_noise_value = unifrnd(0, 2*pi); %unifrnd(0, 1.0) * 2 * pi;
        e_r_n = [cos(random_noise_value); sin(random_noise_value)];    
        d_theta_n = unifrnd(0, 1 * noise_strength); % Actuation Noise
        e_theta_n = randn;
       
        neighbour_list = k1(k2 == i); % contains neighbours id of robot i 

        ni_x = cos(ang(i));   ni_y = sin(ang(i));  % ni hat, that is parallel to the robot heading
        np_x = cos(ang(i) + (pi/2));  np_y = sin(ang(i) + (pi/2));
        fi = [0; 0]; % accumlate the forces for i robot
         
        for q = 1:length(neighbour_list) % to check the nieghbours of i robot
            j = neighbour_list(q); % to get the index of the robot neighbour
            
            % Calculate the euclidean distance 
            r_ij = pos_meas(:,i) - pos_meas(:,j);
            r_norm = norm(r_ij);
            
    
            % Calculate the force
            f_neb = (-k / l_ij(i,j)) * (r_norm - l_ij(i,j)) / r_norm * r_ij;
            fi = fi + f_neb;    
        end

        fi_value(i) = norm(fi); % to get the value of the neighbour force for each robot i

        % Update robot i position
        ni = [ni_x; ni_y];
        new_pos = (V0 * ni + alpha * ((fi + d_r_n * e_r_n)' * ni) * ni); 

        % Update robot i angle
        ni_perp = [np_x; np_y];
        new_ang = (beta *((fi + d_r_n * e_r_n)' * ni_perp) + d_theta_n * e_theta_n); 
        new_ang = new_ang + ang(i);

        ang(i) =  mod(new_ang, 1.5 * pi); 
        
        positions(1, i) = positions(1, i) + new_pos(1);       
        positions(2, i) = positions(2, i) + new_pos(2);      

        pos_meas(:,i) = positions(:,i) * (1 + (2*rand - 1) * 0.001); % adding white noise to position to mimic real-world scenario
        
    end % loop for each robot i

     % ------------------------ Performance metrics ------------------------ 
    % The degree of alignment
    alignment(step) = norm([sum(cos(ang)), sum(sin(ang))]) / n_robots;

    % The total virtual force
    force(step) = sum(fi_value);
    
    % --------------------------------------------------------------------- 



     % 1. Visualise the collective motion of the swarm
    quiver(positions(1,:), positions(2,:), cos(ang), sin(ang), 0.3,'filled','Marker','diamond','MarkerEdgeColor','b','MarkerFaceColor','b','MarkerSize',3); % show the followers in blue color
   
    text(3, 10, '{\psi}: ', 'color', 'k', 'FontSize', 12); 
    text(9, 10, [num2str(alignment(step)*100)], 'color', 'k', 'FontSize', 9);
    xlim([0 L]); % Comment the next 3 lines when I want to have two graphs 1:robot env, 2:f_neb vs step
    ylim([0 L]);
    axis square
    pause(0.1);


    
end





