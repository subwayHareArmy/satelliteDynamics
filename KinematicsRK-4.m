%Abki Baar, Yembarwar
%Program to simulate attitude kinematics of the satellite over one orbit.

%%%%%%%%%%%%%%%%%% MAKE A FUNCCTION MODULE FROM OF THIS AND INTEGRATE WITH
%%%%%%%%%%%%%%%%%% DYNAMICS. TAKE w, qINITIAL AND STEP AS PARAMETERS.

% Not using this statement because yes_or_no function isn't included in Octave
% bleh = yes_or_no (" Do you want to accept values? ");

h = 0.1;                                           % Time step size
tic;
bleh = 0;
timeDuration = 90;                                 % Time duration of the simulation in minutes (One orbit in this case)      
numberOfInstants = timeDuration*60*(1/h)           % Number of instants (54000 in this case)

% Storing attitude quaternion for plotting later - each column corresponds to the quaternion and the column number corresponds to the time instant
q = zeros(4,numberofInstants); 

% Norm values for the quaternion array - Used only to check if norm == 1 
norm = zeros(1,numberofInstants);


if (bleh == 0)                                     % Using hardcoded values if bleh == 0
  q(:,1) = [0.5; 0.5; 0.5; 0.5];                   % Starting attitude quaternion
  w = [0.1;                                        % Starting value of angular velocity (In Satellite Body Frame (SBRF))
       0;
       0];
end

if (bleh == 1)                                     % Accepting user given values if bleh == 1
  printf("Enter values for Attitude quaternion\n");
  for j = 1:4
    q(j,1) = input("");
  end
  
  for j = 1:3
    printf("Enter values for the w (angular velocity in Satellite Body Frame(SBRF)) vector(3x1) \n");
    w(ceil(j/3),1) = input("");
  end
end

%Main Body -

for temp = 1:numberofInstants                       % The propogator 
  matrixTheta = [  0  ,  w(3), -w(2), w(1);         % The rotation matrix for the quaternion 
                 -w(3),   0  ,  w(1), w(2);
                  w(2), -w(1),   0  , w(3);
                 -w(1), -w(2), -w(3),  0  ];
             
  k1 = (matrixTheta*q(:,temp))./2;                  % Numerical integration using Runge-Kutta 4
  k2 = (matrixTheta*(q(:,temp)+(h*k1)./2))./2;      
  k3 = (matrixTheta*(q(:,temp)+(h*k2)./2))./2;
  k4 = (matrixTheta*(q(:,temp)+(h*k3)))./2;
  
  q(:,temp+1) = q(:,temp) + h*(k1 + 2*k2 + 2*k3 + k4)./6;
  norm(1,temp) = ceil(q(1,temp)^2 + q(2,temp)^2 + q(3,temp)^2 + q(4,temp)^2)                     % Storing norm values of q
  
% q(:,temp+1) = q(:,temp) + qDot.*0.1;           

end

time = 0:h:60*timeDuration;

figure
plot(q(1,:),time);
title('Parameter 1');
xlabel('Quaternion parameter 1');
ylabel('Time (0.1 s)');

figure
plot(q(2,:),time);
title('Parameter 2');
xlabel('Quaternion parameter 2');
ylabel('Time (0.1 s)');

figure
plot(q(3,:),time);
title('Parameter 3');
xlabel('Quaternion parameter 3');
ylabel('Time (0.1 s)');

figure
plot(q(4,:),time);
title('Parameter 4');
xlabel('Quaternion parameter 4');
ylabel('Time (0.1 s)');

toc;                                                % Print total time required for the program
