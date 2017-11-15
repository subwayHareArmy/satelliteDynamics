%Abki Baar, Yembarwar
%Program to compute and simulate attitude kinematics
%%%%%%%%%%%%%%%%%% MAKE A FUNCCTION MODULE FROM OF THIS AND INTEGRATE WITH
%%%%%%%%%%%%%%%%%% DYNAMICS. TAKE w, qINITIAAL AND STEP AS PARAMETERS.

%bleh = yes_or_no (" Do you want to accept values? ");
tic;
bleh = 0;
q = zeros(4,54000);
norm = zeros(1,54000);
h = 0.1; %step size

if (bleh == 0) % Using hardcoded values if ans == 0
  q(:,1) = [0.5; 0.5; 0.5; 0.5];
  w = [0.1;
       0;
       0];
end

if (bleh == 1) % Using user given values if ans == 0
  printf("Enter values for q\n");
  for j = 1:4
    q(j,1) = input("");
  end
  
  for j = 1:3
    printf("Enter values for the 3x1 vector matrix \n");
    w(ceil(j/3),1) = input("");
  end
end

%Main Body -

for temp = 1:54000 %Simulation is for 90 minutes 
  matrixTheta = [  0  ,  w(3), -w(2), w(1);
                 -w(3),   0  ,  w(1), w(2);
                  w(2), -w(1),   0  , w(3);
                 -w(1), -w(2), -w(3), 0  ];
             
  k1 = (matrixTheta*q(:,temp))./2;
  k2 = (matrixTheta*(q(:,temp)+(h*k1)./2))./2;
  k3 = (matrixTheta*(q(:,temp)+(h*k2)./2))./2;
  k4 = (matrixTheta*(q(:,temp)+(h*k3)))./2;
  
  q(:,temp+1) = q(:,temp) + h*(k1 + 2*k2 + 2*k3 + k4)./6;
  
%  q(:,temp+1) = q(:,temp) + qDot.*0.1;
  norm(1,temp) = q(1,temp)^2 + q(2,temp)^2 + q(3,temp)^2 + q(4,temp)^2;
end

time = 0:0.1:5400;
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

toc;
