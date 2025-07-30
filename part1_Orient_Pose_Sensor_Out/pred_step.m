function [covarEst,uEst] = pred_step(uPrev,covarPrev,angVel,acc,dt)
%% BEFORE RUNNING THE CODE CHANGE NAME TO pred_step
    %% Parameter Definition
    % uPrev - is the mean of the prev state
    %covarPrev - covar of the prev state
    %angVel - angular velocity input at the time step
    %acc - acceleration at the timestep
    %dt - difference in time 

%To extract the previous state values
x =uPrev(1,1);
y =uPrev(2,1);
z =uPrev(3,1);

roll = uPrev(4,1);
pitch = uPrev(5,1);
yaw = uPrev(6,1);

velocity_x = uPrev(7,1);
velocity_y = uPrev(8,1);
velocity_z = uPrev(9,1);

bias_gyro_x = uPrev(10,1);
bias_gyro_y = uPrev(11,1);
bias_gyro_z = uPrev(12,1);

bias_acc_x = uPrev(13,1);
bias_acc_y = uPrev(14,1);
bias_acc_z = uPrev(15,1);

% To extract the components from the angular velocity 
wmx = angVel(1,1);
wmy = angVel(2,1);
wmz = angVel(3,1);

% To define the angular velocity vectors
wm = [wmx; wmy;wmz];

% To extract the components from the acceleration input
acc_mx = acc(1,1);
acc_my = acc(2,1);
acc_mz = acc(3,1);

% To define the acceleration vectors
am = [acc_mx;acc_my;acc_mz];

% To take gravity vector
g = [0;0;-9.81];

%To define state vectors x1(position), x2(orientation), x3(velocity), x4(gyroscope bias), x5(accelerometer bias)
x1 = [x;y;z];

% X = [x1;x2;x3;x4;x5];

%To process noise covariance matrix
Q = eye(12,12)*0.001; %To consider it as 12x12 sized identity matrix here

%To augment the covariance matrix with Q
P_aug = [covarPrev,zeros(15,12);
         zeros(12,15),Q];
covar_aug = chol(P_aug,'lower');

% To augment the previous state vector with zeros for processing the noise
u_augPrev = [uPrev;zeros(12,1)];

% To define the parameters for the unscented transform (UKF)
alpha = 0.001;
k =1;
beta = 2;   
n_prime = 27;
% length(covar_aug)
lambda = (alpha)^2 *(n_prime+k) - n_prime;

% To initialize arrays to hold augmented sigma points
X_aug_plus = [];
X_aug_minus = [];

% To generate sigma points for the unscented transform (UKF)
for i = 1:length(covar_aug)
    X_aug_p = u_augPrev+ ((n_prime + lambda)^(1/2))*(covar_aug(:,i));
    X_aug_m = u_augPrev- ((n_prime + lambda)^(1/2))*(covar_aug(:,i));
    X_aug_plus = [X_aug_plus,X_aug_p];
    X_aug_minus= [X_aug_minus, X_aug_m];
end

% To include the sigma points into matrix named X_aug
X_aug  = [u_augPrev , X_aug_plus , X_aug_minus];
                                                            
% To initialize arrays for storing augmented state derivatives
x_dot_aug = [];
x_t = [];

%Calculating the Weights %For the unscented transform (UKF)
W0_c = (lambda/(n_prime+lambda)) + (1-(alpha)^2 + beta);
Wi_c = 1/(2*(n_prime + lambda));

W0_m = lambda/(n_prime + lambda);
Wi_m = Wi_c;

for j = 1:55
    % To extract components from the augmented state vector X_aug
    n = X_aug(16:27,j); % To extract noise components 'n'
    n_gyro = n(1:3,:); % Gyroscope bias noise
    n_acc = n(4:6,:); % Accelerometer bias noise
    nb_gyro = n(7:9,:); % Gyroscope bias drift noise
    nb_acc = n(10:12,:); % Accelerometer bias drift noise

    % To extract the roll, pitch, and yaw angles from X_aug
    roll  = X_aug(4,j);
    pitch = X_aug(5,j);
    yaw = X_aug(6,j);

    x3 = X_aug(7:9,j); % To extract velocity components (velocity_x, velocity_y, velocity_z)
    x4 = X_aug(10:12,j); % To extract gyroscope bias components (bias_gyro_x, bias_gyro_y, bias_gyor_z)
    x5 = X_aug(13:15,j); % To extract accelerometer bias components (bias_acc_x, bias_acc_y, bias_acc_z)

    % To calculate orientation rotation matrix
    G_x2_inv =[(cos(yaw)*sin(pitch))/cos(pitch) (sin(pitch)*sin(yaw))/cos(pitch) 1 ; 
               -sin(yaw)                         cos(yaw)                        0; 
               cos(yaw)/cos(pitch)               sin(yaw)/cos(pitch)             0];

    G_x2_inv_xyz = flip(G_x2_inv); % To flip the rotation matrix to get the inverse transformation

    % To calculate individual rotation matrices
    Rx = [1 0 0 ; 0 cos(roll) -sin(roll) ; 0 sin(roll) cos(roll)];
    Ry = [cos(pitch) 0 sin(pitch) ; 0 1 0 ; -sin(pitch) 0 cos(pitch)];
    Rz = [cos(yaw) -sin(yaw) 0 ; sin(yaw) cos(yaw) 0 ;0 0 1];

    % To calculate the total orientation rotation matrix in ZYX rotation order
    Rotation_zyx = (Rz*Ry*Rx);


    % To calculate state derivative x_dot
    x_dot = [x3 ; G_x2_inv_xyz*Rotation_zyx*(wm - x4 - n_gyro) ; g+(Rotation_zyx)*(am - x5 - n_acc); nb_gyro ; nb_acc];
    x_dot_aug =[x_dot_aug, x_dot];

    % To perform the Euler integration to predict the next state = to calculate the first nine elements in each column we first multiply by dt and add it to first nine elements of the previous state and for the next 6 elements in each column we add the noise
    x_t(1:9,j) = x_dot_aug(1:9,j)*dt + X_aug(1:9,j);
    x_t(10:15,j)= x_dot_aug(10:15,j) + X_aug(10:15,j);
end

% To calculate the estimated state vector and covariance
Ut_0 = W0_m * x_t(:,1);
Ut_i = 0;

for i =2:55
    Ut_i =  Ut_i + Wi_m * x_t(:,i);
end
uEst = Ut_0 + Ut_i; % To calculate the estimated state vector 

% To calculate the initial covariance estimate
covar_0 = W0_c*(x_t(:,1) - uEst)*(x_t(:,1) -uEst)';
Covar_i = 0; % To initialize Covar_i to accumulate the weighted covariance predictions


for i = 2:55
    Covar_i =  Covar_i + Wi_c*(x_t(:,i) - uEst)*(x_t(:,i)-uEst)';
end

% To calculate the final estimated covariance
covarEst = covar_0 + Covar_i;


end

