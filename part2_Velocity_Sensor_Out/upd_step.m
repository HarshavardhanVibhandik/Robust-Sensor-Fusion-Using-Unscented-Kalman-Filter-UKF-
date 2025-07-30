function [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst,omega)
%% BEFORE RUNNING THE CODE CHANGE NAME TO upd_step
    %% Parameter Definition
    %z_t - is the sensor data at the time step
    %covarEst - estimated covar of the  state
    %uEst - estimated mean of the state
    

alpha = 0.001; % To take the constant parameter
k =1; % To take the constant parameter
beta = 2; % To take the constant parameter
n_prime = 15; % The number o components in the state vectors
vt = zeros(3,1); % To take the velocity - zero vector

covaraug = chol(covarEst,'lower'); % To compute the Cholesky decomposition

lambda = (alpha)^2 *(n_prime+k) - n_prime; % To calculate the lambda value


%Derived from the image given 
%0.785 radians = 45 degrees
r_cb_b = [-0.04*cos(0.785) , 0 , -0.03]; % To calculate the vector
skew_r_cb_b = [0 0.03 0.0283; -0.03 0 -0.0283; -0.0283 0.0283 0;]; % To calculate the matrix

% To initialize the arrays
X_est_plus = [];
X_est_minus = [];

% To extract the angles (Roll, Pitch, Yaw)
roll = uEst(4,1);
pitch = uEst(5,1);
yaw = uEst(6,1);

% To compute the rotation matrix using the above angles
Rx = [1 0 0 ; 0 cos(roll) -sin(roll) ; 0 sin(roll) cos(roll)];
Ry = [cos(pitch) 0 sin(pitch) ; 0 1 0 ; -sin(pitch) 0 cos(pitch)];
Rz = [cos(yaw) -sin(yaw) 0 ; sin(yaw) cos(yaw) 0 ;0 0 1];

%Rotation of the body frame with respect to camera
Rb_camera = rotz(-45)*rotx(180);
Rc_body = Rb_camera';

Rb_world= (Rz*Ry*Rx);
Rw_body= Rb_world';

g0 = Rb_camera * Rw_body * uEst(7:9) - Rb_camera * skew_r_cb_b * Rc_body * omega' + vt;

%31 is 2*15 +1 
for i = 1:15 

    X_est_p = uEst + ((n_prime + lambda)^(1/2))*(covaraug(:,i)); % To calculate the state estimate with positive perturbation
    X_est_m = uEst - ((n_prime + lambda)^(1/2))*(covaraug(:,i)); % To calculate the state estimate with negative perturbation
    X_est_plus = [X_est_plus,X_est_p];
    X_est_minus= [X_est_minus, X_est_m];
    
end    

X_est = [uEst , X_est_plus , X_est_minus]; % To construct the X_est matrix

% To initialize the variables
g_plus =[];
g_minus = [];

for i = 1:15
    
    Rb_camera = rotz(-45)*rotx(180); % To define the rotation matrix for body to camera frame
    Rc_body = Rb_camera';

    roll_plus = X_est_plus(4,i);
    pitch_plus = X_est_plus(5,i);
    yaw_plus = X_est_plus(6,i);

    
    roll_minus = X_est_minus(4,i);
    pitch_minus = X_est_minus(5,i);
    yaw_minus = X_est_minus(6,i);


    Rx_plus = [1 0 0 ; 0 cos(roll_plus) -sin(roll_plus) ; 0 sin(roll_plus) cos(roll_plus)];
    Ry_plus = [cos(pitch_plus) 0 sin(pitch_plus) ; 0 1 0 ; -sin(pitch_plus) 0 cos(pitch_plus)];
    Rz_plus = [cos(yaw_plus) -sin(yaw_plus) 0 ; sin(yaw_plus) cos(yaw_plus) 0 ;0 0 1];

    Rb_world_plus = (Rz_plus*Ry_plus*Rx_plus);
    Rw_body_plus = Rb_world_plus';

    g_p = Rb_camera * Rw_body_plus * X_est_plus(7:9,i) - Rb_camera * skew_r_cb_b * Rc_body * omega' + vt;
    g_plus = [g_plus , g_p];

    Rx_minus = [1 0 0 ; 0 cos(roll_minus) -sin(roll_minus) ; 0 sin(roll_minus) cos(roll_minus)];
    Ry_minus = [cos(pitch_minus) 0 sin(pitch_minus) ; 0 1 0 ; -sin(pitch_minus) 0 cos(pitch_minus)];
    Rz_minus = [cos(yaw_minus) -sin(yaw_minus) 0 ; sin(yaw_minus) cos(yaw_minus) 0 ;0 0 1];

    Rb_world_minus = (Rz_minus*Ry_minus*Rx_minus);
    Rw_body_minus = Rb_world_minus';

    g_m = Rb_camera * Rw_body_minus * X_est_minus(7:9,i) - Rb_camera * skew_r_cb_b * Rc_body * omega' + vt;
    g_minus = [g_minus,g_m];

end

g = [g0,g_plus , g_minus]; % To construct the g matrix

% X_aug  = [u_augPrev , X_aug_plus , X_aug_minus];

% To calculate the weights and contributions based on g
W0_c = (lambda/(n_prime+lambda)) + (1-(alpha)^2 + beta);
Wi_c = 1/(2*(n_prime + lambda));

W0_m = lambda/(n_prime + lambda);
Wi_m = Wi_c;

% To calculate Ut0 and Uti
Ut0 = W0_m * g(:,1);
Uti = 0;

for i = 2:31
    Uti = Uti + Wi_m*g(:,i);
end
Ut = Ut0 + Uti;

% To calculate the values of S0 and S_i
S0 = W0_c*(g(:,1) - Ut) * (g(:,1) - Ut)' ;
S_i = 0;

for i = 2:31
    S_i = S_i+ Wi_c*(g(:,i) - Ut) * (g(:,i)-Ut)';
end
S = S0+S_i;

R = 0.5*eye(3);

% To add noice covariance matrix
S = S + R;

% To calculate the values of C0 and C_i
C0 = W0_c*(X_est(:,1) - uEst) * (g(:,1) - Ut)';
C_i = 0;

for i =2:31
    C_i =C_i +  Wi_c * (X_est(:,i) - uEst) * (g(:,i) - Ut)';
end

C = C0 + C_i;

Kt = C/S; % To calculate the Kalman Gain

% To update the current state and covariance
uCurr = uEst + Kt*(z_t' - Ut);
covar_curr = covarEst - Kt*S*Kt';

end

