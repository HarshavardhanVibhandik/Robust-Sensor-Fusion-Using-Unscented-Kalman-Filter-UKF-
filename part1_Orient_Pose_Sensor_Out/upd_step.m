function [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst)
%% BEFORE RUNNING THE CODE CHANGE NAME TO upd_step
    %% Parameter Definition
    %z_t - is the sensor data at the time step
    %covarEst - estimated covar of the  state
    %uEst - estimated mean of the state
    
% I = zeros(6,15);
% I(1:6,1:6)= eye(6,6);

% To define the measurement matrix
I = [eye(3),zeros(3),zeros(3),zeros(3),zeros(3);zeros(3),eye(3),zeros(3),zeros(3),zeros(3)];
z = I * uEst + zeros(6,1); % To define the predicted measurement

Ct = I ; % Measurement matrix 

% Wt = eye(6,6);                                                                      

% To define the measurement noise covariance matrix
R = eye(6,6) * 0.1;

% To calculate the Kalman gain
Kt = covarEst * Ct' /(Ct * covarEst* Ct' + R );

% To update the estimated state vector
uCurr = uEst + Kt * (z_t - z );

% To update the covariance matrix
covar_curr = covarEst- Kt* Ct * covarEst;

end

