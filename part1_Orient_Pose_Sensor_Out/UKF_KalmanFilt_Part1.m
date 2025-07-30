clear; % Clear variables
addpath('../data')
datasetNum = 4; % CHANGE THIS VARIABLE TO CHANGE DATASET_NUM %It specifies the dataset number to be used
[sampledData, sampledVicon, sampledTime,proj2Data] = init(datasetNum); % sampledData: resampled sensor data collected from IMU %sampledVicon: resampled Vicon data which includes info about Velocity, position, orientation % sampledTime: timestamps corresponding to each sample in sampledData and sampledVicon

Z = sampledVicon(1:6,:); %All the measurements that you need for the update % It selects the first 6 rows of sampledVicon
% Set initial condition

uPrev = vertcat(sampledVicon(1:9,1),zeros(6,1)); % Copy the Vicon Initial state % Vector that will contain the irst 9 rows of sampledVicon and the zero vector with length 6
covarPrev = 0.1*eye(15); % Covariance constant %It initilizes the covariance matrix to be an Identity matrix of size 15x15
savedStates = zeros(15, length(sampledTime)); %Matrix with 15 rows and number of columns = lenght of sampledTime
prevTime = 0; %Variable to store the previous time step which will be used to track the time progression 

pos = proj2Data.position; %To extract the position data
pose = proj2Data.angle; %To extract the orientation data

for i = 1:length(sampledTime) % This is For loop that iterates over each time step in the sampledTime
    %% Fill in the FOR LOOP
   dt  = sampledTime(i) - prevTime; % To calculate time step %Substract previous time from the current time
   angVel= sampledData(i).omg; %To retrive the angular velocity from IMU data
   acc = sampledData(i).acc; %To retrive linear acceleration from IMU data
   currTime = sampledTime(i); %To extract the current timestamp from the sampleddata 
   prevTime = currTime; %To update the previous time with the current
   z_t = Z(:,i); %Retrive measurement data at current time step

   % The Prediction step
   [covarEst,uEst] =  pred_step(uPrev,covarPrev,angVel,acc,dt); % To update the state and covariance based on the measurement data and the predicted state and covariance

   % The Update step
   [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst); % To updates the state and covariance based on the measurement data and the predicted state and covariance

   uPrev = uCurr; % To update the preious state for the next iteration
   covarPrev = covar_curr; % To update the previous covariance for the next iteration
   
   savedStates(:,i) = uCurr; %To save current state at time step i
end

plotData(savedStates, sampledTime, sampledVicon, 1, datasetNum); % Plot the saved states against time along with the sampledVicon data
