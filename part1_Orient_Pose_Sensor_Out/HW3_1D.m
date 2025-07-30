% To define the point in image 1 (homogeneous coordinates)
point1 = [60; 80; 1];

% To define a suitable homography matrix H that will transform point1 to approximately [210, 220, 1]
H = [3.5, 0, 0;
     0, 2.75, 0;
     0, 0, 1];

% To apply the homography matrix to the point
point2_estimated = H * point1;

% To convert from homogeneous coordinates back to Cartesian coordinates
point2_estimated = point2_estimated / point2_estimated(3);

% To display the transformed point
disp('Estimated Point 2 in Image 2:');
disp(point2_estimated);
