function [next_angle] = GetAction(x_curr,y_curr,curr_angle, roboNumber)

global OptimalPolicy;

r_size = 11;
gridResolution = 0.5;
x_curr = round(x_curr/gridResolution)*gridResolution;
y_curr = round(y_curr/gridResolution)*gridResolution;


curr_idx = floor(y_curr * r_size / gridResolution);
curr_idx = curr_idx+(x_curr/gridResolution)+1;
curr_angle = round(curr_angle/45)*45;
if(curr_angle > 315)
    curr_angle = 0;
end

Angle = 0:45:315;

curr_angle_idx = find(Angle(:)==curr_angle);

next_angle_idx= OptimalPolicy(curr_idx,curr_angle_idx, roboNumber);
next_angle = (next_angle_idx-1)*45;


end