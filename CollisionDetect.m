function obs_flag = CollisionDetect(obstacles,x_new,y_new,row_size,gridResolution)
                        
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
obs_flag = 0;
for i=1:length(obstacles)
    idx = obstacles(i);
    y_obs = (floor((idx-1)/row_size)) * gridResolution;
    x_obs = (round(mod((idx-1),row_size))) * gridResolution;
             
    if (x_new == x_obs) && (y_new == y_obs) 
        obs_flag = 1;
    end
end
        
end

