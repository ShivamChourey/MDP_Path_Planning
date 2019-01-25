function[] = MDP(goalIndex,goalOrientation, roboNumber)

global OptimalPolicy

row_size = 11;
column_size = 11;
gridSize = row_size*column_size;
gridResolution = 0.5 ;
num_actions = 8;
num_states = 8;
num_orientations = 8;
V  = -1 .* zeros(gridSize,num_orientations);
TP_temp = xlsread('TP.xlsx','Sheet1','A1:H8');
%V  = zeros(gridSize,num_orientations);
vel = 0.5;
actionSpace = [0,vel;
    -vel,vel;
    -vel,0;
    -vel,-vel;
    0,-vel;
    vel,-vel;
    vel,0;
    vel,vel];

%Obstacles = randi([0 gridSize],1,ceil(0.5*row_size));
Obstacles = [39,40,41,49,50,51,52,60,61,62,63,71,72,85];
goal_idx = find(Obstacles(:) == goalIndex);
Obstacles(goal_idx) = [];


%V = CreateObstacle(V,Obstacles);

V(goalIndex,goalOrientation) = 0;

V_pre = V ;

OptimalPolicy(goalIndex,goalOrientation, roboNumber) = 0;

for i=1:num_actions
    
    TransitionProbabilities(i,:) = zeros(1,num_states);
end

TransitionProbabilities = TP_temp;
Angle = 0:45:315;
eps = 1E5;
Conv = 1E-4;
while eps > Conv
    for idx = 1:gridSize
        for orient =1:num_orientations
            
            y_curr = (floor((idx-1)/row_size)) * gridResolution;
            x_curr = (round(mod((idx-1),row_size))) * gridResolution;
            
            tempV = zeros(num_actions,1);
            
            for action = 1:num_actions
                
                localAction = Transformation(orient) * actionSpace(action,:)';
                localAction = (localAction(:,:));
                for state = 1:num_states
                    check_extent =0;
                    next_state = zeros(2,1);
                    new_vector = Transformation(state) * localAction;
                    new_vector = (new_vector(:,:));
                    if new_vector == localAction
                        x_new = x_curr + localAction(1);
                        y_new = y_curr + localAction(2);
                        x_new = round(x_new/vel)*vel;
                        y_new = round(y_new/vel)*vel;
                    else
                        x_new = x_curr + new_vector(1);
                        y_new = y_curr + new_vector(2);
                        x_new = round(x_new/vel)*vel;
                        y_new = round(y_new/vel)*vel;
                    end
                    if (x_new <0 || x_new >= (row_size*gridResolution)) || (y_new <0 || y_new >= (column_size*gridResolution))
                        check_extent = 1;
                    end
                    
                    if check_extent ~= 1
                        
                        IsObstacle = 0;
                        next_state(1) = floor(y_new * row_size / gridResolution);
                        next_state(1) = next_state(1)+(x_new/gridResolution)+1;
                        
                        temp = int16((Angle(action)+Angle(state)+Angle(orient))/45)+1;
                        while temp >8
                            temp = temp-8;
                        end
                        
                        next_state(2) = temp;
                        
                        IsObstacle = CollisionDetect(Obstacles,x_new,y_new,row_size,gridResolution);
                        
                        if next_state(1) == goalIndex && next_state(2) == goalOrientation
                            Reward = 1;
                        elseif IsObstacle == 1
                            Reward = -10;
                        else
                            Reward = 0;
                        end
                        
                        tempV(action) = tempV(action) + TransitionProbabilities(action,state)*...
                            (Reward + V_pre(next_state(1),next_state(2)));
                        
                    end
                end
                
            end
            [compVal,maxIndex] = max(tempV);
            if compVal > V_pre(idx,orient)
                V(idx,orient) = compVal;
                OptimalPolicy(idx,orient, roboNumber) = maxIndex;
            end
            V(goalIndex,goalOrientation) = 0;
            OptimalPolicy(goalIndex,goalOrientation, roboNumber) = 0;
        end
    end
    eps = abs(max(max(V -V_pre)));
    V_pre =V;
end
end

