vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);

row_size = 22;
column_size = 22;
gridSize = row_size*column_size;
num_orientations = 8;
global OptimalPolicy
OptimalPolicy = zeros(gridSize,num_orientations,2);

if(clientID ~= -1)
    disp('Connected to VRep');
    
    % Now try to retrieve data in a blocking fashion (i.e. a service call):
    [res,objs]=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_blocking);
    
    %% Orientation
    % 1 - 0degree, 2 - 45degree ......... , 8 - 315 degree
    %% Index - 1 to 121 --- grid indices
    goalIndex1 = 242;
    goalOrientation1 = 5;
    goalAngle1 = (goalOrientation1-1) * 45;
    
    botHeight = 0.1386;
    
    r_size = 22;
    gridResolution = 0.5;
    x_goal1 = (round(mod((goalIndex1-1),r_size))) * gridResolution;
    y_goal1 = (floor((goalIndex1-1)/r_size)) * gridResolution;
    goalPosition1 = [x_goal1, y_goal1, botHeight];
    
    goalIndex2 = 16;
    goalOrientation2 = 5;
    goalAngle2 = (goalOrientation2-1) * 45;
    x_goal2 = (round(mod((goalIndex2-1),r_size))) * gridResolution;
    y_goal2 = (floor((goalIndex2-1)/r_size)) * gridResolution;
    goalPosition2 = [x_goal2, y_goal2, botHeight];
    
    %% Get the optimal policy through MDP
    MDP(goalIndex1,goalOrientation1, 1);
%     MDP(goalIndex2,goalOrientation2, 2);
    
    % start the simulation:
    vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);
    
    %% Get reference frame handle
    [returnCode,RefFrame]=vrep.simxGetObjectHandle(clientID,'ReferenceFrame',vrep.simx_opmode_blocking);
    [returnCode,RefFrameBody1]=vrep.simxGetObjectHandle(clientID,'ReferenceFrame0',vrep.simx_opmode_blocking);
%     [returnCode,RefFrameBody2]=vrep.simxGetObjectHandle(clientID,'ReferenceFrame1',vrep.simx_opmode_blocking);
    
    %% Get the bot handle
    [returnCode,BotHandle1]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx#0',vrep.simx_opmode_blocking);
%     [returnCode,BotHandle2]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_blocking);
    
    %% Get motor handles  
    [returnCode,LeftMotorHandle1]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor#0',vrep.simx_opmode_blocking);
    [~,RightMotorHandle1]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor#0',vrep.simx_opmode_blocking);
%     
%     [returnCode,LeftMotorHandle2]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking);
%     [~,RightMotorHandle2]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking);
%     
    %% Get the current position and orientation of the robot
    [returnCode,position1]=vrep.simxGetObjectPosition(clientID, BotHandle1, RefFrame, vrep.simx_opmode_blocking);
    [returnCode,eulerAngles1]=vrep.simxGetObjectOrientation(clientID,RefFrameBody1,RefFrame,vrep.simx_opmode_blocking);
        
    currentPosition1 = position1;
    currentEulerAngles1 = eulerAngles1;
    
%     [returnCode,position2]=vrep.simxGetObjectPosition(clientID, BotHandle2, RefFrame, vrep.simx_opmode_blocking);
%     [returnCode,eulerAngles2]=vrep.simxGetObjectOrientation(clientID,RefFrameBody2,RefFrame,vrep.simx_opmode_blocking);
% 
%     currentPosition2 = position2;
%     currentEulerAngles2 = eulerAngles2;
    
    ReachedGoal1 = false;
    ReachedGoal2 = false;
    %% Loop till we reach at goal
    while ReachedGoal1 ~= true %|| ReachedGoal2 ~= true
        
        if(ReachedGoal1 ~= true)
            [ReachedGoal1, currentPosition1, currentEulerAngles1] = MoveRobot(clientID, vrep, BotHandle1, LeftMotorHandle1, RightMotorHandle1, currentPosition1, currentEulerAngles1, goalPosition1, goalAngle1, 1, RefFrame, RefFrameBody1);
        end
        
%         if(ReachedGoal2 ~= true)
%             [ReachedGoal2, currentPosition2, currentEulerAngles2] = MoveRobot(clientID, vrep, BotHandle2, LeftMotorHandle2, RightMotorHandle2, currentPosition2, currentEulerAngles2, goalPosition2, goalAngle2, 2, RefFrame, RefFrameBody2);
%         end

    end
    
    %% When complete stop the simulation
    vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);
    
    %% Noisy GPS not used
    %     [res,data]=vrep.simxGetStringSignal(clientID,'myPts',vrep.simx_opmode_streaming);
    %     while(1)
    %         %% Get the GPS Data
    %         vrep.simxGetStringSignal(clientID,'myGpsData',vrep.simx_opmode_streaming); % Initialize streaming
    %         while (1)
    %             [returnCode,data]=vrep.simxGetStringSignal(clientID,'myGpsData',vrep.simx_opmode_buffer); % Try to retrieve the streamed data
    %             if (returnCode==vrep.simx_return_ok) % After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
    %                 gpsData=vrep.simxUnpackFloats(data);
    %                 %% Get the position of robot
    %                 [returnCode,BotHandle]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx#0',vrep.simx_opmode_blocking);
    %                 [returnCode,position]=vrep.simxGetObjectPosition(clientID, BotHandle, RefFrame, vrep.simx_opmode_blocking);
    %
    %             end
    %         end
    %     end
    %
    
    vrep.simxFinish(clientID);
end


function [ReachedGoal, currentPosition, currentEulerAngles] = MoveRobot(clientID, vrep, BotHandle, LeftMotorHandle, RightMotorHandle, currentPosition, currentEulerAngles, goalPosition, goalAngle, roboNumber, RefFrame, RefFrameBody)

       if abs(currentPosition(1)-goalPosition(1)) < 0.3 && abs(currentPosition(2)-goalPosition(2)) < 0.3 && abs(goalAngle-((currentEulerAngles(3)*180)/pi)) < 25
           ReachedGoal = false;
           return;
       end
           
        % calculate the orientation
        orientation = (currentEulerAngles(3) * 180) / pi;
        RotationTimeConstant = 30;   % degrees / sec
        
        RightTurn = false;
        LeftTurn = false;
        
        StraighMotionFactor = 1;
        
        % Get the angle by which we need to turn
        deltaOrientation = GetAction(currentPosition(1), currentPosition(2), orientation, roboNumber);
        
        newOrientation = deltaOrientation + orientation;
        newOrientation = round(newOrientation/45)*45;
        Isdiagonal = false;
        if newOrientation == 45 || newOrientation == 135 || newOrientation == 225 || newOrientation == 315
            StraighMotionFactor = 1.414;
        end
        
        
        if(deltaOrientation < 180)
            LeftTurn = true;
        else
            RightTurn = true;
            deltaOrientation = 360-deltaOrientation;
        end
        
        time = 1.8 * deltaOrientation / RotationTimeConstant ;
        Speed = 0.5;
        
        if (RightTurn == true)
            %% Zero-radius right turn
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID,LeftMotorHandle,Speed,vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID,RightMotorHandle,-Speed,vrep.simx_opmode_blocking);
            
            pause(time);
            
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID,LeftMotorHandle,0,vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID,RightMotorHandle,0,vrep.simx_opmode_blocking);
            
            [returnCode,eulerAngles]=vrep.simxGetObjectOrientation(clientID,BotHandle,RefFrame,vrep.simx_opmode_blocking);
        end
        if (LeftTurn == true)
            %% Zero-radius left turn
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID,LeftMotorHandle,-Speed,vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID,RightMotorHandle,Speed,vrep.simx_opmode_blocking);
            
            pause(time);
            
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID,LeftMotorHandle,0,vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID,RightMotorHandle,0,vrep.simx_opmode_blocking);
            
            [returnCode,eulerAngles]=vrep.simxGetObjectOrientation(clientID,BotHandle,RefFrame,vrep.simx_opmode_blocking);
            
        end
        
        %% Straight motion
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,LeftMotorHandle,Speed,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,RightMotorHandle,Speed,vrep.simx_opmode_blocking);
        
        pause(11 * StraighMotionFactor);
        
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,LeftMotorHandle,0,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,RightMotorHandle,0,vrep.simx_opmode_blocking);
        
        %% Get the  current position and orientation of the robot
        [returnCode,currentPosition]=vrep.simxGetObjectPosition(clientID, BotHandle, RefFrame, vrep.simx_opmode_blocking);

        [returnCode,currentEulerAngles]=vrep.simxGetObjectOrientation(clientID,RefFrameBody, RefFrame,vrep.simx_opmode_blocking);
        if (currentEulerAngles(3) < 0)
            currentEulerAngles(3) = currentEulerAngles(3) + (2*pi);
        end

       if abs(currentPosition(1)-goalPosition(1)) > 0.3 || abs(currentPosition(2)-goalPosition(2)) > 0.3 || abs(goalAngle-((currentEulerAngles(3)*180)/pi)) > 25
           ReachedGoal = false;
       else
           ReachedGoal = true;
       end
end