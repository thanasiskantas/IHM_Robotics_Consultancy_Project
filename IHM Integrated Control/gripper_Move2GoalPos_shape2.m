function [] = gripper_Move2GoalPos(start_coord,end_pos)
    
    % lib_name = '';
    % 
    % if strcmp(computer, 'PCWIN')
    %   lib_name = 'dxl_x86_c';
    % elseif strcmp(computer, 'PCWIN64')
    %   lib_name = 'dxl_x64_c';
    % elseif strcmp(computer, 'GLNX86')
    %   lib_name = 'libdxl_x86_c';
    % elseif strcmp(computer, 'GLNXA64')
    %   lib_name = 'libdxl_x64_c';
    % elseif strcmp(computer, 'MACI64')
    %   lib_name = 'libdxl_mac_c';
    % end
    % 
    % % Load Libraries
    % if ~libisloaded(lib_name)
    %     [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
    % end
    % 

    %% ---- Control Table Addresses ---- %%
    XM.ADDR_PRO_OPERATING_MODE         = 11;
    XM.ADDR_DRIVE_MODE                 = 10;
    XM.ADDR_MOVING_STATUS              = 123;
    XM.ADDR_PRO_TORQUE_ENABLE          = 64;  
    XM.ADDR_PRO_PRESENT_POSITION       = 132; 
    
    XM.ADDR_PRO_GOAL_POSITION          = 116;
    XM.ADDR_PROFILE_VELOCITY           = 112; %sets the time span to reach velocity of profile. When value is zero, infinite velocity.
    XM.ADDR_PROFILE_ACCELERATION       = 108; %sets accelerating time(t1) in millisecond[ms]. When value is zero, infinite acceleration
                                           %Profile Acceleration(108) will not exceed 50% of Profile Velocity(112) value
    XM.ADDR_VELOCITY_TRAJECTORY        = 136;
    XM.ADDR_POSITION_TRAJECTORY        = 140;
    
    XM.ADDR_MAX_POS_LIM                = 48;
    XM.ADDR_MIN_POS_LIM                = 52;
    
    XM.ADDR_GOAL_CURRENT               = 102;
    XM.ADDR_CURRENT_LIM                = 38;
    XM.ADDR_PRESENT_CURRENT            = 126;
    
    
    
    %% ---- Other Settings ---- %%
    
    TORQUE_CONTROL                  = 0;
    POS_CONTROL                     = 3;
    
    % Protocol version
    PROTOCOL_VERSION            = 2.0;          % See which protocol version is used in the Dynamixel
    
    % Default setting
    XM_ID1                      = 1;            % Dynamixel ID: 1
    XM_ID2                      = 4;
    XL_ID3                      = 3;
    XL_ID4                      = 6;
    DXL_ID_array = [XM_ID1,XM_ID2,XL_ID3,XL_ID4];
    
    BAUDRATE                    = 57600;
    DEVICENAME                  = '/dev/tty.usbserial-FT5WIZLL';       % Check which port is being used on your controller
                                                % ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'                                            
    TORQUE_ENABLE               = 1;            % Value for enabling the torque
    TORQUE_DISABLE              = 0;            % Value for disabling the torque
    
    %the next two lines have been swapped in value
    % DXL_MINIMUM_POSITION_VALUE  = 0;      % Dynamixel will rotate between this value
    % DXL_MAXIMUM_POSITION_VALUE  = 4095;       % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
    
    DXL_MOVING_STATUS_THRESHOLD = 20;     % Dynamixel moving status threshold
    ESC_CHARACTER               = 'e';          % Key for escaping loop
    
    COMM_SUCCESS                = 0;            % Communication Success result value
    COMM_TX_FAIL                = -1001;        % Communication Tx Failed
    
    
    
    %% Variables setting
    
    % current limit
    XM.MAX_CURRENT = 200;
    
    XM.MAX_POS = 3466;
    XM.MIN_POS = 500;
    
    % POS
    
    
    %% ------------------ %%
    
    % port_num = portHandler(DEVICENAME);
    % 
    % % Initialize PacketHandler Structs
    % packetHandler();
    % 
    % index = 1;
    % dxl_comm_result = COMM_TX_FAIL;           % Communication result
    % % dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE DXL_MAXIMUM_POSITION_VALUE];         % Goal position
    % 
    % dxl_error = 0;                              % Dynamixel error
    % 
    % % Open port
    % if (openPort(port_num))
    %     fprintf('Port Open\n');
    % else
    %     unloadlibrary(lib_name);
    %     fprintf('Failed to open the port\n');
    %     input('Press any key to terminate...\n');
    %     return;
    % end
    % 
    % % Set port baudrate
    % if (setBaudRate(port_num, BAUDRATE))
    %     fprintf('Baudrate Set\n');
    % else
    %     unloadlibrary(lib_name);
    %     fprintf('Failed to change the baudrate!\n');
    %     input('Press any key to terminate...\n');
    %     return;
    % end
    
    port_num = portHandler(DEVICENAME);
    
    % Initialize PacketHandler Structs
    packetHandler();
    
    index = 1;
    dxl_comm_result = COMM_TX_FAIL;           % Communication result
    % dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE DXL_MAXIMUM_POSITION_VALUE];         % Goal position
    
    dxl_error = 0;                              % Dynamixel error
    
    % Open port
    if (openPort(port_num))
        fprintf('Port Open\n');
    else
        unloadlibrary(lib_name);
        fprintf('Failed to open the port\n');
        input('Press any key to terminate...\n');
        return;
    end
    
    % Set port baudrate
    if (setBaudRate(port_num, BAUDRATE))
        fprintf('Baudrate Set\n');
    else
        unloadlibrary(lib_name);
        fprintf('Failed to change the baudrate!\n');
        input('Press any key to terminate...\n');
        return;
    end

    
    %% XM & XL
    
       
    % check ok
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end
    
    
    %% Variable setting
    
    low_fric = 0;
    high_fric = 1;
    red = 0;
    white = 1;
    
    start_pos = [2048,2048,high_fric,high_fric]; % [XM(white), XM(red), white-DL(index=1), red-DL(index=2)]
    
    %% get trajectory
    trajectory_angle_array = getTrajectory_shape2 (start_coord, end_pos);
    array = trajectory_angle_array;
    disp(array)
    
    sz = size(trajectory_angle_array);
    for move = 1:1:sz(1)
        if trajectory_angle_array(move,1) ~= 0
            trajectory_angle_array(move,1) = (3*pi/2 - trajectory_angle_array(move,1))*4096/(2*pi);
        elseif trajectory_angle_array(move,2) ~= 0
            trajectory_angle_array(move,2) = (pi/2+trajectory_angle_array(move,2))*4096/(2*pi);
        end
    end
    trajectory_angle_array = trajectory_angle_array;
    sz_traj_array = size(trajectory_angle_array);
    
    
    
    %% Start Movement
    disp("------------------Manipulation START-----------------")

    xm_torqueControl (port_num, DXL_ID_array(1), XM)
    xm_torqueControl (port_num, DXL_ID_array(2), XM)

    xl320_controls(port_num,white,trajectory_angle_array(1,3))
    xl320_controls(port_num,red,trajectory_angle_array(1,4))

    % rotate_index = 0;
    
    for move = 1:1:sz_traj_array(1)

        if trajectory_angle_array(move,1) == 0 && trajectory_angle_array(move,2) == 0
            disp("All moves are completed")
            break;
        end

        disp("Move number");
        disp(move);
    
        % if move == 1
        % 
        %     if trajectory_angle_array(move+1,1) == 0 && trajectory_angle_array(move+1,2) ~= 0
        %         torque_index = 1;
        %         xm_torqueControl (port_num, DXL_ID_array(torque_index), XM)
        %     elseif trajectory_angle_array(move+1,1) ~= 0 && trajectory_angle_array(move+1,2) == 0
        %         torque_index = 2;
        %         xm_torqueControl (port_num, DXL_ID_array(torque_index), XM)
        %     elseif trajectory_angle_array(move+1,1) == 0 && trajectory_angle_array(move+1,2) == 0
        %         disp("Next Move is ignored")
        %     end
        % 
        % 
        %     for i = 1:1:2
        %         if trajectory_angle_array(move+1,i) == 0
        %             disp("torque")
        %             torque_index = i;
        %             xm_torqueControl (port_num, DXL_ID_array(i), XM)
        %         else
        %             disp("pos")
        %             pos_index = i; 
        %             xm_posControl (port_num, DXL_ID_array(pos_index), XM, trajectory_angle_array(move,i))
        %             end_pos(pos_index) = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_array(pos_index), XM.ADDR_PRO_GOAL_POSITION);
        %             present_pos(pos_index) = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_array(pos_index), XM.ADDR_PRO_PRESENT_POSITION);
        %         end
        %     end
        % 
        % else
            
        % pos control is high friction finger
        if trajectory_angle_array(move,1) == 0 && trajectory_angle_array(move,2) ~= 0 % white torque, red pos
            if trajectory_angle_array(move,3) == high_fric && trajectory_angle_array(move,4) == high_fric % both high friction, rotate
                pos_index = 1;
                torque_index = 2;
                xm_torqueControl_rotate (port_num, DXL_ID_array(torque_index), XM)
                xm_posControl (port_num, DXL_ID_array(pos_index), XM, trajectory_angle_array(move,pos_index))
                rotate_index = 1;
            % elseif rotate_index == 1
            %     pos_index = 2;
            %     torque_index = 1;
            %     rotate_index = 0;
            %     xm_torqueControl_rotate (port_num, DXL_ID_array(torque_index), XM)
            %     xm_posControl (port_num, DXL_ID_array(pos_index), XM, trajectory_angle_array(move,pos_index))
            else
                pos_index = 2;
                torque_index = 1;
                xm_posControl (port_num, DXL_ID_array(pos_index), XM, trajectory_angle_array(move,pos_index))
            end
        elseif trajectory_angle_array(move,1) ~= 0 && trajectory_angle_array(move,2) == 0 % white pos, red torque
            if trajectory_angle_array(move,3) == high_fric && trajectory_angle_array(move,4) == high_fric % both high friction, rotate
                pos_index = 1;
                torque_index = 2;
                xm_torqueControl_rotate (port_num, DXL_ID_array(torque_index), XM)
                xm_posControl (port_num, DXL_ID_array(pos_index), XM, trajectory_angle_array(move,pos_index))
                rotate_index = 1;
            % elseif rotate_index == 1
            %     pos_index = 1;
            %     torque_index = 2;
            %     rotate_index = 0;
            %     xm_torqueControl_rotate (port_num, DXL_ID_array(torque_index), XM)
            %     xm_posControl (port_num, DXL_ID_array(pos_index), XM, trajectory_angle_array(move,pos_index))
            else
                pos_index = 1;
                torque_index = 2;
                xm_posControl (port_num, DXL_ID_array(pos_index), XM, trajectory_angle_array(move,pos_index))
            end
        end
        
        
        end_pos(pos_index) = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_array(pos_index), XM.ADDR_PRO_GOAL_POSITION);
        present_pos(pos_index) = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_array(pos_index), XM.ADDR_PRO_PRESENT_POSITION);
        disp("start position")
        disp(present_pos)
        disp("goal position")
        disp(end_pos)
        

        disp("wait to reach goal position")
        while abs(present_pos(pos_index) - trajectory_angle_array(move,pos_index)) > 20
            present_pos(pos_index) = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_array(pos_index), XM.ADDR_PRO_PRESENT_POSITION);
            disp(present_pos)
        end
        pause(2)
        
        
        disp("these should match")
        disp(present_pos(pos_index))
        disp(end_pos(pos_index))
        
    
        disp("---------Move Completed---------")
        
    
        disp("---------Prepare For Next Move---------")
        xl320_controls(port_num,white,high_fric)
        xl320_controls(port_num,red,high_fric)
        pause(1)

        prompt = "wait keyboard press";
        while input(prompt) == 1
        end

        if move < sz_traj_array(1)
            if trajectory_angle_array(move+1,1) == 0 && trajectory_angle_array(move+1,2) ~= 0
                torque_index = 1;
                xm_torqueControl (port_num, DXL_ID_array(torque_index), XM)
                write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_array(2), XM.ADDR_PRO_OPERATING_MODE, POS_CONTROL);
            elseif trajectory_angle_array(move+1,1) ~= 0 && trajectory_angle_array(move+1,2) == 0
                torque_index = 2;
                xm_torqueControl (port_num, DXL_ID_array(torque_index), XM)
                write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_array(1), XM.ADDR_PRO_OPERATING_MODE, POS_CONTROL);
            elseif trajectory_angle_array(move+1,1) == 0 && trajectory_angle_array(move+1,2) == 0
                disp("Next Move is ignored") 
                break;
            end
    
            if trajectory_angle_array(move+1,3) == low_fric && trajectory_angle_array(move+1,4) == high_fric
                xl320_controls(port_num,white,low_fric)
                xl320_controls(port_num,red,high_fric)
                pause(1)
                disp("Next Move is: SLIDE")
            elseif trajectory_angle_array(move+1,3) == high_fric && trajectory_angle_array(move+1,4) == low_fric
                xl320_controls(port_num,red,low_fric)
                xl320_controls(port_num,white,high_fric)
                pause(1)
                disp("Next Move is: SLIDE")
            elseif trajectory_angle_array(move+1,3) == high_fric && trajectory_angle_array(move+1,4) == high_fric
                pause(1)
                disp("Next Move is: ROTATE")
            end
        end
    
    end


    
    disp("---------TASK ENDS---------")
    
    
    
    
    %%  check ok
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end
    
    disp("Torque disable in 2s")
    
    
    
    %% Disable Dynamixel Torque
    

end

