function [dxl_error] = gripper_release()

    lib_name = '';

    if strcmp(computer, 'PCWIN')
      lib_name = 'dxl_x86_c';
    elseif strcmp(computer, 'PCWIN64')
      lib_name = 'dxl_x64_c';
    elseif strcmp(computer, 'GLNX86')
      lib_name = 'libdxl_x86_c';
    elseif strcmp(computer, 'GLNXA64')
      lib_name = 'libdxl_x64_c';
    elseif strcmp(computer, 'MACI64')
      lib_name = 'libdxl_mac_c';
    end
    
    % Load Libraries
    if ~libisloaded(lib_name)
        [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
    end
    
    
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

    
    %% XM & XL      
    % Initialize PortHandler Structs
    % Set the port path
    % Get methods and members of PortHandlerLinux or PortHandlerWindows
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

    
    %% get trajectory
    posControl (port_num, DXL_ID_array(1), XM, 2348)
    posControl (port_num, DXL_ID_array(2), XM, 1748)
    pause(1)
    xl320_controls(port_num,red,low_fric)
    xl320_controls(port_num,white,low_fric)
    pause(2)
    



    %% torque disable
    disp("torque disabling")
    for i = 1:1:2
        write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_array(i), XM.ADDR_PRO_TORQUE_ENABLE, 0);
    end
    xl320_disable(port_num)
    

    
    %%  check ok
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end
    
    % Close port
    closePort(port_num);
    fprintf('Port Closed \n');
    
    % Unload Library
    unloadlibrary(lib_name);
    
    close all;

end

    
function torqueControl (port_num, ID, XM)

    PROTOCOL_VERSION                = 2.0;          % See which protocol version is used in the Dynamixel
    TORQUE_CONTROL                  = 0;
    
    if ID == 1
        % goal_torque = 0xFFEC;%-25
        goal_torque = 0xFFEC; %-20
    elseif ID == 4
        goal_torque = 0x0020;
        %goal_torque = 0x0015;
    end
    
    write1ByteTxRx(port_num, PROTOCOL_VERSION, ID, XM.ADDR_PRO_TORQUE_ENABLE, 0);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, ID, XM.ADDR_PRO_OPERATING_MODE, TORQUE_CONTROL);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, ID, XM.ADDR_CURRENT_LIM, XM.MAX_CURRENT);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, ID, XM.ADDR_PRO_TORQUE_ENABLE, 1);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, ID, XM.ADDR_GOAL_CURRENT, goal_torque);
end

function posControl (port_num, ID, XM, goal_pos)

    
    PROTOCOL_VERSION                = 2.0;          % See which protocol version is used in the Dynamixel
    POS_CONTROL                     = 3;
    total_move_time                 = 500;
    acceleration_time               = 1;
    %Drive Mode(10) is time-based profile
    DRIVE_MODE_TIME                 = 4; %'0b00000100'
    %Drive Mode(10) is velocity-based profile
    DRIVE_MODE_VELOCITY             = 0; %'0b00000000'
    
    % 需不需要torque disable
    write1ByteTxRx(port_num, PROTOCOL_VERSION, ID, XM.ADDR_PRO_TORQUE_ENABLE, 0);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, ID, XM.ADDR_PRO_OPERATING_MODE, POS_CONTROL);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, ID, XM.ADDR_PRO_TORQUE_ENABLE, 1);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, ID, XM.ADDR_MAX_POS_LIM, XM.MAX_POS);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, ID, XM.ADDR_MIN_POS_LIM, XM.MIN_POS);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, ID, XM.ADDR_DRIVE_MODE, DRIVE_MODE_TIME);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, ID, XM.ADDR_PROFILE_VELOCITY, total_move_time);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, ID, XM.ADDR_PROFILE_ACCELERATION, acceleration_time);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, ID, XM.ADDR_PRO_GOAL_POSITION, goal_pos);            
end

function xl320_disable (port_num)
    PROTOCOL_VERSION            = 2.0;

    ADDR_PRO_TORQUE_ENABLE       = 24;           % Control table address is different in Dynamixel model
    ADDR_PRO_GOAL_POSITION       = 30; 
    DXL_ID_3                      = 3;            % Dynamixel right xl320 white
    DXL_ID_4                      = 6;            % Dynamixel left xl320 red

    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_PRO_TORQUE_ENABLE, 0);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_PRO_TORQUE_ENABLE, 0);
end