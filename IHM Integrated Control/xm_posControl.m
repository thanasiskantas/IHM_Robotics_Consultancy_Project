function xm_posControl (port_num, ID, XM, goal_pos)
    
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