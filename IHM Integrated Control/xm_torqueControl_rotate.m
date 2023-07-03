function xm_torqueControl_rotate (port_num, ID, XM)

    PROTOCOL_VERSION                = 2.0;          % See which protocol version is used in the Dynamixel
    TORQUE_CONTROL                  = 0;
    
    if ID == 1
        % goal_torque = 0xFFEC;%-25
        goal_torque = 0xFFFB; %-20
    elseif ID == 4
        goal_torque = 0x0005;
        %goal_torque = 0x0015;
    end
    
    write1ByteTxRx(port_num, PROTOCOL_VERSION, ID, XM.ADDR_PRO_TORQUE_ENABLE, 0);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, ID, XM.ADDR_PRO_OPERATING_MODE, TORQUE_CONTROL);
    write4ByteTxRx(port_num, PROTOCOL_VERSION, ID, XM.ADDR_CURRENT_LIM, XM.MAX_CURRENT);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, ID, XM.ADDR_PRO_TORQUE_ENABLE, 1);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, ID, XM.ADDR_GOAL_CURRENT, goal_torque);
end