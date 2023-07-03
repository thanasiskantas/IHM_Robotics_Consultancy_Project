% servo: 0(left); 1(right), friction: 0(low), 1(high); 
% the torque is by default on when friction is low, remember to close when
% exiting

function xl320_controls(port_num, servo,friction)
    PROTOCOL_VERSION                = 2.0;
    
    ADDR_CONTROL_MODE               = 11;
    ADDR_PRO_TORQUE_ENABLE          = 24;           % Control table address is different in Dynamixel model
    ADDR_PRO_GOAL_POSITION          = 30; 
    ADDR_PRO_SPEED                  = 32;
    ADDR_PRESENT_SPEED              = 100;
    DXL_ID_3                        = 3;            % Dynamixel right xl320 white
    DXL_ID_4                        = 6;            % Dynamixel left xl320 red
    
    
    write2ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID_3,ADDR_CONTROL_MODE,2)
    write2ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID_4,ADDR_CONTROL_MODE,2)
    write2ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID_3,ADDR_PRO_SPEED,300)
    write2ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID_4,ADDR_PRO_SPEED,300)

    if (friction==0)
        % 90 degree low friction
        
        if (servo ==0)
           
            write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_PRO_TORQUE_ENABLE, 1);
            write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_PRO_GOAL_POSITION, 214);
            % pause(0.5);
        else
            write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_PRO_TORQUE_ENABLE, 1);
            write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_PRO_GOAL_POSITION, 513);
            % pause(0.5);
        end
    else
        % high friction
        if (servo ==0)
            write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_PRO_TORQUE_ENABLE, 1);
            write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_PRO_GOAL_POSITION, 520);
            % pause(0.5);
            % write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_3, ADDR_PRO_TORQUE_ENABLE, 0);
        else
            write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_PRO_TORQUE_ENABLE, 1);
            write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_PRO_GOAL_POSITION, 190);
            % pause(0.5);
            % write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_4, ADDR_PRO_TORQUE_ENABLE, 0);
        end
    end

end
