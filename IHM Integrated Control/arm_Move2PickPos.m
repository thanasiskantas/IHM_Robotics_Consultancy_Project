function arm_Move2PickPos(x,y)
    
    %rosinit()
    
    chatpub = rospublisher('IHM_arm_control','std_msgs/String');
    startpub = rospublisher('Pick_position','std_msgs/Int8MultiArray');
    msg = rosmessage(chatpub);
    msg.Data = 'pick';
    send(chatpub,msg);
    pause(0.5);
    start_msg = rosmessage(startpub);
    start_msg.Data = [x,y];
    send(startpub,start_msg);
    disp("sent");
    %rosshutdown;
end