function arm_Move2PlacePos(goal_position)
    %rosinit()
    
    chatpub = rospublisher('IHM_arm_control','std_msgs/String');
    goalpub = rospublisher('Goal_position','std_msgs/Float32');
    msg = rosmessage(chatpub);
    msg.Data = 'place';
    send(chatpub,msg);
    pause(0.5);
    goalmsg = rosmessage(goalpub);
    goalmsg.Data = goal_position;
    send(goalpub,goalmsg);
    %rosshutdown;
end