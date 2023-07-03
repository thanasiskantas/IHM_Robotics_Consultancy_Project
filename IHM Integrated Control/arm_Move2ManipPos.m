function [msg] = arm_Move2ManipPos()
    %rosinit()
    
    chatpub = rospublisher('IHM_arm_control','std_msgs/String');
    msg = rosmessage(chatpub);
    msg.Data = 'manipulate';
    send(chatpub,msg);
    recpub = rossubscriber('IHM_start_pos','std_msgs/Float32');
    % pause(0.5);
    msg = receive(recpub).Data;
    %rosshutdown;
end