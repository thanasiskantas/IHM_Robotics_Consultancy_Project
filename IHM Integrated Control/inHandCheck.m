function [] = inHandCheck_thread ()
    % thread function
    % conduct a serious of check either through vision or through gripper 
    
    % Create a thread using parfeval
    thread = parfeval(@inHandCheck_thread, 0);

    % Wait for the thread to complete (optional)
    wait(thread);
end


function inHandCheck()
    disp('Executing thread function...');
    % Thread function implementation
    % ...
end
