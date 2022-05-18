function robotSuck(state)
    % Turn on the tool
    %%
    [toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
    toolStateMsg.Data = [state]; % Send 1 for on and 0 for off 
    send(toolStatePub,toolStateMsg);
end