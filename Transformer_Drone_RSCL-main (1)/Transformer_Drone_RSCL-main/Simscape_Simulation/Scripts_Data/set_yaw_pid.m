function [kp, ki, kd, filtD, limit] = set_yaw_pid(configuration)
    % filtM_attitude = 0.01;
    % kp_attitude    = 128.505;
    % ki_attitude    = 5.9203;
    % kd_attitude    = 78.2000*2;
    % filtD_attitude = 1000;
    % limit_attitude = 800;

    switch(configuration)
        case 1 % X
            filtM      = 0.01;
            kp         = 25.7010*4*2;
            ki         = 5.9203*0.01;
            kd         = 78.2000*0.01;
            filtD      = 100;
            limit      = 20;
        case 2 % O
            filtM      = 0.01;
            kp         = 25.7010*4*2;
            ki         = 5.9203*0.01;
            kd         = 78.2000*0.01;
            filtD      = 100;
            limit      = 20;
        case 3 % H
            filtM      = 0.01;
            kp         = 25.7010*4*2;
            ki         = 5.9203*0.01;
            kd         = 78.2000*0.01;
            filtD      = 100;
            limit      = 20;
        case 4 % T
            filtM      = 0.01;
            kp         = 25.7010*4*2;
            ki         = 5.9203*0.01;
            kd         = 78.2000*0.01;
            filtD      = 100;
            limit      = 20;
        otherwise
            filtM      = 0.01;
            kp         = 25.7010*4*2;
            ki         = 5.9203*0.01;
            kd         = 78.2000*0.01;
            filtD      = 100;
            limit      = 20;
    end
end