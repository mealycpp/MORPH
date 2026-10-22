function [kp, ki, kd, filD, limit] = set_pitch_pid(configuration)
    % filtM_attitude = 0.01;
    % kp_attitude    = 128.505;
    % ki_attitude    = 5.9203;
    % kd_attitude    = 78.2000*2;
    % filtD_attitude = 1000;
    % limit_attitude = 800;

    switch(configuration)
        case 1 % X
            kp = 98.505;
            ki = 5.9203;
            kd = 100.2 * 2;
            filD = 1000;
            limit = 800;
        case 2 % O
            kp = 128.505;
            ki = 5.9203;
            kd = 78.2 * 2;
            filD = 1000;
            limit = 800;
        case 3 % H
            kp = 118.505;
            ki = 8.9203;
            kd = 88.2 * 2;
            filD = 1000;
            limit = 800;
        case 4 % T
            kp = 58.505;
            ki = 5.9203;
            kd = 78.2 * 2;
            filD = 1000;
            limit = 300;
        otherwise
            kp = 128.505;
            ki = 5.9203;
            kd = 78.2 * 2;
            filD = 1000;
            limit = 800;
    end
end