function mixer_matrix = adjust_motor_mixer(configuration)
    % Default Matrix (Standard X/H shapes)
                % Roll Pitch Yaw Thrust
    standard_mixer = [ 1 -1 -2  1; ... % w1
                      -1 -1  2  1; ... % w2
                       1  1  2  1; ... % w3
                      -1  1 -2  1];    % w4
    
    switch (configuration)
        case 1 % X Shape
            mixer_matrix = standard_mixer;
            
        case 2 % O Shape    
            % O-shape matrix: Reduced Pitch/Roll authority, Same Yaw for stability
            mixer_matrix = [ 1  1 -2  1; ... % w1 <- w3
                             1 -1  2  1; ... % w2 <- w1
                            -1  1  2  1; ... % w3 <- w4
                            -1 -1 -2  1];    % w4 <- w2
            
        case 3 % H Shape
            mixer_matrix = standard_mixer;
            
        case 4 % T Shape
            %                Roll   Pitch   Yaw   Thrust
            mixer_matrix = [ 1.0,  -1.0,   -2.0,   1.0; ... % w1: Front-Left (Wide)
                            -1.0,  -1.0,    2.0,   1.0; ... % w2: Front-Right (Wide)
                             0.2,   0.9,    2.0,   0.7; ... % w3: Rear-Left (Tail)
                            -0.2,   0.9,   -2.0,   0.7];    % w4: Rear-Right (Tail)

        case 5 % O Shape Alt
            mixer_matrix = [-1 -1 -2  1; ... % w1 <- w2
                            -1  1  2  1; ... % w2 <- w4
                             1 -1  2  1; ... % w3 <- w1
                             1  1 -2  1];    % w4 <- w3
            % -1 -1  1  1; ... % w1 <- w2
            %                 -1  1 -1  1; ... % w2 <- w4
            %                  1 -1 -1  1; ... % w3 <- w1
            %                  1  1  1  1];    % w4 <- w3

        case 6 % H Shape Alt
            mixer_matrix = standard_mixer;

        case 7 % T Shape Alt 
            mixer_matrix = [  0.2, -0.9, -2.0, 0.7; ... % w1 Front Left (CW)
                              0.2, -0.9,  2.0, 0.7; ... % w2 Front Right (CCW)
                              1.0,  1.0,  2.0, 1.0; ... % w3 Rear Left (CCW)
                             -1.0,  1.0, -2.0, 1.0];    % w4 Rear Right (CW)
        
        otherwise % Default
            mixer_matrix = standard_mixer;
       
    end
end