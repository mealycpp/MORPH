function [waypoints, timespot_spl, spline_data, spline_yaw, wayp_path_vis] = quadcopter_package_select_trajectory(path_number,varargin)
%quadcopter_select_trajectory Obtain parameters for selected quadcopter trajectory
%   [waypoints, timespot_spl, spline_data, spline_yaw] = quadcopter_select_trajectory(path_number)
%   This function returns the essential parameters that define the
%   quadcopter's trajectory. The function returns
%
%       waypoints       Key x-y-z locations the quadcopter will pass through
%       timespot_spl    Times the quadcopter will pass through points along
%                       the spline that defines its path
%       spline_data     Points used for interpolating the spline that
%                       defines the path of the quadcopter
%       spline_yaw      Yaw angle at the spline_data points

% Copyright 2021-2025 The MathWorks, Inc.

if(nargin == 2)
    roundtrip = varargin{1};
else
    roundtrip = false;
end

switch (path_number)
    case 1
        waypoints = [ ...
            -2    -2 0 2 5
            -2    -2 0 0 0
            0.14   6 6 6 0.14];
        max_speed = 1;
        min_speed = 0.1;
        xApproach = [4 0.5];
        vApproach = 0.1;

    case 2
        waypoints = [...
            -2    -2  -2  -2 -2  2  2
            -2    -2  -2   2  2  2  2
            0.15  6   6   6  6  6  0.15];
        max_speed = 1;
        min_speed = 0.1;
        xApproach = [2 0.5];
        vApproach = 0.1;

    case 3
        % Note: This trajectory defines the waypoints, spline data, and yaw
        % data explicitly and does not use the function to calculate the
        % target speed and yaw angle based on the path.
        waypoints = [...
            -2   -2    -2     -2  -2  -2 -2  -2  -2 -2 -2 -2 -2 -2
            -2   -2    -2     -2  -2  -2 -2  -2  -2 -2 -2 -2  0  0
            0.15 0.15   0.15   4   4   4  4   4   4  4  4  4  4  0.14];
        spline_data = waypoints';
        timespot_spl = [0:4:11*4 11*4+6 11*4+6+6]';
        spline_yaw   = [0 0 0 pi/4 pi/4 pi/4 0 0 0 -pi/4 -pi/4 -pi/4 -pi/4 -pi/4];

    case 4
        waypoints = [...
            -3.0000    0.5633    4.5492    7.7662    9.0011    7.3491    3.7145   -0.0156    2.2687    5.0000
            -5.0000   -4.4724   -4.5758   -2.3910    1.5272    5.3013    6.5986    6.8774    9.5797    8.0000
            0.1500    6.0000    6.0000    6.0000    6.0000    6.0000    6.0000    6.0000    6.0000    0.15];
        max_speed = 1;
        min_speed = 0.1;
        xApproach = [4 0.5];
        vApproach = 0.1;

    case 5 % Manual Timing Obstacle Course (30s Hold)
        % config = [1 6 2 1 6 4 1 2]
        % hold time = 30 sec
        wayp_table = [ ...
            % X      Y      Z      
            0.0,    0.0,    0.15;  % T=0   Station 1: Start (X)
            
            0.0,    0.0,    2.00;  % T=30  Station 2: Between walls (X-H)
            0.0,    0.0,    2.00;  % T=35  Buffer
            0.0,    0.0,    2.00;  % T=40  Buffer
            
            3.0,    0.0,    2.00;  % T=60  Station 3: Up rings (H-O)
            3.0,    0.0,    2.00;  % T=65  Buffer (Rotation Start)
            3.0,    0.0,    2.00;  % T=70  Buffer
            
            3.0,    0.0,    4.00;  % T=120 Station 4: X-H shape other walls
            3.0,    0.0,    4.00;  % T=125 Buffer
            3.0,    0.0,    4.00;  % T=130 Buffer
            
            3.0,    2.5,    4.00;  % T=150 Station 5: H-T shape
            3.0,    3.0,    4.00;  % T=155 Buffer
            3.0,    3.0,    4.00;  % T=160 Buffer
            
            3.0,    3.0,    4.00;  % T=180 Station 6: T-X Buffer
            3.0,    3.0,    4.00;  % T=185 Buffer
            3.0,    2.5,    4.00;  % T=190 Buffer
            
            3.0,    4.0,    3.00;  % T=210 Station 7: X-O shape
            3.0,    4.0,    3.00;  % T=215 Buffer
            3.0,    4.0,    3.00;  % T=220 Buffer
            
            3.0,    4.0,    0.14]; % T=230 Station 8: Landing (O)
    
        waypoints = wayp_table';
        spline_data = wayp_table;
        % Synchronized Timestamps
        timespot_spl = [0, ...                 % S1
                        30, 35, 40, ...        % S2
                        60, 65, 70, ...        % S3
                        90, 95, 100, ...       % S4
                        120, 125, 130, ...     % S5
                        150, 155, 160, ...     % S6
                        180, 185, 190, ...     % S7
                        210]';                 % S8
    
        % Yaw Logic:
        % S1-S2: Face 0 rad
        % S3: Rotate to pi/3
        % S4-S9: Face pi/2
        spline_yaw = [ ...
            0, ...                           % Station 1
            0, 0, 0, ...                     % Station 2
            0, pi/4, pi/4, ...               % Station 3 (Rotate)
            pi/4, pi/2, pi/2, ...            % Station 4
            pi/2, pi/2, pi/2, ...            % Station 5
            pi/2, pi/2, pi/2, ...            % Station 6
            pi/2, pi/2, pi/2, ...            % Station 7
            pi/2]';                          % Station 8
            
    case 6 
        waypoints = [ ...
            0   10   20   25   35   45   55   65   75   85   95; % x
            0    0    0   10   20   20   20   10    0    0    0; % y
            0.15 6    6    6    6    6    6    6    6    6    0.15]; % z
       
        max_speed = 1.2;
        min_speed = 0.2; 
        xApproach = [5 1]; 
        vApproach = 0.1;

    case 7
        % Path 1 handles the linear paths
        % Path 2 handles the circular paths
        scale = 3.0;
        xpathL = [0   0  0  0   0  -scale  scale];
        ypathL = [0   0  scale  -scale  0  0   0];
        zpathL = [.15 scale  scale  scale   scale  scale   scale];

        % Change smoothness of rotation path
        % 1 = Linear pathing from max x and y values
        rotation_quality = 6;
        
        xpathCCW = [];
        ypathCCW = [];
        
        for step = 1:rotation_quality * 4
            xpathCCW = [xpathCCW scale*cos(step*pi/(2*rotation_quality))];
            ypathCCW = [ypathCCW scale*sin(step*pi/(2*rotation_quality))];
        end
        [~, points] = size(xpathCCW);
        zpathRotate = scale*ones(1, points);
        [xpathCCW flip(xpathCCW(:, 1:points-1))];

        waypoints = [ ...
            xpathL xpathCCW    flip(xpathCCW(:, 1:points-1))  scale  scale
            ypathL ypathCCW    flip(ypathCCW(:, 1:points-1))  0   0
            zpathL zpathRotate zpathRotate(:, 2:points)       scale  0.15];
        max_speed = 0.5;
        min_speed = 0.3;
        xApproach = [4 1];
        vApproach = 0.1;
end

% Only call the function to calculate target speed and yaw angle if needed
% Paths that define the spline data and yaw angles explictly should not
% define parameter xApproach
if(exist("xApproach","var"))
    if(roundtrip)
        [timespot_spl_re, spline_data_re, spline_yaw_re, ~] = ...
            quadcopter_waypoints_to_trajectory(...
            fliplr(waypoints),max_speed,min_speed,xApproach,vApproach);

        [timespot_spl_to, spline_data_to, spline_yaw_to, wayp_path_vis] = ...
            quadcopter_waypoints_to_trajectory(...
            waypoints,max_speed,min_speed,xApproach,vApproach);
        
        pause_at_target = 5; % sec
        timespot_spl = [timespot_spl_to; timespot_spl_re+timespot_spl_to(end)+pause_at_target];

        spline_data = [spline_data_to;spline_data_re];
        spline_yaw = [spline_yaw_to spline_yaw_re];
        spline_yaw = unwrap(spline_yaw,1.5*pi);
    else
        [timespot_spl, spline_data, spline_yaw, wayp_path_vis] = ...
            quadcopter_waypoints_to_trajectory(...
            waypoints,max_speed,min_speed,xApproach,vApproach);
    end
else
    % Obtain data to visualize path between waypoints
    wayp_path_vis = quadcopter_waypoints_to_path_vis(waypoints);
    if(roundtrip)
        spline_data  = [spline_data; flipud(spline_data)];
        %timespot_spl
        timespot_spl = [timespot_spl; timespot_spl(end)+5; ...
            timespot_spl(end)+5+cumsum(flipud(diff(timespot_spl)))];
        spline_yaw = unwrap([spline_yaw flipud(spline_yaw)+pi],1.5*pi);
    end
end
