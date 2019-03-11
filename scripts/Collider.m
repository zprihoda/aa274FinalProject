classdef Collider < matlab.mixin.Copyable
    properties
        collision = false;
        valid_update = true
        
        pop = rosmessage('geometry_msgs/Pose2D') % previous pose
        pon = rosmessage('geometry_msgs/Pose2D') % next pose
        ptp = rostime() % previous pose time
        ptn = rostime() % next pose time
        
        odp = rosmessage('nav_msgs/Odometry') % previous odometry
        odn = rosmessage('nav_msgs/Odometry') % next odometry
        
        cvp = rosmessage('geometry_msgs/Twist') % previous vel message
        cvn = rosmessage('geometry_msgs/Twist') % next vel message
        
        subp                % pose subscriber
        subo                % odometry subscriber
        subvel              % cmd_vel subscriber
        pub                 % collision publisher
        tf              % transform listener
        
    end
    methods
        % constructor: setup publishers / subscribers
        function obj = Collider()
            % check for / set necessary ros parameters
            p = rosparam;
            if ~p.has('lin_vel_thres')
                p.set('lin_vel_thres', 0.05);
            end
            
            if ~p.has('ang_vel_thres')
                p.set('ang_vel_thres', deg2rad(10));
            end
            
            if ~p.has('pos_dur_thres')
                p.set('pos_dur_thres', 1);
            end
            
            if ~p.has('odm_dur_thres')
                p.set('odm_dur_thres', 1);
            end
            
            if p.has('sim')
                use_gazebo = p.get('sim');
            else
                use_gazebo = false;
            end
            
            if use_gazebo
                obj.subp = rossubscriber('/gazebo/model_states', 'gazebo_msgs/ModelStates', @obj.gazebo_callback);
            else
                obj.subp = [];
            end
            
            obj.subo = rossubscriber('/odom', ('nav_msgs/Odometry'),    @obj.odom_callback);
            
            obj.subvel = rossubscriber('/cmd_vel', 'geometry_msgs/Twist', @obj.cmdvel_callback);
            
            obj.pub  = rospublisher('/collision', 'std_msgs/Bool');
            
            obj.tf = rostf;
            
            pause(1); % wait a literal second so tf doesn't bug out
            
        end
        
        function cmdvel_callback(obj, src, msg)
            
            obj.cvp = obj.cvn;
            obj.cvn = msg;
            
        end
        
        function gazebo_callback(obj, src, gmsg)
            % convert to Pose2D message
            quat = gmsg.Pose.Orientation;
            ang = (quat2eul([quat.W quat.X quat.Y quat.Z]));
            
            msg = rosmessage('geometry_msgs/Pose2D');
            msg.X = gmsg.Pose.Position.X;
            msg.Y = gmsg.Pose.Position.Y;
            msg.Theta = ang(3);
            
            % save next Pose2D message
            obj.pop  = obj.pon;
            obj.pon  = msg;
            
            obj.ptp = obj.ptn;
            obj.ptn = rostime("now");
        end
        
        function pose_callback(obj, src, msg)
            if ~isempty(msg)
                obj.pop = obj.pon;
                obj.pon = msg;
                
                obj.ptp = obj.ptn;
                obj.ptn = rostime("now");
            end
        end
        
        function odom_callback(obj, src, msg)
            obj.odp = obj.odn;
            obj.odn = msg;
        end
        
        function msg = getPoseFromTF(obj)
            
            % get transform
            opts = {'base_footprint','map',rostime(0)};
            msg = rosmessage('geometry_msgs/Pose2D');
            if ~ obj.tf.canTransform(opts{1:2})
                % send empty message
                msg = [];
                return
            else
                try
                    tform = obj.tf.getTransform(opts{:});
                catch ME
                    disp(ME); % display error, but don't die
                    msg = []; % tform still failed: return empty msg
                    return
                end
                
                % convert to pose
                quat = tform.Transform.Rotation;
                ang = (quat2eul([quat.W quat.X quat.Y quat.Z]));
                
                % create msg
                msg.X = tform.Transform.Translation.X;
                msg.Y = tform.Transform.Translation.Y;
                msg.Theta = ang(3);
            end
        end
        
        function bool = collided(obj)
            % get duration between messages
            pos_dur = (obj.ptn - obj.ptp);
            odm_dur = (obj.odp.Header.Stamp - obj.odn.Header.Stamp);
            
            % get avg odom velocity reading
            cmd_lin_vel_x = (obj.cvn.Linear.X +  obj.cvp.Linear.X) ./ 2;
            cmd_lin_vel_y = (obj.cvn.Linear.Y +  obj.cvp.Linear.Y) ./ 2;
            cmd_lin_vel = sqrt(cmd_lin_vel_x.^2 + cmd_lin_vel_y.^2);
            cmd_ang_vel = (obj.cvn.Angular.Z + obj.cvp.Angular.Z) ./ 2;
            
            % get avg pose velocity reading
            dt_inv = 1./pos_dur.Sec;
            pos_lin_vel = dt_inv * sqrt((obj.pop.X - obj.pon.X).^2 + (obj.pop.Y - obj.pon.Y).^2);
            pos_ang_vel = dt_inv * sqrt((obj.pop.Theta - obj.pon.Theta ));
            
            % check velocity and time
            p = rosparam;
            
            % wheels moving faster than position changes / orientation
            % turning faster than supposed to
            fprintf('\ncmd_lin_vel: %0.3f', cmd_lin_vel)
            fprintf('\tpos_lin_vel: %0.3f', pos_lin_vel)
            obj.collision = abs((cmd_lin_vel) - (pos_lin_vel)) > p.get('lin_vel_thres'); ... || ...
%                 abs(cmd_ang_vel - pos_ang_vel) > p.get('ang_vel_thres');
            
            % time diff more than allowed
            obj.valid_update = pos_dur.Sec < p.get('pos_dur_thres') && ...
                odm_dur.Sec < p.get('odm_dur_thres');
            
            % report collision if valid
            bool = obj.collision && obj.valid_update;
        end
        
        function pub_collision(obj)
            msg = rosmessage('std_msgs/Bool');
            msg.Data = obj.collided();
            obj.pub.send(msg);
        end
    end
end