try
    %% initialize collider object
    c = Collider();
    p = rosparam;
    if ~ p.has('collision_detector_rate')
        p.set('collision_detector_rate', 20);
    end
    %% start loop
    while(1)
        % if no pose subscriber, poll
        if isempty(c.subp)
            c.pose_callback(c.getPoseFromTF());
        end
        
        % publish collision
        c.pub_collision();
        
        % wait
        sleep(rosrate(p.get('collision_detector_rate')));
        
    end
    
catch colliderException
    disp(colliderException);
    rethrow(colliderException);
end
