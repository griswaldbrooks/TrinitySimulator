function [stuck_timer, rand_goal] = CheckStuck(poses, VAR_THRESH, stuck_timer, rand_goal)

VAR_THRESH1 = 0.8*VAR_THRESH;
VAR_THRESH2 = 1.2*VAR_THRESH;

pose_var = [var(poses(1,:)), var(poses(2,:))]';
if(stuck_timer.stuck == 0)
    if((pose_var(1) < VAR_THRESH1) && ~any(poses(1,:) == 0))
        rand_goal = 500*rand(2,1);
        stuck_timer.time = stuck_timer.top;
        stuck_timer.count = stuck_timer.count + 1;
        stuck_timer = stuck_timer.updateTop();
        stuck_timer.stuck = 1;
        disp('BANG X');
        %input('Press Enter to Continue:');
    elseif ((pose_var(2) < VAR_THRESH1) && ~any(poses(2,:) == 0))
        rand_goal = 500*rand(2,1);
        stuck_timer.time = stuck_timer.top;
        stuck_timer.count = stuck_timer.count + 1;
        stuck_timer = stuck_timer.updateTop();
        stuck_timer.stuck = 1;
        disp('BANG Y');
        %input('Press Enter to Continue:');
    elseif( (mean([pose_var(1),pose_var(2)] < VAR_THRESH2)) && (~any(poses(2,:) == 0)) && (~any(poses(1,:) == 0)) )
        rand_goal = 500*rand(2,1);
        stuck_timer.time = stuck_timer.top;
        stuck_timer.count = stuck_timer.count + 1;
        stuck_timer = stuck_timer.updateTop();
        stuck_timer.stuck = 1;
        disp('BANG XY');
        %input('Press Enter to Continue:');
    end
end

