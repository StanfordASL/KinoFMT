%% checkCollision.m
%
% Given a v and end state, check whether the resulting edge is free of
% collisions
%
% Given:
%   v- start state
%   w- end state
%   obstacles- obstacle set, row i is lower corner of obstacle and row
% i+1 is upper corner of obstacle, such that ever value in row i is > i+1
%
% Return:
%   valid- true if the path between v and end is valid
%

function [valid] = checkCollision(v, w, obstacles, bounds)
bb_min = min(v,w);
bb_max = max(v,w);

% check spacial environment bounds
if ~isempty(bounds)
    if any(bb_min' < bounds(:,1)) || any(bb_max' > bounds(:,2))
        valid = 0;
        return;
    end
end

% check obstacles
if ~isempty(obstacles)
    for k = 1:2:length(obstacles(:,1))
        obs = obstacles(k:k+1,:);
        % Broadphase Validity check
        if ~(any(bb_max < obs(1,:)) || any(bb_min > obs(2,:)))
    %     if ~BroadphaseValidQ(bb_min, bb_max,obs)
            if ~MotionValidQ(v, w, obs)
                valid = 0;
                return;
            end
        end
    end
end
valid = 1;

%     % check if obstacle within range of path
%     function [isValid] = BroadphaseValidQ(bb_min, bb_max, obs)
%         for i = 1:size(obs,2)
%             if bb_max(i) < obs(1,i)  || obs(2,i) < bb_min(i)
%                 isValid = 1;
%                 return
%             end
%         end
%         isValid = 0;
%     end

    % find where vector v to w would intersect the face
    function [isValid] = MotionValidQ(v, w, obs)
        v_to_w = w-v;
        corner = v < obs(1,:);
        lambdas = (corner.*obs(1,:) + ~corner.*obs(2,:) - v)./v_to_w+5*eps;
        for i = 1:size(obs,2)
            if FaceContainsProjection(v, v_to_w, lambdas(i), i, obs)
                isValid = 0;
                return
            end
        end
        isValid = 1;
    end

    % check whether face is intersected
    function [isValid] = FaceContainsProjection(v, v_to_w, lambda, j, obs)
        for i = 1:size(obs,2)
            if i ~= j && ~(obs(1,i) < v(i) + v_to_w(i)*lambda && ...
                    v(i) + v_to_w(i)*lambda < obs(2,i))
                isValid = 0;
                return
            end
        end
        isValid = 1;
    end
end
