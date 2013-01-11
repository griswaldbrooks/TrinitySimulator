%%% Beam Model %%%
function [range, feature] = getRangeBeam(map, view_angle, max_range, min_range, T)
    
    ranges = [];
    feature_iters = [];
    
    for iter = 1:2:length(map)
        % Parameterized Inclusion Test
        p_0 = T\[map(:,iter);1];
        p_f = T\[map(:,iter+1);1];
        
    %%%% DOT PRODUCT TEST %%%%
        p_1 = [p_0(1),p_0(2)]';
        p_2 = [p_f(1),p_f(2)]';
        p_lam = p_2 - p_1;
        distances = [];
        % Perpendicular Test
        % Calculate perpendicular point
        lambda_perp = (-p_lam'*p_1)/(p_lam'*p_lam);
        p_r = p_1 + lambda_perp*p_lam;
        pp1 = p_r(1)/sqrt(p_r(1)^2 + p_r(2)^2);
        % Check to see if perpendicular point is in the view angle and 
        % the point is on the line segment
        if((pp1 >= cos(0.5*view_angle))&&((lambda_perp <= 1)&&(lambda_perp >= 0)))     
            % Add distance to range candidates
            ranges = [ranges, sqrt(p_r(1)^2 + p_r(2)^2)];
            % Add feature iterator to feature iterator candidates
            feature_iters = [feature_iters, iter];      
        else
            % End Point Test
            % Calculate the unit vector projection along the endpoint line
            % and see if it is within the view angle
            ep1 = p_1(1)/sqrt(p_1(1)^2 + p_1(2)^2);
            ep2 = p_2(1)/sqrt(p_2(1)^2 + p_2(2)^2);
            if(ep1 >= cos(0.5*view_angle))
                distances = [distances, sqrt(p_1(1)^2 + p_1(2)^2)];
            end
            if(ep2 >= cos(0.5*view_angle))
                distances = [distances, sqrt(p_2(1)^2 + p_2(2)^2)];
            end
            % Intersection Test
            % Calculate the intersection point and see if it is on the line
            % segment
            a = (p_lam(1)^2)*(cos(0.5*view_angle)^2 - 1) + p_lam(2)^2;
            b = 2*((p_1(1)*p_lam(1))*(cos(0.5*view_angle)^2 - 1) + p_1(2)*p_lam(2));
            c = (p_1(1)^2)*(cos(0.5*view_angle)^2 - 1) + p_1(2)^2;
            % Check for imaginary solutions
            if((b^2 - 4*a*c) >= 0)
                lambdas_int = roots([a,b,c]);
                % If the solution is within range
                if((lambdas_int(1) <= 1)&&(lambdas_int(1) >= 0))
                    ip1 = p_1 + lambdas_int(1)*p_lam;
                    nrmip1 = ip1(1)/sqrt(ip1(1)^2 + ip1(2)^2);
                    % If the point is within the view angle
                    if(nrmip1 >= cos(0.5*view_angle))
                        distances = [distances, sqrt(ip1(1)^2 + ip1(2)^2)];
                    end
                end
                if((lambdas_int(2) <= 1)&&(lambdas_int(2) >= 0))
                    ip2 = p_1 + lambdas_int(2)*p_lam;
                    nrmip2 = ip2(1)/sqrt(ip2(1)^2 + ip2(2)^2);
                    % If the point is within the view angle
                    if(nrmip2 >= cos(0.5*view_angle))
                        distances = [distances, sqrt(ip2(1)^2 + ip2(2)^2)];
                    end
                end
            end
            % If any distances were added 
            if(~isempty(distances))
                % Add distance to range candidates
                ranges = [ranges, min(distances)];
                % Add feature iterator to feature iterator candidates
                feature_iters = [feature_iters, iter];     
            end
        end
    end
    
    [range, index] = min(ranges);
    % Limit beam range
%     if(range > max_range)
%         range = max_range;
%     end
%     if(range < min_range)
%         range = min_range;
%     end
    iter = feature_iters(index);
    feature = [map(:,iter),map(:,iter+1)];
	
	%%%% ITERATIVE TEST %%%%
%         for lambda = 0:0.05:1
%             % Get x_feature for current lambda
%             x_feature = lambda*p_lam(1) + p_0(1);
%             % Get y_feature for current lambda
%             y_feature = lambda*p_lam(2) + p_0(2);
%             % Get Beam Boundary y_beam for x_feature
%             y_beam = m*abs(x_feature);
%             % Initialize Possible Distances
%             distances = [Inf,Inf];
%             % Test for feature in beam
%             if((y_feature <= y_beam)&&(y_feature >= -y_beam)&&(x_feature >= 0))
%                 % Highlight the feature
%                 %line(map(1,iter:iter+1),map(2,iter:iter+1),'Color','r','LineWidth',1);
%                 p_ftr = T*[x_feature;y_feature;1];
%                 plot(p_ftr(1), p_ftr(2), 'r*');
%                 
%                 C_up   =  (m*p_0(1) - p_0(2))/(p_lam(2) - m*p_lam(1));
%                 C_down = -(m*p_0(1) + p_0(2))/(p_lam(2) + m*p_lam(1));
%                 
%                 % If a valid parameterization was produced, add it for
%                 % testing
%                 if( (C_up <= 1)&&(C_up >= 0) )
%                     x_d = C_up*p_lam(1) + p_0(1);
%                     y_d = C_up*p_lam(2) + p_0(2);
%                     distances(1) = sqrt((x_d)^2 + (y_d)^2);
%                 end
%                 
%                 if( (C_down <= 1)&&(C_down >= 0) )
%                     x_d = C_down*p_lam(1) + p_0(1);
%                     y_d = C_down*p_lam(2) + p_0(2);
%                     distances(2) = sqrt((x_d)^2 + (y_d)^2);
%                 end
%                 
%                 % Add minimum distance to range cadidates
%                 ranges = [ranges, min(distances)];
%                 % Add feature iterator to feature iterator candidates
%                 feature_iters = [feature_iters, iter];
%                 
%                 % Don't check other points since we know this feature is in
%                 % the beam
%                 %break;
%             end
%         end