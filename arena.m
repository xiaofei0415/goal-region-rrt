classdef arena
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        start = [50 20];
        % [x,y, a, b]
        %goal = [50, 80, 5, 3];
        %1: rectangle [left-down point location, up-right position,1]
        %2 :circle [center position, r, 0, 2]

        obs = [30,40,56, 60, 1;58, 40, 70, 60, 1];
       % obs = [30,40,70, 60, 1];
        area = [0, 0, 100, 100]; % left-down point location , width,height
        step = 1;
    end

    methods
        function obj = arena()
            %UNTITLED2 Construct an instance of this class
            %   Detailed explanation goes here
           
        end

        function flag = isMotionValid(obj, near_point, new_point)% from didnot check collision
            seg_len = obj.step;
            len = norm(new_point - near_point);
            seg_num = floor(len/seg_len);
            remain = mod(len, seg_len);
            unit = (new_point-near_point)/len;
            for i = 1:seg_num
                temp_point = near_point + i*seg_len*unit;
                if ~ obj.isSateValid(temp_point)
                    flag = false;
                    return;
                end
                
            end
            if remain ~= 0
                if ~ obj.isSateValid(new_point)
                    flag = false;
                    return;
                end
            end
            
            flag = true;%no collison
            
        end

        function flag = isSateValid(obj, p)
            flag = true;
            for i = 1:size(obj.obs, 1)
                  if obj.obs(i,5) ==1
                        if p(1)>= obj.obs(i,1) && p(2)>= obj.obs(i,2)&& p(1)<= obj.obs(i,3)&& p(2)<= obj.obs(i,4)
                            flag = false;
                            break;
                        end

                  end
                  if obj.obs(i,5) ==2
                        if (p(1)-obj.obs(i,1))^2 + (p(2)-obj.obs(i,2))^2 <= obj.obs(i,3)^2
                            flag = false;
                            break;
                        end

                  end
            end

        end
        function plot_arena(obj)
            hold on;
            tem = obj.area;
              plot([tem(1), tem(1), tem(3), tem(3),tem(1)],[tem(2),tem(4),tem(4),tem(2),tem(2)],'c-');%cyan
              for i = 1:size(obj.obs,1)
                  tem = obj.obs(i,:);
                  if tem(5) == 1
                       fill([tem(1), tem(1), tem(3), tem(3),tem(1)],[tem(2),tem(4),tem(4),tem(2),tem(2)],'c')
                  end
                  if tem(5) == 2
                        t = 0:pi/20:2*pi;
                        x=tem(1) + tem(3)*sin(t);
                        y = tem(2)+tem(3)*cos(t);
                        fill(x,y,'c');
                  end
                
              end

        end

        function obj =addNewObstacle(obj,obss)
            obj.obs(end+1, :) = obss;
        end

        function g = sampleGoal(obj)
%             a = obj.goal;
%             g = rand(1,2).*[2*a(3), 2*a(4)] + [a(1)-a(3), a(2)-a(4)];
%             while ((g(1)-a(1))^2 / a(3)^2 + (g(2)-a(2))^2/a(4)^2 >=1) || ~ obj.isSateValid(g)
%                     g = rand(1,2).*[2*a(3), 2*a(4)] + [a(1)-a(3), a(2)-a(4)];
%             end


%             x = rand*100;
%             if x <45 && 0<=45
%                 y=-20*(x-45)/45+80;
%                 g = [x y];
%             else
%                 g = [x,80];
%             end
                 g = [rand*100,80];


        end
        function s = sampleUniform(obj)
            a = obj.area;
                s =rand(1,2).*[a(3), a(4)] + [a(1), a(2)];
        end
        function path = shrink(obj, totalPath)
            mm=1;
            nn=3;
            path = totalPath(mm,:);
            while nn<=size(totalPath,1)
                if obj.isMotionValid(totalPath(mm,1:2),totalPath(nn,1:2))
                    nn=nn+1;
                else
                    path(end+1,:)  = totalPath(nn-1,:);
                    mm = nn-1;
                    nn = mm+2;
                end            
            end   
            
            path(end+1,:) = totalPath(end,:);
        end
    end
end