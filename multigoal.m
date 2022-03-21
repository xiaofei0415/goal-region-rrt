classdef multigoal
    %UNTITLED7 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        %Property1
        area
        goalset
        cbest
        Lset
        Cset
        
        T
        d
        step
        numgoals
        
        path
        tu
        xpos
    end

    methods
        function obj = multigoal()
            %UNTITLED7 Construct an instance of this class
            %   Detailed explanation goes here
            %obj.Property1 = inputArg1 + inputArg2;
            obj.area = arena;
            obj.cbest = Inf;
            obj.d = 2;
            obj.step = 5;
            obj.numgoals =4;
            obj.T=[obj.area.start 0 0];% position parent_index, this_seg_length
            obj.path = [];
            obj.Cset={};
            obj.Lset={};
            
            obj.goalset=[];
            obj.tu=[];
            obj.xpos=[];
        end
        
        function obj= plan(obj)
            tstart=tic;
            for iter = 1: 6000
                
                [temcbest,tempath] = getCbest(obj);
                if temcbest < obj.cbest
                    obj.cbest = temcbest;
                    obj.path=tempath;
                    obj = updataLandCandelse(obj);

                end
                if size(obj.goalset,1) == 0 ||(rand < 0.1 && size(obj.goalset,1) < obj.numgoals)
                    tem = obj.area.sampleGoal();
                    if norm(tem - obj.area.start) < obj.cbest
                        obj.goalset(end+1,:) = [tem -1];
                        %obj.xcenterset(end+1,:) = [(obj.area.start + tem)/2 0];
                        %obj.x_center=[(obj.start+obj.goal)/2,0]';
                        cmin = norm(tem-obj.area.start);
                        a_l=[(tem-obj.area.start)/cmin 0]';
                        id_t=[1,0,0];
                        M=a_l*id_t;
                        [U,~,Vh] = svd(M);
                        obj.Cset{end+1}=(U*diag([1,1,det(U)*det(Vh')]))*(Vh);
                        obj.Lset{end+1}=[];
                        if obj.cbest < Inf
                            obj = updataLandCandelse(obj);
                        end

                    end


                end
                [xrand, flag] = obj.sample();
                if ~flag
                    continue;
                end
                [xnearest,parentindex] = obj.Nearest(xrand);
                xnew = obj.Steer(xnearest, xrand);

                if obj.area.isMotionValid(xnearest, xnew)
                    nearlist = obj.Near(xnew);
                    minlen = obj.Cost(parentindex)+norm(xnew-xnearest);
                    minindex = parentindex;
                    for i =nearlist
                        if i ~= parentindex
                            temlen = norm(xnew-obj.T(i,1:obj.d))+obj.Cost(i);
                            if temlen < minlen
                                minlen = temlen;
                                minindex = i;
                            end
                        end


                    end
                    obj.T(end+1,:) = [xnew, minindex, norm(xnew-obj.T(minindex,1:obj.d))];
                    
                    for i = nearlist
                        if obj.Cost(i)>norm(obj.T(i,1:obj.d)-xnew)+minlen
                            obj.T(i,obj.d+1) = size(obj.T,1);
                            obj.T(i,obj.d+2) = norm(obj.T(i,1:obj.d)-xnew);
                        end
                    end
                    pindex = size(obj.T,1);
                    for i=1:size(obj.goalset,1)
                        if obj.goalset(i,obj.d+1)== -1 && norm(xnew - obj.goalset(i,1:obj.d))<= obj.step && obj.area.isMotionValid(xnew, obj.goalset(i,1:obj.d))
                            obj.T(end+1,:)= [obj.goalset(i,1:obj.d) pindex norm(xnew - obj.goalset(i,1:obj.d))];
                            obj.goalset(i,obj.d+1) = size(obj.T,1);
                        end
                                
                    end
                end
                if obj.cbest < inf && mod(iter,30)==0
                    obj.tu(1:2,end+1)=[toc(tstart),obj.cbest];
%                     tem=obj.path(end,1);
%                     if tem >= 50
%                       obj.xpos(1:2,end+1)=[toc(tstart);tem-50];
%                     else
%                         obj.xpos(1:2,end+1)=[toc(tstart);50-tem];
%                     end
                end
                if  iter == 1000|| iter == 5000|| iter == 3000
                    plot(obj);
                    fprintf("obj.cbest:  ");disp([obj.cbest,toc(tstart)]);
                end

            end

        end

        function plot(obj)
            figure;
            hold on;
            obj.area.plot_arena();
            Xa = obj.T;
            for i=1:size(Xa,1)
                if i==1
                    plot(Xa(1,1),Xa(1,2),'*r','MarkerSize',10);
                else
                    index = Xa(i,3);
                    plot([Xa(i,1) Xa(index,1)],[Xa(i,2),Xa(index,2)], 'b-');
                    plot(Xa(i,1),Xa(i,2), 'k.', 'MarkerSize', 3);
                end
            end
            if size(obj.goalset,1)~= 0
                plot(obj.goalset(:,1),obj.goalset(:,2),'+r','MarkerSize',10);
            end
            if obj.cbest < Inf
                plot(obj.path(:,1),obj.path(:,2),'g-',"LineWidth",2.0);
                %fprintf("obj.cbest:  ");disp([obj.cbest,toc(tstart)]);
                if size(obj.goalset,1)>=1
                    for i = 1:size(obj.goalset,1)
                        plot_ecllipse(obj,i);
                    end
                end
            else
                disp("no inital path yet");   
            end
            xlabel('X/m');ylabel('Y/m');
            %drawnow;
        end
        function plot_ecllipse(obj,index)
            x_center = (obj.area.start + obj.goalset(index,1:obj.d))/2;
            x1=x_center(1);
                y1=x_center(2);
                a1=obj.cbest/2;
                v=obj.goalset(index,1:obj.d)-obj.area.start;
                cmin = norm(v);
                b1=sqrt(obj.cbest.^2-cmin.^2)/2;
                sita=0:pi/100:2*pi;
                fi1=atan2(v(2),v(1));
                %plot ellipse
                toplot = [x1+a1*cos(fi1)*cos(sita)-b1*sin(fi1)*sin(sita);y1+a1*sin(fi1)*cos(sita)+b1*cos(fi1)*sin(sita)];
                for i = size(toplot,2):-1:1
                    if toplot(1,i)<0 || toplot(1,i)>100 || toplot(2,i)<0 ||toplot(2,i)>100
                        toplot(:,i)=[];
                    end
                end
                hold on;
                plot(toplot(1,:),toplot(2,:),'--g');
        end
        function len=Cost(obj,i)
            len = 0;
           while obj.T(i,obj.d+1)~=0
               len = len+obj.T(i,obj.d+2);
               i = obj.T(i,obj.d+1);
           end
        end
        function [best,path] = getCbest(obj)
            best =Inf;
            path = [];
            minindex=0;
            for i = 1:size(obj.goalset,1)
                index = obj.goalset(i,obj.d+1);
                if index~= -1
                    tem = obj.Cost(index);
                    if tem<best
                        best = tem;
                        minindex = index;
                    end
                 end
            end
            if minindex~=0
                path = getPath(obj,minindex);
            end
        end

        function path = getPath(obj,index)
            path = obj.T(index, 1:obj.d);
            parent_index = obj.T(index,obj.d+1);
            
            while parent_index~= 0
                path(end+1,:) = obj.T(parent_index,1:obj.d);
                
                parent_index = obj.T(parent_index,obj.d+1);    
                
            end
            path=flip(path);
        end

        function obj = updataLandCandelse(obj)
            for i=size(obj.goalset,1):-1:1
                if norm(obj.goalset(i,1:obj.d)-obj.area.start)>= obj.cbest
                    obj.Cset(i)=[];
                    obj.Lset(i)=[];
                    obj.goalset(i,:)=[];
                end
            end
            for i=1:size(obj.goalset,1)
                cmin = norm(obj.goalset(i,1:obj.d)-obj.area.start);
                tem = sqrt(obj.cbest^2-cmin^2)/2;
                %r=[obj.cbest/2, ones(1,obj.d-1).*tem];
                r=[obj.cbest/2, tem, tem];
                obj.Lset{i}=diag(r);
            end
        end
        function [xrand,flag] = sample(obj)
            if obj.cbest< Inf
                s = size(obj.goalset,1);
                if s==0
                    xrand=[];
                    flag = false;
                else
                    xrand = obj.informedsample();
                    while xrand(1)<0 || xrand(1)>100 || xrand(2)<0 || xrand(2)>100
                        xrand = obj.informedsample();
                    end
                    flag = true;
                end
            else
                flag = true;
                xrand = obj.area.sampleUniform();
            end

        end
        function xrand = informedsample(obj)
            s = size(obj.goalset,1);
              r = randi(s);
                    a=rand();
                    b=rand();
                    if b<a
                        tem=b;
                        b=a;
                        a=tem;
                    end
                    x_ball = [b*cos(2*pi*a/b);b*sin(2*pi*a/b);0];
%                     disp(["C",size(obj.Cset{r})]);
%                     disp(["L",size(obj.Lset{r})]);
                    xrand = obj.Cset{r} * obj.Lset{r} * x_ball + [(obj.area.start + obj.goalset(r,1:obj.d))/2 0]';
                    xrand = xrand(1:obj.d,1)';
        end
        function [xnearest, index] = Nearest(obj, xrand)
            temp=obj.T(:,1:obj.d)-ones(size(obj.T,1),1)*xrand;           
            [~,index] = min(sum(temp.^2,2));
            xnearest = obj.T(index,1:obj.d);
        end

        function xnew = Steer(obj, xnearest, xrand)
            len = norm(xrand-xnearest);
                if len>obj.step
                    xnew = xnearest+(xrand-xnearest)*obj.step/len;
                    
                else
                    xnew = xrand;
                end
        end

        function nearlist = Near(obj,newpoint)
            nearlist=[];
            r2 = obj.step^2;
                    M1 = obj.T(:,1:obj.d) - ones(size(obj.T,1),1)*newpoint;
                    M1 = sum(M1.^2,2);
                    list1 = find(M1<=r2);
                    %disp(["list1num",size(list1,1)]);
                    for i = list1'
                        if obj.area.isMotionValid(obj.T(i,1:obj.d),newpoint)
                            nearlist(1,end+1)=i;
                        end
                    end
        end
        
    end
end