classdef TrajectoryPlanning < handle

  properties

    robot = 0; pointStart = zeros(1, 3);
    pointEnd = zeros(1, 3);
    coordCurrent = zeros(1, 3);
    distance = 0; iterations = 0; indexNode = 0;
    minDistEndPoint = 200; pts = []; ptsManipulator = [];
    offsetAxi = [100, 100, 0];
    coordOld = zeros(1, 3); coord = zeros(1, 3);
    cost = 0; bitEmptyReached = 0; reachable = struct;
    explored = struct; node = struct; pointsOfTrajectory = [];
    finalTrajectory = []; obst = [];

  end

  methods

    function obj = TrajectoryPlanning(manipulator, objects3D, color)

      obj.robot = manipulator;
      obj.obst = objects3D.circle;
      obj.pointEnd = objects3D.pointEnd(obj.robot.priority, :);
      obj.pointStart = obj.robot.coordLinks(1, :);
      obj.coordCurrent = obj.pointStart;
      obj.coordOld = obj.coordCurrent;
      obj.coord = obj.coordOld;

      obj.findPath(); % find first trajectory
      obj.finalTrajectory(end+1:end+size(obj.pointsOfTrajectory, 1), :) = flipud(obj.pointsOfTrajectory);

      disp('Planning ...'); hold on;
      plot3 (obj.finalTrajectory(:, 1), obj.finalTrajectory(:, 2), ...
             obj.finalTrajectory(:, 3), color, 'LineWidth', 2);
      drawnow;

    end

    function findPath(obj)

      obj.reachable = struct('nodeCurrent', obj.pointStart, ...
                             'nodePrevious', obj.pointStart, 'cost', 0);
      obj.explored = obj.reachable; obj.node = obj.reachable;
      obj.bitEmptyReached = false;

      while (~obj.bitEmptyReached)

        try
          obj.bitEmptyReached = isempty(obj.reachable);
          [node, indexNode] = obj.chooseNode(obj.reachable, obj.pointEnd);
        catch
          obj.pointsOfTrajectory = []; disp('Trajectory NOT Found'); break;
        end

        obj.distance = pdist([node.nodeCurrent; obj.pointEnd], 'euclidean');
        obj.calculateOffsetAxi();

        if (obj.distance <= obj.minDistEndPoint); obj.pointsOfTrajectory = obj.buildPath(obj.explored, node); break; end

        obj.reachable(indexNode) = []; obj.explored(end+1) = node;
        aux = obj.computesPointNeighborhood(node.nodeCurrent, obj.offsetAxi);
        obj.pts = aux(1:8, :);

        for i = 1 : size(obj.pts, 1)

          if (~obj.verifyElementsInStruct(obj.pts(i, :), obj.explored))

            obj.robot.getFootprintRobot(obj.pts(i, :));
            obj.cost = 0; bit = obj.verifyCollisionRobot();
            if (bit); obj.cost = inf; end %disp('obstaculo!');
            newReachable(i) = struct('nodeCurrent', obj.pts(i, :), ...
                                     'nodePrevious', node.nodeCurrent, ...
                                     'cost', obj.cost);

          end

        end

        for i = 1 : size(newReachable, 2)

          adjacent = newReachable(i);
          [bit, ~] = obj.verifyElementsInStruct(adjacent.nodeCurrent, obj.reachable);

          if (~bit); obj.reachable(end+1) = adjacent; end

          if ((node.cost + max(obj.offsetAxi)) < adjacent.cost)

            adjacent.nodePrevious = node.nodeCurrent;
            adjacent.cost = node.cost + max(obj.offsetAxi);

          end

        end

        obj.iterations = obj.iterations + 1;
        %hold on; plot3(obj.pts(:, 1), obj.pts(:, 2), obj.pts(:, 3), '*r');
        %plot3(node.nodeCurrent(1), node.nodeCurrent(2), node.nodeCurrent(3), '*g');
        %drawnow;

      end

    end

    function calculateOffsetAxi(obj)

      if (obj.distance > 400); aux = 200; else; aux = 100; end

      obj.offsetAxi = aux*[ones(1, 2) 0];

    end

    function [bestNode, indexNode] = chooseNode(obj, reachable, pointEnd)

    	minCost = 100000000; bestNode = []; %indexDeletar = [];
    	totalCost = 0; costStartToNode = 0; costNodeToGoal = 0;

    	for i = 1 : size(reachable, 2)

    		if (~isequal(reachable(i).cost, inf))

    			costStartToNode = pdist([reachable(i).nodeCurrent; reachable(i).nodePrevious], 'euclidean');
    			costNodeToGoal = pdist([reachable(i).nodeCurrent; pointEnd], 'euclidean');
    			totalCost = costStartToNode + costNodeToGoal;
    			reachable(i).cost = totalCost;

          if (minCost > totalCost)

    			  minCost = totalCost; bestNode = reachable(i); indexNode = i;

          end

        end

    	end
      
    end

    function pathFound = buildPath(obj, explored, node)

    	pathFound = []; bitIsMemberOfStruct = 1;

    	for i = 1 : size(explored, 2)

    		pathFound(end+1, :) = node.nodeCurrent;
    		[bitIsMemberOfStruct, index] = obj.verifyElementsInStruct(node.nodePrevious, explored);
    		node = explored(index);

    	end

    end

    function [bitIsMemberOfStruct, index] = verifyElementsInStruct(obj, element, structRequired)

    	bitIsMemberOfStruct = 0; index = 0;

    	for i = 1 : size(structRequired, 2)

    		if (isequal(element, structRequired(i).nodeCurrent))

    			bitIsMemberOfStruct = 1; index = i; break;

        end

    	end

    end

    function pts = computesPointNeighborhood(obj, ptOpenList, offsetAxi)

      x = ptOpenList(1); y = ptOpenList(2); z = ptOpenList(3);
    	pts = zeros(3, 12);

    	pts(:, 1)  = [x+offsetAxi(1);   y-offsetAxi(2); z];
    	pts(:, 2)  = [x+offsetAxi(1);   y; 				z];
    	pts(:, 3)  = [x+offsetAxi(1);   y+offsetAxi(2); z];
    	pts(:, 4)  = [x; 				y-offsetAxi(2); z];
    	pts(:, 5)  = [x; 				y+offsetAxi(2); z];
    	pts(:, 6)  = [x-offsetAxi(1);   y-offsetAxi(2); z];
    	pts(:, 7)  = [x-offsetAxi(1);   y;              z];
    	pts(:, 8)  = [x-offsetAxi(1);   y+offsetAxi(2); z];

    	pts(:, 9)  = [x+offsetAxi(1);   y-offsetAxi(2);   z+offsetAxi(3)];
    	pts(:, 10) = [x+offsetAxi(1);   y; 						    z+offsetAxi(3)];
    	pts(:, 11) = [x+offsetAxi(1);   y+offsetAxi(2);   z+offsetAxi(3)];
      pts(:, 12) = [x; 							  y-offsetAxi(2);   z+offsetAxi(3)];
    	pts(:, 13) = [x;						    y+offsetAxi(2);   z+offsetAxi(3)];
      pts(:, 14) = [x-offsetAxi(1);   y-offsetAxi(2);   z+offsetAxi(3)];
      pts(:, 15) = [x-offsetAxi(1);   y;						    z+offsetAxi(3)];
      pts(:, 16) = [x-offsetAxi(1);   y+offsetAxi(2);   z+offsetAxi(3)];

    	pts(:, 17) = [x+offsetAxi(1);   y-offsetAxi(2);   z-offsetAxi(3)];
    	pts(:, 18) = [x+offsetAxi(1);   y; 						    z-offsetAxi(3)];
    	pts(:, 19) = [x+offsetAxi(1);   y+offsetAxi(2);   z-offsetAxi(3)];
    	pts(:, 20) = [x; 							  y-offsetAxi(2);   z-offsetAxi(3)];
    	pts(:, 21) = [x; 							  y+offsetAxi(2);   z-offsetAxi(3)];
    	pts(:, 22) = [x-offsetAxi(1);   y-offsetAxi(2);   z-offsetAxi(3)];
    	pts(:, 23) = [x-offsetAxi(1);   y; 							  z-offsetAxi(3)];
    	pts(:, 24) = [x-offsetAxi(1);   y+offsetAxi(2);   z-offsetAxi(3)];

    	pts = pts';

    end

    function bitCollision = verifyCollisionRobot(obj)

      bitCollision = 0;

      for i = 1 : size(obj.obst, 2)

        [bitInside, ~] = intersect(obj.robot.footprint, obj.obst(i).Vertices);
        if (~isempty(bitInside)); bitCollision = 1; break; end;

      end

    end

  end

end
