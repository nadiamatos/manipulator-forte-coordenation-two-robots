classdef Coordenation < handle

  properties

    intersection = struct;
    footprints = struct;
    pointsConfigurations = struct;
    quantityRobots = 0;
    quantityPoints = 0;
    rb = 0; trajectory = 0;
    k = 2;
    rbState = struct;

  end

  methods

    function obj = Coordenation(quantityRobots, quantityPoints, rb, trajectory)

      disp('Coordinating ...');
      obj.quantityRobots = quantityRobots;
      obj.quantityPoints = quantityPoints;
      obj.rb = rb;
      obj.trajectory = trajectory;

      for i = 1 : obj.quantityRobots

        obj.footprints.(['rb' num2str(i)]) = polyshape;
        obj.pointsConfigurations.(['rb' num2str(i)]) = polyshape;

      end

      obj.constructionFootprints();
      obj.findPointsCollision();

    end

    function constructionFootprints(obj)

      %{

      This function to build the region that englobe the robot,
      using a fixed radius for each.

      %}

      t = 0 : 1 : 2*pi; radius = 1.5*obj.rb(1).radiusRobot;

      for j = 1 : obj.quantityRobots

        for i = 1 : 2 : max(obj.quantityPoints)

          if (i <= obj.quantityPoints(j))

            obj.footprints.(['rb' num2str(j)])(1, end+1) = polyshape(obj.trajectory(j).finalTrajectory(i, 1)+radius*sin(t), ...
                                                                     obj.trajectory(j).finalTrajectory(i, 2)+radius*cos(t));

          end

        end

      end

      %{
      hold on;
      plot(obj.footprints.rb1, 'FaceColor', obj.rb(1).color);
      plot(obj.footprints.rb2, 'FaceColor', obj.rb(2).color);
      plot(obj.footprints.rb3, 'FaceColor', obj.rb(3).color);
      plot(obj.footprints.rb4, 'FaceColor', obj.rb(4).color);
      drawnow;
      %}

    end

    function findPointsCollision(obj)
      %{

      This function work for to find regions of collision between two paths
      of two differents robots. Finding regions that can to occur intersection.

      %}

      k = 2;

      for i = 1 : obj.quantityRobots

        rb1 = ['rb' num2str(i)];

        for j = obj.quantityRobots : - 1 : k

          if (i == j); continue; end
          rb2 = ['rb' num2str(j)];

          for a = 1 : size(obj.footprints.(rb1), 2)

            for b = 1 : size(obj.footprints.(rb2), 2)
              %out = intersect(obj.footprints.(rb1)(1, a), obj.footprints.(rb2)(1, b));
              %if ( ~isempty(out) && (out.NumRegions ~= 0) )
              out = overlaps([obj.footprints.(rb1)(1, a) obj.footprints.(rb2)(1, b)]);
              if (out ~= 0)                
                obj.pointsConfigurations.(rb1)(1) = union(obj.pointsConfigurations.(rb1)(1), obj.footprints.(rb1)(1, a));
                obj.pointsConfigurations.(rb2)(1) = union(obj.pointsConfigurations.(rb2)(1), obj.footprints.(rb2)(1, b));
              end

            end

          end

        end

        k = k + 1;

      end
      %{
      hold on;
      plot(obj.pointsConfigurations.rb1, 'FaceColor', obj.rb(1).color);
      plot(obj.pointsConfigurations.rb2, 'FaceColor', obj.rb(2).color);
      plot(obj.pointsConfigurations.rb3, 'FaceColor', obj.rb(3).color);
      plot(obj.pointsConfigurations.rb4, 'FaceColor', obj.rb(4).color);
      drawnow;
      %}

    end

    function bit = verifyRobotInRegionCollision(obj, index)

      bit = zeros(1, obj.quantityRobots); aux = bit; threshold = 4000; 
      reference = obj.rb(index).getPositionBase( obj.rb(index).links(1).Vertices, 0 );
      %distance = 0;

      for i = 1 : obj.quantityRobots

        if isequal(i, index); continue; end
        %distance = pdist([obj.rb(i).coordLinks(1, 1:2); reference(1, 1:2)]);
        bit(i) = isinterior(obj.pointsConfigurations.(['rb' num2str(index)])(1), obj.rb(i).coordLinks(1, 1:2)) && ...
                 isinterior(obj.pointsConfigurations.(['rb' num2str(index)])(1), reference(1, 1:2));
        aux(i) = isinterior(obj.pointsConfigurations.(['rb' num2str(i)])(1), obj.rb(i).coordLinks(1, 1:2)) && ...
                 isinterior(obj.pointsConfigurations.(['rb' num2str(i)])(1), reference(1, 1:2));

        if ( isequal(aux, 1) && isequal(bit(i), 1) )
          obj.rb(index).status = 1; obj.rb(i).status = 0; %break;
        elseif ( isequal(aux, 1) )
          obj.rb(index).status = 1; obj.rb(i).status = 0; %break;
        elseif ( isequal(bit(i), 1) )
          obj.rb(index).status = 0; obj.rb(i).status = 1; %break;
        else
          obj.rb(i).status = 1;
        end
        %{
          PENSAR NA IDEIA DE REGIAO DE OCUPACAO.                
          FAZER SEMPRE UMA COMPARAÇÃO DE UM ROBÔ COM TODOS OS OUTROS.
            EX: 1 - 2, 1 - 3, 1 - 4
                2 - 3, 2 - 4
                3 - 4
        %}
      end

    end

  end

end
