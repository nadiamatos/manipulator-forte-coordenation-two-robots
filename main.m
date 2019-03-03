clc; clear ('all'); close all;


axis([-5000 5000 -5000 5000 0 5000]);
rotate3d on; grid on;
set(get(gca, 'XLabel'), 'String', 'Axis - X');
set(get(gca, 'YLabel'), 'String', 'Axis - Y');
set(get(gca, 'ZLabel'), 'String', 'Axis - Z');


nameImage = 'cenario2.bmp';
%pointsInitial = [500, -2000, 0; 500 0 0; 500 2000 0; 500 4000 0];
%pointsEnd     = [6000, 4000, 0; 6000, 2000, 0; 6000, 0, 0; 6000, -2000, 0];
%pointsInitial = [0, 1500, 0; 0 0 0; 0 -1500 0; 1000, -2000, 0;];
%pointsEnd     = [6000, 4000, 0; 6000, 4000, 0; 6000, 4000, 0; 6000, 4000, 0];
pointsInitial = [0, 1500, 0; 0 0 0;];
pointsEnd     = [6000, 4000, 0; 6000, 4000, 0;];

colorTraj = ['b' 'g' 'y' 'r'];
colorRobot = [0.0 0.0 1.0; 0.0 1.0 0.0; 1.0 1.0 0.0; 1.0 0.0 0.0];
quantityRobots = size(pointsInitial, 1); quantityPoints = zeros(1, quantityRobots);
objects3D = Object3D(nameImage, quantityRobots, pointsInitial, pointsEnd);


for i = 1 : quantityRobots
  drawnow;
  forte(i) = ManipulatorForte(pointsInitial(i, :), colorRobot(i, :), i);
  trajectory(i) = TrajectoryPlanning(forte(i), objects3D, colorTraj(i));
  quantityPoints(i) = size(trajectory(i).finalTrajectory, 1);
end

%
coordenation = Coordenation(quantityRobots, quantityPoints, forte, trajectory);
index = ones(size(quantityPoints, 2), 1); bitIncresing = index;

while true

  for i = 1 : quantityRobots

    coordenation.verifyRobotInRegionCollision(i);
    if (forte(i).status == 0); continue; end
    % Going
    if bitIncresing(i)

      if (~isequal(index(i), quantityPoints(i)))
        index(i) = index(i) + 1;
      else
        bitIncresing(i) = 0;
      end
    % Coming
    elseif ~bitIncresing(i)

      if (~isequal(index(i), 1))
        index(i) = index(i) - 1;
      else
        bitIncresing(i) = 1;
      end

    end
    %{
    if (forte(i).status == 1)
      forte(i).updateBaseForte(trajectory(i).finalTrajectory(index(i), :));
    end
    %}

  end

  for i = 1 : quantityRobots

    if (forte(i).status == 1)
      forte(i).updateBaseForte(trajectory(i).finalTrajectory(index(i), :));
    end

  end

end
%}
