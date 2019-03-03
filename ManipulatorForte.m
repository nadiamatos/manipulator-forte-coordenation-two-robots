classdef ManipulatorForte < handle

  properties

    joints = zeros(1, 7);   % first joint is for the mobile base
    coordCurrent = zeros(1, 3); coordOld = zeros(1, 3);
    coordLinks = zeros(7, 3);  % first link coordinate is for the mobile base
    T01 = zeros(4); T02 = zeros(4); T03 = zeros(4);
    T04 = zeros(4); T05 = zeros(4); T06 = zeros(4);
    T01old = zeros(4); T02old = zeros(4); T03old = zeros(4);
    T04old = zeros(4); T05old = zeros(4); T06old = zeros(4);
    links = gobjects(1, 7); wheels = gobjects(1, 4);
    offsetOrigin = zeros(1, 2); positionInitialBase = zeros(1, 2);
    velocity = 50; radiusRobot = 400;
    priority = 0; pointMiddle = 0; footprint = polyshape;
    status = 0; % if status = 1, the robot is mobile, if not, the robot is not mobile.
    color = 'r';

  end

  methods

    function obj = ManipulatorForte(positionInitialBase, color, priority)

      obj.priority = priority;
      obj.positionInitialBase = positionInitialBase;
      obj.getLinksForte(7, [pwd '\files_objects_3D\'], color);
      %obj.getFootprintRobot(obj.coordCurrent)
      obj.moveRobotForPosition(positionInitialBase, 0); %TIRAR COMENTARIO!!!
      obj.color = color;
      %{
      obj.updateBaseForte(positionInitialBase, bitMoveManipulator);
      % na condicao inicial, sempre calcular a cinematica direta e atualizar as transformacoes
      obj.forwardKinematicForte(obj.joints(2:7));
      obj.updateTransformations(); % atualizar fora, tem momentos que nao e' desejado.
      obj.updateForte(obj.joints(2:7));
      %}

    end

    function getLinksForte(obj, quantityLinks, pathFiles, color)

      offsetX2 = 179;
      obj.links(1) = patch(stlread([pathFiles 'Base_Forte.stl']), ...
                                    'FaceColor', color);
      obj.links(1).XData = offsetX2 + obj.links(1).XData;
      obj.links(1).ZData = 225      + obj.links(1).ZData;

      for i = quantityLinks : -1 : 2

        obj.links(i) = patch(stlread([pathFiles 'Link' num2str(i-1) '_Forte.stl']), ...
                                      'FaceColor', color);
        obj.links(i).XData = 250 + obj.links(i).XData;
        obj.links(i).ZData = 250 + obj.links(i).ZData;

        if (i <= 5)

          obj.wheels(i-1) = patch(stlread([pathFiles 'Roda' num2str(i-1) '.stl']), ...
                                           'FaceColor', [1 1 1]);
          set(obj.wheels(i-1), 'XData', offsetX2 + get(obj.wheels(i-1), 'XData'));
          set(obj.wheels(i-1), 'ZData', 225      + get(obj.wheels(i-1), 'ZData'));

        end

      end
      
    end

    function updateBaseForte(obj, poseRequired)

      position = obj.getPositionBase(obj.links(1).Vertices, 1);
      dist = pdist([poseRequired(1:2); position(1:2)], 'euclidean');
      theta = obj.calculeAng(position(1:2), poseRequired(1:2));
      obj.moveBaseForte(theta - obj.joints(1), 0, 1);
      obj.joints(1) = theta;

      while true

        if (dist <= 0); break; end
        position = obj.getPositionBase(obj.links(1).Vertices, 1);
        dist = pdist([poseRequired(1:2); position(1:2)], 'euclidean');
        theta = obj.calculeAng(position(1:2), poseRequired(1:2));
        obj.moveBaseForte(theta, dist, 0);

      end

      obj.coordLinks(1, :) = obj.getPositionBase(obj.links(1).Vertices, 0); %% REVER DEPOIS !!!

    end

    function ang = calculeAng(obj, ptA, ptB)

      %{
        This function is returning the angle between ptA and ptB, the
        angle in degree.
      %}

      ang = round(atan2d(ptB(2) - ptA(2), ptB(1) - ptA(1)));
      if (ang < 0) ang = ang + 360; end

    end

    function object = rotationZ(obj, object, theta, center)

      rotate(object, [0 0 1], theta, center);

    end

    function deplacement = moveBaseForte(obj, theta, velocity, isrotation)

      %{
        This function is replacing the base of the robot, doing the
        rotation when necessary.
      %}

      deplacement = velocity*[cosd(theta) sind(theta)];
      obj.offsetOrigin = obj.offsetOrigin + deplacement;
      position = obj.getPositionBase(obj.links(1).Vertices, 1);

      for i = 1 : 7
        if (isrotation)
          obj.links(i) = obj.rotationZ(obj.links(i), theta, position);
        end
        obj.links(i).XData = obj.links(i).XData + deplacement(1);
        obj.links(i).YData = obj.links(i).YData + deplacement(2);

        if (i <= 4)
          if (isrotation)
            obj.wheels(i) = obj.rotationZ(obj.wheels(i), theta, position);
          end
          obj.wheels(i).XData = obj.wheels(i).XData + deplacement(1);
          obj.wheels(i).YData = obj.wheels(i).YData + deplacement(2);
        end

      end

      drawnow;

    end

    function moveRobotForPosition(obj, position, index)

      %{
        This function is moving the robot for a required position.
      %}

      obj.updateBaseForte(position);

      if isequal(index, 0)

        % Movimentation without manipulator
        obj.forwardKinematicForte(obj.joints(2:7));
        obj.updateTransformations();
        obj.updateForte(zeros(1, 6));

      else

        % Movimentation with manipulator
        obj.joints(2:7) = obj.inverseKinematicForte(obj.coordCurrent, position, obj.joints(2:7));
        obj.updateForte(obj.joints(2:7));

      end

      %obj.updateTransformations();

    end

    function position = getPositionBase(obj, vertices, bitMiddle)

      %{
        This function is returning the middle position of the mobile
        base of the robot.
      %}
      XData = vertices(:, 1); YData = vertices(:, 2); ZData = vertices(:, 3);

      xmin = min(XData); xmin = xmin(1); xmax = max(XData); xmax = xmax(1);
      ymin = min(YData); ymin = ymin(1); ymax = max(YData); ymax = ymax(1);
      z0 = max(ZData); z0 = z0(end);

      if (bitMiddle)
        position = round([(xmax + xmin)/2, (ymax + ymin)/2, z0]);
      else
        position = round([xmax, (ymax + ymin)/2, z0]);
      end
      %hold on;
      %plot3(position(1), position(2), position(3),'*r')

    end

    function anglesJoints = controlPositionJointsForte(obj, anglesJoints)

      %{
        This function is doing the control of the limit each joint
        of the manipulator.
      %}

      upperLimitJoint    = [ 130,  60,   75,  140,  60,  120];
      inferiorLimitJoint = [-130, -50, -110, -140, -70, -120];

      for i = 1 : length(anglesJoints)

        if ( anglesJoints(i)  > upperLimitJoint(i) )

          anglesJoints(i) = upperLimitJoint(i) - 1;

        elseif ( anglesJoints(i) < inferiorLimitJoint(i) )

          anglesJoints(i) = inferiorLimitJoint(i) + 1;

        end

      end

    end

    function updateTransformations(obj)

      %{
        This function is doing the update every transformations of
        each joint of the manipulator.
      %}

      obj.T01old = obj.T01; obj.T02old = obj.T02; obj.T03old = obj.T03;
    	obj.T04old = obj.T04; obj.T05old = obj.T05; obj.T06old = obj.T06;

    end

    function updateForte(obj, joints)

      %{
        This function is doing the movimentation of the objects 3D of
        the manipulator.
      %}

    	obj.forwardKinematicForte(joints);
      j = 1; k = 0; old = 0;

      while (j ~= 7)
          if (j == 1) % j é a variavel inteira para indicar o numero da junta j.
    			  peca = obj.links(2); T1 = obj.T01old^-1; T2 = obj.T01;
    			elseif (j == 2)
    	      peca = obj.links(3); T1 = obj.T02old^-1; T2 = obj.T02;
    			elseif (j == 3)
    	      peca = obj.links(4); T1 = obj.T03old^-1; T2 = obj.T03;
          elseif (j == 4)
            peca = obj.links(5); T1 = obj.T04old^-1; T2 = obj.T04;
          elseif (j == 5)
            peca = obj.links(6); T1 = obj.T05old^-1; T2 = obj.T05;
    			elseif (j == 6)
            peca = obj.links(7); T1 = obj.T06old^-1; T2 = obj.T06;
          end

          if (old == 0)
            if (T1^-1 == T2)
              j = j + 1;
            else
              old = 1;
            end
          elseif (old == 1)
              X = get(peca, 'XData'); x1 = X(1,:);
              if size(X,1) == 3
                x2 = X(2,:); x3 = X(3,:);
              end

              Y = get(peca, 'YData'); y1 = Y(1,:);
              if size(Y,1) == 3
                y2 = Y(2,:); y3 = Y(3,:);
              end

              Z = get(peca, 'ZData'); z1 = Z(1,:);
              if size(Z,1) == 3
                z2 = Z(2,:); z3 = Z(3,:);
              end

              if size(X,1) == 1
                  objecto = [x1; y1; z1; ones(1,size(x1,2))];
                  if k == 0
                      final = T1*objecto;
                  elseif k == 1
                      final = T2*objecto;
                  end
                  set(peca, 'XData', final(1, 1:size(x1,2)),  ...
    									peca, 'YData', final(2, 1:size(x1,2)), ...
    									peca, 'ZData', final(3, 1:size(x1,2)));
              end
              if size(X,1) == 3
              	objecto = [x1 x2 x3; y1 y2 y3; z1 z2 z3; ones(1,3*size(x1,2))];
              	if k == 0
                	final = T1*objecto;
              	elseif k == 1
                	final = T2*objecto;
              	end
              	set(peca,'XData',[final(1, 1 : size(x1, 2) ); ...
                                	final(1, size(x1, 2)+1 : 2*size(x1, 2) ); ...
                                	final(1, 2*size(x1, 2)+1 : 3*size(x1, 2)) ]);
              	set(peca,'YData',[final(2, 1 : size(x1, 2)); ...
                                  final(2, size(x1, 2)+1 : 2*size(x1, 2)); ...
                                	final(2, 2*size(x1, 2)+1 : 3*size(x1, 2))]);
              	set(peca,'ZData',[final(3, 1 : size(x1, 2)); ...
                                  final(3, size(x1, 2)+1 : 2*size(x1, 2)); ...
                                	final(3, 2*size(x1, 2)+1 : 3*size(x1, 2))]);
              end

              if (j == 1) 	  obj.links(2) = peca;
              elseif (j == 2) obj.links(3) = peca;
              elseif (j == 3) obj.links(4) = peca;
              elseif (j == 4) obj.links(5) = peca;
              elseif (j == 5) obj.links(6) = peca;
    					elseif (j == 6) obj.links(7) = peca;
              end

              k = k + 1;
              if (k == 2) j = j + 1; k = 0; end
          end
      end
      obj.updateTransformations(); drawnow;

    end

    function coordCurrent = forwardKinematicForte(obj, t)

      %{
        This function calculate the FORWARD kinematic for forte manipulator.
      %}

      T1=[cosd(t(1))  -sind(t(1))  0  250+obj.offsetOrigin(1);
          sind(t(1))   cosd(t(1))  0  obj.offsetOrigin(2);
              0             0      1  193+250;
              0             0      0  1];

      T2=[sind(t(2))   cosd(t(2))      0   0;
         	 0              0            1   0;
         	cosd(t(2))   -sind(t(2))     0   0;
         	 0              0            0   1];

      T3=[cosd(t(3))  -sind(t(3))  0  190+335; %525
          sind(t(3))   cosd(t(3))  0  0;
             0             0       1  0;
             0             0       0  1];

      T4=[cosd(t(4))   -sind(t(4))     0   60+45;
           	 0              0          1   0;
         -sind(t(4))   -cosd(t(4))     0   0;
           	 0              0          0   1];

      T5 =[cosd(t(5))  -sind(t(5))  0   0;
              0              0     -1   0;
           sind(t(5))   cosd(t(5))  0   290+360;
              0              0      0   1];

      T6 =[cosd(t(6))  -sind(t(6))  0   0;
              0              0     -1   70+150;
           sind(t(6))  cosd(t(6))   0   0;
              0              0      0   1];

      obj.T01 = T1;         obj.T02 = obj.T01*T2; obj.T03 = obj.T02*T3;
      obj.T04 = obj.T03*T4; obj.T05 = obj.T04*T5; obj.T06 = obj.T05*T6;

      obj.coordLinks(2, :) = round(obj.T01(1:3, 4)');
      obj.coordLinks(3, :) = round(obj.T02(1:3, 4)');
      obj.coordLinks(4, :) = round(obj.T03(1:3, 4)');
      obj.coordLinks(5, :) = round(obj.T04(1:3, 4)');
      obj.coordLinks(6, :) = round(obj.T05(1:3, 4)');
      obj.coordLinks(7, :) = round(obj.T06(1:3, 4)');
      obj.coordOld = obj.coordCurrent;
      obj.coordCurrent = round(obj.T06(1:3, 4)');

      coordCurrent = obj.coordCurrent;

    end

    function anglesJoints = inverseKinematicForte(obj, ptStart, ptRequired, anglesJoints)

      %{
        This function calculate the INVERSE kinematic for forte manipulator.
      %}

      iterations = 0; distancePt = 1e+2; errorDist = zeros(1, 3); alpha = 0.5;
    	aux = 0; dq = []; Jm = [];

      while ( (distancePt > 0.1) )

        if (iterations > 1000)

          %angles_joints = [0 0 0 0 0 0];
          disp('Problema na trajetoria'); break;

        end

        errorDist = [ptRequired - ptStart]';
    		Jm = jacobianForte(anglesJoints); dq = alpha*Jm*errorDist;
        % update of the angles.

      	for i = 1 : length(anglesJoints)

          aux = anglesJoints(i);
    			anglesJoints(i) = round( aux + 180*(dq(i, 1))/pi );

      	end

        coord = obj.forwardKinematicForte(anglesJoints);
        distancePt = pdist([coord; ptStart], 'euclidean');
        iterations = iterations + 1; ptStart = coord;

      end

      anglesJoints = obj.controlPositionJointsForte(anglesJoints);

    end

    function getFootprintRobot(obj, point)

      t = 0 : 0.5 : 2*pi;

      obj.footprint = polyshape(point(1, 1)+obj.radiusRobot*sin(t), ...
                                point(1, 2)+obj.radiusRobot*cos(t));
      %hold on; plot(obj.footprint, 'FaceColor', obj.color);

    end

  end

end
