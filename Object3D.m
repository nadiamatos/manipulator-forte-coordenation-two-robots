classdef Object3D < handle

  %{
    This class is for generate objects in world 3D, being obstacles or not.
  %}

  properties

    pointStart = []; pointEnd = []; quantity = 0; radius = 1000;
    mesuareObjectPointStartAndEnd = [100, 100, 0];
    circle = patch; heigthObst = 500; % object = patch;

  end

  methods

    function obj = Object3D(nameImage, quantityRobots, ptsStart, ptsEnd)

      img = Imagem(nameImage, obj.heigthObst); img.processing();
      obj.quantity = img.numObjects; t = 0 : 0.5 : 2*pi;
      obj.pointStart = ptsStart; obj.pointEnd = ptsEnd;
      radius = 0; k = 0.7; j = 0;

      for i = 1 : quantityRobots

        obj.inclusionObject3D('r', obj.mesuareObjectPointStartAndEnd, ptsStart(i, :));
        obj.inclusionObject3D('c', obj.mesuareObjectPointStartAndEnd, ptsEnd(i, :));

      end

      for i = 1 : obj.quantity
        
        radius = [max(img.boundary{i}(:, 2)) - min(img.boundary{i}(:, 2)), ...
                  max(img.boundary{i}(:, 1)) - min(img.boundary{i}(:, 1))];
        obj.circle(i) = patch(img.centroidObstacles(i).Centroid(2) + k*radius(2)*sin(t), ...
                              img.centroidObstacles(i).Centroid(1) + k*radius(1)*cos(t), 'k', 'FaceAlpha', 0.3);
        hold on; plot(img.objects{i}, 'FaceColor', 'k');
        %axis([-1000 img.dimensions(1)+2000 -1000 img.dimensions(2)+2000 0 2000]);
        axis([-1000 8000 -5000 5000 0 1200]);

      end

    end

    function final = inclusionObject3D(~, colorObject, sizeObject, ptReferenceObject)

      %{

        This function draw the objects 3D in the world.

        colorObject - it is a caracther indicating the color of the object
        sizeObject - are the mesuare of object, such as: [width, depth, heigth]
        ptReferenceObject - is the point at where the object will be draw

        final - is the patch of the object drawn.

      %}

      ptsObject = [ptReferenceObject(1) - sizeObject(1)/2, ...
                   ptReferenceObject(2) - sizeObject(2)/2, ...
                   ptReferenceObject(3) - sizeObject(3)/2];
      final = cube_plot(ptsObject, sizeObject(1), sizeObject(2), sizeObject(3), ...
                        colorObject);

    end

  end

end
