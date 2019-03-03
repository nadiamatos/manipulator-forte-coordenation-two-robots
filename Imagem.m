classdef Imagem < handle

  properties

    dimensions = 10*ones(1, 2); name = [];
    image = []; maxGray = 255; minGray = 0;
    obstacles = polyshape;
    objects = {}; boundary = []; numObjects = 0;
    centroidObstacles = []; heightObstacles = 100;
    
  end

  methods

    function obj = Imagem(name, heightObstacles)

      obj.heightObstacles = heightObstacles;
      obj.name = name;
      obj.image = obj.maxGray*ones(obj.dimensions(1), obj.dimensions(2));

    end

    function read(obj)

      aux = rgb2gray(imread(obj.name)); obj.image = imbinarize(aux);
      [obj.boundary, ~, obj.numObjects] = bwboundaries(obj.image);
      obj.dimensions = size(obj.image);

    end

    function write(obj, data)

      if (~isempty(data)); obj.image = data; end
      obj.image = uint8(mat2gray(obj.image));
      imwrite(obj.image, ['out' obj.name]);

    end

    function processing(obj)

      obj.read();
      obj.centroidObstacles = regionprops(obj.image, 'centroid');

      %{
      rotate3d on; grid on;
      set(get(gca, 'XLabel'), 'String', 'Axis - X');
      set(get(gca, 'YLabel'), 'String', 'Axis - Y');
      set(get(gca, 'ZLabel'), 'String', 'Axis - Z');
      axis([0 obj.dimensions(1) 0 obj.dimensions(2) 0 5000]);
      %}

      for i = 1 : obj.numObjects

        obj.centroidObstacles(i).Centroid = round(obj.centroidObstacles(i).Centroid);
        obj.objects{i} = alphaShape(repmat(obj.boundary{i}(:, 1), 2, 1), ...
                                    repmat(obj.boundary{i}(:, 2), 2, 1), ...
                                    [zeros(size(obj.boundary{i}, 1), 1); ...
                                    obj.heightObstacles*ones(size(obj.boundary{i}, 1), 1)]);
        %hold on; plot(obj.objects{i}, 'FaceColor', 'k');
        %disp(obj.centroidObstacles(i).Centroid)
        
      end

    end

  end

end
