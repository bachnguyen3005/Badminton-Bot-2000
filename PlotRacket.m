function [RacketMesh_h] = PlotRacket
bricknum = [-0.25, 0.7, 1.2];
[f,v,data] = plyread('racket.PLY','tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
%RacketVertexCount = size(v,1);
RacketMesh_h = trisurf(f,v(:,1)+bricknum(1,1),v(:,2)+bricknum(1,2), v(:,3)+bricknum(1,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','none','EdgeLighting','none');
end


