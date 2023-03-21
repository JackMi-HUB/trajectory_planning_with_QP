function [o] = get_Box_hart(Box_Vertexs)%获取走廊的中心和半宽度
o=[];
for i = 1:size(Box_Vertexs)
    o=[o;(Box_Vertexs(i,1)+(Box_Vertexs(i,2)-Box_Vertexs(i,1))/2)*10,(Box_Vertexs(i,3)+(Box_Vertexs(i,4)-Box_Vertexs(i,3))/2)*10,(Box_Vertexs(i,2)-Box_Vertexs(i,1))/2*10,(Box_Vertexs(i,4)-Box_Vertexs(i,3))/2*10];
end
end