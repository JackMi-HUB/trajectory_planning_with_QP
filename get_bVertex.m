function[temp] = get_bVertex(costmap_)
temp = [];
for i = 1:201
    k = i+1;
    if(k>=201)
        k=200;
    end   
    for j = 1:201
        h = j+1;
        if(h>=201)
            h=200;
        end
     
        if(i == 1 && j == 1)%对四个顶点进行遍历:左上
            if(costmap_(i,j) == 1)
                temp = [i,j];
            end
        end
        if(i == 201 && j == 1)%左下
            if(costmap_(i,j) == 1)
                temp = [temp;i,j];
            end
        end        
        if(i == 1 && j == 201)%右上
            if(costmap_(i,j) == 1)
                temp = [temp;i,j];
            end
        end
        if(i == 201 && j == 201)%右下
            if(costmap_(i,j) == 1)
                temp = [temp;i,j];
            end
        end

        if( i==1 && costmap_(1,h) == 1)%对于边缘的检测：up
            if~(costmap_(1,h-1) == 1 && costmap_(1,h+1) ==1 && costmap_(2,h) ==1)
                temp = [temp;1,h];
            end
        end          
        if( i==201 && costmap_(201,h) == 1)%对于边缘的检测：down
            if~(costmap_(201,h-1) == 1 && costmap_(201,h+1) ==1 && costmap_(200,h) ==1)
                 temp = [temp;201,h];
            end
        end          

        if(j==1 && costmap_(k,1) == 1)%对于边缘的检测：left
           if~(costmap_(k-1,1) == 1 && costmap_(k+1,1) ==1 && costmap_(k,2) ==1)
                 temp = [temp;k,1];
           end
        end          
        if(j==201 && costmap_(k,201) == 1)%对于边缘的检测：right
            if~(costmap_(k+1,201) == 1 && costmap_(k-1,201) ==1 && costmap_(k,200) ==1)
                 temp = [temp;k,201];
            end
        end
        if(costmap_(k,h) == 1)
            if~((costmap_(k-1,h-1) == 1 && costmap_(k+1,h+1) == 1) || (costmap_(k+1,h-1) == 1 && costmap_(k-1,h+1) == 1) || (costmap_(k,h-1) == 1 && costmap_(k,h+1) == 1) || (costmap_(k+1,h) == 1 && costmap_(k-1,h) == 1))
                temp = [temp;k,h];
            end
        end
    end
end
end