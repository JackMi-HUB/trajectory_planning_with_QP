function [Q, M] = getQM(n_seg, n_order, ts)
    Q = [];
    M = [];
    % 因为我们将时间映射到[0,1]区间内，所以这个getM函数可以固化下来
    M_k = getM(n_order);
    % d_order = 4;
    for k = 1:n_seg
        %#####################################################
        % STEP 2.1 calculate Q_k of the k-th segment 
        % Q_k = [];
        fac = @(x) x*(x-1)*(x-2)*(x-3);
        Q_k = zeros(n_order+1,n_order+1);

        for i = 0:n_order
            for j = 0:n_order
                if (i < 4) || (j < 4)
                    continue;
                else
                    % 也即 i!/(i-4)! * j!/(j-4)! * 1/(i+j-7) * t^(j+i-7)
                    Q_k(i+1,j+1) = fac(i) * fac(j)/(i + j - 7) * ts(k)^(j+i-7); 
                end
            end
        end
        Q = blkdiag(Q, Q_k);
        M = blkdiag(M, M_k);
    end
end
%      for k = 1:n_seg
%         Q_k = zeros(8,8);
%         %#####################################################
%         % STEP 2.1 calculate Q_k of the k-th segment 
%         for i=4:n_order
%             for l=4:n_order
%                 L = factorial(l)/factorial(l-4);%factorial，求阶乘
%                 I = factorial(i)/factorial(i-4);
%                 Q_k(i+1,l+1) = L*I/(i+l-7)*ts(k)^(i+l-7);
%             end
%         end       
%         Q = blkdiag(Q, Q_k);
%         M = blkdiag(M, M_k);%M矩阵无需通过循环迭代，已由getM函数完整得出
%     end
% end