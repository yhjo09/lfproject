%  pts_r= [801, 380, 407, 1195, 1892; 1376, 1510, 1547, 1298, 718]; % a2
%  pts_l= [787, 361, 389, 1181,1876 ; 1382, 1519, 1555, 1304, 726]; % a1
% t1_max=0; t1_min=0;
% t2_max=0; t2_min=0;
a1 = imread('a1.jpg');
a2 = imread('a2.jpg');
[m, n, o]=size(a1);
%  for i=1:m
%     for j=1:n
%         t=H*[i;j;1];
%         t1_temp=t(1); 
%         t2_temp=t(2);
%         
%             if(t1_temp>t1_max)
%                 t1_max=t1_temp;        
%             end
% 
%              if(t2_temp>t2_max)
%                 t2_max=t2_temp;        
%              end
% 
%              if(t1_temp<t1_min)
%                 t1_min=t1_temp;        
%              end
% 
%              if(t2_temp<t2_min)
%                 t2_min=t2_temp;        
%             end
%     
%     end
% end
%  t1_min=round(t1_min);
%   t1_max=round(t1_max);
%  t2_min=round(t2_min);
%   t2_max=round(t2_max);
% homography_solve(pts_r, pts_l);
% [m, n, o]=size(a1); % m=1634, n=2450, o=3
new_a2=zeros(m,n,o);
for i=1:m
    for j=1:n
        t=H*[j;i;1];  % H가 2를 1로 바꾸는 매트릭스임
        t=round(t);
        if (t(2) <1 || t(1) <1 || t(2) > m || t(1) > n)
            continue
        end
        new_a2(i,j,:)=a2(t(2),t(1),:);
    end
end
% imshow(uint8(new_a1));
% [m, n, o]=size(a1); % m=1634, n=2450, o=3
new_a2_many=zeros(m,n,o);
for i=1:m
    for j=1:n
        t=H_many*[j;i;1];
        t=round(t);
        if (t(2) <1 || t(1) <1 || t(2) > m || t(1) > n)
            continue
        end
        new_a2_many(i,j,:)=a2(t(2),t(1),:);
    end
end

% f=checkerboard(50);
% t=maketform('projective',H);
% new_a1=imtransform(f, t);
% imwrite(uint8(new_a1),'homo_a1.jpg','jpg' );
% imwrite(uint8(new_a1_many),'homo_a1_many.jpg','jpg' );
% vgg_gui_H(uint8(new_a1), a2, [1 0 0; 0 1 0 ; 0 0 1]);
% tform=maketform('projective',v);
% g3=imtransform(a1,tform);