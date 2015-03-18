matchfeature % feature 찾기

Haffine_from_x_MLE_ex % 특징점들로부터 Homography matrix 찾기
disp('안끝났음. 기다리세요')
make_homography_image %  구해진 H행렬로 부터 새로운 이미지 만들기




% % Newimage11=homo_a1-a2;
% Newimage10=uint8(new_a2_many)/2+a1/2;
% figure;
%  imshow(uint8(Newimage10));title('특징점 200여개');
% imwrite(uint8(Newimage10), 'sum_4point.jpg','jpg');

Newimage11=uint8(new_a2)/2+a1/2;
figure;
  imshow(uint8(Newimage11));title('특징점 여러개');
imwrite(uint8(Newimage11), 'sum_manypoint.jpg','jpg');

Newimage12=a1/2+a2/2;
figure;
 imshow(uint8(Newimage12));title('a1, a2 그냥 더함');
imwrite(uint8(Newimage12), 'sum_a1plusa2.jpg','jpg');