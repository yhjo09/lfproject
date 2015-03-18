% Example of using vgg_Haffine_from_x_MLE

% data is square, rotated square

% exem1 = [ 0 1 1 0 ; 0 0 1 1 ]; % a1
% exem2 = [ 0.5 1 0.5 0; 0 0.5 1 0.5 ]; % a2

% 내 데이터
 exem1= [449 1797 1595 391; 49 317 1550 1557];
 exem2= [464 1811 1610 410; 45 308 1543 1547];
% 자동추출
 exem1_many= (mph1(:,1:2))';
 exem2_many= (mph2(:,1:2))';

H = vgg_Haffine_from_x_MLE(vgg_get_homg(exem1) ,vgg_get_homg(exem2)); % 2를 1로 바꾸는거
H_many = vgg_Haffine_from_x_MLE(vgg_get_homg(exem1_many) ,vgg_get_homg(exem2_many));
disp('Affine H relating two noise free point sets:');
disp(H);

% % add noise
% 
% dat = 0.05 * randn(2,4);
% exem1 = exem1 + dat;
% 
% dat = 0.05 * randn(2,4);
% exem2 = exem2 + dat;

H2 = vgg_Haffine_from_x_MLE(vgg_get_homg(exem1) ,vgg_get_homg(exem2));
disp(' ')
disp('MLE estimate of affine H relating the point sets with added noise:');
disp(H2);
a1 = imread('a1.jpg');
a2 = imread('a2.jpg');
% 임의로 추가한거
% vgg_gui_H(a1, a2, H);