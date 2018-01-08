function Code = Car_Code_Recognition(FILE,SHOW_PROCESS)
close all
clc
I=imread(FILE);
figure(1),imshow(I);title('原图')
I1=rgb2gray(I);
if(SHOW_PROCESS)
    figure(2),subplot(1,2,1),imshow(I1);title('灰度图');
    subplot(1,2,2),imhist(I1);title('灰度图直方图');
end
I2=edge(I1,'roberts',0.15,'both');

se=[1;1;1];
I3=imerode(I2,se);
se=strel('rectangle',[25,25]);
I4=imclose(I3,se);
I5=bwareaopen(I4,2000);
if(SHOW_PROCESS)
    figure(3),imshow(I2);title('robert算子边缘检测');
    figure(4),imshow(I3);title('腐蚀后图像');
    figure(5),imshow(I4);title('平滑图像的轮廓');
    figure(6),imshow(I5);title('从对象中移除小对象');
end
[y,x,z]=size(I5);
myI=double(I5);
 Blue_y=zeros(y,1);
 for i=1:y
    for j=1:x
             if(myI(i,j,1)==1) 
                Blue_y(i,1)= Blue_y(i,1)+1;%蓝色像素点统计 
            end  
     end       
 end
 [~,MaxY]=max(Blue_y);%Y方向车牌区域确定
 PY1=MaxY;
 while ((Blue_y(PY1,1)>=5)&&(PY1>1))
        PY1=PY1-1;
 end    
 PY2=MaxY;
 while ((Blue_y(PY2,1)>=5)&&(PY2<y))
        PY2=PY2+1;
 end
 IY=I(PY1:PY2,:,:);
 %%%%%% X方向 %%%%%%%%%
 Blue_x=zeros(1,x);%进一步确定x方向的车牌区域
 for j=1:x
     for i=PY1:PY2
            if(myI(i,j,1)==1)
                Blue_x(1,j)= Blue_x(1,j)+1;               
            end  
     end       
 end
  
 PX1=1;
 while ((Blue_x(1,PX1)<3)&&(PX1<x))
       PX1=PX1+1;
 end    
 PX2=x;
 while ((Blue_x(1,PX2)<3)&&(PX2>PX1))
        PX2=PX2-1;
 end
 PX1=PX1-1;%对车牌区域的校正
 PX2=PX2+1;
 dw=I(PY1:PY2-8,PX1:PX2,:);

imwrite(dw,'process\dw.jpg');
[filename,filepath]=uigetfile('process\dw.jpg','输入一个定位裁剪后的车牌图像');
strcat(filepath,filename);
a=imread('process\dw.jpg');
b=rgb2gray(a);
imwrite(b,'process\1.车牌灰度图像.jpg');

g_max=double(max(max(b)));
g_min=double(min(min(b)));
T=round(g_max-(g_max-g_min)/3); % T 为二值化的阈值
[m,n]=size(b);
d=(double(b)>=T);  % d:二值图像
imwrite(d,'process\2.车牌二值图像.jpg');
if(SHOW_PROCESS)
    figure(8);subplot(3,2,1),imshow(b),title('1.车牌灰度图像');
    figure(8);subplot(3,2,2),imshow(d),title('2.车牌二值图像')
    figure(8),subplot(3,2,3),imshow(d),title('3.均值滤波前');
end


% 滤波
h=fspecial('average',3);
d=im2bw(round(filter2(h,d)));
imwrite(d,'process\4.均值滤波后.jpg');
if(SHOW_PROCESS)
    figure(8),subplot(3,2,4),imshow(d),title('4.均值滤波后');
end

% 某些图像进行操作
% 膨胀或腐蚀
% se=strel('square',3);  % 使用一个3X3的正方形结果元素对象对创建的图像进行膨胀
% 'line'/'diamond'/'ball'...
se=eye(2); % eye(n) returns the n-by-n identity matrix 单位矩阵
[m,n]=size(d);
if bwarea(d)/m/n>=0.365
    d=imerode(d,se);
elseif bwarea(d)/m/n<=0.235
    d=imdilate(d,se);
end
imwrite(d,'process\5.膨胀或腐蚀处理后.jpg');
if(SHOW_PROCESS)
    figure(8),subplot(3,2,5),imshow(d),title('5.膨胀或腐蚀处理后');
end

% 寻找连续有文字的块，若长度大于某阈值，则认为该块有两个字符组成，需要分割
d=qiege(d);
[m,n]=size(d);
if(SHOW_PROCESS)
    figure(9),subplot(2,1,1),imshow(d),title(n);
end
k1=1;
k2=1;
s=sum(d);
j=1;
while j~=n
    while s(j)==0
        j=j+1;
    end
    k1=j;
    while s(j)~=0 && j<=n-1
        j=j+1;
    end
    k2=j-1;
    if k2-k1>=round(n/6.5)
        [~,num]=min(sum(d(:,[k1+5:k2-5])));
        d(:,k1+num+5)=0;  % 分割
    end
end
% 再切割
d=qiege(d);
% 切割出 7 个字符
y1=10;y2=0.25;flag=0;word1=[];
while flag==0
    [m,n]=size(d);
    left=1;wide=0;
    while sum(d(:,wide+1))~=0
        wide=wide+1;
    end
    if wide<y1   % 认为是左侧干扰
        d(:,[1:wide])=0;
        d=qiege(d);
    else
        temp=qiege(imcrop(d,[1 1 wide m]));
        [m,n]=size(temp);
        all=sum(sum(temp));
        two_thirds=sum(sum(temp([round(m/3):2*round(m/3)],:)));
        if two_thirds/all>y2
            flag=1;word1=temp;   % WORD 1
        end
        d(:,[1:wide])=0;d=qiege(d);
    end
end
% 分割出第二个字符
[word2,d]=getword(d);
% 分割出第三个字符
[word3,d]=getword(d);
% 分割出第四个字符
[word4,d]=getword(d);
% 分割出第五个字符
[word5,d]=getword(d);
% 分割出第六个字符
[word6,d]=getword(d);
% 分割出第七个字符
[word7,d]=getword(d);
if(SHOW_PROCESS)
subplot(5,7,1),imshow(word1),title('1');
subplot(5,7,2),imshow(word2),title('2');
subplot(5,7,3),imshow(word3),title('3');
subplot(5,7,4),imshow(word4),title('4');
subplot(5,7,5),imshow(word5),title('5');
subplot(5,7,6),imshow(word6),title('6');
subplot(5,7,7),imshow(word7),title('7');
end

[m,n]=size(word1);
% 商用系统程序中归一化大小为 40*20,此处演示
word1=imresize(word1,[40 20]);
word2=imresize(word2,[40 20]);
word3=imresize(word3,[40 20]);
word4=imresize(word4,[40 20]);
word5=imresize(word5,[40 20]);
word6=imresize(word6,[40 20]);
word7=imresize(word7,[40 20]);
if(SHOW_PROCESS)

subplot(5,7,15),imshow(word1),title('1');
subplot(5,7,16),imshow(word2),title('2');
subplot(5,7,17),imshow(word3),title('3');
subplot(5,7,18),imshow(word4),title('4');
subplot(5,7,19),imshow(word5),title('5');
subplot(5,7,20),imshow(word6),title('6');
subplot(5,7,21),imshow(word7),title('7');
end
imwrite(word1,'1.jpg');
imwrite(word2,'2.jpg');
imwrite(word3,'3.jpg');
imwrite(word4,'4.jpg');
imwrite(word5,'5.jpg');
imwrite(word6,'6.jpg');
imwrite(word7,'7.jpg');
liccode=char(['0':'9' 'A':'Z' '苏豫陕鲁']);  %建立自动识别字符代码表  
SubBw2=zeros(40,20);
l=1;
for I=1:7
      ii=int2str(I);
     t=imread([ii '.jpg']);
      SegBw2=imresize(t,[40 20],'nearest');
        if l==1                 %第一位汉字识别
            kmin=37;
            kmax=40;
        else
            if l==2             %第二位 A~Z 字母识别
             kmin=11;
             kmax=36;
        else if(l>=3)               %第三位以后是字母或数字识别
            kmin=1;
            kmax=36;
            end
            end
        end
        
        for k2=kmin:kmax
            fname=strcat('Models\',liccode(k2),'.jpg');
            SamBw2 = imread(fname);
            for  i=1:40
                for j=1:20
                    SubBw2(i,j)=abs(SegBw2(i,j)-SamBw2(i,j));
                end
            end
           % 以上相当于两幅图相减得到第三幅图
            Dmax=sum(sum(SubBw2));
            Error(k2)=Dmax;
        end
        Error1=Error(kmin:kmax);
        MinError=min(Error1);
        findc=find(Error1==MinError);
        Code(l*2-1)=liccode(findc(1)+kmin-1);
        Code(l*2)=' ';
        l=l+1;
end
figure(10),imshow(dw),title (['车牌号码:', Code],'Color','b');