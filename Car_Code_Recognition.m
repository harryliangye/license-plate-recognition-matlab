function Code = Car_Code_Recognition(FILE,SHOW_PROCESS)
close all
clc
I=imread(FILE);
figure(1),imshow(I);title('ԭͼ')
I1=rgb2gray(I);
if(SHOW_PROCESS)
    figure(2),subplot(1,2,1),imshow(I1);title('�Ҷ�ͼ');
    subplot(1,2,2),imhist(I1);title('�Ҷ�ͼֱ��ͼ');
end
I2=edge(I1,'roberts',0.15,'both');

se=[1;1;1];
I3=imerode(I2,se);
se=strel('rectangle',[25,25]);
I4=imclose(I3,se);
I5=bwareaopen(I4,2000);
if(SHOW_PROCESS)
    figure(3),imshow(I2);title('robert���ӱ�Ե���');
    figure(4),imshow(I3);title('��ʴ��ͼ��');
    figure(5),imshow(I4);title('ƽ��ͼ�������');
    figure(6),imshow(I5);title('�Ӷ������Ƴ�С����');
end
[y,x,z]=size(I5);
myI=double(I5);
 Blue_y=zeros(y,1);
 for i=1:y
    for j=1:x
             if(myI(i,j,1)==1) 
                Blue_y(i,1)= Blue_y(i,1)+1;%��ɫ���ص�ͳ�� 
            end  
     end       
 end
 [~,MaxY]=max(Blue_y);%Y����������ȷ��
 PY1=MaxY;
 while ((Blue_y(PY1,1)>=5)&&(PY1>1))
        PY1=PY1-1;
 end    
 PY2=MaxY;
 while ((Blue_y(PY2,1)>=5)&&(PY2<y))
        PY2=PY2+1;
 end
 IY=I(PY1:PY2,:,:);
 %%%%%% X���� %%%%%%%%%
 Blue_x=zeros(1,x);%��һ��ȷ��x����ĳ�������
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
 PX1=PX1-1;%�Գ��������У��
 PX2=PX2+1;
 dw=I(PY1:PY2-8,PX1:PX2,:);

imwrite(dw,'process\dw.jpg');
[filename,filepath]=uigetfile('process\dw.jpg','����һ����λ�ü���ĳ���ͼ��');
strcat(filepath,filename);
a=imread('process\dw.jpg');
b=rgb2gray(a);
imwrite(b,'process\1.���ƻҶ�ͼ��.jpg');

g_max=double(max(max(b)));
g_min=double(min(min(b)));
T=round(g_max-(g_max-g_min)/3); % T Ϊ��ֵ������ֵ
[m,n]=size(b);
d=(double(b)>=T);  % d:��ֵͼ��
imwrite(d,'process\2.���ƶ�ֵͼ��.jpg');
if(SHOW_PROCESS)
    figure(8);subplot(3,2,1),imshow(b),title('1.���ƻҶ�ͼ��');
    figure(8);subplot(3,2,2),imshow(d),title('2.���ƶ�ֵͼ��')
    figure(8),subplot(3,2,3),imshow(d),title('3.��ֵ�˲�ǰ');
end


% �˲�
h=fspecial('average',3);
d=im2bw(round(filter2(h,d)));
imwrite(d,'process\4.��ֵ�˲���.jpg');
if(SHOW_PROCESS)
    figure(8),subplot(3,2,4),imshow(d),title('4.��ֵ�˲���');
end

% ĳЩͼ����в���
% ���ͻ�ʴ
% se=strel('square',3);  % ʹ��һ��3X3�������ν��Ԫ�ض���Դ�����ͼ���������
% 'line'/'diamond'/'ball'...
se=eye(2); % eye(n) returns the n-by-n identity matrix ��λ����
[m,n]=size(d);
if bwarea(d)/m/n>=0.365
    d=imerode(d,se);
elseif bwarea(d)/m/n<=0.235
    d=imdilate(d,se);
end
imwrite(d,'process\5.���ͻ�ʴ�����.jpg');
if(SHOW_PROCESS)
    figure(8),subplot(3,2,5),imshow(d),title('5.���ͻ�ʴ�����');
end

% Ѱ�����������ֵĿ飬�����ȴ���ĳ��ֵ������Ϊ�ÿ��������ַ���ɣ���Ҫ�ָ�
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
        d(:,k1+num+5)=0;  % �ָ�
    end
end
% ���и�
d=qiege(d);
% �и�� 7 ���ַ�
y1=10;y2=0.25;flag=0;word1=[];
while flag==0
    [m,n]=size(d);
    left=1;wide=0;
    while sum(d(:,wide+1))~=0
        wide=wide+1;
    end
    if wide<y1   % ��Ϊ��������
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
% �ָ���ڶ����ַ�
[word2,d]=getword(d);
% �ָ���������ַ�
[word3,d]=getword(d);
% �ָ�����ĸ��ַ�
[word4,d]=getword(d);
% �ָ��������ַ�
[word5,d]=getword(d);
% �ָ���������ַ�
[word6,d]=getword(d);
% �ָ�����߸��ַ�
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
% ����ϵͳ�����й�һ����СΪ 40*20,�˴���ʾ
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
liccode=char(['0':'9' 'A':'Z' '��ԥ��³']);  %�����Զ�ʶ���ַ������  
SubBw2=zeros(40,20);
l=1;
for I=1:7
      ii=int2str(I);
     t=imread([ii '.jpg']);
      SegBw2=imresize(t,[40 20],'nearest');
        if l==1                 %��һλ����ʶ��
            kmin=37;
            kmax=40;
        else
            if l==2             %�ڶ�λ A~Z ��ĸʶ��
             kmin=11;
             kmax=36;
        else if(l>=3)               %����λ�Ժ�����ĸ������ʶ��
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
           % �����൱������ͼ����õ�������ͼ
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
figure(10),imshow(dw),title (['���ƺ���:', Code],'Color','b');