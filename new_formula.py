import math
a=sqrt(x**2+y**2)
if(y*sin(degree))<0:
    n1=-(y*math.sin(degree))+x*math.cos(degree)
else:
    n1=(y*math.sin(degree))+x*math.cos(degree)
n2=(a**2*math.cos(degree)-x*n1)/y
