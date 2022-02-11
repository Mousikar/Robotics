import numpy as np
import matplotlib.pyplot as plt


data=np.array([
    [80,200],
    [95,230],
    [104,245],
    [112,247],
    [125,259],
    [135,262]
])
'''
fig=plt.figure(figsize=(6,6),dpi=80)
plt.xlim(70,140)
plt.ylim(0,300)
plt.scatter(data[:,0],data[:,1],c='r',marker='o',label='like')

x=np.linspace(70,140,7)
price=np.ones(7)*100
plt.plot(x,price)

plt.annotate(r'$y=0x+100$',
             xy=(100,100),xytext=(+10,+50),
             textcoords='offset points',fontsize=12,
             arrowprops=dict(arrowstyle='->'))      #标注，画箭头
'''
#MAE:mean absolute error最小绝对差
#问题：线在最中央会不变，无数解
#MSE:mean square error最小均方差
#长度的平方，面积和，最小二乘法
'''
fig = plt.figure(figsize=(6, 6), dpi=80)
X = np.linspace(1, 400, 400)
for b in X:
    mse = np.sum(np.power((b - data[:,1]),2))
    print("b={},mse={}".format(b,mse))
    plt.scatter(b,mse,c='r',marker='o',label='like')
plt.show()
'''

#信号量：mse的变化率，mse求导：2（预测值-真实值）,学习速率（步伐）：0.01，预测值-变化率*学习速率
'''
from sympy import *  #导入计算库
i, n, b ,Actual_i= symbols('i, n, b ,Actual_i') #声明变量x,y,z
init_printing(pretty_print=True) #初始化latex显示
expr = Sum((b-Actual_i)**2, (i, 1, 10))
diff(expr,b)
'''
