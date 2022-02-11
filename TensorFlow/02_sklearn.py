#-*- codeing = utf-8 -*-
#@Time : 2022/2/9 18:16
#@Author : 任萌
#@File : sklearn.py.py
#@Software : PyCharm

## sk-learn常见的机器学习框架
#导包 导入线性回归的模型
'''
from sklearn.linear_model import LinearRegression
import numpy as np
#实例化模型
model = LinearRegression()
#准备数据
feature=np.array([80,95,104,112,125,135])
label=np.array([200,230,245,247,259,262])
#模型训练
model.fit(feature,label)
print(model.coef_)#斜率
print(model.intercept_)#截距
## 预测
model.predict(np.array([[80]]))

'''

import numpy as np
data = np.array([
    [9,0.5],
    [10,9.36],
    [11,52],
    [12,191],
    [13,350.19],
    [14,571],
    [15,912.17],
    [16,1207],
    [17,1682],
    [18,2135]
])

x = data[:,0]
actual = data[:,1]
import matplotlib.pyplot as plt
fig=plt.figure(figsize=(6,6),dpi=80)
plt.scatter(x,actual)
plt.show()

np.set_printoptions(suppress=True)
from sympy import *  #导入计算库
m_1, m_2,m_3, b = symbols('m_1, m_2, m_3, b') #声明变量x,y,z
init_printing(pretty_print=True) #初始化latex显示

from sympy import *  #导入计算库
x_i,Actual_i,i= symbols('x_i,Actual_i,i') #声明变量x,y,z
init_printing(pretty_print=True) #初始化latex显示
expr = Sum((m_1*x_i**3 + m_2*x_i**2+m_3*x_i + b-Actual_i)**2, (i, 2009, 2018))

print(expr)
