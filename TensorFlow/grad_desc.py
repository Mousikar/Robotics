#-*- codeing = utf-8 -*-
#@Time : 2022/1/30 16:54
#@Author : 任萌
#@File : grad_desc.py.py
#@Software : PyCharm

'''
梯度下降步骤
随机选取一个b值
计算这个b值对应的mse的斜率
如果mse的斜率非常大，那要根据mse的斜率，去修改b的值
mse值越大，b的修改值越大，mse斜率为正，b需要减少，mse斜率为负，b需要增加
选取一个很小的值，叫learningrate，学习速率，可以理解为迈开的步伐
用learningrate和mse的信号去更新b的值
重复2-6的步骤，直到mse的斜率接近于0
'''

'''
import numpy as np
import matplotlib.pyplot as plt
data = np.array([
    [80,200],
    [95,230],
    [104,245],
    [112,247],
    [125,259],
     [135,262]
])

b = 1
learningrate = 0.0001

def gradentdecent():
    global b
    slop = 0
    for item in data[:,1]:
        slop = slop + (b - item)
    b = b -slop*learningrate
    print("slop={},b={}".format(slop,b))

gradentdecent()
for i in range(10000):
    gradentdecent()
'''


#图表显示
'''
import numpy as np
data = np.array([
    [80,200],
    [95,230],
    [104,245],
    [112,247],
    [125,259],
     [135,262]
])
bhistory = []
msehistory = []

b = 1
learningrate = 0.5

#信号量：mse的变化率，mse求导：2（预测值-真实值）,学习速率（步伐）：0.01，预测值-变化率*学习速率
#为什么要有learningrate，因为要有一个步伐，不能太大，不能太小，要符合下降梯度
def gradentdecent():
    global b
    slop = 0
    mse = 0
    for item in data[:,1]:
        slop = slop + (b - item)
        mse = mse + (b-item)**2
    b = b -slop*learningrate
    print("slop={},b={},mse={}".format(slop,b,mse))
    bhistory.append(b)
    msehistory.append(mse)

for i in range(100):
    gradentdecent()

import matplotlib.pyplot as plt
import numpy as np
fig = plt.figure(figsize=(6, 6), dpi=80)

plt.scatter(bhistory,msehistory)
plt.show()
'''


#为什么要用梯度下降，不直接算出来？
#因为实际情况不可能这么简单，维度也不仅仅是2二维
#费马大定律

#分别对mx+b中的m和b求偏导数
'''
import numpy as np
data = np.array([
    [80,200],
    [95,230],
    [104,245],
    [112,247],
    [125,259],
     [135,262]
])
mhistory = []
bhistory = []
msehistory = []

b = 1
m = 1
learningrate = 0.00002

def gradentdecent():
    global b ,m
    bslop = 0
    mslop = 0
    mse = 0
    for index ,item in enumerate(data[:,1]):
        bslop = bslop + (b - item + data[:,0][index] * m)
        mslop = mslop + (b - item + data[:,0][index] * m)*data[:,0][index]
        mse = mse + (m*data[:,0][index] + b-item)**2
    b = b -bslop*learningrate
    m = m - mslop*learningrate
    return mse

i = 0
for i in range(2000000):
    mse = gradentdecent()
    i = i+1
    if(i%100000 == 0):
        print("b={},m={},mse={}".format(b,m,mse))
'''