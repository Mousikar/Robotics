# TensorFlow学习代码
## 环境：jupyter notebook
内容：  
梯度下降  
sklearn
动态调整学习速率
感知机  
逻辑回归  
解析数据集  
多分类逻辑回归  
手写数字识别  

TensorFlow入门  

### TensorFlow环境安装
pycharm→file→settings→Project interpreter→点“设置”图标构建Python的虚拟环境→确定
pycharm→Teminal→ 更新pip在虚拟环境目录更新pip：python -m pip install --upgrade pip→pip install tensorflow 
报错则键入  
“sudo pip install six --upgrade --ignore-installed six”  
或“pip install -U --ignore-installed wrapt enum34 simplejson netaddr”

### 推荐西瓜书

### 推荐网站
1. 目前有超级都的learningrate优化的框架和论文：  
[adam](https://ruder.io/optimizing-gradient-descent/)  
[adaptive moment estimation](https://arxiv.org/pdf/1412.6980.pdf)  
[adagrad](https://medium.com/konvergen/an-introduction-to-adagrad-f130ae871827)  
[RMSProp](https://towardsdatascience.com/understanding-rmsprop-faster-neural-network-learning-62e116fcf29a)  
[Momentum](https://engmrk.com/gradient-descent-with-momentum/)  

2. 高级API  
https://keras.io/api  
[优化器](https://keras.io/api/optimizers/)  
[论文](https://paper.nips.cc/paper/7003-the-marginal-value-of-adaptive-gradient-methods-in-machine-learning.pdf)
[https://arxiv.org/pdf/1705.08292 ](https://arxiv.org/pdf/1705.08292)

3. 加载读取数据集

数据集网站：http://yann.lecun.com/exdb/mnist/

框图网站：http://alexlenail.me/NN-SVG/index.html

[http://playground.tensorflow.org/](http://playground.tensorflow.org/#activation=tanh&batchSize=10&dataset=circle&regDataset=reg-plane&learningRate=0.03&regularizationRate=0&noise=0&networkShape=4,2&seed=0.34480&showTestData=false&discretize=false&percTrainData=50&x=true&y=true&xTimesY=false&xSquared=false&ySquared=false&cosX=false&sinX=false&cosY=false&sinY=false&collectStats=false&problem=classification&initZero=false&hideText=false)

http://ais.uni-bonn.de/papers/icann2010_maxpool.pdf