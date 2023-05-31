---
layout: default
title: Convolutional AutoEncooder
parent: Deep Learning
grand_parent: Artificial Intelligence
permalink: /docs/artifirial_intelligence/deep_learning/convolutional_autoencoder/
---

Convolutional Autoencoder
{: .fs-7 .fw-700 }

Convolutional Autoencoder란 Convolutional neural network(CNN)을 주요 네트워크로 사용한 Autoencoder이다. 이미지를 처리하는 Autoencoder를 만들 때 주로 사용하며, 이미지의 가장 중요한 feature를 찾고 그 feature를 활용할 수 있다. 전체적인 구조는 다음과 같다. Input image에서 Latent Space Representation을 거쳐 Reconstructed image로 가는 사이사이(점선)에 CNN이 사용되고 있다.

![convolutional ae](../../../../assets/images/artificial_intelligence/convolutional_ae.png){: width="60%" height="60%"}

---

코드 설명
{: .fs-6 .fw-500 }

**Autoencoder 모델**

Autoencoder는 크게 Encoder와 Decoder로 나뉜다. 아래 코드는 Encoder와 Decoder class를 각각 생성한 후, 하나의 Autoencoder 모델로 만든 것이다. 
   
```python
class Encoder(nn.Module):
    def __init__(self,):
        super(Encoder, self).__init__()
        self.encode = nn.Sequential(nn.Conv2d(3, 6, 5, 1, 0),
                                    nn.ReLU(),
                                    nn.MaxPool2d(2, 2),
                                    nn.Conv2d(6, 16, 5, 1, 0),
                                    nn.ReLU(),
                                    nn.MaxPool2d(2, 2),
        )
    def forward(self, input):
        return self.encode(input)


class Decoder(nn.Module):
    def __init__(self, ):
        super(Decoder, self).__init__()
        self.decode = nn.Sequential(nn.Conv2d(16, 16, 13, 1, 12),
                                    nn.ReLU(),
                                    nn.Conv2d(16, 16, 13, 1, 12),
                                    nn.ReLU(),
                                    nn.Conv2d(16, 6, 13, 1, 12),
                                    nn.ReLU(),
                                    nn.Conv2d(6, 6, 13, 1, 12),
                                    nn.ReLU(),
                                    nn.Conv2d(6, 6, 13, 1, 12),
                                    nn.ReLU(),
                                    nn.Conv2d(6, 3, 19, 1, 18),
                                    nn.Tanh(),
        )
    def forward(self, input):
        return self.decode(input)

class AutoEncoder(nn.Module):
    def __init__(self):
        super(AutoEncoder, self).__init__()
        self.encoder = Encoder()
        self.decoder = Decoder()

    def forward(self, input):
        z = self.encoder(input)
        x_hat = self.decoder(z)
        return z, x_hat
```

Encoder는 두개의 Layer로 이루어져 있으며, 각각의 Layer는 convolutional layer와 sub-sampling(pooling) layer를 포함한다. kernel size와 output의 개수는 LeNet-5를 참고하였다.   
   
Decoder는 여섯개의 Layer로 이루어져 있으며, Transposed convolution(convolution의 역순으로, fearture로부터 이미지를 계산하는 방법. convolution 연산으로 결과값을 추론함.)을 convolution으로 구현하였다. zero pedding을 통해 convolution을 적용하더라도 차원을 늘릴 수 있었고, kernel size를 적절히 조절하여 zero pedding된 곳만 convolution되지 않도록 주의하였다.  

**Autoencoder 학습** 

```python
batch_size = 255

train_dataloader = torch.utils.data.DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
test_dataloader = torch.utils.data.DataLoader(test_dataset, batch_size=batch_size)

autoencoder = AutoEncoder().to(device)
optimizer = optim.Adam(autoencoder.parameters(), lr=0.001)
criterion = nn.MSELoss()

epochs = 100
train_avg_costs = []

autoencoder.train()
for epoch in range(epochs):
    autoencoder.train()
    avg_cost = 0
    total_batch_num = len(train_dataloader)
    
    for b_x, b_y in train_dataloader:
        b_x = b_x.to(device)
        z, b_x_hat = autoencoder(b_x) # forward propagation
        loss = criterion(b_x_hat, b_x) # get cost
        
        avg_cost += loss / total_batch_num
        optimizer.zero_grad()
        loss.backward() # backward propagation
        optimizer.step() # update parameters
    train_avg_costs.append(avg_cost.detach())
    print('Epoch : {} / {}, cost : {}'.format(epoch+1, epochs, avg_cost))
```

학습 시 사용하는 batch size는 255로 하였고, Adam optimize


**Classifier 모델**

```python
class Classifier(nn.Module):
    def __init__(self, ):
        super(Classifier, self).__init__()
        self.classify = nn.Sequential(nn.Conv2d(16, 32, 3, 1, 0),
                                      nn.ReLU(),
                                      nn.MaxPool2d(2, 2),
                                      nn.Conv2d(32, 64, 3, 1, 0),
                                      nn.ReLU(),
                                      nn.MaxPool2d(2, 2),
                                      nn.Flatten(),
                                      nn.Linear(64*4*4, 512),
                                      nn.ReLU(),
                                      nn.Linear(512, 102)
        )
    def forward(self, input):
        return self.classify(input)
```

**classfier 학습**

```python
##### Classifier 학습 코드 #####

classifier = Classifier().to(device)
cls_criterion = nn.CrossEntropyLoss()
optimizer = optim.Adam(
    [
        {"params": autoencoder.parameters(), "lr": 0.001},
        {"params": classifier.parameters(), "lr": 0.001},
    ]
)

autoencoder.train()
classifier.train()
total_batch_num = len(train_dataloader)

epochs = 60
classifier_avg_costs = []

for epoch in range(epochs):
  avg_cost = 0

  for b_x, b_y in train_dataloader:
    b_x = b_x.to(device)
    b_y = b_y.to(device)
    z, b_x_hat = autoencoder(b_x)
    logits = classifier(z) # classification
    loss = cls_criterion(logits, b_y) # get cost

    avg_cost += loss / total_batch_num

    optimizer.zero_grad()
    loss.backward() # backward propagation
    optimizer.step() # update param

  classifier_avg_costs.append(avg_cost.detach())
  print('Epoch : {} / {}, cost : {}'.format(epoch+1, epochs, avg_cost))
```   
   
----

실험 결과
{: .fs-6 .fw-500 }

우선 Autoencoder 학습결과이다. 총 6149개의 테스트 이미지 중 10개를 임의로 선택하여 학습결과를 확인해보았다.

![base_model_autoencoder_output.png](../../../../assets/images/artificial_intelligence/base_model_qutoencoder_output.png){: width="60%" height="60%"}

이상적으로는 x와 x_hat이 거의 같아야하지만 그렇지 않은 것을 볼 수 있다. 

```yaml
Accuracy of the network on test images: 21.71084729224264 %
```

**Batch normalization 적용**

아래와 같이 encoder, decoder, classifier 모델 layer의 사이사이에 batch normalization을 적용하여 covariate shift가 커지는 것을 방지하였다.  
   
```python
class Encoder(nn.Module):
        ...
        self.encode = nn.Sequential(nn.Conv2d(3, 6, 5, 1, 0),
                                    nn.BatchNorm2d(6),
                                    nn.ReLU(),
                                    nn.MaxPool2d(2, 2),
                                    nn.Conv2d(6, 16, 5, 1, 0),
                                    nn.BatchNorm2d(16),
                                    nn.ReLU(),
                                    nn.MaxPool2d(2, 2),
        )
    ...
```

다음은 batch normalization을 적용한 Autoencoder의 결과이다.

![batch_normalization_ae_output.png](../../../../assets/images/artificial_intelligence/batch_normalization_ae_output.png){: width="60%" height="60%" }

위 base network의 결과와 비교해봤을 때, 조금 더 feature가 반영된 output x_hat을 볼 수 있다.
또한 classifier의 accuracy 역시 base network보다 향상된 값을 보였다.

```yaml
Accuracy of the network on test images: 21.71084729224264 %
```


**Batch normalization & drop out 적용**
위 batch normalization이 accuracy를 높이는 데 큰 도움이 되었으므로, batch normalization은 유지한 채, 아래와 같이 classifier 모델의 layer에 drop out을 적용해보았다.

```python
class Classifier(nn.Module):
        ...
        self.classify = nn.Sequential(
                                      nn.Conv2d(16, 32, 3, 1, 0),
                                      nn.BatchNorm2d(32),
                                      nn.ReLU(),
                                      nn.MaxPool2d(2, 2),
                                      nn.Dropout2d(0.2),
                                      nn.Conv2d(32, 64, 3, 1, 0),
                                      nn.BatchNorm2d(64),
                                      nn.ReLU(),
                                      nn.MaxPool2d(2, 2),
                                      nn.Flatten(),
                                      nn.Dropout(0.5),
                                      nn.Linear(64*4*4, 512),
                                      nn.BatchNorm1d(512),
                                      nn.ReLU(),
                                      nn.Dropout(0.25),
                                      nn.Linear(512, 102),
        )
    ...
```
그 결과 batch normalization만 적용한 것보다 더 향상 된 accuracy를 볼 수 있었다.

```yaml
Accuracy of the network on the test images: 23.288339567409334 %
```

실제로 batch normalization & drop out을 적용한 것이 overfitting을 막아주는 지 확인해보았다. 첫번째 그래프는 base network의 train/test loss 값을 나타낸 것이고 두번째 그래프는 batch normalization & drop out을 적용한 network의 train/test loss 값을 나타낸것이다. 두 값의 차가 두번째 그래프가 훨씬 적다는 것을 볼 수 있다. 

![base_loss_graph.png](../../../../assets/images/artificial_intelligence/base_loss_graph.png){: width="60%" height="60%" }
![better_version_loss_graph.png](../../../../assets/images/artificial_intelligence/better_version_loss_graph.png){: width="60%" height="60%" }
   
 
**Decoder 구조 변경**
