---
layout: default
title: Convolutional AutoEncooder
parent: Deep Learning
grand_parent: Artificial Intelligence
permalink: /docs/artifirial_intelligence/deep_learning/convolutional_autoencoder/
---

Convolutional Autoencoder
{: .fs-7 .fw-700 }

Convolutional Autoencoder란 Convolutional neural network(CNN)을 주요 네트워크로 사용한 Autoencoder이다. 이미지를 처리하는 Autoencoder를 만들 때 주로 사용하며, 이미지의 가장 중요한 feature를 찾고 그 feature를 활용할 수 있다. 전체적인 구조는 다음과 같다. Input image에서 Latent Space Representation을 거쳐 Reconstructed image로 가는 사이사이에 CNN이 사용되고 있다.

![convolutional ae](../../../../assets/images/artificial_intelligence/convolutional_ae.png){: width="100%" height="100%"}
   
과제: 수업시간에 배운 다양한 방법들을 이용해서 AqtoEncoder 네트워크를 완성하고, 이미지 classification 수행   
Dataset: Oxford flower 102   
   
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
   

*Encoder*는 두개의 Layer로 이루어져 있으며, 각각의 Layer는 convolutional layer와 sub-sampling(pooling) layer를 포함한다. kernel size와 output의 개수는 LeNet-5를 참고하였고 max pooling을 sub-sampling layer로 사용하였다. Convolution과 max pooling에 따른 feature map의 변화는 다음 그림과 같다.

![encoder_structure.jpg](../../../../assets/images/artificial_intelligence/encoder_structure.jpg){: width="70%" height="70%"}

이를 통해 100x100x3이었던 input image가 22x22x16의 latent code로 표현되었다.   
    
   
*Decoder*는 여섯개의 Layer로 이루어져 있으며, Transposed convolution(convolution의 역순으로, fearture로부터 이미지를 계산하는 방법. convolution 연산으로 결과값을 추론함)을 convolution으로 구현하였다. 13x13사이즈의 Kernel 6개와 19x19 사이즈의 kernel 1개를 차례대로 적용하였고, 이때 feature map 외부에 0을 채우는 zero pedding을 사용하여 차원을 늘렸다. 주의해야할 점은 zero pedding이 된 곳만 convolution하지않는 것인데, kernel size와 pedding의 개수를 적절히 조절하여 이러한 문제를 방지하였다. Convolution에 따른 feature map의 변화는 다음 그림과 같다.
   
![decoder_structure.JPG](../../../../assets/images/artificial_intelligence/decoder_structure.JPG){: width="90%" height="90%"}   
    
   
**Autoencoder 학습** 
   
위에서 만든 autoencoder 모델을 기반으로 supervised learning을 통해 autoencoder를 학습시킬 수 있다. training data의 image x가 input으로 들어가면, autoencoder의 encoder가 latent code z를 생성하고, decoder가 z를 다시 input으로 하여 새로운 image x_hat을 생성한다. 최종 목표는 input image x와 output image x_hat이 같도록 하는 것이다.   
   
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

Autoencoder 학습 시 사용하는 데이터의 batch size는 255로 설정하였고, dataloader를 통해 batch size만큼 학습데이터를 지정해주었다. 사용한 hyperprameter는 각각 epochs=100, optimizer=Adam, loss function=MSELoss이다.   
     
   
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
   
다음은 classifier 모델이다. 2개의 convolution layer와 2개의 linear layer를 사용하였으며 전체적인 구조는 다음과 같다.
   
![classifier_structure](../../../../assets/images/artificial_intelligence/classifier_structure.JPG){: width="120%" height="120%"}   
   
    
**Classfier 학습**
   
Classifier학습은 Fine-tuning autoencoder를 사용했다. 즉 미리 학습된 autoencoder weight를 task(classification)맞게 미세하게 조정하여 학습시켰다. Encoder에서 생성한 feature vector를 input으로 받으면, 그 input이 총 102개의 class 중 어디에 속해있는지에 대한 정보가 담겨있는 ouput 102개를 출력한다. 이는 unsupervised learning인 encoder부분과 supervised learning인 classifier가 결합된 semisupervised learning이다.
   
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
   
학습 시 사용한 hyperprameter로는 epochs는 60, optimizer는 Adam, loss function은 Cross-entropy Loss가 있다.

   
----

실험 결과
{: .fs-6 .fw-500 }

우선 Autoencoder 학습결과이다. 총 6149개의 테스트 이미지 중 10개를 임의로 선택하여 학습결과를 확인해보았다.

![base_model_ae_output](../../../../assets/images/artificial_intelligence/base_model_ae_output.png){: width="100%" height="100%"}

input image의 형태는 갖추고 있지만, detail한 부분까지는 생성을 하지 못하고 있는 것을 볼 수 있었다.
다음으로는 classification 학습결과이다. 

![base_model_accuracy](../../../../assets/images/artificial_intelligence/base_model_accuracy.png){: width="90%" height="90%"}

Accuracy가 약 15%로 매우 낮은 값이 나왔다.   

원인을 분석하기 위해 train loss와 test loss를 출력해 본 결과 아래와 같이 굉장히 큰 variance가 발생한 것을 볼 수 있었다. 

![base_loss_graph](../../../../assets/images/artificial_intelligence/base_loss_graph.png){: width="60%" height="60%"}

이는 overftting과 너무 복잡한 classifier를 사용한 것을 원인으로 생각할 수 있다. 이와 같은 문제를 완하시키고 성능 향상을 위해 다음과 같은 실험들을 진행하였다.   
   
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

batch normalization을 적용한 Autoencoder의 결과이다.

![batch_normalization_ae_output](../../../../assets/images/artificial_intelligence/batch_normalization_ae_output.png){: width="100%" height="100%" }

위 base network의 결과와 비교해봤을 때, 조금 더 feature가 반영된 output x_hat을 볼 수 있다.
또한 classifier의 accuracy 역시 base network보다 향상된 값을 보였다.

![batcho_normalization_accuracy](../../../../assets/images/artificial_intelligence/batch_normalization_accuracy.png){: width="80%" height="80%"}

   
**Batch normalization & drop out 적용**   
  
위 batch normalization이 accuracy를 높이는 데 큰 도움이 되었으므로, batch normalization은 유지한 채, 아래와 같이 classifier 모델의 layer에 dropout을 적용해 너무 복잡한 classifier문제를 완화하였다.   

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

![batch_normalization_n_dropout_accuracy](../../../../assets/images/artificial_intelligence/batch_normalization_&_dropout_accuracy.png){: width="80%" height="80%"}
   
실제로 batch normalization & drop out을 적용한 것이 overfitting을 얼마나 완화시켜주는지 확인해보기위해 batch normalization & drop out을 적용한 network의 train/test loss 값 역시 출력해보았다.

![better_version_loss_graph](../../../../assets/images/artificial_intelligence/better_version_loss_graph.png){: width="65%" height="65%"}

위 base model의 그래프와 비교해보았을 때 두 값의 차가 두번째 그래프가 훨씬 적다는 것을 볼 수 있다. 
   
   
**Decoder 구조 변경**
    
마지막으로 Decoder 구조를 변경해보았다. 기존의 Decoder 구조는 차원을 늘리기 위해 큰 size의 filter와(13x13 사이즈 6개, 19x19 사이즈의 kernel 1개) 많은 수의 zero padding(12개, 18개)를 사용하였다. 이는 원본 이미지의 공간적인 정보를 보존한다는 장점이 있지만, detail한 feature 정보를 잃을 수도 있다는 단점이 존재한다. 또한 학습 데이터에 비해 많은 수의 layer를 사용하였으므로 Overfitting 역시 발생할 가능성도 있다.  이를 방지하기 위해 아래와 같이 feature간의 interpolation을 사용한 Upsampling class를 통해 차원을 늘리는 방식으로 Decoder 구조를 변경해보았다.   
    
```python
...
class Decoder(nn.Module):
    def __init__(self, ):
        super(Decoder, self).__init__()
        self.decode = nn.Sequential(Upsample(),
                                    nn.Conv2d(16, 6, 5, 1, 4),
                                    nn.BatchNorm2d(6),
                                    nn.ReLU(),
                                    Upsample(),
                                    nn.Conv2d(6, 3, 5, 1, 4),
                                    nn.BatchNorm2d(3),
                                    nn.Tanh(),
        )
    def forward(self, input):
        return self.decode(input)
...
class Upsample(nn.Module):
  def __init__(self):
    super(Upsample, self).__init__()

  def forward(self, input):
    batch_size, channels, height, width = input.shape

    # Define the scale factors for height and width
    scale_factor_h = 2
    scale_factor_w = 2

    input = input.detach().cpu().numpy()
    # Reshape the tensor to a 2D array
    reshaped_tensor = np.reshape(input, (batch_size * channels, height, width))

    # Upsample using np.kron
    upsampled_tensor = np.kron(reshaped_tensor, np.ones((scale_factor_h, scale_factor_w)))

    # Reshape the upsampled tensor back to the original shape
    upsampled_tensor = np.reshape(upsampled_tensor, (batch_size, channels, height * scale_factor_h, width * scale_factor_w))
    upsampled_tensor = torch.Tensor(upsampled_tensor).to(device)

    return upsampled_tensor
```

구현한 Upsample class는 Kronecker Product를 사용하여 input feature map의 요소를 반복하며 그 크기를 2배로 확장시키는 class이다. 아래 그림과 같이 동작한다.

![upsample_example](../../../../assets/images/artificial_intelligence/upsample_example.png){: width="50%" height="50%"}
   
다음은 upsample을 통해 학습시킨 autoencoder의 출력값이다.

![upsampling_ae_output](../../../../assets/images/artificial_intelligence/upsampling_ae_output.png){: width="120%" height="120%" }

지금까지 했던 방법 중 가장 잘 input image를 detail하게 나타낸 것을 볼 수 있다.   
classifier의 accuracy 역시 약 26%로 가장 높은 값을 기록하였다. 

![upsampling_accuracy](../../../../assets/images/artificial_intelligence/upsampling_accuracy.png){: width="85%" height="85%" }







