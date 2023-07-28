# model.py
import torch.nn as nn
import torch
from torch.utils.tensorboard import SummaryWriter


class AlexNet(nn.Module):
    def __init__(self, num_classes=10):
        super(AlexNet, self).__init__()
        self.features = nn.Sequential(
            nn.Conv2d(3, 64, kernel_size=3, stride=2, padding=1),
            nn.ReLU(inplace=True),  # 卷积后，.5的部分去掉，变成16*16
            nn.MaxPool2d(kernel_size=2),  # 最大池化后，16*16变成8*8
            nn.Conv2d(64, 192, kernel_size=3, padding=1),  # 还是8*8
            nn.ReLU(inplace=True),
            nn.MaxPool2d(kernel_size=2),  # 4*4了
            nn.Conv2d(192, 384, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(384, 256, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(256, 256, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(kernel_size=2),  # 到这里变成2*2了，
        )
        self.classifier = nn.Sequential(
            nn.Dropout(),
            nn.Linear(256 * 2 * 2, 4096),
            nn.ReLU(inplace=True),
            nn.Dropout(),
            nn.Linear(4096, 4096),
            nn.ReLU(inplace=True),
            nn.Linear(4096, num_classes),
        )

    def forward(self, x):
        x = self.features(x)
        x = x.view(x.size(0), 256 * 2 * 2)
        x = self.classifier(x)
        return x


def alexnet():
    return AlexNet()


if __name__ == '__main__':
    net = alexnet()
    y = net(torch.ones(1, 3, 32, 32))  # 针对CIFAR-10的训练
    writer = SummaryWriter("modellog")
    writer.add_graph(net, torch.ones(1, 3, 32, 32))
    writer.close()
    print(y.size())
