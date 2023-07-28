import torch
import torchvision


class BCNN(torch.nn.Module):
    def __init__(self, num_classes, is_all):
        torch.nn.Module.__init__(self)
        self._is_all = is_all

        if self._is_all:
            # self.features = torchvision.models.vgg16(pretrained=True).features
            self.features = torchvision.models.resnet18(pretrained=True).bn1
            # self.features = torch.nn.Sequential(*list(self.features.children())[:-2])
        self.relu5_3 = torch.nn.ReLU(inplace=False)
        self.fc = torch.nn.Linear(
            in_features=512 * 512, out_features=num_classes, bias=True)
        if not self._is_all:
            self.apply(BCNN._initParameter)

    def _initParameter(module):
        if isinstance(module, torch.nn.BatchNorm2d):
            torch.nn.init.constant_(module.weight, val=1.0)
            torch.nn.init.constant_(module.bias, val=0.0)
        elif isinstance(module, torch.nn.Conv2d):
            torch.nn.init.kaiming_normal_(module.weight, a=0, mode='fan_out',
                                          nonlinearity='relu')
            if module.bias is not None:
                torch.nn.init.constant_(module.bias, val=0.0)
        elif isinstance(module, torch.nn.Linear):
            if module.bias is not None:
                torch.nn.init.constant_(module.bias, val=0.0)

    def forward(self, X):
        N = X.size()[0]
        if self._is_all:
            assert X.size() == (N, 3, 448, 448)
            X = self.features(X)
        assert X.size() == (N, 512, 28, 28)
        X = self.relu5_3(X)
        assert X.size() == (N, 512, 28, 28)
        X = torch.reshape(X, (N, 512, 28 * 28))
        X = torch.bmm(X, torch.transpose(X, 1, 2)) / (28 * 28)
        assert X.size() == (N, 512, 512)
        X = torch.reshape(X, (N, 512 * 512))
        X = torch.sqrt(X + 1e-5)
        X = torch.nn.functional.normalize(X)
        X = self.fc(X)
        return X


def BCNN_16(num_classes=8):
    return BCNN(num_classes=num_classes, is_all=True)
