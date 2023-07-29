import os
import argparse
import shutil

import torchvision.models
from PIL import ImageFile

ImageFile.LOAD_TRUNCATED_IMAGES = True

import torch
import torch.optim as optim
from torch.utils.tensorboard import SummaryWriter
from torchvision import transforms
from torchvision.datasets import ImageFolder

from dataset import MyDataSet
from utils import read_split_data, train_one_epoch, evaluate, plot_matrix, test_for_all
from models.MobilenetV2 import mobilenetv2
from models.MobilenetV3 import mobilenetv3
from models.Mobileone import mobileone
from models.ShufflenetV2 import shufflenetv2
from models.Repvgg import RepVGG_A1, RepVGG_A0, RepVGG_A2
from models.BCNN import BCNN_16
from models.BCNN_ResNet50 import BCNN_ResNet18



import re
from torchvision.datasets import ImageFolder

def natural_sort_key(s):
    return [int(text) if text.isdigit() else text.lower()
            for text in re.split('([0-9]+)', s)]

class NaturalSortImageFolder(ImageFolder):
    def _find_classes(self, dir):
        classes = [d.name for d in os.scandir(dir) if d.is_dir()]
        classes.sort(key=natural_sort_key)
        class_to_idx = {cls_name: i for i, cls_name in enumerate(classes)}
        return classes, class_to_idx

def train(args):
    torch.cuda.empty_cache()
    device = torch.device(args.device if torch.cuda.is_available() else "cpu")

    if os.path.exists(args.save_path) is False:
        print("创建新文件夹存储权重")
        os.makedirs(args.save_path)
    else:
        print("清空之前的数据并重新训练")
        shutil.rmtree(args.save_path)
        os.makedirs(args.save_path)

    if args.tb_writer:
        tb_writer = SummaryWriter()
    img_size = args.img_size
    data_transform = {
        "train": transforms.Compose([transforms.RandomResizedCrop(img_size),
                                     transforms.RandomHorizontalFlip(),
                                     transforms.ToTensor(),
                                     transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])]),
        "val": transforms.Compose([transforms.Resize(int(img_size * 1.143)),
                                   transforms.CenterCrop(img_size),
                                   transforms.ToTensor(),
                                   transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])])}
    if args.custom_data == True:
        train_images_path, train_images_label, val_images_path, val_images_label = read_split_data(args.data_path)
        # 实例化训练数据集
        train_dataset = MyDataSet(images_path=train_images_path, images_class=train_images_label,
                                  transform=data_transform["train"])

        # 实例化验证数据集
        val_dataset = MyDataSet(images_path=val_images_path, images_class=val_images_label,
                                transform=data_transform["val"])

    else:  # 使用的传统数据集
        train_images_path = args.data_path + 'train'
        val_images_path = args.data_path + 'val'
        test_images_path = args.data_path + 'test'

        train_dataset = NaturalSortImageFolder(train_images_path, transform=data_transform["train"])
        val_dataset = NaturalSortImageFolder(val_images_path, transform=data_transform["val"])
        test_dataset = NaturalSortImageFolder(val_images_path, transform=data_transform["val"])

        train_data_size = len(train_dataset)
        val_data_size = len(val_dataset)
        print("Train_size:{}".format(train_data_size))
        print("Val_size:{}".format(val_data_size))

    batch_size = args.batch_size
    nw = min([os.cpu_count(), batch_size if batch_size > 1 else 0, 8])  # number of workers

    train_loader = torch.utils.data.DataLoader(train_dataset,
                                               batch_size=batch_size,
                                               shuffle=True,
                                               pin_memory=True,
                                               num_workers=nw
                                               )

    val_loader = torch.utils.data.DataLoader(val_dataset,
                                             batch_size=batch_size,
                                             shuffle=False,
                                             pin_memory=True,
                                             num_workers=nw
                                             )
    test_loader = torch.utils.data.DataLoader(test_dataset,
                                              batch_size=batch_size,
                                              shuffle=False,
                                              pin_memory=True,
                                              num_workers=nw
                                              )

    model = mobilenetv2(num_classes=args.num_classes).to(device)

    if args.weights != "":
        assert os.path.exists(args.weights), "weights file: '{}' not exist.".format(args.weights)
        weights_dict = torch.load(args.weights, map_location=device)
        weights_dict = weights_dict["model"] if "model" in weights_dict else weights_dict
        # 删除有关分类类别的权重
        for k in list(weights_dict.keys()):
            if "classifier" in k:
                del weights_dict[k]
        print(model.load_state_dict(weights_dict, strict=False))

    if args.freeze_layers:
        for name, para in model.named_parameters():
            # 除head外，其他权重全部冻结
            if "classifier" not in name:
                para.requires_grad_(False)
            else:
                print("training {}".format(name))

    pg = [p for p in model.parameters() if p.requires_grad]
    optimizer = optim.SGD(pg, lr=args.lr)

    best_acc = 0.
    for epoch in range(args.epochs):
        # train
        train_loss, train_acc = train_one_epoch(model=model,
                                                optimizer=optimizer,
                                                data_loader=train_loader,
                                                device=device,
                                                epoch=epoch)

        # validate
        val_loss, val_acc = evaluate(model=model,
                                     data_loader=val_loader,
                                     device=device,
                                     epoch=epoch)

        tags = ["train_loss", "train_acc", "val_loss", "val_acc", "learning_rate"]
        if args.tb_writer:
            tb_writer.add_scalar(tags[0], train_loss, epoch)
            tb_writer.add_scalar(tags[1], train_acc, epoch)
            tb_writer.add_scalar(tags[2], val_loss, epoch)
            tb_writer.add_scalar(tags[3], val_acc, epoch)
            tb_writer.add_scalar(tags[4], optimizer.param_groups[0]["lr"], epoch)

        if val_acc > best_acc:
            best_acc = val_acc
            torch.save(model, args.save_path + "best.pth")
        torch.save(model, args.save_path + "last.pth")

        if args.hide_result == False and epoch == args.epochs - 1:
            test_for_all(model=model, data_loader=test_loader, device=device, path=args.save_path)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--num_classes', type=int, default=17)
    parser.add_argument('--img-size', type=int, default=224)
    parser.add_argument('--epochs', type=int, default=30)
    parser.add_argument('--batch-size', type=int, default=32)
    parser.add_argument('--hide-result', nargs='?', const=True, default=False)
    parser.add_argument('--lr', type=float, default=0.01)
    parser.add_argument('--data-path', type=str, default="/root/autodl-tmp/dataset/")
    parser.add_argument('--custom-data', nargs='?', const=True, default=False, help='use custom dataset')
    parser.add_argument('--weights', type=str, default='', help='initial weights path')
    parser.add_argument('--tb_writer', type=bool, default=False)
    parser.add_argument('--freeze-layers', type=bool, default=False)
    parser.add_argument('--save_path', type=str, default="../weights/")
    parser.add_argument('--device', default='cuda:0', help='device id (i.e. 0 or 0,1 or cpu)')
    opt = parser.parse_args()
    train(opt)
