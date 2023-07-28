import os
import argparse

from PIL import ImageFile

ImageFile.LOAD_TRUNCATED_IMAGES = True

import torch
import torch.optim as optim
from torch.utils.tensorboard import SummaryWriter
from torchvision import transforms
from torchvision.datasets import ImageFolder

from dataset import MyDataSet
from utils import read_split_data, test_for_all
from models.MobilenetV2 import mobilenetv2
from models.Mobileone import mobileone


def val(args):
    torch.cuda.empty_cache()
    device = torch.device(args.device if torch.cuda.is_available() else "cpu")
    img_size = args.img_size
    data_transform = {
        "test": transforms.Compose([transforms.Resize(img_size),
                                    transforms.CenterCrop(img_size),
                                    transforms.ToTensor(),
                                    transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])])}
    if args.custom_data == True:
        val_images_path, val_images_label = read_split_data(args.data_path)
        val_dataset = MyDataSet(images_path=val_images_path, images_class=val_images_label,
                                transform=data_transform["test"])
    else:
        val_images_path = args.data_path + 'test'
        val_dataset = ImageFolder(val_images_path, transform=data_transform["test"])
    batch_size = args.batch_size
    nw = min([os.cpu_count(), batch_size if batch_size > 1 else 0, 8])  # number of workers
    val_loader = torch.utils.data.DataLoader(val_dataset, batch_size=batch_size, shuffle=False, pin_memory=True,
                                             num_workers=nw)
    model = torch.load(args.model_path)
    test_for_all(model=model, data_loader=val_loader, device=device, path=args.save_path)
    print("Test Finish!")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--num_classes', type=int, default=8)
    parser.add_argument('--model-path', type=str, default="../weights/last.pth")
    parser.add_argument('--img-size', type=int, default=448)
    parser.add_argument('--batch-size', type=int, default=16)
    parser.add_argument('--data-path', type=str, default="/root/autodl-tmp/dataset/")
    parser.add_argument('--custom-data', nargs='?', const=True, default=False, help='use custom dataset')
    parser.add_argument('--save_path', type=str, default="../weights/")
    parser.add_argument('--device', default='cuda:0', help='device id (i.e. 0 or 0,1 or cpu)')
    opt = parser.parse_args()
    val(opt)
