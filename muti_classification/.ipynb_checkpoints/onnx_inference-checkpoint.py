import os
import argparse

import onnxruntime
import numpy as np
import torch
import torchvision.transforms as transforms
from PIL import Image
def to_numpy(tensor):
    return tensor.detach().cpu().numpy() if tensor.requires_grad else tensor.cpu().numpy()
class_names = ["Background", "Corn_Plant", "Cucumber_Plant", "Rice_Plant", "Wheat_Plant","Corn_1", "Corn_2", "Corn_3", "Corn_4","Cucumber_1", "Cucumber_2", "Cucumber_3", "Cucumber_4","Watermelen_1", "Watermelen_2", "Watermelen_3", "Watermelen_4"]

# 模型图片输入的预处理，可以抄pth或者pt的图片处理
def get_img_tensor(img_path, use_cuda, get_size=False):
    img = Image.open(img_path)
    original_w, original_h = img.size

    img_size = (224, 224)  # crop image to (224, 224)
    img.thumbnail(img_size, Image.ANTIALIAS)
    img = img.convert('RGB')
    normalize = transforms.Normalize(
        mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    data_tf = transforms.Compose([
        # transforms.RandomResizedCrop(224),
        transforms.RandomHorizontalFlip(),
        transforms.Resize((416, 416)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])])
    img_tensor = data_tf(img)
    img_tensor = torch.unsqueeze(img_tensor, 0)
    if use_cuda:
        img_tensor = img_tensor.cuda()
    if get_size:
        return img_tensor, original_w, original_w
    else:
        return img_tensor

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--model-path', type=str,default="/root/autodl-tmp/weights/export.onnx") #储存节点的地方
    parser.add_argument('--data-path', type=str,default="/root/autodl-tmp/dataset/train/1_Corn_Plant/1073_0.jpg")
    args = parser.parse_args()
    to_tensor = get_img_tensor(args.data_path, False)
    onet_session = onnxruntime.InferenceSession(args.model_path)
    inputs = {onet_session.get_inputs()[0].name: to_numpy(to_tensor)}
    outs = onet_session.run(None, inputs)
    preds = outs[0]
    tops_type = [1]  # 输出topk的值
# np.argsort(a)  返回的是元素值从小到大排序后的索引值的数组   [::-1] 将元素倒序排列
    indexes = np.argsort(preds[0])[::-1]
    print('np.argsort(preds[0]):', np.argsort(preds[0]))
    print('np.argsort(preds[0][::-1]):', np.argsort(preds[0])[::-1])
    for topk in tops_type:
        idxes = indexes[:topk]
        print('[ Top%d Attribute Prediction ]' % topk)
        for idx in idxes:
            print(class_names[idx])
        
        
        