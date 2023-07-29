import os
import argparse
import shutil
from sklearn.model_selection import train_test_split

def split_dataset(dataset_path, output_path, train_size):
    if not os.path.exists(output_path):
        os.makedirs(output_path)

    classes = os.listdir(dataset_path)
    for c in classes:
        if not os.path.exists(os.path.join(output_path, 'train', c)):
            os.makedirs(os.path.join(output_path, 'train', c))
        if not os.path.exists(os.path.join(output_path, 'val', c)):
            os.makedirs(os.path.join(output_path, 'val', c))

        images = os.listdir(os.path.join(dataset_path, c))
        train_images, val_images = train_test_split(images, train_size=train_size)

        for img in train_images:
            shutil.copy(os.path.join(dataset_path, c, img), os.path.join(output_path, 'train', c, img))
        for img in val_images:
            shutil.copy(os.path.join(dataset_path, c, img), os.path.join(output_path, 'val', c, img))

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset_path', default='/root/autodl-tmp/dataset_orgin')
    parser.add_argument('--output_path', default='/root/autodl-tmp/dataset')
    parser.add_argument('--train_size', default=0.8, type=float)
    args = parser.parse_args()
    split_dataset(args.dataset_path, args.output_path, args.train_size)
