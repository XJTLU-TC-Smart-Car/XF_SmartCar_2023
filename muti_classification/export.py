import argparse

import torch

def export(args):
        model_path = args.model_path
        model = torch.load(model_path)
        torch.onnx.export(
                model,
                torch.randn(1, 3, args.img_size, args.img_size, device=args.device),
                args.save_path + 'export.onnx',
                verbose=args.verbose,
                export_params=True,
                opset_version=9,
                input_names=['input'],
                output_names=['output'],
                do_constant_folding=args.do_constant_folding)
        print("Export Done!")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--num_classes', type=int, default=8)
    parser.add_argument('--img-size', type=int, default=416)
    parser.add_argument('--verbose', nargs='?', const=True, default=False)
    parser.add_argument('--do_constant_folding', nargs='?', const=True, default=False)
    parser.add_argument('--model_path', type=str, default="/root/autodl-tmp/success_smart_model/torch_mobilenetv3_416/best.pth")
    parser.add_argument('--weights', type=str, default='', help='initial weights path')
    parser.add_argument('--save_path', type=str, default="/root/autodl-tmp/success_smart_model/torch_mobilenetv3_416/")
    parser.add_argument('--device', default='cuda:0', help='device id (i.e. 0 or 0,1 or cpu)')
    opt = parser.parse_args()
    export(opt)