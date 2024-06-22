import os

import numpy as np
from pdf2image import convert_from_path
import imageio

def pdfs_to_video(pdf_folder, output_path, fps=30):
    images = []
    for filename in sorted(os.listdir(pdf_folder), key=lambda x: int(os.path.splitext(x)[0][4:])):
        if filename.endswith(".pdf"):
            pdf_path = os.path.join(pdf_folder, filename)
            print(pdf_path)
            pdf_images = convert_from_path(pdf_path)
            for image in pdf_images:
                images.append(image)

    with imageio.get_writer(output_path, mode='I', fps=fps) as writer:
        for image in images:
            print(image)
            image = np.array(image)
            writer.append_data(image)

# 调用函数进行转换
pdf_folder = "G:\Apg_proj\env_build\hier_decision\\results\episode27"  # 包含PDF文件的文件夹路径
output_path = "G:\Apg_proj\env_build\hier_decision\\results\episode27\\video.mp4"  # 输出视频的文件路径
pdfs_to_video(pdf_folder, output_path)
