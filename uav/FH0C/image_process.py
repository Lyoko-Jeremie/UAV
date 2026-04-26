import cv2
import numpy as np


def image_file_2_mat(imageFileData: bytes):
    im_arr = np.frombuffer(imageFileData, dtype=np.uint8)  # im_arr is one-dim Numpy array
    img = cv2.imdecode(im_arr, flags=cv2.IMREAD_COLOR)
    return img

def write_mat_2_file(m, path: str):
    """
    :param m: jpg formatted image data
    :param path: 
    :return:
    """
    im_arr = np.frombuffer(m, dtype=np.uint8)
    cv2.imwrite(path, im_arr)
    print(f"write_mat_2_file Image saved to {path}")
    pass
