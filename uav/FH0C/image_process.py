import cv2
import numpy as np


def image_file_2_mat(imageFileData: bytes):
    im_arr = np.frombuffer(imageFileData, dtype=np.uint8)  # im_arr is one-dim Numpy array
    img = cv2.imdecode(im_arr, flags=cv2.IMREAD_COLOR)
    return img

def write_mat_2_file(m: np.ndarray, path: str):
    cv2.imwrite(path, m)
    pass
