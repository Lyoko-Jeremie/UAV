import cv2
import numpy as np


def image_file_2_mat(imageFileData: bytes):
    im_arr = np.frombuffer(imageFileData, dtype=np.uint8)  # im_arr is one-dim Numpy array
    img = cv2.imdecode(im_arr, flags=cv2.IMREAD_COLOR)
    return img

def write_mat_2_file(m, path: str):
    """
    :param m: jpg formatted image bytes or a decoded image matrix (numpy.ndarray)
    :param path: file path to save the image, e.g., "image.jpg"
    :return:
    """
    if isinstance(m, np.ndarray):
        # m is already an image matrix (H x W x C or H x W)
        ok = cv2.imwrite(path, m)
        if not ok:
            raise IOError(f"Failed to write image matrix to {path}")
    elif isinstance(m, (bytes, bytearray, memoryview)):
        # m is encoded image data (e.g. jpg bytes); write it directly.
        with open(path, "wb") as f:
            f.write(m)
    else:
        raise TypeError(f"Unsupported image data type: {type(m)}")

    print(f"write_mat_2_file Image saved to {path}")
