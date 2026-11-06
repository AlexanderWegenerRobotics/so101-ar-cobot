import cv2
import numpy as np
import qrcode

FAMILIES = {
    "tag36h11": cv2.aruco.DICT_APRILTAG_36h11,
    "tag25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "tag16h5": cv2.aruco.DICT_APRILTAG_16h5,
}


def tag_face_image(family, tag_id, face_fraction=1.0, margin_rgb=(255, 255, 255), cell_px=16):
    dictionary = cv2.aruco.getPredefinedDictionary(FAMILIES[family])
    marker_cells = dictionary.markerSize + 2
    marker = cv2.aruco.generateImageMarker(dictionary, tag_id, marker_cells * cell_px)
    tag = cv2.copyMakeBorder(marker, cell_px, cell_px, cell_px, cell_px, cv2.BORDER_CONSTANT, value=255)
    tag = cv2.cvtColor(tag, cv2.COLOR_GRAY2RGB)
    face_px = int(round(tag.shape[0] / face_fraction))
    pad = (face_px - tag.shape[0]) // 2
    face = np.empty((tag.shape[0] + 2 * pad,) * 2 + (3,), dtype=np.uint8)
    face[:] = margin_rgb
    face[pad:pad + tag.shape[0], pad:pad + tag.shape[0]] = tag
    return face


def black_edge(family, printed_size):
    dictionary = cv2.aruco.getPredefinedDictionary(FAMILIES[family])
    cells_black = dictionary.markerSize + 2
    return printed_size * cells_black / (cells_black + 2)


def qr_face_image(payload, border_modules=2, module_px=16):
    qr = qrcode.QRCode(error_correction=qrcode.constants.ERROR_CORRECT_M, border=border_modules)
    qr.add_data(payload)
    qr.make(fit=True)
    cells = np.array(qr.get_matrix(), dtype=bool)
    img = np.where(cells, 0, 255).astype(np.uint8)
    img = np.kron(img, np.ones((module_px, module_px), dtype=np.uint8))
    return cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
