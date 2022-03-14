import ipdb
import sys
import cv2
import h5py
import numpy as np
from common import plot_helpers
import pathlib
import json
from common.realsense_params import MIN_VALID_DEPTH_METERS
from common.realsense_params import MAX_VALID_DEPTH_METERS
from tqdm import tqdm
from scipy.spatial.distance import directed_hausdorff

MIN_VALID_DEPTH_MM = MIN_VALID_DEPTH_METERS*1000
MAX_VALID_DEPTH_MM = MAX_VALID_DEPTH_METERS*1000


def auto_canny(image, sigma=0.33):
    # compute the median of the single channel pixel intensities
    v = np.median(image)
    # apply automatic Canny edge detection using the computed median
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image, lower, upper)
    # return the edged image
    return edged


subj = 'QKZF-SWAL-UJPN'
hand = 'left'
file_name = f'/home/mjsobrep/Downloads/data/{subj}/target-touch/full_data-vid.hdf5'
cam = 'upper'
data = h5py.File(file_name, 'r')
color_idx = 100
depth_idx = data['vid/lower/color/matched_depth_index'][color_idx]

img_stacked = data['vid/lower/color/data'][color_idx]
# img_stacked = np.median(img_stacked, axis=0).astype('uint8')

# cv2.imshow('stacked color', img_stacked)

img_stacked_gray = cv2.cvtColor(img_stacked, cv2.COLOR_BGR2GRAY)
img_stacked_smooth = cv2.GaussianBlur(img_stacked_gray, (5, 5), 0)
img_stacked_canny = auto_canny(img_stacked_smooth)
# cv2.imshow('color 9 canny', img_stacked_canny)

# depth_img_stacked = data['vid/lower/depth/data'][depth_idx-4:depth_idx+4]
# depth_img_stacked = np.median(depth_img_stacked, axis=0).astype('uint16')
depth_img_stacked = data['vid/lower/depth/data'][depth_idx]
# depth_img_stacked = cv2.bilateralFilter(
#     depth_img_stacked.astype('float32'), 9, 75, 75).astype('uint16')
cv2.imshow('Depth color 9 image median', depth_img_stacked)


# for each zero pixel, find nearest non-zero pixel in each direction and take max
# shaddows always occur at the edge of objects. The correct value is the missing
# bacground value

# TODO: optimize this
# valid[v_idx] = ((depth_img[v_row, v_col] > MIN_VALID_DEPTH_MM) &
#                 (depth_img[v_row, v_col] < MAX_VALID_DEPTH_MM))
# depth_img_stacked

depth_img_stacked[depth_img_stacked < MIN_VALID_DEPTH_MM] = 0
depth_img_stacked[depth_img_stacked > MAX_VALID_DEPTH_MM] = 0

# new_img = depth_img_stacked.copy()
for loc in tqdm(np.array(np.where(depth_img_stacked == 0)).T, desc='filling depth holes'):
    # loc[0]: row
    # loc[1]: col
    # depth_img_stacked.shape->(height, width)
    # depth_img_stacked[row, col]
    surrounding = []
    # horizontal
    idx = loc[0]
    last_val = 0
    while idx < depth_img_stacked.shape[0]-1 and last_val == 0 and idx-loc[0] < 50:
        idx = idx+1
        last_val = depth_img_stacked[idx, loc[1]]
    if last_val != 0:
        surrounding.append(last_val)
    idx = loc[0]
    last_val = 0
    while idx > 0 and last_val == 0 and loc[0]-idx < 50:
        idx = idx - 1
        last_val = depth_img_stacked[idx, loc[1]]
    if last_val != 0:
        surrounding.append(last_val)
    idx = loc[1]
    last_val = 0
    while idx < depth_img_stacked.shape[1]-1 and last_val == 0 and idx-loc[1] < 50:
        idx = idx+1
        last_val = depth_img_stacked[loc[0], idx]
    if last_val != 0:
        surrounding.append(last_val)
    idx = loc[1]
    last_val = 0
    while idx > 0 and last_val == 0 and loc[1]-idx < 50:
        idx = idx - 1
        last_val = depth_img_stacked[loc[0], idx]

    if len(surrounding):
        depth_img_stacked[loc[0], loc[1]] = np.max(surrounding)
        # new_img[loc[0], loc[1]] = 60000   # np.max(surrounding)


cv2.imshow('Depth color 9 image median - filled', depth_img_stacked)
cv2.waitKey(0)
cv2.destroyAllWindows()
# depth_img_stacked_stretched = plot_helpers.stretch_histogram(depth_img_stacked)
# depth_img_stacked_stretched = cv2.convertScaleAbs(
#     depth_img_stacked_stretched, alpha=(0xFF/0xFFFF))
# cv2.imshow('Depth color 9 image median - stretched',
#            depth_img_stacked_stretched)

depth_img_stacked = cv2.convertScaleAbs(depth_img_stacked, alpha=(0xFF/0xFFFF))
cv2.imshow('Depth color 9 image median - converted', depth_img_stacked)

depth_img_stacked_smooth = cv2.GaussianBlur(depth_img_stacked, (3, 3), 0)
depth_img_stacked_canny = auto_canny(depth_img_stacked_smooth)
# cv2.imshow('Depth color 9 - converted - canny', depth_img_stacked_canny)

# # depth_img_stacked_smooth = cv2.GaussianBlur(
# #     depth_img_stacked_stretched, (3, 3), 0)
# # depth_img_stacked_canny = auto_canny(depth_img_stacked_smooth)
# # cv2.imshow('Depth color 9 - stretched - canny', depth_img_stacked_canny)


sift = cv2.SIFT_create(nfeatures=10000, nOctaveLayers=10)
kp1, des1 = sift.detectAndCompute(img_stacked, None)
kp2, des2 = sift.detectAndCompute(depth_img_stacked, None)

cv2.imshow('color_sift', cv2.drawKeypoints(img_stacked, kp1,
                                           None, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS))
cv2.imshow('depth_sift', cv2.drawKeypoints(depth_img_stacked,
                                           kp2, None, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS))
cv2.waitKey(0)
cv2.destroyAllWindows()

kph1 = cv2.cornerHarris(img_stacked_canny, 2, 3, 0.04)
kph2 = cv2.cornerHarris(depth_img_stacked_canny, 2, 3, 0.04)

img_harris = cv2.cvtColor(img_stacked_canny, cv2.COLOR_GRAY2RGB)
depth_harris = cv2.cvtColor(depth_img_stacked_canny, cv2.COLOR_GRAY2RGB)
img_harris[kph1 > 0.01*kph1.max()] = [255, 0, 0]
depth_harris[kph2 > 0.01*kph2.max()] = [255, 0, 0]
cv2.imshow('color harris', img_harris)
cv2.imshow('depth harris', depth_harris)

cv2.waitKey(0)
cv2.destroyAllWindows()

directory = pathlib.Path('/home/mjsobrep/Downloads/data/')
with open(directory/'transforms.json', encoding='utf-8') as json_file:
    transforms = json.load(json_file)

trans = (np.array([trans['translation']
                   for trans in transforms['robot'][cam].values()]))
rot = (np.array([trans['rotation']
                 for trans in transforms['robot'][cam].values()]))

# TODO: construct H matrix and use cv2 built in cv2.warpPerspective
r_cd = np.asarray(rot[0]).reshape(3, 3)
t_cd = np.asarray(trans[0]).reshape(3, 1)
k_c = data['vid/lower/color/data'].attrs['K'].reshape(3, 3)
inv_kc = np.linalg.inv(k_c)
k_d = data['vid/lower/depth/data'].attrs['K'].reshape(3, 3)
edge_points = np.array(np.where(img_stacked_canny)).T
edge_points = np.concatenate(
    (edge_points, np.ones((len(edge_points), 1))), axis=1)
edge_points_in_depth = (
    k_d @ (r_cd.T @ ((inv_kc @ edge_points.T) - t_cd))).T

for col in range(3):
    edge_points_in_depth[:, col] = (
        edge_points_in_depth[:, col] /
        edge_points_in_depth[:, 2])
idx = np.round(edge_points_in_depth).astype('uint16')
idx = idx[:, :2]
depth_shape = depth_img_stacked_canny.shape

color_in_depth_img = np.zeros(depth_img_stacked_canny.shape)
idx = idx[np.all(idx >= 0, axis=1)]
idx = idx[idx[:, 0] < depth_shape[0]]
idx = idx[idx[:, 1] < depth_shape[1]]
color_in_depth_img[idx[:, 0], idx[:, 1]] = 255

# H = np.concatenate((r_cd[:, :2], t_cd), axis=1)
# color_in_depth_img2 = cv2.warpPerspective(
#     cv2.undistort(img_stacked_canny, k_c, None), H, (depth_shape[1], depth_shape[0]))

merged_image = depth_img_stacked_canny.copy()
merged_image[idx[:, 0], idx[:, 1]] = 50
cv2.imshow('overlayed', merged_image)
cv2.waitKey(0)

edge_points_depth = np.array(
    np.where(depth_img_stacked_canny)).T.astype('uint16')
edge_points_depth = np.concatenate(
    (edge_points_depth, np.ones((len(edge_points_depth), 1), dtype='uint16')), axis=1)
edge_points_depth_world_d = (np.linalg.inv(k_d)@edge_points_depth.T).T
edge_points_depth_world_d = (edge_points_depth_world_d *
                             np.atleast_2d(
                                 depth_img_stacked_smooth[edge_points_depth[:, 0], edge_points_depth[:, 1]]).T)
edge_points_depth_world_d = np.concatenate(
    (edge_points_depth_world_d, np.ones((len(edge_points_depth_world_d), 1))), axis=1)
HT = np.eye(4)
HT[:3, :3] = r_cd
HT[:3, 3] = t_cd.T
HT_inv = HT.copy()
HT_inv[:3, :3] = np.linalg.inv(HT[:3, :3])
HT_inv[:3, 3] = -HT_inv[:3, :3]@HT[:3, 3]

cv2.imshow('depth canny', depth_img_stacked_canny)
for HT_t, name in zip((HT, HT_inv, np.eye(4)), ('HT', 'HTinv', 'identity')):
    edge_points_depth_world_c = HT_inv@edge_points_depth_world_d.T
    edge_points_depth_cam_c = k_c@edge_points_depth_world_c[:3, :]
    edge_points_depth_cam_c = (edge_points_depth_cam_c /
                               np.atleast_2d(edge_points_depth_cam_c.T[:, 2]))
    edge_points_depth_cam_c = edge_points_depth_cam_c.astype('uint16')
    depth_in_color_img = np.zeros(img_stacked_canny.shape)
    edge_points_depth_cam_c = edge_points_depth_cam_c[:, edge_points_depth_cam_c[0, :]
                                                      < depth_in_color_img.shape[0]]
    edge_points_depth_cam_c = edge_points_depth_cam_c[:, edge_points_depth_cam_c[1, :]
                                                      < depth_in_color_img.shape[1]]
    depth_in_color_img[edge_points_depth_cam_c[0, :],
                       edge_points_depth_cam_c[1, :]] = 255
    cv2.imshow(f'depth in color: {name}', depth_in_color_img)

    merged_image = depth_in_color_img.copy()
    color_edge_points = np.array(np.where(img_stacked_canny)).T
    merged_image[color_edge_points[:, 0], color_edge_points[:, 1]] = 100
    cv2.imshow(f'merged: {name}', merged_image)
cv2.waitKey(0)
ipdb.set_trace()
