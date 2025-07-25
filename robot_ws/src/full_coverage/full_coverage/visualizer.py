# #!/usr/bin/env python3

# # import os
# # import yaml
# # import cv2

# # # ==== CONFIG ====
# # map_yaml_path = os.path.expanduser('/home/samah/robot_ws/src/map_server/config/room_area.yaml')  # <--- UPDATE THIS PATH
# # robot_size = 0.4  # in meters

# # # ==== LOAD MAP ====
# # with open(map_yaml_path, 'r') as f:
# #     map_data = yaml.safe_load(f)

# # map_img_path = os.path.join(os.path.dirname(map_yaml_path), map_data['image'])
# # map_img = cv2.imread(map_img_path, cv2.IMREAD_GRAYSCALE)
# # color_img = cv2.cvtColor(map_img, cv2.COLOR_GRAY2BGR)

# # height, width = map_img.shape
# # resolution = map_data['resolution']
# # origin = map_data['origin']

# # step_pixels = int(robot_size / resolution)
# # print(f'Loaded map: {width}x{height}, resolution: {resolution}, step: {step_pixels}px')

# # # ==== DRAW WAYPOINTS ====
# # for y in range(0, height, step_pixels):
# #     row = range(0, width, step_pixels) if (y // step_pixels) % 2 == 0 else range(width - 1, -1, -step_pixels)
# #     for x in row:
# #         if map_img[y, x] > 250:
# #             cv2.circle(color_img, (x, y), 3, (0, 255, 0), -1)

# # # ==== SHOW / SAVE IMAGE ====
# # scale = 10.0 # You can try 2.5 or 3.0 depending on your screen size
# # resized_img = cv2.resize(color_img, (0, 0), fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)

# # cv2.imshow("Zigzag Path", resized_img)
# # cv2.waitKey(0)
# # cv2.destroyAllWindows()
# # # Optionally save it
# # # cv2.imwrite("zigzag_waypoints.png", color_img)

# import os
# import yaml
# import cv2
# import numpy as np

# # === CONFIGURATION ===
# map_yaml_path = '/home/samah/robot_ws/src/map_server/config/room_area.yaml'
# robot_size = 0.4  # meters
# overlap = 0.1     # meters
# resolution_threshold = 250

# # === LOAD MAP YAML ===
# with open(map_yaml_path, 'r') as f:
#     map_data = yaml.safe_load(f)

# map_img_path = os.path.join(os.path.dirname(map_yaml_path), map_data['image'])
# resolution = map_data['resolution']
# origin_x, origin_y = map_data['origin'][0], map_data['origin'][1]

# # === LOAD MAP IMAGE ===
# map_img = cv2.imread(map_img_path, cv2.IMREAD_GRAYSCALE)
# color_map = cv2.cvtColor(map_img, cv2.COLOR_GRAY2BGR)  # for visualization

# height, width = map_img.shape
# step_px = int((robot_size - overlap) / resolution)

# # === BINARIZE MAP ===
# _, bin_map = cv2.threshold(map_img, resolution_threshold, 255, cv2.THRESH_BINARY)
# bin_map = bin_map // 255  # Convert to 0 or 1

# # === GENERATE WAYPOINTS IN ZIGZAG PATTERN ===
# waypoints = []
# direction = 1
# for row in range(0, height, step_px):
#     cols = range(0, width, step_px) if direction == 1 else range(width - 1, -1, -step_px)
#     for col in cols:
#         if bin_map[row, col] == 1:
#             waypoints.append((col, row))
#     direction *= -1

# print(f"Generated {len(waypoints)} waypoints")

# # === DRAW WAYPOINTS ON IMAGE ===
# for i, (x, y) in enumerate(waypoints):
#     cv2.circle(color_map, (x, y), 3, (0, 0, 255), -1)
#     if i > 0:
#         cv2.line(color_map, waypoints[i - 1], (x, y), (0, 255, 0), 1)

# # === DISPLAY IMAGE ===
# cv2.imshow("Coverage Path Preview", cv2.resize(color_map, (800, 800)))
# cv2.waitKey(0)
# cv2.destroyAllWindows()


#!/usr/bin/env python3
import os
import yaml
import cv2
import numpy as np

# === CONFIGURATION ===
map_yaml_path = '/home/samah/robot_ws/src/map_server/config/room_area.yaml'
robot_size = 0.4  # meters
overlap = 0.1     # meters
resolution_threshold = 250

# === LOAD MAP YAML ===
with open(map_yaml_path, 'r') as f:
    map_data = yaml.safe_load(f)

map_img_path = os.path.join(os.path.dirname(map_yaml_path), map_data['image'])
resolution = map_data['resolution']
origin_x, origin_y = map_data['origin'][0], map_data['origin'][1]

# === LOAD MAP IMAGE ===
map_img = cv2.imread(map_img_path, cv2.IMREAD_GRAYSCALE)
color_map = cv2.cvtColor(map_img, cv2.COLOR_GRAY2BGR)  # for visualization

height, width = map_img.shape
step_px = int((robot_size - overlap) / resolution)

# === BINARIZE MAP ===
_, bin_map = cv2.threshold(map_img, resolution_threshold, 255, cv2.THRESH_BINARY)
bin_map = bin_map // 255  # Convert to 0 or 1



# === DILATE OBSTACLES FOR 30cm BUFFER ===
dilation_distance_m = 0.3  # 30 cm
dilation_radius_px = int(dilation_distance_m / resolution)
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * dilation_radius_px + 1, 2 * dilation_radius_px + 1))
dilated_obstacles = cv2.dilate(1 - bin_map, kernel)  # Dilate obstacles (invert first)
bin_map = 1 - dilated_obstacles  # Invert back: 1 = free, 0 = obstacle

# === GENERATE SPARSE ZIGZAG WAYPOINTS ===
waypoints = []
direction = 1
for row in range(0, height, step_px):
    cols = range(0, width) if direction == 1 else range(width - 1, -1, -1)
    valid_points = [col for col in cols if bin_map[row, col] == 1]

    if len(valid_points) > 0:
        start = valid_points[0]
        end = valid_points[-1]
        waypoints.append((start, row))
        waypoints.append((end, row))
    
    direction *= -1

print(f"Generated {len(waypoints)} optimized waypoints")

# === DRAW WAYPOINTS ===
for i, (x, y) in enumerate(waypoints):
    cv2.circle(color_map, (x, y), 4, (0, 0, 255), -1)
    if i > 0:
        cv2.line(color_map, waypoints[i - 1], (x, y), (0, 255, 0), 1)

# === DISPLAY ===
cv2.imshow("Optimized Coverage Path Preview", cv2.resize(color_map, (800, 800)))
cv2.waitKey(0)
cv2.destroyAllWindows()










