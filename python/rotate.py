import cv2
import imutils
import shutil
import os



def create_rotation_frames(img, full_path, dim):
    for i in range(max_images):
        filename = f"bird_{i}.png"
        angle = i * offs
        rotated = imutils.rotate(img, -angle)
        resized = cv2.resize(rotated, (dim, dim))
        current_file_name = os.path.join(full_path, filename)
        cv2.imwrite(current_file_name, resized) 


# Paths are relative to the repository root
path = "resources/matrix.png"
offs = 4
max_images = 90
min_dimension = 5
dimension_num = 40
img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
if img is None:
    raise SystemExit(f"Cannot read {path}: run this script from the repository root")
directory_path = "resources"
dim_dir_name = "dim"

# Only the generated dimNN directories are removed: everything else in
# resources/ (matrix.png included) is left untouched.
for filename in os.listdir(directory_path):
    full_path = os.path.join(directory_path, filename)

    if not filename.startswith(dim_dir_name) or not os.path.isdir(full_path):
        continue

    shutil.rmtree(full_path)

for i in range (dimension_num):
    full_path = os.path.join(directory_path, dim_dir_name)
    full_path = full_path + str(i+min_dimension)
    os.makedirs(full_path, exist_ok=True)
    create_rotation_frames(img, full_path, min_dimension+i)


