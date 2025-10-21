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
        print(resized.shape)
        current_file_name = os.path.join(full_path, filename)
        cv2.imwrite(current_file_name, resized) 


path = "resources/matrix.png" 
offs = 4
max_images = 90
min_dimension = 5
dimension_num = 40
img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
directory_path = "resources"
file_to_save = "matrix.png"
dim_dir_name = "dim"

for filename in os.listdir(directory_path):
    full_path = os.path.join(directory_path, filename)

    if(filename == file_to_save):
        continue

    if os.path.isdir(full_path):
        shutil.rmtree(full_path)
    elif os.path.isfile(full_path):
        os.remove(full_path)

for i in range (dimension_num):
    full_path = os.path.join(directory_path, dim_dir_name)
    full_path = full_path + str(i+min_dimension)
    os.mkdir(full_path)
    create_rotation_frames(img, full_path, min_dimension+i)


