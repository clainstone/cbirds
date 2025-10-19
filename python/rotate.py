import cv2
import imutils
path = "resources/matrix.png" 
offs = 1
max_images = 360
img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
print(img.shape)

if img is None:
    print(f"Error: Could not read image from path: {path}")
else:
    for i in range(max_images):
        angle = i * offs
        rotated = imutils.rotate(img, -angle)
        resized = cv2.resize(rotated, (15, 15))
        print(resized.shape)
        current_file_name = f"resources/bird_{i}.png" 
        cv2.imwrite(current_file_name, resized) 
        print(f"Saved: {current_file_name}")

print("Rotation and saving complete.")
