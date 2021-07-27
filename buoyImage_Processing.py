# import modules needed
import cv2
import matplotlib.pyplot as plt
import numpy as np

def sensor_position_m (self, pix_x, pix_y, res_x, res_y):

        x = pix_x - (0.5 * res_x)
        y = (0.5 * res_y) - pix_y

        x_prop = (3.68 / 1000) / res_x
        y_prop = (2.76 / 1000) / res_y

        sensor_pos_x = x * x_prop
        sensor_pos_y = (y * y_prop)

        return (sensor_pos_x, sensor_pos_y)

def sensor_position_px (self, pix_x, pix_y, res_x, res_y):

        x = pix_x - (0.5 * res_x)
        y = (0.5 * res_y) - pix_y

        return (x, y)

def get_angles (self, x, y):

        f = (3.04 / 1000)

        hor_ang = np.degrees(np.arctan2(x, f))
        vert_ang = np.degrees(np.arctan2(y, f))

        return hor_ang, vert_ang
    
# loads image, resizes, and applies box filter
fn = 'frame_1627328707.jpg'
img = cv2.imread(fn)
img = cv2.resize(img, (640, 480))
img_brg_filtered = cv2.boxFilter(img, -1, (10,10))

# puts image thresholds using RGB makes 3 boolean arrays depending on if pixel meets thresholds
img_threshold_blue = np.logical_not(np.logical_and(img_brg_filtered[:, :, 0] >= 210, img_brg_filtered[:, :, 0] <= 255))
img_threshold_green = (np.logical_and(img_brg_filtered[:, :, 1] >= 140, img_brg_filtered[:, :, 1] <= 255))
img_threshold_red = (np.logical_and(img_brg_filtered[:, :, 2] >= 20, img_brg_filtered[:, :, 1] <= 140))

# combines all three logical statements to find pixels that meet all three conditions
img_red_buoys = np.logical_and(img_threshold_green, img_threshold_blue)
img_green_buoys = np.logical_and(img_threshold_red, img_threshold_blue)

fig, ax = plt.subplots(3, 3)

ax[0, 0].imshow(img[:, :, 0], cmap = "Blues")
ax[0, 0].title.set_text("Blue")
ax[0, 1].imshow(img[:, :, 1], cmap = "Greens")
ax[0, 1].title.set_text("Green")
ax[0, 2].imshow(img[:, :, 2], cmap = "Reds")
ax[0, 2].title.set_text("Red")

ax[1, 0].imshow(img_brg_filtered[:, :, 0], cmap = "Blues")
ax[1, 0].title.set_text("Blue")
ax[1, 1].imshow(img_brg_filtered[:, :, 1], cmap = "Greens")
ax[1, 1].title.set_text("Green")
ax[1, 2].imshow(img_brg_filtered[:, :, 2], cmap = "Reds")
ax[1, 2].title.set_text("Red")

ax[2, 0].imshow(img_threshold_blue, cmap = "Blues")
ax[2, 0].title.set_text("Blue")
ax[2, 1].imshow(img_threshold_green, cmap = "Greens")
ax[2, 1].title.set_text("Green")
ax[2, 2].imshow(img_threshold_red, cmap = "Reds")
ax[2, 2].title.set_text("Red")

#plt.show()

#steps 13/14 of lab 15. looks at pixel intensity values of it's neighborhood and scales to 255
img_points_red = cv2.boxFilter(img_red_buoys.astype(int), -1, (50, 50), normalize = False)
img_points_red_value_scaled = (img_points_red * (255 / np.max(img_points_red))).astype(np.uint8)

img_points_green = cv2.boxFilter(img_green_buoys.astype(int), -1, (50, 50), normalize = False)
img_points_green_value_scaled = (img_points_green * (255 / np.max(img_points_green))).astype(np.uint8)

thresh_red, img_out_red = cv2.threshold(img_points_red_value_scaled, 217, 255, cv2.THRESH_BINARY)
thresh_green, img_out_green = cv2.threshold(img_points_green_value_scaled, 217, 255, cv2.THRESH_BINARY)

contours_red, hierarchy_red = cv2.findContours(img_out_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
contours_green, hierarchy_green = cv2.findContours(img_out_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

contours_red_distances = []
contours_red_center = []

contours_green_distances = []
contours_green_center = []

# should find and plot center of red buoy / contour
contours_red_x = [item[0][0] for item in contours_red[0]]
contours_red_y = [[item[0][1] for item in contours_red[0]]]
avg_contours_red_x = np.average(contours_red_x)
avg_contours_red_y = np.average(contours_red_y)
plt.plot(avg_contours_red_x, avg_contours_red_y, 'bo')
plt.annotate("red center", (avg_contours_red_x, avg_contours_red_y))

# should find and plot center of green buoy / contour
contours_green_x = [item[0][0] for item in contours_green[0]]
contours_green_y = [[item[0][1] for item in contours_green[0]]]
avg_contours_green_x = np.average(contours_green_x)
avg_contours_green_y = np.average(contours_green_y)
plt.plot(avg_contours_green_x, avg_contours_green_y, 'yo')
plt.annotate("green center", (avg_contours_green_x, avg_contours_green_y))

# =============================================================================
# distance to object = (f(mm) * ow(mm) * iw(px)) / (ow(px) * sw(mm))
# f = focal length = 3.04mm
# ow = object width = .25m = 250 mm = (np.max([item[0][0] for item in contour]) - np.min([item[0][0] for item in contour])) in px
# iw = image width = img.shape[1] (px)
# sw = sensor width = 3.68mm
# =============================================================================
for contour in contours_red:
    dist = (3.04*250*img.shape[1])/((np.max([item[0][0] for item in contour]) - np.min([item[0][0] for item in contour]))*3.68)/1000
    print(dist)
    contours_red_distances.append(dist)

    # find center of each contour in contours_red
    contour_x = int(np.mean([item[0][0] for item in contour]))
    contour_y = int(np.mean([item[0][1] for item in contour]))
    avg_contour_x = np.average(contour_x)
    avg_contour_y = np.average(contour_y)
    contours_red_center.append((avg_contour_x, avg_contour_y))
    

for contour in contours_green:
    dist = (3.04*250*img.shape[1])/((np.max([item[0][0] for item in contour]) - np.min([item[0][0] for item in contour]))*3.68)/1000
    print(dist)
   # find center of each contour in contours_red
    contour_x = int(np.mean([item[0][0] for item in contour]))
    contour_y = int(np.mean([item[0][1] for item in contour]))
    avg_contour_x = np.average(contour_x)
    avg_contour_y = np.average(contour_y)
    contours_green_center.append((avg_contour_x, avg_contour_y))

if len(contours_red_distances) > 0: # checks if we have at least 1 contour detected
    # for each contour finds the index of the smallest contour
    red_index = contours_red_distances.index(min(contours_red_distances))
    
    # finds angle to the buoys
    red_center_m_x = (((3.68 / 1000) / img.shape[1]) * contours_red_center[red_index][0])
    red_center_m_y = (((2.76 / 1000) * img.shape[0]) / contours_red_center[red_index][1])
    red_angle_x, red_angle_y = red_center_m_x, red_center_m_y
    red = ((np.tan(np.radians(red_angle_x)) * contours_red_distances[red_index]), contours_red_distances[red_index])
    
if len(contours_green_distances) > 0:
    green_index = contours_green_distances.index(min(contours_green_distances))
    green_center_m_x = (((3.68 / 1000) / img.shape[1]) * contours_green_center[green_index][0])
    green_center_m_y = (((2.76 / 1000) * img.shape[0]) / contours_green_center[green_index][1])
    green_angle_x, green_angle_y = get_angles(green_center_m_x, green_center_m_y)
    green = ((np.tan(np.radians(green_angle_x)) * contours_green_distances[green_index]), contours_green_distances[green_index])
    
