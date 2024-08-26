import cv2  # OpenCV library for image processing
import numpy  # NumPy library for numerical operations

# Load the image from the specified path
image = cv2.imread('/home/kashyap/shot.png')

# Callback function to handle mouse events
def mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:  # If the left mouse button is clicked
        h = image[y, x, 0]  # Get the Hue value at the clicked position
        s = image[y, x, 1]  # Get the Saturation value at the clicked position
        v = image[y, x, 2]  # Get the Value (brightness) at the clicked position
        print("H:", h)
        print("S:", s)
        print("V:", v)

# Create a window to display the image and set the mouse callback function
cv2.namedWindow('mouse')
cv2.setMouseCallback('mouse', mouse)

# Display the original image in a window
cv2.imshow("original image", image)
cv2.imshow("mouse", image)
cv2.waitKey(0)  # Wait for a key press
cv2.destroyAllWindows()  # Close all OpenCV windows

# Define the color range for masking (in HSV space)
light_line = numpy.array([120, 120, 0])
dark_line = numpy.array([150, 150, 10])
mask = cv2.inRange(image, light_line, dark_line)  # Create a mask based on the color range

# Display the mask

cv2.imshow('mask', mask)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Apply Canny edge detection on the mask
canny = cv2.Canny(mask, 30, 5)
cv2.imshow('edge', canny)
cv2.waitKey(0)
cv2.destroyAllWindows()
print(canny.shape)  # Print the shape of the Canny edge-detected image

# Crop a region of interest (ROI) from the Canny edge-detected image
r1 = 200; c1 = 0
img = canny[r1:r1+200, c1:c1+512]
cv2.imshow('crop', img)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Find the edges in the specified row of the cropped image
edge = []
row = 150

for i in range(512):
    if img[row, i] == 255:  # Check if the pixel is an edge
        edge.append(i)
print(edge)

# Determine the left and right edges based on the detected edges
if len(edge) == 4:
    left_edge = edge[0]
    right_edge = edge[2]
    print(edge)
elif len(edge) == 3:
    if edge[1] - edge[0] > 5:
        left_edge = edge[0]
        right_edge = edge[1]
    else:
        left_edge = edge[0]
        right_edge = edge[2]

# Calculate the width of the road and the midpoint
road_width = (right_edge - left_edge)
frame_mid = left_edge + (road_width / 2)
mid_point = 512 / 2
img[row, int(mid_point)] = 255  # Mark the midpoint on the image
print(mid_point)

# Calculate the error between the midpoint and the frame's center
error = mid_point - frame_mid

# Determine the action based on the error
if error < 0:
    action = "Go Right"
else:
    action = "Go Left"

print("error", error)

# Mark the midpoint of the frame on the image
img[row, int(frame_mid)] = 255
print("midpoint of the frame", frame_mid)

# Add text to the image indicating the action to take
f_image = cv2.putText(img, action, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 1, cv2.LINE_AA)

# Display the final image with the action text
cv2.imshow('final image', f_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
