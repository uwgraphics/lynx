import cv2
import numpy as np

def resize_image(img, scalar):
    width = int(img.shape[1] * scalar)
    height = int(img.shape[0] * scalar)
    dim = (width, height)
    resized = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)
    return resized

def show_image(img):
    cv2.imshow('window', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def cv2_coordinates_to_world_coordinates(px_x, px_y, image_pixel_width, image_pixel_height, world_width, world_height = 1.0, clamp=False):
    '''
    remember, world coordinates is VERTICAL AXIS DOWN, then HORIZ. AXIS OVER TO THE RIGHT
    :param image_pixel_width:
    :param image_pixel_height:
    :return:
    '''
    world_space_y = float(px_y) / image_pixel_height
    world_space_y = world_space_y * world_height
    if clamp:
        world_space_y = max(min(world_space_y, world_height), 0.0)

    world_space_x = float(px_x) / image_pixel_width
    world_space_x = world_space_x * world_width
    if clamp:
        world_space_x = max(min(world_space_x, world_width), 0.0)

    return (world_space_y, world_space_x)

def cv2_scalar_to_world_scalar(cv2_scalar, image_pixel_height, world_height=1.0):
    return (float(cv2_scalar) / image_pixel_height) * world_height

def world_coordinates_to_cv2_coordinates(world_y, world_x, image_pixel_height, image_pixel_width, world_width, world_height = 1.0):
    '''
    cv2 coordinate space origin is in upper left, order is (horizontal axis right, vertical axis down)
    :param world_y:
    :param world_x:
    :param image_pixel_height:
    :param image_pixel_width:
    :param world_width:
    :param world_height:
    :return:
    '''
    image_space_y = round((image_pixel_height*world_y)/world_height)
    image_space_y = int(max(min(image_space_y, image_pixel_height-1), 0))

    image_space_x = round((image_pixel_width*world_x)/world_width)
    image_space_x = int(max(min(image_space_x, image_pixel_width-1), 0))

    return (image_space_x, image_space_y)

def world_scalar_to_cv2_scalar(world_scalar, image_pixel_height):
    return int(round(image_pixel_height * world_scalar))

def matplotlib_plot_to_cv2_img(plot, resize_scalar = 1.0):
    plot.canvas.draw()
    img = np.fromstring(plot.canvas.tostring_rgb(), dtype=np.uint8, sep='')
    img = img.reshape(plot.canvas.get_width_height()[::-1] + (3,))
    img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
    img = resize_image(img, resize_scalar)
    return img