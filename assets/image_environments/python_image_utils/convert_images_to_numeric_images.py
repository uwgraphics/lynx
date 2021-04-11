import cv2
import os
import file_utils as f
from cv2_utils import resize_image

########################################################################################################################

def _convert_and_output_all_collision_files():
    path_to_src = os.path.dirname(__file__)
    all_collision_file_directories = f.get_all_directories_in_given_dir(path_to_src + '/../collision_images')

    for dirname in all_collision_file_directories:
        already_converted = f.is_file_in_dir(path_to_src + '/../collision_images/' + dirname, 'numeric_image.txt')
        if not already_converted:
            filenames = f.get_all_files_in_given_dir(path_to_src + '/../collision_images/' + dirname)
            if len(filenames) > 1:
                print 'WARNING: image directories should only have one image in them.  Taking first one by default.'
            if len(filenames) == 0:
                print 'ERROR: image directories must have at least one image in them.  Exiting.'
                return

            filename = filenames[0]

            img = cv2.imread(path_to_src + '/../collision_images/' + dirname + '/' + filename)
            numeric_img = _make_collision_numeric_image(img, 0.5)
            _save_collision_numeric_image(dirname, numeric_img)

def _convert_and_output_all_objective_files():
    path_to_src = os.path.dirname(__file__)
    all_objective_file_directories = f.get_all_directories_in_given_dir(path_to_src + '/../objective_images')

    for dirname in all_objective_file_directories:
        already_converted = f.is_file_in_dir(path_to_src + '/../objective_images/' + dirname, 'numeric_image.txt')
        if not already_converted:
            filenames = f.get_all_files_in_given_dir(path_to_src + '/../objective_images/' + dirname)
            if len(filenames) > 1:
                print 'WARNING: image directories should only have one image in them.  Taking first one by default.'
            if len(filenames) == 0:
                print 'ERROR: image directories must have at least one image in them.  Exiting.'
                return

            filename = filenames[0]

            img = cv2.imread(path_to_src + '/../objective_images/' + dirname + '/' + filename)
            numeric_img = _make_objective_output_image(img, 0.5)
            _save_objective_numeric_image(dirname, numeric_img)

########################################################################################################################

def _make_collision_numeric_image(img, resize_scalar):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img = cv2.threshold(img, 120, 255, cv2.THRESH_BINARY)[1]
    img = resize_image(img, resize_scalar)
    return img

def _make_objective_output_image(img, resize_scalar):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img = cv2.normalize(img, None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_64F)
    img = resize_image(img, resize_scalar)
    return img

def _save_collision_numeric_image(dirname, img):
    path_to_src = os.path.dirname(__file__)
    outfile = open(path_to_src + '/../collision_images/' + dirname + '/' + 'numeric_image.txt', 'w+')
    for i in range(0, img.shape[0]):
        for j in range(0, img.shape[1] - 1):
            if img[i,j] > 120:
                outfile.write(str(0) + ',')
            else:
                outfile.write(str(1) + ',')
        if img[i, img.shape[1]-1] > 120:
            outfile.write(str(0) + ',')
        else:
            outfile.write(str(1) + ',')
        outfile.write('\n')

def _save_objective_numeric_image(dirname, img):
    path_to_src = os.path.dirname(__file__)
    outfile = open(path_to_src + '/../objective_images/' + dirname + '/' + 'numeric_image.txt', 'w+')
    for i in range(0,img.shape[0]):
        for j in range(0, img.shape[1] - 1):
            outfile.write(str(1.0 - img[i,j]) + ',')
        outfile.write(str(1.0 - img[i, img.shape[1]-1]))
        outfile.write('\n')

def _mark_file_as_done():
    path_to_src = os.path.dirname(__file__)
    outfile = open(path_to_src + '/../completion_metadata.txt', 'w+')
    outfile.write(' ')
    outfile.close()

########################################################################################################################

if __name__ == '__main__':
    _convert_and_output_all_collision_files()
    _convert_and_output_all_objective_files()
    # print 'images successfully converted using python script.'
