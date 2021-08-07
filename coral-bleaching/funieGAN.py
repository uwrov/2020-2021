"""
# > Script for testing FUnIE-GAN 
# > Notes and Usage:
#    - set data_dir and model paths
#    - python test_funieGAN.py
"""
import os
import ntpath
import numpy as np
from PIL import Image
from os.path import join, exists
from keras.models import model_from_json
## local libs
from utils.data_utils import read_and_resize, preprocess, deprocess
import cv2

def load_model():
    ## test funie-gan
    checkpoint_dir  = 'models/gen_p/'
    model_name_by_epoch = "model_15320_" 
    ## test funie-gan-up
    #checkpoint_dir  = 'models/gen_up/'
    #model_name_by_epoch = "model_35442_" 

    model_h5 = checkpoint_dir + model_name_by_epoch + ".h5"  
    model_json = checkpoint_dir + model_name_by_epoch + ".json"
    # sanity
    assert (exists(model_h5) and exists(model_json))

    # load model
    with open(model_json, "r") as json_file:
        loaded_model_json = json_file.read()
    funie_gan_generator = model_from_json(loaded_model_json)
    # load weights into new model
    funie_gan_generator.load_weights(model_h5)
    print("\nLoaded data and model")
    return funie_gan_generator

# testing loop
def infer(img_path, funie_gan_generator):
    # prepare data
    inp_img = read_and_resize(img_path, (256, 256))
    im = preprocess(inp_img)
    im = np.expand_dims(im, axis=0) # (1,256,256,3)
    # generate enhanced image
    gen = funie_gan_generator.predict(im)
    gen_img = deprocess(gen)[0]

    # ## create dir for log and (sampled) validation data
    # samples_dir = "C:/Users/zhouc/vscode-workspace/2020-2021/coral-bleaching/blue output/"
    # if not exists(samples_dir): os.makedirs(samples_dir)
    # # save output images
    # img_name = ntpath.basename(img_path)
    # out_img = np.hstack((inp_img, gen_img)).astype('uint8')
    # Image.fromarray(out_img).save(join(samples_dir, img_name))

    return gen_img

# img and gen_img are RGB
def infer_img(img, funie_gan_generator):
    original_size = img.shape
    # prepare data
    inp_img = cv2.resize(img, (256, 256)).astype(np.float32)
    im = preprocess(inp_img)
    im = np.expand_dims(im, axis=0) # (1,256,256,3)
    # generate enhanced image
    gen = funie_gan_generator.predict(im)
    gen_img = deprocess(gen)[0]
    # return gen_img
    return cv2.resize(gen_img, (original_size[1], original_size[0]))
