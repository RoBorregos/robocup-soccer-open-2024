'''
This module contains functions for finding and drawing blobs in an image.
'''

def find_blobs(img, thresholds):
    '''Find blobs in an image using the given thresholds.'''
    return img.find_blobs([thresholds, thresholds] , area_threshold=1, merge=True)

def draw_blobs(img, blobs):
    '''Draw the blobs found in the image.'''
    for blob in blobs:
        img.draw_rectangle(blob.rect(), color=(0,255,0))
        img.draw_cross(blob.cx(), blob.cy(), color=(0,255,0))
