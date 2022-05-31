from img_segmentation import ImageSegmentation


img_seg = ImageSegmentation(    src_directory = './train_fire/',
                                dst_directory = './ground_truth/',
                                img_range = [1, 100])

if __name__ == '__main__':
    img_seg.start_segmentation()