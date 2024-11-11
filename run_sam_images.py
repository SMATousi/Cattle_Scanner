import glob
import numpy as np
import torch
from segment_anything import sam_model_registry, SamAutomaticMaskGenerator, SamPredictor

points_path = '/home/vigir3d/Datasets/cattle_scans/maskrcnn_data/points/'
#mask_path = '/home/vigir3d/Datasets/cattle_scans/maskrcnn_data/masks/'
new_mask_path = '/home/vigir3d/Datasets/cattle_scans/maskrcnn_data/new_masks/'

files = glob.glob(points_path+'*.npy')
files.sort()

for idx in range(len(files)):
    file = files[idx]
    file_name = file.split('.npy')[0]
    file_name = file_name.split('/')[-1]
    file_name = file_name.split('_points')[0]
    
    with open(files[idx], 'rb') as fp:
        input_point = np.load(fp)
    input_label = np.ones(len(input_point))
    print(file_name)
    img = cv2.imread(img_path+file_name+'.jpg')
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    predictor.set_image(img)


    masks, scores, logits = predictor.predict(
        point_coords=input_point,
        point_labels=input_label,
        multimask_output=False,
    )
    mask = masks[0]
    h, w = mask.shape[-2:]
    color = np.array([30/255, 144/255, 255/255, 0.6])
    mask_image = mask.reshape(h, w, 1) * color.reshape(1, 1, -1)
    m = mask.astype(np.uint8)  #convert to an unsigned byte
    m*=255
    cv2.imwrite(new_mask_path+file_name+"_mask.png", m)
    