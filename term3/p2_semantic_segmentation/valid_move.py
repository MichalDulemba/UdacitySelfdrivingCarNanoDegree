import os
import shutil
import numpy as np

source_image_folder="./training/image_2/"
source_gt_folder="./training/gt_image_2/"

original_files = os.listdir(source_image_folder)
original_target_files = os.listdir(source_gt_folder)

print(original_files)
print("number of files",len(original_files))
print(original_target_files)

selected= np.random.choice(289, 48, replace=False)
print ("selected", selected)

target_gt_folder="./validation/gt_image_2/"
images_folder= "./validation/image_2/"

files=os.listdir(images_folder)
print ("PLIKI", files, "\n\n")

#print(type(files))
#print(type(files[0]))

#print (dir(files[0]))

for i in selected:
    selected_file=original_files[i]
    print("Selected", selected_file)

    name=selected_file.split(".")[0]
    print (name)
    starter = name.split("_")[0]
    number = name.split("_")[1]

    if starter == "umm":
       print ("umm_road_"+number)
       selected_gt_file="umm_road_"+number+".png"

    elif starter == "um":
        print("um_road_"+number)
        selected_gt_file="um_road_"+number+".png"
    else:
        print("uu_road_"+number)
        selected_gt_file="uu_road_"+number+".png"
    #if uu
    print(source_image_folder+selected_file)
    print(images_folder+selected_file)
    #shutil.move(source_image_folder+selected_file, images_folder+selected_file)

    try:
        shutil.move(source_image_folder+selected_file, images_folder+selected_file)
    except FileNotFoundError:
        print("Not found")


    print(source_gt_folder+selected_gt_file)
    print(target_gt_folder+selected_gt_file)

    try:
        shutil.move(source_gt_folder+selected_gt_file, target_gt_folder+selected_gt_file)
    except FileNotFoundError:
        print("Not found")
