# Semantic Segmentation


### Goal
For this project my main task was creating Fully Connected Convolutional neural network. 
Instead of last layers being fully connected like in classification cnns, here we want to "label" every pixel of the source image.

### Environment
I used docker with tensorflow and opencv to avoid any dependency issues. You can build it from scratch using Dockerfile or pull from docker hub using "docker pull dulemba/tfcv2"

### Data 
Data is based on Kitty dataset and it is a really small sample (200-300 examples depending on the split training/validation).

#### Normalization
Images were normalized using standard x/255-0.5 formula. Later I removed this step and achieved even better results - probably VGG was trained using 0-255 range. 

### Model
First I load pretrained layers from VGG16, then passed them to "Layers" function. 
Next i add 1x1 layer    

    conv1x1 = tf.layers.conv2d(inputs=vgg_layer7_out, filters=512, kernel_size=(1,1), strides=(1,1), padding="SAME")

The main difference here was that I didn't kill all my data by going here into 2 categories, I decided to use 512 layers to help creating skip connections.


And now comes the decoder part (with skip connections):

    up1 = tf.layers.conv2d_transpose(conv1x1, 512, kernel_size=(4,4), strides=(2,2), padding="SAME" )
    up1_and_vgg4 =tf.add(up1, 0.01*vgg_layer4_out)

    up2 = tf.layers.conv2d_transpose(up1_and_vgg4, 256, kernel_size=(8,8), strides=(2,2), padding="SAME" )
    up2_and_vgg3=tf.add(up2, 0.001*vgg_layer3_out)

    last_layer = tf.layers.conv2d_transpose(up2_and_vgg3, num_classes, kernel_size=(16,16), strides=(8,8), padding="SAME" )

To print layer sizes I also add tf.print nodes with the same names as layers.
Later i also tested idea of adding L2 regularizer but it made results worse, so I decided to use "pure" crossentropy loss.



### Hyperparameters
#### Batch size
Tested 16 and 8 and 8 worked better for me.

#### Epochs
I started with 20 and after adding a little bit of data augmentation I increased epochs to 40.

#### Scalling
In skip layer I tested multiple scalling factors and 0.01 for first skip connection and 0.001 for the second one were working best.

#### Kernel Initialiser
I used truncated_normal_initialiser with stddev=0.01


### Validation
There wasn't any validation data, so i decided to write short script to "extract" some images with corresponding ground truth to be able to check my ideas for improvement. I moved random 48 images from training to validation. Now i probably shouldn't "cheat" and check validation in every run, because that could "skew" model to make it work on validation set. But it seemed like reasonable thing to do, when having so little data. 
Later I also added printing validation set with green overlay to see predicted classes. 


### Overfitting
After initial model creation, I added showing loss calculations after each batch - this allowed me to see if my ideas are improving results or not. 

At first I only used changing (Keep/dropout rate) as a way of preventing overfiting. Later I added "coin flip" function that allowed me randomly adding horizontal flip. Pictures are very contrasty, so I applied CLAHE method to soften contrast a little bit. That also helped a little bit. 


### Sample images

You can see images in three notebooks:
Inference images - part 1.ipynb
Inference images - part 2.ipynb
Inference images - part 3.ipynb
and in "Latest inference" folder.


### Ideas for improvement
Main thing that could help here would be much larger dataset. Of course more data augmentations (shifts, small rotations could also help). Also I'm not sure if using VGG is the best way to go. Most of experiments are now run using Resnet variations because it gives better results and is usually faster than vgg (with dense layers). 



 
