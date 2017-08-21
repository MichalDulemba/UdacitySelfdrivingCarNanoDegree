
# Vehicle Detection Project #

The goals / steps of this project are the following:
- choosing method (deep learning vs hog/svm)
- detecting cars on video
- creating video with bounding boxes for detected cars

I decided to go with SSD (Single shot detection) which is much master than hog based solutions. 

Disclaimer: I started working on my solution with code from Pierluigi Ferrari who ported SSD from Caffe to Keras.
Gradually my model became a lot different than his base model but I used his decoding functions and anchor-box implementation. 
Also it took quite a lot of effort to train my model and tune hyperparameters to get a good detection even without averaging between frames.

### Basics of Single Shot Dectection. 

The main ideas behind SSD is to run whole computation in a single trainable model. 
So called classic solutions are often based on many steps that have to be tuned separately.
In SSD classes of objects, different scales and different aspect ratios are all calulated during single prediction.

### Original SSD model
Original model called often SSD300 consists of base model and prediction layers. 
Image that is commonly used to show "ssd" architecture is not doing a very good job in my opinion.
So I decided to draw my own based on Pierre's (and later mine) code. 

<IMAGE>

Base model is:
1) VGG feature extractor (light blue on the left)
2) model "compressing" (1x1 convolutions and dilated convolutions) - (orange part)
3) Getting out of model different level of details  - (red part)
4) Gray - normalizer

And from 3) and 4) we have 2 separate pipelines 
1) is detecting classes (softmax) - (light green/turqoise part)
   It contains inputs, reshape, concatenation of results and softmax.
2) is working on different scales - (purple part)
3) is working on different aspect ratios - (yellow part)

At the end of this model, those two pipelines are joined into single big tensor of predictions - green rectangle.

---
### Creating model
I started with very small network initially proposed by Pierluigi. 
It was good enough to detect some cars, and show me what i can expect from this kind of detections.

Next step was making it a lot deeper (more layers and more filters) 
- I did it in couple of steps, gradually getting better results.

My final working model is in <FILE>
I also tried to use VGG19 as feature extractor but training was horribly slow and I had to dump this idea.
After research about what network is as good and faster, I decided to integrate Resnet-like network into my model.

I used keras functions and changed it a little bit to avoid such spatial compression. 
Overall it was kind of fight between accuracy and available memory. 
Even 11GB on 1080 Ti is not that much if you work on larger models.

I also added more possible aspect ratios because first tests showed that model is having hard time to "fit" car nicely
into rectangle.


### Decoding/thresholding

I had to play with parameters 

confidence_thresh=0.95, iou_threshold=0.05,

### Video Implementation

#### 1. Creating video
Video is created using <FILE> that simply creates model, loads weights and then procesess video. 



#### 2. Final video

You can see my final result at
<LINK>
or here
<FILE>

#### 3. Removing false positives
As usual - the more you want detect, the bigger change that you will get false positives. 
Therefore I used confidence_threshold of 0.95 that allowed to remove most unwanted detections. 




### Discussion

#### 1. Ideas for improvement

Next step would be testing how many layers we can remove from this model to make it fast/lightweight. At this moment we could test higher resolution images without resizing (and losing information). 

Also i found that "greedy nms" based on confidance has some issues when one box is completely inside the other. On PyimageSearch.com I found method by dr Malisiewicz, that is using box area to make better NMS.
http://www.pyimagesearch.com/2015/02/16/faster-non-maximum-suppression-python/


#### 2. Failures
Definitely this model with very little data to train is not very generalized. I tested detecting car at night and it was a complete failure.

