import os.path
import tensorflow as tf
import helper
import warnings
from distutils.version import LooseVersion
import project_tests as tests

# global step is keeping total number of training epochs (can be helpfull for loading model and continuing training)

# logits - function operates on unscaled output of earlier layers

# Check TensorFlow Version
assert LooseVersion(tf.__version__) >= LooseVersion('1.0'), 'Please use TensorFlow version 1.0 or newer.  You are using {}'.format(tf.__version__)
print('TensorFlow Version: {}'.format(tf.__version__))

# Check for a GPU
if not tf.test.gpu_device_name():
    warnings.warn('No GPU found. Please use a GPU to train your neural network.')
else:
    print('Default GPU Device: {}'.format(tf.test.gpu_device_name()))



def test_code():
    tests.test_load_vgg(load_vgg, tf)
    tests.test_layers(layers)
    tests.test_optimize(optimize)
    tests.test_train_nn(train_nn)
    tests.test_for_kitti_dataset(data_dir)

def load_vgg(sess, vgg_path):
    """
    Load Pretrained VGG Model into TensorFlow.
    :param sess: TensorFlow Session
    :param vgg_path: Path to vgg folder, containing "variables/" and "saved_model.pb"
    :return: Tuple of Tensors from VGG model (image_input, keep_prob, layer3_out, layer4_out, layer7_out)
    """
    # TODO: Implement function
    #   Use tf.saved_model.loader.load to load the model and weights



    vgg_tag = 'vgg16'
    vgg_input_tensor_name = 'image_input:0'
    vgg_keep_prob_tensor_name = 'keep_prob:0'
    vgg_layer3_out_tensor_name = 'layer3_out:0'
    vgg_layer4_out_tensor_name = 'layer4_out:0'
    vgg_layer7_out_tensor_name = 'layer7_out:0'

    tf.saved_model.loader.load(sess, [vgg_tag], vgg_path)
    graph = tf.get_default_graph()

    first_input_layer = graph.get_tensor_by_name(vgg_input_tensor_name)
    keep_prob = graph.get_tensor_by_name(vgg_keep_prob_tensor_name)
    vgg_layer3_out = graph.get_tensor_by_name(vgg_layer3_out_tensor_name)
    vgg_layer4_out = graph.get_tensor_by_name(vgg_layer4_out_tensor_name)
    vgg_layer7_out = graph.get_tensor_by_name(vgg_layer7_out_tensor_name)

    print ("First", first_input_layer.get_shape())

    return first_input_layer, keep_prob, vgg_layer3_out, vgg_layer4_out, vgg_layer7_out



def layers(vgg_layer3_out, vgg_layer4_out, vgg_layer7_out, num_classes):
    """
    Create the layers for a fully convolutional network.  Build skip-layers using the vgg layers.
    :param vgg_layer3_out: TF Tensor for VGG Layer 3 output
    :param vgg_layer4_out: TF Tensor for VGG Layer 4 output
    :param vgg_layer7_out: TF Tensor for VGG Layer 7 output
    :param num_classes: Number of classes to classify
    :return: The Tensor for the last layer of output
    """

    # input [ 1 160 576 3]
    # vgg3 [ 1 20 72 256]
    # vgg4 [ 1 10 36 512]
    # vgg7 [ 1 5 18 4096]


    # TODO: Implement function
    vgg_layer7_out = tf.Print(vgg_layer7_out, [tf.shape(vgg_layer7_out)], message= "Shape of vgg_layer7_out:", summarize=10, first_n=1)


    conv1x1 = tf.layers.conv2d(inputs=vgg_layer7_out, filters=512, kernel_size=(1,1), strides=(1,1), padding="SAME", kernel_regularizer=tf.contrib.layers.l2_regularizer(0.001), kernel_initializer = tf.truncated_normal_initializer(stddev=0.1))
    print ("show new layers info")
    conv1x1 = tf.Print(conv1x1, [tf.shape(conv1x1)], message= "Shape of layer7_1x1:", summarize=10, first_n=1)

     # possibly add kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-3)
     #kernel size 4 stride 2 - from walkthrough
     # 4 / (2,2)
     # 8 / (2,2)
     # 16 / (8,8)

   #without scalling it works pretty ok

    #kernel_initializer = tf.truncated_normal_initializer(stddev=0.03)

    up1 = tf.layers.conv2d_transpose(conv1x1, 512, kernel_size=(4,4), strides=(2,2), padding="SAME", kernel_regularizer=tf.contrib.layers.l2_regularizer(0.001), kernel_initializer = tf.truncated_normal_initializer(stddev=0.01))
    up1 = tf.Print(up1, [tf.shape(up1)], message= "Shape of up1:", summarize=10, first_n=1)
    up1_and_vgg4 =tf.add(up1, 0.01*vgg_layer4_out)
   
    up2 = tf.layers.conv2d_transpose(up1_and_vgg4, 256, kernel_size=(8,8), strides=(2,2), padding="SAME", kernel_regularizer=tf.contrib.layers.l2_regularizer(0.001), kernel_initializer = tf.truncated_normal_initializer(stddev=0.01))
    up2 = tf.Print(up2, [tf.shape(up2)], message= "Shape of up2:", summarize=10, first_n=1)
    up2_and_vgg3=tf.add(up2, 0.001*vgg_layer3_out)
   

    up3 = tf.layers.conv2d_transpose(up2_and_vgg3, num_classes, kernel_size=(16,16), strides=(8,8), padding="SAME", kernel_regularizer=tf.contrib.layers.l2_regularizer(0.001), kernel_initializer = tf.truncated_normal_initializer(stddev=0.01))
    up3 = tf.Print(up3, [tf.shape(up3)], message= "Shape of up3:", summarize=10, first_n=1)


    #conv1x1_2 = tf.layers.conv2d(up3, filters=2, kernel_size=(1,1), strides=(1,1), padding="SAME")
    #conv1x1_2 = tf.Print(conv1x1_2, [tf.shape(conv1x1_2)], message = "last layer:", summarize=10, first_n=1)

   

   
    return up3



def optimize(nn_last_layer, correct_label, learning_rate, num_classes):
    """
    Build the TensorFLow loss and optimizer operations.
    :param nn_last_layer: TF Tensor of the last layer in the neural network
    :param correct_label: TF Placeholder for the correct label image
    :param learning_rate: TF Placeholder for the learning rate
    :param num_classes: Number of classes to classify
    :return: Tuple of (logits, train_op, cross_entropy_loss)
    """
    # TODO: Implement function
    logits = tf.reshape(nn_last_layer, (-1, num_classes))
    
    correct_label = tf.reshape(correct_label, (-1, num_classes))

    loss = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits=logits,labels=correct_label))

    reg_losses = tf.get_collection(tf.GraphKeys.REGULARIZATION_LOSSES)

    reg_constant = 0.5  # Choose an appropriate one.
    
    total_loss = loss + reg_constant * sum(reg_losses)

    # calculates cross entropy of result AFTER pplying softmax function
    #softmax is squashing vector/tensor from real numbers to range 0-1 (sum of all will be 1)
    #cross entropy calculates error between prediction and result across all input samples

    optimizer = tf.train.AdamOptimizer(learning_rate=learning_rate)
    train_op = optimizer.minimize(loss, global_step=tf.train.get_global_step())

    return logits, train_op, loss



def train_nn(sess, epochs, batch_size, valid_get_batches_fn, get_batches_fn, train_op, cross_entropy_loss, input_image,
             correct_label, keep_prob, learning_rate):
    """
    Train neural network and print out the loss during training.
    :param sess: TF Session
    :param epochs: Number of epochs
    :param batch_size: Batch size
    :param get_batches_fn: Function to get batches of training data.  Call using get_batches_fn(batch_size)
    :param train_op: TF Operation to train the neural network
    :param cross_entropy_loss: TF Tensor for the amount of loss
    :param input_image: TF Placeholder for input images
    :param correct_label: TF Placeholder for label images
    :param keep_prob: TF Placeholder for dropout keep probability
    :param learning_rate: TF Placeholder for learning rate
    """
    # TODO: Implement function

    #print (tf.shape(keep_prob))
    print(input_image.get_shape())
    #print ("keep", keep_prob)
    #keep_prob_value = keep_prob
    total_result = 0
    steps = 0


    for image_batch, label_batch in get_batches_fn(batch_size):

        _, result = sess.run( [train_op, cross_entropy_loss], feed_dict = {input_image: image_batch, correct_label: label_batch, keep_prob: 0.5})

        print ("Loss", result)

    print( " --- validation ---")
    for image_batch, label_batch in valid_get_batches_fn(batch_size):

        #print(image_batch, label_batch)
        result = sess.run( [cross_entropy_loss], feed_dict = {input_image: image_batch, correct_label: label_batch, keep_prob: 1})
        print (result[0])
        total_result+=result[0]
        steps+=1
    avg_loss = total_result/steps
    print ("avg validation loss", avg_loss)
    return avg_loss


def run():

    # Download pretrained vgg model
    # fix scalling, add better loss calc

    num_classes = 2
    image_shape = (160, 576)
    data_dir = './data'
    runs_dir = './runs'

    helper.maybe_download_pretrained_vgg(data_dir)

    epochs=40
    batch_size=8
    learning_rate=1e-5
    dropout = 0.5
     

    previous_validation_loss = 100000000
    validation = 100000000
    best_model = None

    #input_image_place_holder = tf.placeholder(shape=[None, image_shape[0], image_shape[1], 3], dtype=tf.float32, name="Input_image")
    correct_label_place_holder = tf.placeholder(shape=[None, image_shape[0], image_shape[1], 2], dtype=tf.float32, name = "labels")

    # OPTIONAL: Train and Inference on the cityscapes dataset instead of the Kitti dataset.
    # You'll need a GPU with at least 10 teraFLOPS to train on.
    #  https://www.cityscapes-dataset.com/

    with tf.Session() as sess:


        vgg_path = os.path.join(data_dir, 'vgg')

        get_batches_fn = helper.gen_batch_function(os.path.join(data_dir, 'data_road/training'), image_shape)
        valid_get_batches_fn = helper.gen_batch_function(os.path.join(data_dir, 'data_road/validation'), image_shape)

        # OPTIONAL: Augment Images for better results
        #  https://datascience.stackexchange.com/questions/5224/how-to-prepare-augment-images-for-neural-network

        input_layer, keep_prob, vgg3, vgg4, vgg7 = load_vgg(sess, vgg_path)

        last_layer = layers(vgg3, vgg4, vgg7, num_classes)

        logits, train_op, loss = optimize(last_layer, correct_label_place_holder, learning_rate,num_classes)

        sess.run(tf.global_variables_initializer())
        saver = tf.train.Saver(max_to_keep=5)

        for epoch in range(epochs):
                print ("Epoch", epoch)
		# test with/without regularizer
                # test with and without image norm
                validation_loss = train_nn(sess, epochs, batch_size, valid_get_batches_fn, get_batches_fn, train_op, loss, input_layer, correct_label_place_holder, keep_prob, learning_rate)
                if ((validation_loss < previous_validation_loss) and (epoch > 5)):
                   link = "./models/model_"+str(epoch)+"/loss_"+str(validation_loss)
                   print ("Better model, saving... at"+link)
                   saver.save(sess, link)
                   best_model = link
                   previous_validation_loss = validation_loss
        if best_model != None:
            print ("Loading best model: ", best_model)
            saver.restore(sess, best_model)

        # TODO: Save inference data using helper.save_inference_samples
        helper.save_inference_samples(runs_dir, data_dir, "data_road/validation", sess, image_shape, logits, keep_prob, input_layer)
        helper.save_inference_samples(runs_dir, data_dir, "data_road/testing", sess, image_shape, logits, keep_prob, input_layer)

        # OPTIONAL: Apply the trained model to a video


if __name__ == '__main__':
    #test_code()
    run()
