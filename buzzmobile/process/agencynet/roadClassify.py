import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt

import glob
import cv2
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('iterations', type=int, help='Number of iterations')
args = parser.parse_args()

# Helper functions
def weight_variable(shape):
    initial = tf.truncated_normal(shape, stddev=0.1)
    return tf.Variable(initial)

def bias_variable(shape):
    initial = tf.constant(0.1, shape=shape)
    return tf.Variable(initial)

def conv2d(x, W):
    return tf.nn.conv2d(x, W, strides=[1, 1, 1, 1], padding='SAME')

def max_pool_2x2(x):
    return tf.nn.max_pool(x, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding='SAME')


sess = tf.Session()
g = {} # globals

def get_next_example(index):

    image_filenames = glob.glob("./images/*.png")
    label_filenames = map(lambda e: e.replace("images", "labels"), image_filenames)

    if index in images.keys():
        return g[index][0], g[index][1], len(images)

    realIndex = index % len(image_filenames)

    image = np.reshape(cv2.imread(image_filenames[realIndex]), [310*94*3])

    preLabel = cv2.imread(label_filenames[realIndex])

    blue, green, red = preLabel[:,:,0], preLabel[:,:,1], preLabel[:,:,2]
    mask = ((red == 255) & (green == 0) & (blue == 0)) | ((red == 255) & (green == 255) & (blue == 0))

    label = np.zeros((preLabel.shape[0], preLabel.shape[1]), dtype=np.uint8)
    label[:,:][mask] = 255;

    # This line turns on edge detection
    # label = cv2.Laplacian(label, -1)

    label = np.reshape(label, [78*24])

    g[index] = (image, label)

    return image, label, len(images)

#image_batch, label_batch = tf.train.batch([image, label], batch_size=2)

# Placeholders
x = tf.placeholder(tf.float32, [310*94*3])
y_ = tf.placeholder(tf.float32, [78*24])

# Reshape the input image
x_image = tf.reshape(x, [-1, 310, 94, 3])

# Define convolution layer 1 (32 features for each 10x10 patch)
W_conv1 = weight_variable([10, 10, 3, 32])
b_conv1 = bias_variable([32])

h_conv1 = tf.nn.relu(conv2d(x_image, W_conv1) + b_conv1)
h_pool1 = max_pool_2x2(h_conv1)

# Define convolution layer 2 (32 -> 64 features for each 10x10 patch)
W_conv2 = weight_variable([10, 10, 32, 64])
b_conv2 = bias_variable([64])

h_conv2 = tf.nn.relu(conv2d(h_pool1, W_conv2) + b_conv2)
h_pool2 = max_pool_2x2(h_conv2)

# Global Activation Pooling

W_gap = weight_variable([1, 1, 64])
#b_gap = bias_variable([1, 1, 64])

h_pool2_rs = tf.reshape(h_pool2, [78, 24, 64])
h_mul_gap = tf.mul(h_pool2_rs, W_gap)

h_gap = tf.reduce_mean(h_mul_gap, 2)

# Dropout
keep_prob = tf.placeholder(tf.float32)
h_gap_drop = tf.nn.dropout(h_gap, keep_prob)

y_conv = tf.reshape(h_gap_drop, [78*24])

# Loss
loss = tf.sqrt(tf.reduce_mean(tf.square(tf.sub(y_, y_conv * 255))))

# Training parameters
global_step = tf.Variable(1, name='global_step', trainable=False)
train_step = tf.train.AdamOptimizer(1e-4).minimize(loss, global_step=global_step)
correct_prediction = tf.equal(tf.round(y_conv) * 255, y_)
accuracy = tf.reduce_mean(tf.to_float(correct_prediction))

sess.run(tf.initialize_all_variables())

saver = tf.train.Saver()

#ckpt = tf.train.get_checkpoint_state("checkpoints/")
#if ckpt and ckpt.model_checkpoint_path:
#    saver.restore(sess, ckpt.model_checkpoint_path)
saver.restore(sess, "final_weights.model")
#    print "Model restored from latest checkpoint"
#else:
#    print "No checkpoints, runnning from scratch"

current_step = global_step.eval(session=sess)
for i in range(current_step, current_step+args.iterations):

    imgs, lbls, count = get_next_example(i % 3)

    train_accuracy = accuracy.eval(session=sess, feed_dict={x: imgs, y_: lbls, keep_prob: 1.0})
    lossValue = loss.eval(session=sess, feed_dict={x: imgs, y_: lbls, keep_prob: 1.0})
    print ("step %d, loss %f, training accuracy %f" % (i, lossValue, train_accuracy))

    train_step.run(session=sess, feed_dict={x: imgs, y_: lbls, keep_prob: 1.0})

    if i % 100 == 0:
        saver.save(sess, "checkpoints/" + "model.ckpt", global_step=global_step)

        imgs, lbls, count = get_next_example(0)
        result = y_conv.eval(session=sess, feed_dict={x: imgs, y_: lbls, keep_prob: 1.0})
        result = np.reshape(result, [24, 78])
        cv2.imwrite("result.png", ((result * 255) > 127) * 255)

        print ("Loss: %f" % loss.eval(session=sess, feed_dict={x: imgs, y_: lbls, keep_prob: 1.0}))
        print ("**** %d iterations/example ****" % (i / count))

if True:
    imgs, lbls, count = get_next_example(0) # 9 is fun
    result = y_conv.eval(session=sess, feed_dict={x: imgs, y_: lbls, keep_prob: 1.0})
    result = ((np.reshape(result, [24, 78]) * 255) > 127) * 255

    fig = plt.figure()
    raw = fig.add_subplot(2, 1, 1)
    plt.imshow(np.reshape(imgs, [94, 310, 3]))
    raw.set_title("Raw Image")

    classified = fig.add_subplot(2, 1, 2)

    plt.imshow(result.astype(np.uint8), cmap='gray')
    classified.set_title("AgencyNet Output")
    plt.show()

    fig.savefig('composite.png')
