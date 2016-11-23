"""Classify roads."""
from __future__ import print_function

import tensorflow as tf
import numpy as np

import glob
import cv2
import argparse


PARSER = argparse.ArgumentParser()
PARSER.add_argument('iterations', type=int, help='Number of iterations')
ARGS = PARSER.parse_args()

g = {} # globals


# Helper functions
def weight_variable(shape):
    """Creates weight tensor."""
    initial = tf.truncated_normal(shape, stddev=0.1)
    return tf.Variable(initial)

def bias_variable(shape):
    """Creates bias tensor."""
    initial = tf.constant(0.1, shape=shape)
    return tf.Variable(initial)

def conv2d(x, weight):
    """Creates conv tensor."""
    return tf.nn.conv2d(x, weight, strides=[1, 1, 1, 1], padding='SAME')

def max_pool_2x2(x):
    """Creates max pool tensor."""
    return tf.nn.max_pool(x, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1],
            padding='SAME')

def get_next_example(index):
    """Given index, returns next example."""

    image_filenames = glob.glob("./images/*.png")
    label_filenames = [e.replace("images", "labels") for e in image_filenames]

    #  if index in images.keys():
    if index in image_filenames:
        return g[index][0], g[index][1], len(image_filenames)

    real_index = index % len(image_filenames)

    image = np.reshape(cv2.imread(image_filenames[real_index]), [310*94*3])

    pre_label = cv2.imread(label_filenames[real_index])

    blue, green, red = (pre_label[:, :, 0], pre_label[:, :, 1],
            pre_label[:, :, 2])
    mask = (((red == 255) & (green == 0) & (blue == 0))
            | ((red == 255) & (green == 255) & (blue == 0)))

    label = np.zeros((pre_label.shape[0], pre_label.shape[1]), dtype=np.uint8)
    label[:, :][mask] = 255

    # This line turns on edge detection
    # label = cv2.Laplacian(label, -1)

    label = np.reshape(label, [78*24])

    g[index] = (image, label)

    return image, label, len(image_filenames)

def _create_main_graph(x_image):
    """Creates main graph."""
    # Define convolution layer 1 (32 features for each 10x10 patch)
    w_conv1 = weight_variable([10, 10, 3, 32])
    b_conv1 = bias_variable([32])

    h_conv1 = tf.nn.relu(conv2d(x_image, w_conv1) + b_conv1)
    h_pool1 = max_pool_2x2(h_conv1)

    # Define convolution layer 2 (32 -> 64 features for each 10x10 patch)
    w_conv2 = weight_variable([10, 10, 32, 64])
    b_conv2 = bias_variable([64])

    h_conv2 = tf.nn.relu(conv2d(h_pool1, w_conv2) + b_conv2)
    h_pool2 = max_pool_2x2(h_conv2)

    # Global Activation Pooling

    w_gap = weight_variable([1, 1, 64])
    #b_gap = bias_variable([1, 1, 64])

    h_pool2_rs = tf.reshape(h_pool2, [78, 24, 64])
    h_mul_gap = tf.mul(h_pool2_rs, w_gap)

    h_gap = tf.reduce_mean(h_mul_gap, 2)

    return h_gap

def create_graph(x, y_truth):
    """Creates graph."""
    # Reshape the input image
    x_image = tf.reshape(x, [-1, 310, 94, 3])

    h_gap = _create_main_graph(x_image)

    # Dropout
    keep_prob = tf.placeholder(tf.float32)
    h_gap_drop = tf.nn.dropout(h_gap, keep_prob)

    y_conv = tf.reshape(h_gap_drop, [78*24])

    # Loss
    loss = tf.sqrt(tf.reduce_mean(tf.square(tf.sub(y_truth, y_conv * 255))))

    return y_conv, loss, keep_prob

def main():
    """main method for roadClassify."""
    sess = tf.Session()

    # Placeholders
    x = tf.placeholder(tf.float32, [310*94*3])
    y_truth = tf.placeholder(tf.float32, [78*24])

    y_conv, loss, keep_prob = create_graph(x, y_truth)

    # Training parameters
    global_step = tf.Variable(1, name='global_step', trainable=False)
    train_step = tf.train.AdamOptimizer(1e-4).minimize(loss,
            global_step=global_step)
    correct_prediction = tf.equal(tf.round(y_conv) * 255, y_truth)
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

    for i in range(global_step.eval(session=sess),
            global_step.eval(session=sess)+ARGS.iterations):

        imgs, lbls, count = get_next_example(i % 3)

        print("step %d, loss %f, training accuracy %f"
                % (i,
                   sess.run(loss,
                       feed_dict={x: imgs, y_truth: lbls, keep_prob: 1.0}),
                   sess.run(accuracy,
                       feed_dict={x: imgs, y_truth: lbls, keep_prob: 1.0})))

        train_step.run(session=sess,
                feed_dict={x: imgs, y_truth: lbls, keep_prob: 1.0})

        if i % 100 == 0:
            saver.save(sess, "checkpoints/" + "model.ckpt",
                    global_step=global_step)

            imgs, lbls, count = get_next_example(0)
            cv2.imwrite("result.png", (
                (np.reshape(
                    sess.run(y_conv,
                        feed_dict={x: imgs, y_truth: lbls, keep_prob: 1.0}),
                    [24, 78])
                * 255) > 127) * 255)

            print("Loss: %f" % loss.eval(session=sess,
                feed_dict={x: imgs, y_truth: lbls, keep_prob: 1.0}))
            print("**** %d iterations/example ****" % (i / count))

    # uncomment if you want to save from trained model.
    #  if True:
        #  imgs, lbls, count = get_next_example(0) # 9 is fun
        #  result = sess.run(y_conv, feed_dict={x: imgs, y_truth: lbls,
            #  keep_prob: 1.0})
        #  result = ((np.reshape(result, [24, 78]) * 255) > 127) * 255

        #  fig = plt.figure()
        #  raw = fig.add_subplot(2, 1, 1)
        #  plt.imshow(np.reshape(imgs, [94, 310, 3]))
        #  raw.set_title("Raw Image")

        #  classified = fig.add_subplot(2, 1, 2)

        #  plt.imshow(result.astype(np.uint8), cmap='gray')
        #  classified.set_title("AgencyNet Output")
        #  plt.show()

        #  fig.savefig('composite.png')

if __name__ == '__main__': main()
