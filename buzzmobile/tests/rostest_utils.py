import roslaunch
import functools
import subprocess
import rospy
import contextlib
import rosnode
import time

def with_roscore(obj):
    """Decorator to run all tests in a testcase with their own roscore.

    This wraps the setUp and tearDown methods to start by first spinning up a
    roscore process, and tears it down at the very end. This adds a small time
    penalty, but its worth it.

    Its worth deciding whether to make this work on a per-method, per object,
    or both basis.
    """
    old_setup = obj.setUp
    old_teardown = obj.tearDown

    def new_setup(self):
        self.roscore = subprocess.Popen(['roscore'])
        old_setup(self)

    def new_teardown(self): 
        old_teardown(self)
        self.roscore.kill()
        subprocess.call(['killall', '-9', 'rosmaster'])
        self.roscore.wait()

    obj.setUp = new_setup
    obj.tearDown = new_teardown
    return obj


class ROSLauncher(roslaunch.scriptapi.ROSLaunch):
    """
    ROSLaunch that allows the use of launch files.
    """
    def __init__(self, files):
        super().__init__()
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, True) 
        self.parent = roslaunch.parent.ROSLaunchParent(uuid, files, is_core=False) 


def launch_node(package, name, launch):
    """Decorator to manage running a node and shutting it down gracefully.
    """
    f = roslaunch.rlutil.resolve_launch_arguments([package, launch])
    def launcher(func):
        @functools.wraps(func)
        def new_test(self):
            node = roslaunch.core.Node(package, name)
            launch = ROSLauncher(f)
            launch.start()

            # can we do something async here?
            process = launch.launch(node)
            while not any(nn.startswith('/' + name.replace('.', '_')) 
                    for nn in rosnode.get_node_names()):
                time.sleep(.1)
            try:
                temp = func(self)
            except:
                raise
            finally:
                process.stop()
            return temp

        return new_test
    return launcher


class MockNode:
    def __init__(self, topic, msg_type, node):
        self.topic = topic
        self.msg_type = msg_type
        self.node = node
    
    def send(self, value):
        self.node.publish(value)
        time.sleep(1)

@contextlib.contextmanager
def mock_node(topic, rosmsg_type, queue_size=1):
    """Mocks a node and cleans it up when done.
    """
    pub = rospy.Publisher(topic, rosmsg_type, queue_size=queue_size)
    yield MockNode(topic, rosmsg_type, pub)
    pub.unregister()

class TestNode:
    def __init__(self, topic, msg_type, callback):
        self.topic = topic
        self.msg_type = msg_type
        self.callback = callback

@contextlib.contextmanager
def test_node(topic, rosmsg_type, callback):
    rospy.init_node('test_'+topic, anonymous=True)
    rospy.Subscriber(topic, rosmsg_type, callback)
    yield TestNode(topic, rosmsg_type, callback)
    rospy.signal_shutdown('test complete')

    # Ros really doesn't want you to reinitialize a node once it's been
    # shutdown because there can be bad side effects, but we are good
    # at cleaning up after ourselves.
    rospy.client._init_node_args = None  # pylint: disable=protected-access
    rospy.core._shutdown_flag = False  # pylint: disable=protected-access
    rospy.core._in_shutdown = False  # pylint: disable=protected-access
    
