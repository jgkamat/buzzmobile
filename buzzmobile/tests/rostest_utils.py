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
    ROSLaunch but allows the use of launch files.

    This was found by peering into the roslaunch util source code from
    `which roslaunch`. 
    """
    def __init__(self, files):
        super().__init__()
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, True) 
        self.parent = roslaunch.parent.ROSLaunchParent(uuid, files, is_core=False) 

# TODO(joshuamorton@gatech.edu): swap this to a Map[int, launcher], where the
# int is incrementally generated and stored within the calling decorator. This
# will prevent the issue of overwriting or cross writing if it becomes a
# danger.
_launcher = None

def with_launch_file(package, launch):
    """Decorator to source a launch file for running nodes.

    This should always be run first.

    This and launch nodes work together gracefully, but this poses a danger.
    Because they rely on a global variable, multiple running simultaneously
    (in the same thread) can cause issues by overwriting the 'launcher'
    value. This could be fixed if needed, but I don't think it will be an
    issue.
    """
    f = roslaunch.rlutil.resolve_launch_arguments([package, launch])
    def launcher(func):
        @functools.wraps(func)
        def new_test(self):
            launch = ROSLauncher(f)
            launch.start()
            global _launcher
            _launcher = launch

            temp = func(self)
            _launcher = None
            return temp
        return new_test
    return launcher
            
def launch_node(package, name, namespace=None):
    """Decorator to manage running a node and shutting it down gracefully.

    Note that this will wrap itself up cleanly and launch all nodes with a
    single launcher, instead of multiples.
    """
    if not namespace:
        namespace = '/'+package
    def launcher(func):
        @functools.wraps(func)
        def new_test(self):
            node = roslaunch.core.Node(package, name, namespace=namespace)
            is_master = False
            global _launcher
            if _launcher is None:
                launch = roslaunch.scriptapi.ROSLaunch()
                launch.start()
                _launcher = launch
                is_master = True
            else:
                launch = _launcher

            process = launch.launch(node)
            # Beware this is a bit of a hack, and will currently not work if we
            # want to run more than 1 node with the same name.
            while not any(nn.split('/')[-1].startswith(name.replace('.', '_')) 
                    for nn in rosnode.get_node_names()):
                time.sleep(.1)
            try:
                temp = func(self)
            except:
                raise
            finally:
                process.stop()
            if is_master:
                _launcher = None
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
def mock_pub(topic, rosmsg_type, queue_size=1):
    """Mocks a node and cleans it up when done.
    """
    pub = rospy.Publisher(topic, rosmsg_type, queue_size=queue_size)
    yield MockNode(topic, rosmsg_type, pub)
    pub.unregister()

class TestNode:
    def __init__(self, topic, msg_type):
        self.topic = topic
        self.msg_type = msg_type

@contextlib.contextmanager
def check_topic(topic, rosmsg_type, callback):
    rospy.init_node('test_'+topic.split('/')[-1], anonymous=True)
    tn = TestNode(topic, rosmsg_type)
    cb  = functools.partial(callback, tn)
    rospy.Subscriber(topic, rosmsg_type, cb)
    yield tn 
    rospy.signal_shutdown('test complete')

    # Ros really doesn't want you to reinitialize a node once it's been
    # shutdown because there can be bad side effects, but we are good
    # at cleaning up after ourselves.
    rospy.client._init_node_args = None  # pylint: disable=protected-access
    rospy.core._shutdown_flag = False  # pylint: disable=protected-access
    rospy.core._in_shutdown = False  # pylint: disable=protected-access
    
