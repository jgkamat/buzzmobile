import roslaunch
import functools
import subprocess

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

    obj.setUp = new_setup
    obj.tearDown = new_teardown
    return obj

def launch_node(package, name):
    """Decorator to manage running a node and shutting it down gracefully.
    """
    def launcher(func):
        @functools.wraps(func)
        def new_test(self):
            node = roslaunch.core.Node(package, name)
            launch = roslaunch.scriptapi.ROSLaunch()
            launch.start()

            # can we do something async here?
            process = launch.launch(node)
            temp = func(self)
            process.stop()
            return temp

        return new_test
    return launcher



