#  BSD 3-Clause License
#
#  Copyright (c) 2026, National Institute of Advanced Industrial Science
#  and Technology(AIST)
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#
#  3. Neither the name of the copyright holder nor the names of its
#     contributors may be used to endorse or promote products derived from
#     this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
#  OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
#  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
#  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
#  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
#  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
#  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#  Author: Toshio Ueshiba (t.ueshiba@aist.go.jp)
#
import rclpy, sys, threading
from rclpy.node                import Node
from example_interfaces.action import Fibonacci
from task_wrappers             import SimpleActionClient

class TestSimpleActionClient(Node):
    def __init__(self):
        super().__init__('test_simple_action_client')
        self._client = SimpleActionClient(self, Fibonacci, 'fibonacci')
        self._client.wait_for_server()

        threading.Thread(target=self.interactive, daemon=True).start()

    def interactive(self):
        def is_int(s):
            try:
                int(s)
            except ValueError:
                return False
            else:
                return True

        while rclpy.ok():
            print('=== Commands ===')
            print('  <numeric>: send a goal with specified order value')
            print('  w:         wait until current goal terminated')
            print('  c:         cancel current goal')
            print('  q:         quit')

            key = input('>> ')
            if is_int(key):
                self._client.send_goal(Fibonacci.Goal(order=int(key)),
                                       feedback_callback=self._feedback_cb,
                                       timeout_sec=0.0)
            elif key == 'w':
                status, result = self._client.wait()
                print('status=%s, result=%s'
                      % (SimpleActionClient.goal_status_str(status),
                         list(result.sequence) if result else 'None'))
            elif key == 'c':
                self._client.cancel_goal()
            elif key == 'q':
                break

        self.destroy_node()
        rclpy.shutdown()

    def _feedback_cb(self, feedback):
        self.get_logger().info('feedback=%s'
                               % list(feedback.feedback.sequence))

def main():
    rclpy.init(args=sys.argv)

    test = TestSimpleActionClient()
    rclpy.spin(test)
