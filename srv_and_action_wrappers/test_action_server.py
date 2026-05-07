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
import rclpy, sys, time
from rclpy.node                import Node
from rclpy.executors           import MultiThreadedExecutor
from rclpy.callback_groups     import (MutuallyExclusiveCallbackGroup,
                                       ReentrantCallbackGroup)
from rclpy.action.server       import GoalResponse, CancelResponse
from example_interfaces.action import Fibonacci
from srv_and_action_wrappers.action_server import ActionServer

class TestActionServer(Node):
    def __init__(self):
        super().__init__('test_action_server')

        policy   = self.declare_parameter('policy', 'single').value
        grouping = self.declare_parameter('grouping', False).value
        self._server = ActionServer(self, Fibonacci, 'fibonacci',
                                    self._execute_cb, None,
                                    MutuallyExclusiveCallbackGroup(),
                                    policy, grouping)

    def _execute_cb(self, goal_handle):
        feedback = Fibonacci.Feedback(sequence=[0, 1])

        # Start executing the action
        for i in range(1, goal_handle.request.order):
            # If goal is flagged as no longer active (ie. another goal was accepted),
            # then stop executing
            if not goal_handle.is_active:
                return Fibonacci.Result()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Fibonacci.Result()

            # Update Fibonacci sequence
            feedback.sequence.append(feedback.sequence[i] +
                                     feedback.sequence[i-1])

            self.get_logger().info('feedback=%s' % list(feedback.sequence))

            # Publish the feedback
            goal_handle.publish_feedback(feedback)

            # Sleep for demonstration purposes
            time.sleep(1)

        if not goal_handle.is_active:
            return Fibonacci.Result()

        # Populate result message
        goal_handle.succeed()
        return Fibonacci.Result(sequence=feedback.sequence)

def main():
    try:
        rclpy.init(args=sys.argv)

        node = TestActionServer()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except Exception as e:
        print('*** Terminate the node due to exception: %s' % e)
