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
import rclpy.action.server, threading
from collections           import deque
from rclpy.action.server   import GoalResponse, CancelResponse
from action_msgs.msg       import GoalStatus
from .action_server        import ActionServer

from rclpy.node            import Node
from rclpy.callback_groups import CallbackGroup
from rclpy.action.server   import ServerGoalHandle
from typing                import Optional, Union

#************************************************************************
#  class TaskServer                                                     *
#************************************************************************
class TaskServer(ActionServer):
    """ROS Action server supporting multiple policies of processing goals.

    This class wraps ``rclpy.action.server.ActionServer``.
    """
    def __init__(self, node: Node, action_type, action_name: str,
                 execute_callback, stages, *,
                 callback_group: Optional[CallbackGroup]=None,
                 goal_callback=None,
                 goal_processing_policy: str='single',
                 group_field: str=''):
        """Create an ActionServer.

        :param node: The ROS node to add the action server to.
        :param action_type: Type of the action.
        :param action_name: Name of the action.
            Used as part of the underlying topic and service names.
        :param execute_callback: Callback function for processing accepted
            goals. This is called if when :class:`ServerGoalHandle.execute()`
            is called for a goal handle that is being tracked by this action
            server.
        :param callback_group: Callback group to add the action server to.
            If ``None``, then the node's default callback group is used.
        :param goal_callback: Callback function for handling new goal requests.
            If ``None``, then any goal request will be accepted.
        :param goal_processing_policy: Specifies policy of processing goals.
            - If 'single', then the server processes only one goal at a time.
              When a new goal request is recieved, the current goal under
              processing will be aborted and then new goal will be accepted.
            - If 'queued', then the server processes only one goal at a time.
              When a new goal request is recieved, it will be appended to
              FIFO queue. Its processing will be deferred until all the
              preceeding goals are completed.
            - If 'multi', then the server processes multiple goals in parallel.
            - Otherwise, raises ``ValueError``.
        :param group_field: Field name of the goal request which is used for
            grouping the incoming goals. This parameter  has no effect when
            `goal_processing_policy` is 'multi'.
            - If non-empty string, grouping is enabled and the requests
              belonging to different groups will be processed in parallel
              while the requests in a same group will be processed
              according to the policy specified by ``goal_processing_policy``.
            - If empty string, grouping is disabled.
        """
        super().__init__(node, action_type, action_name,
                         self._execute_callback,
                         callback_group=callback_group,
                         goal_callback=goal_callback,
                         goal_processing_policy=goal_processing_policy,
                         group_field=group_field)

    def wait_for_stage_completed(stage_name: str, timeout_sec=None):
        pass

    def _execute_cb(self, goal_handle):
        self._logger.info('goal[%s] started'
                          % ActionServer.goal_id_str(goal_handle))
        try:
            return self._user_execute_cb(goal_handle)
        finally:
            self._goal_handles.remove(goal_handle)
            if goal_handle.status == GoalStatus.STATUS_SUCCEEDED:
                self._logger.info('goal[%s] SUCCEEDED'
                                  % ActionServer.goal_id_str(goal_handle))
            elif goal_handle.status == GoalStatus.STATUS_CANCELED:
                self._logger.warn('goal[%s] CANCELED'
                                  % ActionServer.goal_id_str(goal_handle))
            elif goal_handle.status == GoalStatus.STATUS_ABORTED:
                self._logger.error('goal[%s] ABORTED'
                                   % ActionServer.goal_id_str(goal_handle))
            else:
                self._logger.error('goal[%s] terminated with status[%d]'
                                   % (ActionServer.goal_id_str(goal_handle),
                                      goal_handle.status))
