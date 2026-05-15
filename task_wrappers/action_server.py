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

from rclpy.node            import Node
from rclpy.callback_groups import CallbackGroup
from rclpy.action.server   import ServerGoalHandle
from typing                import Optional, Union

#************************************************************************
#  stuffs concerning with ServerGoalHandle                              *
#************************************************************************
class ServerGoalHandleBuffer(object):
    """Buffer storing a single ``ServerGoalHandle``.
    """
    def __init__(self):
        super().__init__()
        self._lock        = threading.Lock()
        self._goal_handle = None

    def append(self, goal_handle: ServerGoalHandle):
        with self._lock:
            if self._goal_handle is not None:
                self._goal_handle.abort()
            self._goal_handle = goal_handle
        goal_handle.execute()

    def remove(self, goal_handle: ServerGoalHandle):
        with self._lock:
            self._goal_handle = None

class ServerGoalHandleQueue(object):
    """FIFO queue storing ``ServerGoalHandle``.
    """
    def __init__(self):
        super().__init__()
        self._lock  = threading.Lock()
        self._deque = deque()

    def append(self, goal_handle: ServerGoalHandle):
        with self._lock:
            if len(self._deque) == 0:
                goal_handle.execute()
            self._deque.append(goal_handle)

    def remove(self, goal_handle: ServerGoalHandle):
        with self._lock:
            self._deque.remove(goal_handle)
            if len(self._deque) > 0:
                self._deque[0].execute()

class ServerGoalHandlePassthrough(object):
    """Dummy buffer storing no ``ServerGoalHandle``.
    """
    def __init__(self):
        super().__init__()

    def append(self, goal_handle: ServerGoalHandle):
        goal_handle.execute()

    def remove(self, goal_handle: ServerGoalHandle):
        pass

class ServerGoalHandlesDict(object):
    """Dictionary of containers of ``ServerGoalHandle`` with string keys
    """
    def __init__(self,
                 buffer_type: Union[ServerGoalHandleBuffer,
                                    ServerGoalHandleQueue]):
        super().__init__()
        self._buffer_type = buffer_type
        self._dict        = {}

    def append(self, goal_handle: ServerGoalHandle) -> None:
        group_name = goal_handle.request.group_name
        if not group_name in self._dict:
            self._dict[group_name] = self._buffer_type()
        self._dict[group_name].append(goal_handle)

    def remove(self, goal_handle: ServerGoalHandle) -> None:
        group_name = goal_handle.request.group_name
        self._dict[group_name].remove(goal_handle)

#************************************************************************
#  class ActionServer                                                   *
#************************************************************************
class ActionServer(object):
    """ROS Action server supporting multiple policies of processing goals.

    This class wraps ``rclpy.action.server.ActionServer``.
    """
    def __init__(self, node: Node, action_type, action_name: str,
                 execute_callback, *,
                 callback_group: Optional[CallbackGroup]=None,
                 goal_callback=None,
                 goal_processing_policy: str='single',
                 grouping: bool=False):
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
        :param grouping: Enables/Disables grouping of incoming goals
            by a field named `group_name` in the goal request. This parameter
            has no effect when `goal_processing_policy` is 'multi'.
            - If `True`, grouping is enabled and the requests belonging to
              different groups will be processed in parallel while the requests
              in a same group will be processed according to the policy
              specified by ``goal_processing_policy``.
            - If ``False``, grouping is disabled.
        """
        super().__init__()

        self._logger = node.get_logger()

        # Server settings
        if goal_processing_policy == 'single':
            if grouping:
                self._goal_handles \
                    = ServerGoalHandlesDict(ServerGoalHandleBuffer)
            else:
                self._goal_handles = ServerGoalHandleBuffer()
        elif goal_processing_policy == 'queued':
            if grouping:
                self._goal_handles \
                    = ServerGoalHandlesDict(ServerGoalHandleQueue)
            else:
                self._goal_handles = ServerGoalHandleQueue()
        elif goal_processing_policy == 'multi':
            self._goal_handles = ServerGoalHandlePassthrough()
        else:
            raise ValueError()

        self._user_execute_cb = execute_callback
        if not goal_callback:
            goal_callback = self._default_goal_cb
        self._server = rclpy.action.server.ActionServer(
                           node, action_type, action_name,
                           callback_group=callback_group,
                           execute_callback=self._execute_cb,
                           goal_callback=goal_callback,
                           handle_accepted_callback=self._handle_accepted_cb,
                           cancel_callback=self._cancel_cb)
        self._logger.info('action server[%s] started' % action_name)

    @staticmethod
    def goal_id_str(goal_handle):
        s = '0x'
        for i in goal_handle.goal_id.uuid:
            s += format(i, '02x')
        return s

    def _default_goal_cb(self, goal_request):
        self._logger.info('new goal ACCEPTED')
        return GoalResponse.ACCEPT

    def _handle_accepted_cb(self, goal_handle):
        self._goal_handles.append(goal_handle)

    def _cancel_cb(self, goal_handle):
        self._logger.warn('cancel request for goal[%s] received'
                          % ActionServer.goal_id_str(goal_handle))
        return CancelResponse.ACCEPT

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
