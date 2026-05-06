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
from collections         import deque
from rclpy.node          import Node
from rclpy.action.server import GoalResponse, CancelResponse
from action_msgs.msg     import GoalStatus

#########################################################################
#  stuffs concerning with ServerGoalHandle                              #
#########################################################################
class ServerGoalHandleBuffer(object):
    def __init__(self):
        super().__init__()
        self._lock        = threading.Lock()
        self._goal_handle = None

    def append(self, goal_handle):
        with self._lock:
            if self._goal_handle is not None:
                self._goal_handle.abort()
            self._goal_handle = goal_handle
        goal_handle.execute()

    def remove(self, goal_handle):
        with self._lock:
            self._goal_handle = None

class ServerGoalHandleQueue(object):
    def __init__(self):
        super().__init__()
        self._lock  = threading.Lock()
        self._deque = deque()

    def append(self, goal_handle):
        with self._lock:
            if len(self._deque) == 0:
                goal_handle.execute()
            self._deque.append(goal_handle)

    def remove(self, goal_handle):
        with self._lock:
            self._deque.remove(goal_handle)
            if len(self._deque) > 0:
                self._deque[0].execute()

class ServerGoalHandlePassthrough(object):
    def __init__(self):
        super().__init__()

    def append(self, goal_handle):
        goal_handle.execute()

    def remove(self, goal_handle):
        pass

class ServerGoalHandlesDict(object):
    def __init__(self, buffer_type):
        super().__init__()
        self._buffer_type = buffer_type
        self._dict        = {}

    def append(self, goal_handle):
        group_name = goal_handle.request.group_name
        if not group_name in self._dict:
            self._dict[group_name] = self._buffer_type()
        self._dict[group_name].append(goal_handle)

    def remove(self, goal_handle):
        group_name = goal_handle.request.group_name
        self._dict[group_name].remove(goal_handle)

########################
#  class ActionServer  #
########################
class ActionServer(object):
    def __init__(self, node, action_type, action_name, user_execute_callback,
                 check_goal_request=None, callback_group=None,
                 goal_processing_policy='single', grouping=False):
        super().__init__()

        self._check_goal_request \
            = check_goal_request if check_goal_request else \
              ActionServer._check_goal_request_default
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
        else:
            self._goal_handles = ServerGoalHandlePassthrough()

        self._user_execute_cb = user_execute_callback
        self._server = rclpy.action.server.ActionServer(
                           node, action_type, action_name,
                           callback_group=callback_group,
                           execute_callback=self._execute_cb,
                           goal_callback=self._goal_cb,
                           handle_accepted_callback=self._handle_accepted_cb,
                           cancel_callback=self._cancel_cb)
        self._logger.info('action server[%s] started' % action_name)

    @staticmethod
    def goal_uuid_str(goal_handle):
        s = '0x'
        for i in goal_handle.goal_id.uuid:
            s += format(i, '02x')
        return s

    def _check_goal_request_default(goal_request):
        return True

    def _goal_cb(self, goal_request):
        if not self._check_goal_request(goal_request):
            self._logger.error('new goal REJECTED')
            return GoalResponse.REJECT
        self._logger.info('new goal ACCEPTED')
        return GoalResponse.ACCEPT

    def _handle_accepted_cb(self, goal_handle):
        self._goal_handles.append(goal_handle)

    def _cancel_cb(self, goal_handle):
        self._logger.warn('cancel request for goal[%s] received'
                          % ActionServer.goal_uuid_str(goal_handle))
        return CancelResponse.ACCEPT

    def _execute_cb(self, goal_handle):
        self._logger.info('goal[%s] started'
                          % ActionServer.goal_uuid_str(goal_handle))
        try:
            return self._user_execute_cb(goal_handle)
        finally:
            self._goal_handles.remove(goal_handle)
            if goal_handle.status == GoalStatus.STATUS_SUCCEEDED:
                self._logger.info('goal[%s] SUCCEEDED'
                                  % ActionServer.goal_uuid_str(goal_handle))
            elif goal_handle.status == GoalStatus.STATUS_CANCELED:
                self._logger.warn('goal[%s] CANCELED'
                                  % ActionServer.goal_uuid_str(goal_handle))
            elif goal_handle.status == GoalStatus.STATUS_ABORTED:
                self._logger.error('goal[%s] ABORTED'
                                   % ActionServer.goal_uuid_str(goal_handle))
            else:
                self._logger.error('goal[%s] terminated with status[%d]'
                                   % (ActionServer.goal_uuid_str(goal_handle),
                                      goal_handle.status))
