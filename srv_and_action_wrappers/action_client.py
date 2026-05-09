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
import rclpy.action.client, threading
from action_msgs.msg            import GoalStatus
from action_msgs.srv            import CancelGoal

from typing                     import Optional
from rclpy.node                 import Node
from unique_identifier_msgs.msg import UUID
from builtin_interfaces.msg     import Time

#*********************************************************************
#  class ClientGoalHandle                                            *
#*********************************************************************
class ClientGoalHandle(object):
    """Goal handle for Action Clients with a function awaiting result.

    This class wraps ``rclpy.action.client.ClientGoalHandle``.
    """
    def __init__(self, goal_handle: rclpy.action.client.ClientGoalHandle):
        super().__init__()

        self._goal_handle = goal_handle
        self._result      = None
        self._result_cond = threading.Condition()

    @property
    def accepted(self) -> bool:
        return self._goal_handle.accepted

    @property
    def goal_id(self) -> UUID:
        return self._goal_handle.goal_id

    @property
    def stamp(self) -> Time:
        return self._goal_handle.stamp

    @property
    def status(self) -> int:
        return self._goal_handle.status

    @property
    def goal_id_str(self) -> str:
        """String representation of goal's UUID.

        :return: goal UUID encoded to string.
        """
        s = '0x'
        for i in self.goal_id.uuid:
            s += format(i, '02x')
        return s

    def wait(self, timeout_sec: Optional[float]=None):
        """Wait for the result of the goal request.

        Wait until the response to the goal request issued by
        `ActionClient.send_goal()` with non-positive ``timeout_sec``
        value becomes available.

        :param timeout_sec: If positive, seconds to wait. If ``None``,
            then wait forever. Return immediately otherwise.
        :return: Tuple of the goal status and the action result
            if ``timeout_sec`` is positive or ``None``. Returns tuple
            of the current (non-terminal) goal state and ``None`` otherwise.
        """
        def _result_cb(future):
            with self._result_cond:
                self._result = (future.result().status, future.result().result)
                self._result_cond.notify_all()

        if not self._result:
            self._goal_handle.get_result_async().add_done_callback(_result_cb)
            with self._result_cond:
                if not self._result_cond.wait_for(lambda:
                                                  self._result is not None,
                                                  timeout_sec):
                    return (self.status, None)
        return self._result

    def cancel(self) -> None:
        """Asynchronous request for the goal be canceled.

        Result of the cancel request is available by calling ``wait()``.
        """
        def _cancel_response_cb(future):
            cancel_response = future.result()
            if cancel_response.return_code != CancelGoal.Response.ERROR_NONE:
                with self._result_cond:
                    self._result = (self.status, None)
                    self._result_cond.notify_all()

        self._goal_handle.cancel_goal_async() \
                         .add_done_callback(_cancel_response_cb)

######################################################################
#  class ActionClient                                                #
######################################################################
class ActionClient(object):
    """ROS Action client synchronously awaiting goal handle.

    This class wraps ``rclpy.action.client.ActionClient``.
    """
    _GoalStatus = [
        'UNKNOWN',    # 0: GoalStatus.STATUS_UNKNOWN
        'ACCEPTED',   # 1: GoalStatus.STATUS_ACCEPTED
        'EXECUTING',  # 2: CancelGoal.STATUS_EXECUTING
        'CANCELING',  # 3: CancelGoal.STATUS_CANCELING
        'SUCCEEDED',  # 4: GoalStatus.STATUS_SUCCEEDED
        'CANCELED',   # 5: GoalStatus.STATUS_CANCELED
        'ABORTED',    # 6: GoalStatus.STATUS_ABORTED
    ]

    def __init__(self, node, action_type, action_name: str,
                 callback_group=None):
        """Create an ActionClient.

        :param node: The ROS node to add the action client to.
        :param action_type: Type of the action.
        :param action_name: Name of the action.
            Used as part of the underlying topic and service names.
        :param callback_group: Callback group to add the action client to.
            If None, then the node's default callback group is used.
        """
        super().__init__()

        self._logger = node.get_logger()
        self._client = rclpy.action.client.ActionClient(
                           node, action_type, action_name,
                           callback_group=callback_group)
        self._logger.info('action client[%s] started' % action_name)

    @property
    def logger(self):
        return self._logger

    @staticmethod
    def goal_status_str(status: int) -> str:
        return ActionClient._GoalStatus[status]

    def wait_for_server(self, timeout_sec: Optional[float]=None) -> bool:
        """Wait for a action server to become ready.

        Returns as soon as a server becomes ready or if the timeout expires.

        :param timeout_sec: Seconds to wait. If ``None``, then wait forever.
        :return: ``True`` if server became ready while waiting
            or ``False`` on a timeout.
        """
        if not self._client.wait_for_server(timeout_sec):
            self._logger.error('timeout[%fsec] expired before connection to action server[%s] establised'
                               % (timeout_sec, self._client._action_name))
            return False
        self._logger.info('connection to action server[%s] established'
                          % self._client._action_name)
        return True

    def send_goal(self, goal, feedback_callback=None,
                  timeout_sec: Optional[float]=None):
        """Send a goal request to the server and wait until the corresponding
        goal handle will be returned.

        :param goal: The goal request.
        :param timeout_sec: Seconds to wait. If ``None``, then wait forever.
            Raise ``TimeoutError`` on a timeout.
        :return: The goal handle if the request is accepted.
            Returns ``None`` otherwise.
        """
        goal_handle      = None
        goal_handle_cond = threading.Condition()

        def _goal_response_cb(future):
            nonlocal goal_handle
            goal_handle = ClientGoalHandle(future.result())
            with goal_handle_cond:
                goal_handle_cond.notify_all()

        self._client.send_goal_async(goal,
                                     feedback_callback=feedback_callback) \
                    .add_done_callback(_goal_response_cb)
        with goal_handle_cond:
            if not goal_handle_cond.wait_for(lambda: goal_handle is not None,
                                             timeout_sec):
                self._logger.error('timeout[%fsec] has expired' % timeout_sec)
                raise TimeoutError()
            elif not goal_handle.accepted:
                self._logger.error('goal REJECTED')
                return
            return goal_handle

#*********************************************************************
#  class SimpleActionClient                                          *
#*********************************************************************
class SimpleActionClient(ActionClient):
    """ ROS Action client that tracks only one goal at a time.
    """
    def __init__(self, node: Node, action_type, action_name: str,
                 callback_group=None):
        """Create a SimpleActionClient.

        :param node: The ROS node to add the action client to.
        :param action_type: Type of the action.
        :param action_name: Name of the action.
            Used as part of the underlying topic and service names.
        :param callback_group: Callback group to add the action client to.
            If None, then the node's default callback group is used.
        """
        super().__init__(node, action_type, action_name, callback_group)

        self._goal_handle = None

    def send_goal(self, goal, feedback_callback=None,
                  timeout_sec: Optional[float]=None,
                  goal_handle_timeout_sec: Optional[float]=None):
        """Send a goal request to the server and wait for the result.

        :param goal: The goal request.
        :param timeout_sec: If positive, seconds to wait. If ``None``,
            then wait forever. Return immediately otherwise,
            i.e. asynchronous request.
        :param goal_handle_timeout_sec: Seconds to wait for goal handle.
            If ``None``, then wait forever. Raise ``TimeoutError``
            on a timeout.
        :return: Tuple of the goal status and the action result
            if ``timeout_sec`` is positive or ``None``. Returns tuple
            of the current (non-terminal) goal state and ``None`` otherwise.
        """
        self._goal_handle = super().send_goal(goal, feedback_callback,
                                              goal_handle_timeout_sec)
        return self.wait(timeout_sec)

    @property
    def status(self) -> int:
        return self._goal_handle.status if self._goal_handle is not None else \
               GoalStatus.STATUS_UNKNOWN

    def wait(self, timeout_sec: Optional[float]=None):
        """Wait for the status and the result of the goal or cancel request.

        Wait until the response to the goal request issued by `send_goal()`
        with non-positive ``timeout_sec`` value becomes available.

        :param timeout_sec: If positive, seconds to wait. If ``None``,
            then wait forever. Return immediately otherwise.
        :return: Tuple of the goal status and the action result
            if ``timeout_sec`` is positive or ``None``. Returns tuple
            of the current (non-terminal) goal state and ``None`` otherwise.
        """
        if not self._goal_handle:
            self.logger.error('no goals awaited')
            return (GoalStatus.STATUS_UNKNOWN, None)
        return self._goal_handle.wait(timeout_sec)

    def cancel(self) -> None:
        """Asynchronous request for the current goal be canceled.

        You can get the result of the cancel request by calling ``wait()``.
        """
        if not self._goal_handle:
            return
        self._goal_handle.cancel()

#*********************************************************************
#  class SimpleActionGroupClient                                     *
#*********************************************************************
class SimpleActionGroupClient(ActionClient):
    """ ROS Action client that tracks only one goal for each group at a time.
    """
    def __init__(self, node, action_type, action_name, callback_group=None):
        super().__init__(node, action_type, action_name, callback_group)

        self._goal_handles = {}

    def send_goal(self, goal, feedback_callback=None,
                  timeout_sec: Optional[float]=None,
                  goal_handle_timeout_sec: Optional[float]=None):
        """Send a goal request to the server and wait for the result.

        :param goal: The goal request.
        :param timeout_sec: If positive, seconds to wait. If ``None``,
            then wait forever. Return immediately otherwise,
            i.e. asynchronous request.
        :param goal_handle_timeout_sec: Seconds to wait for goal handle.
            If ``None``, then wait forever. Raise ``TimeoutError``
            on a timeout.
        :return: Tuple of the goal status and the action result
            if ``timeout_sec`` is positive or ``None``. Returns tuple
            of the current (non-terminal) goal state and ``None`` otherwise.
        """
        self._goal_handles[goal.group_name] \
            = super().send_goal(goal, feedback_callback,
                                goal_handle_timeout_sec)
        return self.wait(goal.group_name, timeout_sec)

    def status(self, group_name: str) -> int:
        goal_handle = self._goal_handles.get(group_name)
        return goal_handle.status if goal_handle else GoalStatus.STATUS_UNKNOWN

    def wait(self, group_name: str, timeout_sec: Optional[float]=None):
        """Wait for the status and the result of the goal or cancel request.

        Wait until the response to the goal request issued by `send_goal()`
        with positive on ``None`` ``timeout_sec`` value becomes available.

        :param group_name: Group name of the goal to be waited for.
        :param timeout_sec: If positive, seconds to wait. If ``None``,
            then wait forever. Return immediately otherwise.
        :return: Tuple of the goal status and the action result
            if ``timeout_sec`` is positive or ``None``. Returns tuple
            of the current (non-terminal) goal state and ``None`` otherwise.
        """
        goal_handle = self._goal_handles.get(group_name)
        if not goal_handle:
            self.logger.error('no goals awaited')
            return (GoalStatus.STATUS_UNKNOWN, None)
        return goal_handle.wait(timeout_sec)

    def cancel(self, group_name: str) -> None:
        """Asynchronous request for the current goal be canceled.

        You can get the result of the cancel request by calling ``wait()``.

        :param group_name: Group name of the goal to be canceled.
        """
        goal_handle = self._goal_handles.get(group_name)
        if not goal_handle:
            return
        goal_handle.cancel()
