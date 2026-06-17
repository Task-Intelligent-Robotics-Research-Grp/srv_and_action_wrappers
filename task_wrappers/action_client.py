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
    """ Goal handle for Action Clients with a function awaiting result.
    This class wraps ``rclpy.action.client.ClientGoalHandle``.
    """
    def __init__(self, goal_handle: rclpy.action.client.ClientGoalHandle):
        """
        Args:
          goal_handle: Goal handle to be wrapped.
        """
        super().__init__()

        self._goal_handle  = goal_handle
        self._result       = None
        self._result_cond  = threading.Condition()
        self._target_stage = None

    @property
    def accepted(self) -> bool:
        """ Flag indicating whether this goal handle is accepted or not.
        """
        return self._goal_handle.accepted

    @property
    def goal_id(self) -> UUID:
        """ UUID of this goal handle.
        """
        return self._goal_handle.goal_id

    @property
    def stamp(self) -> Time:
        """ Timestamp of this goal handle.
        """
        return self._goal_handle.stamp

    @property
    def status(self) -> int:
        """ Status of this goal handle.
        """
        return self._goal_handle.status

    @property
    def goal_id_str(self) -> str:
        """ String representation of goal's UUID.
        """
        s = '0x'
        for i in self.goal_id.uuid:
            s += format(i, '02x')
        return s

    def wait(self, *,
             target_stage: Optional[str]=None,
             timeout_sec: Optional[float]=None):
        """ Wait for result of the goal/cancel request.
        Blocked until the result of goal or cancel request issued by
        `ActionClient.send_goal()` or `ClientGoalHandle.cancel_goal()`
        respecitvely becomes available.

        Args:
          target_stage: Stage name waited for, unless `None`.
          timeout_sec: Timeout time waiting for the result. Seconds to wait,
            if positive. Wait forever, if `None`.

        Returns:
          * A tuple of the goal status and the action result,
            if the result becomes available within `timeout_sec`.
          * A tuple of the current (non-terminal) goal status and `None`,
            if the target stage is reached within `timeout_sec`.
          * A tuple of `None` and `None`, otherwise.

        Raises:
          ValueError: if `timeout_sec` is zero or negative.
        """
        if timeout_sec is not None and timeout_sec <= 0.0:
            raise ValueError()

        self._target_stage = target_stage

        def _result_cb(future):
            with self._result_cond:
                self._result = (future.result().status, future.result().result)
                self._result_cond.notify_all()

        if not self._result:
            self._goal_handle.get_result_async().add_done_callback(_result_cb)
            with self._result_cond:
                if not self._result_cond.wait_for(lambda:
                                                  self._result is not None or \
                                                  self._target_stage == '',
                                                  timeout_sec):
                    return (None, None)
        return self._result if self._result else (self.status, None)

    def cancel_goal(self) -> None:
        """Asynchronous request for the goal be canceled.
        Result of the cancel request is available by calling `wait()`.
        """
        def _cancel_response_cb(future):
            cancel_response = future.result()
            if cancel_response.return_code != CancelGoal.Response.ERROR_NONE:
                with self._result_cond:
                    self._result = (self.status, None)
                    self._result_cond.notify_all()

        self._goal_handle.cancel_goal_async() \
                         .add_done_callback(_cancel_response_cb)

    def _reached_stage(self, current_stage):
        if current_stage == self._target_stage:
            with self._result_cond:
                self._target_stage = ''
                self._target_stage_cond.notifyAll()

#*********************************************************************
#  class ActionClient                                                *
#*********************************************************************
class ActionClient(object):
    """ ROS Action client synchronously awaiting goal handle.
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

    def __init__(self, node, action_type, action_name: str, *,
                 callback_group=None):
        """
        Args:
          node: The ROS node to add the action client to.
          action_type: Type of the action.
          action_name: Name of the action.
            Used as part of the underlying topic and service names.
          callback_group: Callback group to add the action client to.
            If None, then the node's default callback group is used.
        """
        super().__init__()

        self._client = rclpy.action.client.ActionClient(
                           node, action_type, action_name,
                           callback_group=callback_group)

        self.logger.info('action client[%s] started' % action_name)

    @property
    def node(self):
        return self._client._node

    @property
    def logger(self):
        return self.node.get_logger()

    @staticmethod
    def goal_status_str(status: int) -> str:
        return ActionClient._GoalStatus[status]

    def wait_for_server(self, timeout_sec: Optional[float]=None) -> bool:
        """ Wait for a action server to become ready.
        Returns as soon as a server becomes ready or if the timeout expires.

        Args:
          timeout_sec: Seconds to wait. If `None`, then wait forever.

        Returns:
          `True` if server became ready while waiting or `False` on a timeout.
        """
        if not self._client.wait_for_server(timeout_sec):
            self.logger.error('timeout[%fsec] expired before connection to action server[%s] establised'
                              % (timeout_sec, self._client._action_name))
            return False
        self.logger.info('connection to action server[%s] established'
                          % self._client._action_name)
        return True

    def send_goal(self, goal, *, feedback_callback=None,
                  goal_handle_timeout_sec: Optional[float]=None):
        """ Send a goal request to the server and wait until the corresponding
        goal handle will be returned.
        This call is synchronous, that is, blocked until the goal handle
        will be returned or the specified timeout expires.

        Args:
          goal: The goal request.
          feedback_callback: Callback function for feedback associated
            with the goal.
          goal_handle_timeout_sec: Timeout time waiting for the goal handle.
            Seconds to wait, if positive. Wait forever, if `None`.

        Returns:
          Goal handle, if the request is accepted within
          `goal_handle_timeout_sec`. `None`, if the request is rejected.

        Raises:
          ValueError: if `goal_handle_timeout_sec` is zero or negative.
          TimeoutError: on a timeout.
        """
        if goal_handle_timeout_sec and goal_handle_timeout_sec <= 0.0:
            raise ValueError()

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
                                             goal_handle_timeout_sec):
                self.logger.error('timeout[%fsec] has expired'
                                  % goal_handle_timeout_sec)
                raise TimeoutError()
            elif not goal_handle.accepted:
                self.logger.error('goal REJECTED')
                return
            return goal_handle

    def stage_feedback_cb(self, feedback):
        feedback.goal_handle._reached_stage(feedback.feedback.current_stage)

#*********************************************************************
#  class SimpleActionClient                                          *
#*********************************************************************
class SimpleActionClient(ActionClient):
    """ ROS action client that tracks only one goal at a time.
    """
    def __init__(self, node: Node, action_type, action_name: str, *,
                 callback_group=None):
        """
        Args:
          node: The ROS node to add the action client to.
          action_type: Type of the action.
          action_name: Name of the action.
            Used as part of the underlying topic and service names.
          callback_group: Callback group to add the action client to.
            If None, then the node's default callback group is used.
        """
        super().__init__(node, action_type, action_name,
                         callback_group=callback_group)

        self._goal_handle = None

    def send_goal(self, goal, *, feedback_callback=None,
                  timeout_sec: Optional[float]=0.0,
                  goal_handle_timeout_sec: Optional[float]=None):
        """Send a goal request to the server and wait for the result.
        After sending request, wait for the goal handle first and then
        wait for its result, if the goal is accepted. If zero or negative
        timeout time is specified, the call returns immediately after
        the goal handle becomes available, that is, asynchronous call.
        In this case, the result should be obtained by calling 'wait()'.

        Args:
          goal: The goal request.
          feedback_callback: Callback function for feedback associated
            with the goal.
          timeout_sec: Timeout time waiting for the result in seconds.
            Seconds to wait for result, if positive. Wait forever, if `None`.
            Return immediately, i.e. asynchronous request, if zero or negative.
          goal_handle_timeout_sec: Timeout time waiting for the goal
            handle in seconds. Seconds to wait for goal handle, if positive.
            Wait forever, if `None`.

        Returns:
          * A tuple of the goal status and the action result,
            if the result becomes available within `timeout_sec`.
          * A tuple of the current (non-terminal) goal state
            and `None` on a timeout or if the goal request has not been
            accepted or goal handle has not been returned within
            `goal_handle_timeout_sec`.

        Raises:
          ValueError: if `goal_handle_timeout_sec` is zero or negative.
          TimeoutError: on a timeout waiting for goal handle.
        """
        goal_handle = super().send_goal(
                          goal, feedback_callback=feedback_callback,
                          goal_handle_timeout_sec=goal_handle_timeout_sec)
        if not goal_handle:
            return GoalStatus.STATUS_UNKNOWN, None  # goal REJECTED

        self._goal_handle = goal_handle             # goal ACCEPTED
        if timeout_sec is not None and timeout_sec <= 0.0:
            return self.status, None

        return self.wait(timeout_sec=timeout_sec)

    @property
    def status(self) -> int:
        """ Status of the underlying goal handle
        """
        return self._goal_handle.status if self._goal_handle else \
               GoalStatus.STATUS_UNKNOWN

    def wait(self, *,
             target_stage: Optional[str]=None,
             timeout_sec: Optional[float]=None):
        """ Wait for status and result of the goal/cancel request.
        Blocked until the result of goal or cancel request issued by
        `send_goal()` or `cancel_goal()` respecitvely becomes available.

        Args:
          target_stage: Stage name waited for,
            or `None` if wait for only the result.
          timeout_sec: Timeout time waiting for the result.
            Seconds to wait, if positive. Wait forever, if `None`.

        Returns:
          * A tuple of the goal status and the action result,
            if the result becomes available within `timeout_sec`.
          * A tuple of the current (non-terminal) goal status and `None`,
            if the target stage is reached within `timeout_sec`.
          * A tuple of `None` and `None`, otherwise.

        Raises:
          ValueError: if `timeout_sec` is zero or negative.
        """
        if timeout_sec is not None and timeout_sec <= 0.0:
            raise ValueError()
        if not self._goal_handle:
            self.logger.error('no goals awaited')
            return GoalStatus.STATUS_UNKNOWN, None
        return self._goal_handle.wait(target_stage=target_stage,
                                      timeout_sec=timeout_sec)

    def cancel_goal(self) -> None:
        """Asynchronous request for the current goal be canceled.
        You can get the result of the cancel request by calling `wait()`.
        """
        if not self._goal_handle:
            return
        self._goal_handle.cancel_goal()

#*********************************************************************
#  class GroupedSimpleActionClient                                   *
#*********************************************************************
class GroupedSimpleActionClient(ActionClient):
    """ ROS action client that tracks only one goal for each group at a time.
    """
    def __init__(self, node: Node, action_type, action_name: str, *,
                 callback_group=None, group_field: str=''):
        """
        Args:
          node: The ROS node to add the action client to.
          action_type: Type of the action.
          action_name: Name of the action.
            Used as part of the underlying topic and service names.
          callback_group: Callback group to add the action client to.
            If None, then the node's default callback group is used.
          group_field: Name of the field in the goal request specifying
            a group.
        """
        super().__init__(node, action_type, action_name,
                         callback_group=callback_group)

        self._group_field  = group_field
        self._goal_handles = {}

    def send_goal(self, goal, *, feedback_callback=None,
                  timeout_sec: Optional[float]=0.0,
                  goal_handle_timeout_sec: Optional[float]=None):
        """ Send a goal request to the server and wait for the result.
        After sending request, wait for the goal handle first.
        If the request is accepted, then wait for its result. If zero
        or negative `timeout_time` is specified, the call returns
        immediately after the goal handle becomes available, that is,
        asynchronous call. In this case, the result should be obtained
        by calling 'wait()'.

        Args:
          goal: Goal request.
          feedback_callback: Callback function for feedback associated
            with the goal.
          timeout_sec: Timeout time waiting for the result in seconds.
            Seconds to wait for result, if positive. Wait forever, if `None`.
            Return immediately, i.e. asynchronous request, if zero or negative.
          goal_handle_timeout_sec: Timeout time waiting for the goal
            handle in seconds.Seconds to wait for goal handle, if positive.
            Wait forever, if `None`.

        Returns:
          * A tuple of the goal status and the action result,
            if the result becomes available within `timeout_sec`.
          * A tuple of the current (non-terminal) goal state and `None`
            on a timeout or if the goal request has not been accepted or goal
            handle has not been returned within `goal_handle_timeout_sec`.

        Raises:
          ValueError: if `goal_handle_timeout_sec` is zero or negative.
          TimeoutError: on a timeout.
        """
        goal_handle = super().send_goal(
                          goal, feedback_callback=feedback_callback,
                          goal_handle_timeout_sec=goal_handle_timeout_sec)
        if not goal_handle:
            return GoalStatus.STATUS_UNKNOWN, None  # goal REJECTED

        group = getattr(goal, self._group_field)
        self._goal_handles[group] = goal_handle
        if timeout_sec is not None and timeout_sec <= 0.0:
            return self.status(group), None
        return self.wait(group, timeout_sec=timeout_sec)

    def status(self, group) -> int:
        """ Status of the underlying goal handle of the specified group
        """
        goal_handle = self._goal_handles.get(group)
        if not goal_handle:
            return GoalStatus.STATUS_UNKNOWN
        return goal_handle.status

    def wait(self, group, *,
             target_stage: Optional[str]=None,
             timeout_sec: Optional[float]=None):
        """ Wait for status and result of the goal/cancel request.
        Blocked until the result of goal or cancel request issued by
        `send_goal()` or `cancel_goal()` respecitvely becomes available.

        Args:
          group: Group of the goal to be waited for.
          target_stage: Stage name waited for,
            or `None` if wait for only the result.
          timeout_sec: Timeout time waiting for the result.
            Seconds to wait, if positive. Wait forever, if `None`.

        Returns:
          * A tuple of the goal status and the action result,
            if the result becomes available within `timeout_sec`.
          * A tuple of the current (non-terminal) goal status and `None`,
            if the target stage is reached within `timeout_sec`.
          * A tuple of `None` and `None`, otherwise.

        Raises:
          ValueError: if `timeout_sec` is zero or negative.
        """
        if timeout_sec is not None and timeout_sec <= 0.0:
            raise ValueError()
        goal_handle = self._goal_handles.get(group)
        if not goal_handle:
            self.logger.error('no goals awaited')
            return GoalStatus.STATUS_UNKNOWN, None
        return goal_handle.wait(target_stage=target_stage,
                                timeout_sec=timeout_sec)

    def cancel_goal(self, group) -> None:
        """ Asynchronous request for the current goal be canceled.
        You can get the result of the cancel request by calling `wait()`.

        Args:
          group: Group of the goal to be canceled.
        """
        goal_handle = self._goal_handles.get(group)
        if not goal_handle:
            return
        goal_handle.cancel_goal()
