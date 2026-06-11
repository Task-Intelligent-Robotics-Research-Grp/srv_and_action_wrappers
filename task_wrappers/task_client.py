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
from .action_client             import ActionClient

from typing                     import Optional
from rclpy.node                 import Node
from unique_identifier_msgs.msg import UUID
from builtin_interfaces.msg     import Time

#*********************************************************************
#  class TaskClient                                                  *
#*********************************************************************
class TaskClient(ActionClient):
    """ ROS Action client synchronously awaiting goal handle.
    This class wraps ``rclpy.action.client.TaskClient``.
    """
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
        super().__init__(node, action_type, action_name,
                         callback_group=callback_group)
        self._target_stage_cond = threading.Condition()
        self._target_stage = None

        self.logger.info('task client[%s] started' % action_name)

    def send_goal(self, goal, *,
                  goal_handle_timeout_sec: Optional[float]=None):
        """Send a goal request to the server and wait until the corresponding
        goal handle will be returned.
        This call is synchronous, that is, blocked until the goal handle
        will be returned or the specified timeout expires.

        Args:
          goal: The goal request.
          goal_handle_timeout_sec: Timeout time waiting for the goal handle.
            Seconds to wait, if positive. Wait forever, if `None`.

        Returns:
          Goal handle, if the request is accepted within
          `goal_handle_timeout_sec`. `None`, if the request is rejected.

        Raises:
          ValueError: if `goal_handle_timeout_sec` is zero or negative.
          TimeoutError: on a timeout.
        """
        self._current_stage = None
        return super().send_goal(
                   goal, feedback_callback=self._feedback_cb,
                   goal_handle_timeout_sec=goal_handle_timeout_sec)

    def wait_for_stage(self, stage, *, timeout_sec=None):
        self._target_stage = stage
        with self._target_stage_cond:
            return self._target_stage_cond.wait_for(lambda:
                                                    self._target_stage is None,
                                                    timeout_sec)

    def _feedback_cb(self, feedback):
        if feedback.current_stage == self._target_stage:
            with self._target_stage_cond:
                self._target_stage = None
                self._target_stage_cond.notifyAll()

#*********************************************************************
#  class SimpleTaskClient                                            *
#*********************************************************************
class SimpleTaskClient(SimpleActionClient):
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

    def wait(self, *, timeout_sec: Optional[float]=None):
        """ Wait for status and result of the goal/cancel request.
        Blocked until the result of goal or cancel request issued by
        `send_goal()` or `cancel_goal()` respecitvely becomes available.

        Args:
          timeout_sec: Timeout time waiting for the result.
            Seconds to wait, if positive. Wait forever, if `None`.

        Returns:
          * A tuple of the goal status and the action result,
            if the result becomes available within `timeout_sec`.
          * A tuple of the current (non-terminal) goal state
            and `None`. otherwise.

        Raises:
          ValueError: if `timeout_sec` is zero or negative.
        """
        if timeout_sec is not None and timeout_sec <= 0.0:
            raise ValueError()
        if not self._goal_handle:
            self.logger.error('no goals awaited')
            return GoalStatus.STATUS_UNKNOWN, None
        return self._goal_handle.wait(timeout_sec=timeout_sec)

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

        group = getattr(goal_handle.request, self._group_field)
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

    def wait(self, group, *, timeout_sec: Optional[float]=None):
        """ Wait for status and result of the goal/cancel request.
        Blocked until the result of goal or cancel request issued by
        `send_goal()` or `cancel_goal()` respecitvely becomes available.

        Args:
          group: Group of the goal to be waited for.
          timeout_sec: Timeout time waiting for the result.
            Seconds to wait, if positive. Wait forever, if `None`.

        Returns:
          * A tuple of the goal status and the action result,
            if the result becomes available within `timeout_sec`.
          * A tuple of the current (non-terminal) goal state
            and `None`. otherwise.

        Raises:
          ValueError: if `timeout_sec` is zero or negative.
        """
        if timeout_sec is not None and timeout_sec <= 0.0:
            raise ValueError()
        goal_handle = self._goal_handles.get(group)
        if not goal_handle:
            self.logger.error('no goals awaited')
            return GoalStatus.STATUS_UNKNOWN, None
        return goal_handle.wait(timeout_sec=timeout_sec)

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
