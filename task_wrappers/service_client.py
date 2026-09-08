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
import rclpy, threading

from rclpy.node            import Node
from typing                import Optional
from rclpy.callback_groups import CallbackGroup
#from rclpy.type_support    import Srv, SrvRequestT, SrvResponseT

#************************************************************************
#  class ServiceClient                                                  *
#************************************************************************
class ServiceClient(object):
    """ ROS service client with a method awaiting response from the server.
    This class wraps ``rclpy.Client``.
    """
    def __init__(self,
                 node: Node,
                 srv_type,
                 srv_name: str,
                 *,
                 callback_group: Optional[CallbackGroup]=None):
        """
        Args:
          node: The ROS node to add the service client to.
          srv_type: The service type.
          srv_name: The name of the service.
          callback_group: The callback group for the service client.
            If `None`, then the default callback group for the node is used.
        """
        super().__init__()

        self._logger        = node.get_logger()
        self._response      = None
        self._response_cond = threading.Condition()
        self._client        = node.create_client(srv_type, srv_name,
                                                 callback_group=callback_group)
        self._logger.info('service client[%s] started' % srv_name)

    def wait_for_service(self, timeout_sec: Optional[float]=None) -> bool:
        """ Wait for a service server to become ready.
        Returns as soon as a server becomes ready or if the timeout expires.

        Args:
          timeout_sec: Seconds to wait. If `None`, then wait forever.

        Returns:
          `True` if server became ready while waiting or `False` on a timeout.
        """
        if not self._client.wait_for_service(timeout_sec):
            self._logger.error('timeout[%fsec] expired before connection to service[%s] establised'
                               % (timeout_sec, self._client.srv_name))
            return False
        self._logger.info('connection to service[%s] established'
                          % self._client.srv_name)
        return True

    def call(self, request, *, timeout_sec: Optional[float]=None):
        """ Make a synchronous or asynchronous service request.
        If zero or negative `timeout_sec` value is specified, the response
        to the request should be obtaied by calling `wait()`.

        Args:
          request: The service request.
          timeout_sec: Timeout time waiting for the response.
            Seconds to wait, if positive. Wait forever, if `None`.
            Return immediately, i.e. asynchronous request, if zero or negative.

        Returns:
          Service response, if it becomes available within `timeout_sec`.

        Raises:
          TimeoutError: on a timeout.
        """
        def _response_cb(future):
            with self._response_cond:
                self._response = future.result()
                self._response_cond.notify_all()

        self._response = None
        self._client.call_async(request).add_done_callback(_response_cb)
        if timeout_sec is not None and timeout_sec <= 0.0:
            return
        return self.wait(timeout_sec=timeout_sec)

    def wait(self, *, timeout_sec: Optional[float]=None):
        """ Wait for a response to the service request.
        Blocked until a response to the request issued by `call()` with
        zero or negative `timeout_sec` value becomes available.

        Args:
          timeout_sec: Timeout time waiting for the response.
            Seconds to wait, if positive. Wait forever, if `None`.

        Returns:
          Service response, if it becomes available within `timeout_sec`.

        Raises:
          ValueError: if `timeout_sec` is zero or negative.
          TimeoutError: on a timeout.
        """
        if timeout_sec is not None and timeout_sec <= 0.0:
            raise ValueError('non-positive timeout value[%fsec]' % timeout_sec)

        with self._response_cond:
            if not self._response_cond.wait_for(lambda:
                                                self._response is not None,
                                                timeout_sec):
                raise TimeoutError('timeout[%fsec] has expired' % timeout_sec)
            return self._response
