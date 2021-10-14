# Copyright (c) 2013, Oregon State University
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Oregon State University nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL OREGON STATE UNIVERSITY BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Author Dan Lazewatsky/lazewatd@engr.orst.edu

import psutil

ID = '/NODEINFO'


class NodeInfo(object):
    nodes = dict()

    def __init__(self, node):
        self._node = node

    def get_node_info(self, node_name, skip_cache=False):
        if node_name not in self.nodes:
            self.nodes = self.get_all_node_info()
            if node_name not in self.nodes:
                return False
        return self.nodes[node_name]

        # TODO(brawner) ROS2 doesn't currently expose other node's GUIDs or process id in rclpy
        # node_api = rosnode.get_api_uri(rospy.get_master(), node_name, skip_cache=skip_cache)
        # try:
        #     _, _, pid = ServerProxy(node_api[2]).getPid(ID)
        #     if node_name in self.nodes:
        #         return self.nodes[node_name]
        #     else:
        #         try:
        #             p = psutil.Process(pid)
        #             self.nodes[node_name] = p
        #             return p
        #         except e:
        #             return False
        # except SocketError:
        #     if not skip_cache:
        #         return self.get_node_info(node_name, skip_cache=True)
        #     else:
        #         return False

    def get_all_node_info(self):
        infos = []
        node_names = self._node.get_node_names()
        for name in node_names:
            infos.append({'node_name': name})
        return infos

        # TODO(brawner) ROS2 doesn't currently expose other node's GUIDs or process id in rclpy
        # self.remove_dead_nodes()
        # for node_name in rosnode.get_node_names():
        #     info = self.get_node_info(node_name)
        #     if info is not False:
        #         infos.append((node_name, info))
        # return infos

    def get_all_node_fields(self, fields):
        return self.get_all_node_info()

        # TODO(brawner) ROS2 doesn't currently expose other node's GUIDs or process id in rclpy
        # processes = self.get_all_node_info()
        # infos = []
        # psutil_v2_api = int(psutil.__version__.split('.')[0]) >= 2
        # for name, p in processes:
        #     all_fields = fields + ['cmdline', 'get_memory_info']
        #     if psutil_v2_api:
        #         all_fields = [
        #             f[4:] if f.startswith('get_') else f
        #             for f in all_fields]
        #     infos.append(self.as_dict(p, all_fields))
        #     infos[-1]['node_name'] = name
        # return infos

    def remove_dead_nodes(self):
        running_nodes = self._node.get_node_names()
        dead_nodes = [node_name for node_name in self.nodes if node_name not in running_nodes]
        for node_name in dead_nodes:
            self.nodes.pop(node_name, None)

    def kill_node(self, node_name):
        # TODO(brawner) Killing nodes not yet implemented in rclpy
        raise NotImplementedError
        # success, fail = rosnode.kill_nodes([node_name])
        # return node_name in success

    def as_dict(self, p, attrs=[], ad_value=None):
        # copied code from psutil.__init__ from a newer version
        excluded_names = set(['send_signal', 'suspend', 'resume', 'terminate',
                              'kill', 'wait', 'is_running', 'as_dict', 'parent',
                              'get_children', 'nice'])
        retdict = dict()
        for name in set(attrs or dir(p)):
            if name.startswith('_'):
                continue
            if name.startswith('set_'):
                continue
            if name in excluded_names:
                continue
            try:
                attr = getattr(p, name)
                if callable(attr):
                    if name == 'get_cpu_percent':
                        ret = attr(interval=0)
                    else:
                        ret = attr()
                else:
                    ret = attr
            except psutil.AccessDenied:
                ret = ad_value
            except NotImplementedError:
                # in case of not implemented functionality (may happen
                # on old or exotic systems) we want to crash only if
                # the user explicitly asked for that particular attr
                if attrs:
                    raise
                continue
            if name.startswith('get'):
                if name[3] == '_':
                    name = name[4:]
                elif name == 'getcwd':
                    name = 'cwd'
            retdict[name] = ret
        return retdict
