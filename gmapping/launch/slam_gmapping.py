# Copyright 2017 ROS-M.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


# <launch>
#     <param name="use_sim_time" value="true"/>
#     <node pkg="gmapping" type="slam_gmapping" name="slam_gmapping" output="screen">
#       <remap from="scan" to="base_scan"/>
#       <param name="map_update_interval" value="5.0"/>
#       <param name="maxUrange" value="16.0"/>
#       <param name="sigma" value="0.05"/>
#       <param name="kernelSize" value="1"/>
#       <param name="lstep" value="0.05"/>
#       <param name="astep" value="0.05"/>
#       <param name="iterations" value="5"/>
#       <param name="lsigma" value="0.075"/>
#       <param name="ogain" value="3.0"/>
#       <param name="lskip" value="0"/>
#       <param name="srr" value="0.1"/>
#       <param name="srt" value="0.2"/>
#       <param name="str" value="0.1"/>
#       <param name="stt" value="0.2"/>
#       <param name="linearUpdate" value="1.0"/>
#       <param name="angularUpdate" value="0.5"/>
#       <param name="temporalUpdate" value="3.0"/>
#       <param name="resampleThreshold" value="0.5"/>
#       <param name="particles" value="30"/>
#       <param name="xmin" value="-50.0"/>
#       <param name="ymin" value="-50.0"/>
#       <param name="xmax" value="50.0"/>
#       <param name="ymax" value="50.0"/>
#       <param name="delta" value="0.05"/>
#       <param name="llsamplerange" value="0.01"/>
#       <param name="llsamplestep" value="0.01"/>
#       <param name="lasamplerange" value="0.005"/>
#       <param name="lasamplestep" value="0.005"/>
#     </node>
# </launch>
from ros2run.api import get_executable_path


def launch(descriptor, argv):
    """Launch the slam gmapping node.

    * ld -- the launch descriptor object
    * argv -- the command line arguments

    """
    args = []

    # TODO: support input arguments
    
    slamGmappingExecutable = get_executable_path(
        package_name="gmapping",
        executable_name="slam_gmapping")
    
    descriptor.add_process([slamGmappingExecutable] + args)
