<?xml version='1.0' encoding='UTF-8' standalone='yes' ?>
<tagfile>
  <compound kind="file">
    <name>an_read2joint_states.cpp</name>
    <path>/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin/src/</path>
    <filename>an__read2joint__states_8cpp</filename>
    <member kind="function">
      <type>int</type>
      <name>main</name>
      <anchorfile>an__read2joint__states_8cpp.html</anchorfile>
      <anchor>a3c04138a5bfe5d72780bb7e82a18e627</anchor>
      <arglist>(int argc, char **argv)</arglist>
    </member>
    <member kind="function">
      <type>double</type>
      <name>map2joint_states</name>
      <anchorfile>an__read2joint__states_8cpp.html</anchorfile>
      <anchor>ad9c143b83d941db97bda24178275506f</anchor>
      <arglist>(YAML::Node servo, boost::shared_ptr&lt; const urdf::Joint &gt;::element_type *joint, std::vector&lt; uint16_t &gt; an_read)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>arduino_2_rviz.cpp</name>
    <path>/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin/src/</path>
    <filename>arduino__2__rviz_8cpp</filename>
    <class kind="struct">aux_struct</class>
    <member kind="function">
      <type>int</type>
      <name>main</name>
      <anchorfile>arduino__2__rviz_8cpp.html</anchorfile>
      <anchor>a3c04138a5bfe5d72780bb7e82a18e627</anchor>
      <arglist>(int argc, char **argv)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>receiveData</name>
      <anchorfile>arduino__2__rviz_8cpp.html</anchorfile>
      <anchor>aca3eea8c961337fa5143787e0e60e435</anchor>
      <arglist>(const pumpkin_messages::analog_arrayConstPtr &amp;reads)</arglist>
    </member>
    <member kind="variable">
      <type>std::vector&lt; aux_struct &gt;</type>
      <name>aux_vec</name>
      <anchorfile>arduino__2__rviz_8cpp.html</anchorfile>
      <anchor>a57b1efe07b45508a2520183e73e6ef3e</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>bool</type>
      <name>received</name>
      <anchorfile>arduino__2__rviz_8cpp.html</anchorfile>
      <anchor>a6dbe87354d47906dcbb42c7ba634b102</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>calibrate.cpp</name>
    <path>/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin/src/</path>
    <filename>calibrate_8cpp</filename>
    <member kind="function">
      <type>int</type>
      <name>main</name>
      <anchorfile>calibrate_8cpp.html</anchorfile>
      <anchor>a3c04138a5bfe5d72780bb7e82a18e627</anchor>
      <arglist>(int argc, char **argv)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>load_config.cpp</name>
    <path>/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin/src/</path>
    <filename>load__config_8cpp</filename>
    <member kind="function">
      <type>int</type>
      <name>main</name>
      <anchorfile>load__config_8cpp.html</anchorfile>
      <anchor>a0ddf1224851353fc92bfbff6f499fa97</anchor>
      <arglist>(int argc, char *argv[])</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>mosfet_status_publisher.cpp</name>
    <path>/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin/src/old/</path>
    <filename>mosfet__status__publisher_8cpp</filename>
    <member kind="function">
      <type>int</type>
      <name>main</name>
      <anchorfile>mosfet__status__publisher_8cpp.html</anchorfile>
      <anchor>a3c04138a5bfe5d72780bb7e82a18e627</anchor>
      <arglist>(int argc, char **argv)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>moveit.cpp</name>
    <path>/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin/src/</path>
    <filename>moveit_8cpp</filename>
  </compound>
  <compound kind="file">
    <name>planner.cpp</name>
    <path>/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin/src/</path>
    <filename>planner_8cpp</filename>
    <class kind="class">pumpkin::PumpkinPlanner</class>
    <namespace>pumpkin</namespace>
    <member kind="function">
      <type>int</type>
      <name>main</name>
      <anchorfile>planner_8cpp.html</anchorfile>
      <anchor>a0ddf1224851353fc92bfbff6f499fa97</anchor>
      <arglist>(int argc, char *argv[])</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>playback.cpp</name>
    <path>/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin/src/old/</path>
    <filename>playback_8cpp</filename>
    <member kind="define">
      <type>#define</type>
      <name>SSC_BAUDRATE</name>
      <anchorfile>playback_8cpp.html</anchorfile>
      <anchor>a03640ffbbd7a7f54cee97fa9603d59b4</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>int</type>
      <name>main</name>
      <anchorfile>playback_8cpp.html</anchorfile>
      <anchor>a0ddf1224851353fc92bfbff6f499fa97</anchor>
      <arglist>(int argc, char *argv[])</arglist>
    </member>
    <member kind="function">
      <type>uint16_t</type>
      <name>map</name>
      <anchorfile>playback_8cpp.html</anchorfile>
      <anchor>a6a7286260528c53df8e2f25966003382</anchor>
      <arglist>(YAML::Node servo, std::vector&lt; uint16_t &gt; an_read)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>playback_action.cpp</name>
    <path>/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin/src/</path>
    <filename>playback__action_8cpp</filename>
    <class kind="struct">pumpkin::PlaybackActionServer::auxiliar_calibration</class>
    <class kind="class">pumpkin::PlaybackActionServer</class>
    <namespace>pumpkin</namespace>
    <member kind="function">
      <type>int</type>
      <name>main</name>
      <anchorfile>playback__action_8cpp.html</anchorfile>
      <anchor>a0ddf1224851353fc92bfbff6f499fa97</anchor>
      <arglist>(int argc, char *argv[])</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>playback_joint_states.cpp</name>
    <path>/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin/src/</path>
    <filename>playback__joint__states_8cpp</filename>
    <member kind="function">
      <type>int</type>
      <name>main</name>
      <anchorfile>playback__joint__states_8cpp.html</anchorfile>
      <anchor>a3c04138a5bfe5d72780bb7e82a18e627</anchor>
      <arglist>(int argc, char **argv)</arglist>
    </member>
    <member kind="function">
      <type>double</type>
      <name>map2joint_states</name>
      <anchorfile>playback__joint__states_8cpp.html</anchorfile>
      <anchor>ad9c143b83d941db97bda24178275506f</anchor>
      <arglist>(YAML::Node servo, boost::shared_ptr&lt; const urdf::Joint &gt;::element_type *joint, std::vector&lt; uint16_t &gt; an_read)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>playback_moveit.cpp</name>
    <path>/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin/src/</path>
    <filename>playback__moveit_8cpp</filename>
    <member kind="define">
      <type>#define</type>
      <name>SSC_BAUDRATE</name>
      <anchorfile>playback__moveit_8cpp.html</anchorfile>
      <anchor>a03640ffbbd7a7f54cee97fa9603d59b4</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>fakeControllerJointStatesExecutionCallback</name>
      <anchorfile>playback__moveit_8cpp.html</anchorfile>
      <anchor>a715674100c5b613d72f51f9ec87aca9e</anchor>
      <arglist>(const sensor_msgs::JointStateConstPtr &amp;ptr)</arglist>
    </member>
    <member kind="function">
      <type>int</type>
      <name>main</name>
      <anchorfile>playback__moveit_8cpp.html</anchorfile>
      <anchor>a0ddf1224851353fc92bfbff6f499fa97</anchor>
      <arglist>(int argc, char *argv[])</arglist>
    </member>
    <member kind="function">
      <type>double</type>
      <name>mapJointStates2pumpkin</name>
      <anchorfile>playback__moveit_8cpp.html</anchorfile>
      <anchor>aebe323171f47546e66f93e745256da17</anchor>
      <arglist>(YAML::Node servo, boost::shared_ptr&lt; const urdf::Joint &gt;::element_type *joint, std::vector&lt; double &gt; positions, int positions_i)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>plannedPathCallback</name>
      <anchorfile>playback__moveit_8cpp.html</anchorfile>
      <anchor>a4326085550929ad3433aee64eaee3eef</anchor>
      <arglist>(const moveit_msgs::DisplayTrajectoryConstPtr &amp;trajectories)</arglist>
    </member>
    <member kind="variable">
      <type>std::vector&lt; std::string &gt;</type>
      <name>joint_names</name>
      <anchorfile>playback__moveit_8cpp.html</anchorfile>
      <anchor>a9e7923dd434ecb1c52b6789e40426c26</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>std::vector&lt; boost::shared_ptr&lt; const urdf::Joint &gt;::element_type * &gt;</type>
      <name>joints</name>
      <anchorfile>playback__moveit_8cpp.html</anchorfile>
      <anchor>a23c547f22771af1b15ebd6ce8e31b20c</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>std::vector&lt; trajectory_msgs::JointTrajectoryPoint &gt;</type>
      <name>points</name>
      <anchorfile>playback__moveit_8cpp.html</anchorfile>
      <anchor>a71efd89fee193baac3f5283f935a93ad</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>double</type>
      <name>ros_rate</name>
      <anchorfile>playback__moveit_8cpp.html</anchorfile>
      <anchor>a939db3bd867923e4d9b8cb65d67a2dea</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>std::vector&lt; YAML::Node &gt;</type>
      <name>servos</name>
      <anchorfile>playback__moveit_8cpp.html</anchorfile>
      <anchor>a8e788f5c81b7d9a6fe3e4ad81c850d30</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>std::string</type>
      <name>ssc_port</name>
      <anchorfile>playback__moveit_8cpp.html</anchorfile>
      <anchor>ae64d0f609fe23ff7de172e5ef5207e73</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>pumpkin_trajectory_exec.cpp</name>
    <path>/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin/src/old/</path>
    <filename>pumpkin__trajectory__exec_8cpp</filename>
    <member kind="function">
      <type>void</type>
      <name>fakeControllerJointStatesExecutionCallback</name>
      <anchorfile>pumpkin__trajectory__exec_8cpp.html</anchorfile>
      <anchor>a715674100c5b613d72f51f9ec87aca9e</anchor>
      <arglist>(const sensor_msgs::JointStateConstPtr &amp;ptr)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>jointStatesExecutionCallback</name>
      <anchorfile>pumpkin__trajectory__exec_8cpp.html</anchorfile>
      <anchor>a7158514e4ec57068fd2fdfcdf3fefd86</anchor>
      <arglist>(const sensor_msgs::JointStateConstPtr &amp;ptr)</arglist>
    </member>
    <member kind="function">
      <type>int</type>
      <name>main</name>
      <anchorfile>pumpkin__trajectory__exec_8cpp.html</anchorfile>
      <anchor>a0ddf1224851353fc92bfbff6f499fa97</anchor>
      <arglist>(int argc, char *argv[])</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>trajectorExecutionCallback</name>
      <anchorfile>pumpkin__trajectory__exec_8cpp.html</anchorfile>
      <anchor>a50e2d150f967c08e87f8b6fb1abe4792</anchor>
      <arglist>(const std_msgs::StringConstPtr &amp;str)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>recorder.cpp</name>
    <path>/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin/src/</path>
    <filename>recorder_8cpp</filename>
    <class kind="class">pumpkin::RecordActionServer</class>
    <namespace>pumpkin</namespace>
    <member kind="function">
      <type>int</type>
      <name>main</name>
      <anchorfile>recorder_8cpp.html</anchorfile>
      <anchor>a3c04138a5bfe5d72780bb7e82a18e627</anchor>
      <arglist>(int argc, char **argv)</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>sensor_reads.cpp</name>
    <path>/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin/src/</path>
    <filename>sensor__reads_8cpp</filename>
    <member kind="define">
      <type>#define</type>
      <name>MAX_N_A_READS</name>
      <anchorfile>sensor__reads_8cpp.html</anchorfile>
      <anchor>a2de2e292b960584546cc8076a8ae3b15</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>analogReadCallback</name>
      <anchorfile>sensor__reads_8cpp.html</anchorfile>
      <anchor>a9cd33c4830d8f26dc4a633dac2fc64c7</anchor>
      <arglist>(const pumpkin_messages::analog_arrayConstPtr &amp;msg)</arglist>
    </member>
    <member kind="function">
      <type>int</type>
      <name>main</name>
      <anchorfile>sensor__reads_8cpp.html</anchorfile>
      <anchor>a3c04138a5bfe5d72780bb7e82a18e627</anchor>
      <arglist>(int argc, char **argv)</arglist>
    </member>
    <member kind="function">
      <type>float</type>
      <name>map</name>
      <anchorfile>sensor__reads_8cpp.html</anchorfile>
      <anchor>a264e330f75560ae91aa9bf745916234e</anchor>
      <arglist>(uint16_t x, uint16_t in_min, uint16_t in_max, float out_min, float out_max)</arglist>
    </member>
    <member kind="variable">
      <type>float</type>
      <name>angles</name>
      <anchorfile>sensor__reads_8cpp.html</anchorfile>
      <anchor>aa89f83f2405da29c4b97e061212ec513</anchor>
      <arglist>[MAX_N_A_READS]</arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>avg_a_reads</name>
      <anchorfile>sensor__reads_8cpp.html</anchorfile>
      <anchor>a6f07628841578781c6bf2cd190c99d12</anchor>
      <arglist>[MAX_N_A_READS]</arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>max_a_reads</name>
      <anchorfile>sensor__reads_8cpp.html</anchorfile>
      <anchor>a2002c986114dd7e836b8ccd6a421be6c</anchor>
      <arglist>[MAX_N_A_READS]</arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>min_a_reads</name>
      <anchorfile>sensor__reads_8cpp.html</anchorfile>
      <anchor>a9938b796255ddf11c3c82e407eafdefb</anchor>
      <arglist>[MAX_N_A_READS]</arglist>
    </member>
  </compound>
  <compound kind="file">
    <name>state_publisher.cpp</name>
    <path>/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin/src/</path>
    <filename>state__publisher_8cpp</filename>
    <member kind="define">
      <type>#define</type>
      <name>MAX_N_A_READS</name>
      <anchorfile>state__publisher_8cpp.html</anchorfile>
      <anchor>a2de2e292b960584546cc8076a8ae3b15</anchor>
      <arglist></arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>analogReadCallback</name>
      <anchorfile>state__publisher_8cpp.html</anchorfile>
      <anchor>a9cd33c4830d8f26dc4a633dac2fc64c7</anchor>
      <arglist>(const pumpkin_messages::analog_arrayConstPtr &amp;msg)</arglist>
    </member>
    <member kind="function">
      <type>int</type>
      <name>main</name>
      <anchorfile>state__publisher_8cpp.html</anchorfile>
      <anchor>a3c04138a5bfe5d72780bb7e82a18e627</anchor>
      <arglist>(int argc, char **argv)</arglist>
    </member>
    <member kind="function">
      <type>float</type>
      <name>map</name>
      <anchorfile>state__publisher_8cpp.html</anchorfile>
      <anchor>a264e330f75560ae91aa9bf745916234e</anchor>
      <arglist>(uint16_t x, uint16_t in_min, uint16_t in_max, float out_min, float out_max)</arglist>
    </member>
    <member kind="variable">
      <type>float</type>
      <name>angles</name>
      <anchorfile>state__publisher_8cpp.html</anchorfile>
      <anchor>aa89f83f2405da29c4b97e061212ec513</anchor>
      <arglist>[MAX_N_A_READS]</arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>avg_a_reads</name>
      <anchorfile>state__publisher_8cpp.html</anchorfile>
      <anchor>a6f07628841578781c6bf2cd190c99d12</anchor>
      <arglist>[MAX_N_A_READS]</arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>max_a_reads</name>
      <anchorfile>state__publisher_8cpp.html</anchorfile>
      <anchor>a2002c986114dd7e836b8ccd6a421be6c</anchor>
      <arglist>[MAX_N_A_READS]</arglist>
    </member>
    <member kind="variable">
      <type>uint16_t</type>
      <name>min_a_reads</name>
      <anchorfile>state__publisher_8cpp.html</anchorfile>
      <anchor>a9938b796255ddf11c3c82e407eafdefb</anchor>
      <arglist>[MAX_N_A_READS]</arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>aux_struct</name>
    <filename>structaux__struct.html</filename>
    <member kind="variable">
      <type>double</type>
      <name>filter</name>
      <anchorfile>structaux__struct.html</anchorfile>
      <anchor>a8262fb457aac77cabe82ce6347a64788</anchor>
      <arglist>[6]</arglist>
    </member>
    <member kind="variable">
      <type>double</type>
      <name>in</name>
      <anchorfile>structaux__struct.html</anchorfile>
      <anchor>a888016bd707257e495196e39e6e58569</anchor>
      <arglist>[6]</arglist>
    </member>
    <member kind="variable">
      <type>double</type>
      <name>in_max</name>
      <anchorfile>structaux__struct.html</anchorfile>
      <anchor>a2040747eece635138c487c35dd7d8bf5</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>double</type>
      <name>in_min</name>
      <anchorfile>structaux__struct.html</anchorfile>
      <anchor>a85c30ed7e87af6af57fe3e55036b5263</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>std::string</type>
      <name>joint_name</name>
      <anchorfile>structaux__struct.html</anchorfile>
      <anchor>a21edc7ae954ca74652a5b049393a2296</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>double</type>
      <name>out_lower</name>
      <anchorfile>structaux__struct.html</anchorfile>
      <anchor>ad13511706ffd86a563c7d285581c5dec</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>double</type>
      <name>out_upper</name>
      <anchorfile>structaux__struct.html</anchorfile>
      <anchor>a0a08b081e409b8c6fde339f10ba355d0</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>int</type>
      <name>pin</name>
      <anchorfile>structaux__struct.html</anchorfile>
      <anchor>a9739fab541e554bd57058abb507649b9</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>double</type>
      <name>value</name>
      <anchorfile>structaux__struct.html</anchorfile>
      <anchor>aed55dee1e84f06ab84770c7e7aca7cb9</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="namespace">
    <name>pumpkin</name>
    <filename>namespacepumpkin.html</filename>
    <class kind="class">pumpkin::PlaybackActionServer</class>
    <class kind="class">pumpkin::PumpkinPlanner</class>
    <class kind="class">pumpkin::RecordActionServer</class>
  </compound>
  <compound kind="class">
    <name>pumpkin::PlaybackActionServer</name>
    <filename>classpumpkin_1_1PlaybackActionServer.html</filename>
    <class kind="struct">pumpkin::PlaybackActionServer::auxiliar_calibration</class>
    <member kind="function">
      <type>void</type>
      <name>calibrate</name>
      <anchorfile>classpumpkin_1_1PlaybackActionServer.html</anchorfile>
      <anchor>abd19500a78bed7fba289c61b7ab20c78</anchor>
      <arglist>(int arduino_pin, int arduino_min, int arduino_max, int ssc_min, int ssc_max, int ssc_pin, const std::string &amp;joint_name, double joint_lower, double joint_upper)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>endCalibrate</name>
      <anchorfile>classpumpkin_1_1PlaybackActionServer.html</anchorfile>
      <anchor>a61d849d1145cb8d9f02fa1cea1750a87</anchor>
      <arglist>(double rate)</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>onGoal</name>
      <anchorfile>classpumpkin_1_1PlaybackActionServer.html</anchorfile>
      <anchor>a79e8c6f305dc69067b9dd6103e23a743</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>onPreempt</name>
      <anchorfile>classpumpkin_1_1PlaybackActionServer.html</anchorfile>
      <anchor>aa8d4c17e10292b38daf46efab7a83e4e</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>PlaybackActionServer</name>
      <anchorfile>classpumpkin_1_1PlaybackActionServer.html</anchorfile>
      <anchor>ad5950d3be00aff2faad0e1d072c587cf</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>step</name>
      <anchorfile>classpumpkin_1_1PlaybackActionServer.html</anchorfile>
      <anchor>a914b26a9ff20891b86eec5e0c42c32ec</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~PlaybackActionServer</name>
      <anchorfile>classpumpkin_1_1PlaybackActionServer.html</anchorfile>
      <anchor>a056bf339be63f20c8d017f3cd2fbe071</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="enumeration" protection="private">
      <type></type>
      <name>State</name>
      <anchorfile>classpumpkin_1_1PlaybackActionServer.html</anchorfile>
      <anchor>a18ac5e5d3ce9d0b5b0b474b1c24584f1</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue" protection="private">
      <type>@</type>
      <name>Moving</name>
      <anchorfile>classpumpkin_1_1PlaybackActionServer.html</anchorfile>
      <anchor>a18ac5e5d3ce9d0b5b0b474b1c24584f1a8e8ea508a4a6247fff1b53f0267e2599</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue" protection="private">
      <type>@</type>
      <name>RequestPlanning</name>
      <anchorfile>classpumpkin_1_1PlaybackActionServer.html</anchorfile>
      <anchor>a18ac5e5d3ce9d0b5b0b474b1c24584f1aa304ea4e8a1ca484368950bd401a4657</anchor>
      <arglist></arglist>
    </member>
    <member kind="enumvalue" protection="private">
      <type>@</type>
      <name>ExecutingPlanning</name>
      <anchorfile>classpumpkin_1_1PlaybackActionServer.html</anchorfile>
      <anchor>a18ac5e5d3ce9d0b5b0b474b1c24584f1a51a2335c8004fd36e8d6c969c98e6ce8</anchor>
      <arglist></arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>change</name>
      <anchorfile>classpumpkin_1_1PlaybackActionServer.html</anchorfile>
      <anchor>a257dfb9fc845023d9e0aa18f07293580</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>loadFile</name>
      <anchorfile>classpumpkin_1_1PlaybackActionServer.html</anchorfile>
      <anchor>ae48207a9c8ea70c46fd21458b3ab9f72</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>move</name>
      <anchorfile>classpumpkin_1_1PlaybackActionServer.html</anchorfile>
      <anchor>ab9dbde102a09ea084502fd3ff05badc3</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function" protection="private">
      <type>void</type>
      <name>prepare</name>
      <anchorfile>classpumpkin_1_1PlaybackActionServer.html</anchorfile>
      <anchor>af823744b7894dd9b3a0008b6ab6087e5</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::vector&lt; auxiliar_calibration &gt;</type>
      <name>_aux_vec</name>
      <anchorfile>classpumpkin_1_1PlaybackActionServer.html</anchorfile>
      <anchor>a1cce1e22bb54bc1adeccda3440853469</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::vector&lt; YAML::Node &gt;::iterator</type>
      <name>_end_it</name>
      <anchorfile>classpumpkin_1_1PlaybackActionServer.html</anchorfile>
      <anchor>af926cdeec5b52919fd3da75d63182e36</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>PlaybackFeedback</type>
      <name>_feedback</name>
      <anchorfile>classpumpkin_1_1PlaybackActionServer.html</anchorfile>
      <anchor>a483a5793e0a2bcf5e5ee7a1ad302cdbb</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>sensor_msgs::JointState</type>
      <name>_joint_state</name>
      <anchorfile>classpumpkin_1_1PlaybackActionServer.html</anchorfile>
      <anchor>a977302145546d70908c327f193a21879</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>ros::Publisher</type>
      <name>_joints</name>
      <anchorfile>classpumpkin_1_1PlaybackActionServer.html</anchorfile>
      <anchor>adbebd35d35801d7d2e2d60d5cbc1fa09</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>ros::Rate</type>
      <name>_loop</name>
      <anchorfile>classpumpkin_1_1PlaybackActionServer.html</anchorfile>
      <anchor>a591ab1e3ae40ae130e6c0512752c83d2</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::vector&lt; YAML::Node &gt;</type>
      <name>_movement</name>
      <anchorfile>classpumpkin_1_1PlaybackActionServer.html</anchorfile>
      <anchor>a5d9fb4bc492045ab7ee58cb9b612ef51</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::vector&lt; std::string &gt;</type>
      <name>_movement_files</name>
      <anchorfile>classpumpkin_1_1PlaybackActionServer.html</anchorfile>
      <anchor>a984ed300feffc17d23f2b76447d080e5</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>unsigned int</type>
      <name>_movement_index</name>
      <anchorfile>classpumpkin_1_1PlaybackActionServer.html</anchorfile>
      <anchor>ae021c795e3782eeed1b159c8b6f2ada6</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>ros::NodeHandle</type>
      <name>_nh</name>
      <anchorfile>classpumpkin_1_1PlaybackActionServer.html</anchorfile>
      <anchor>a898eed02b36ca61425970935bf8c6dc3</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>double</type>
      <name>_percentage_step</name>
      <anchorfile>classpumpkin_1_1PlaybackActionServer.html</anchorfile>
      <anchor>a2b78ef869d9a23f3c786635bef3ecb73</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>unsigned int</type>
      <name>_plan_index</name>
      <anchorfile>classpumpkin_1_1PlaybackActionServer.html</anchorfile>
      <anchor>acdaaec6ef24de5eda46278878f01a644</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>Planner</type>
      <name>_planner</name>
      <anchorfile>classpumpkin_1_1PlaybackActionServer.html</anchorfile>
      <anchor>a2e9e62b8ce0b0f36f09d651e5eebb967</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>ros::ServiceClient</type>
      <name>_planner_client</name>
      <anchorfile>classpumpkin_1_1PlaybackActionServer.html</anchorfile>
      <anchor>a06a3bc5be9fcc90e573f81de7863cda8</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>double</type>
      <name>_rate</name>
      <anchorfile>classpumpkin_1_1PlaybackActionServer.html</anchorfile>
      <anchor>a3cacdd231939e5cddad593b58fb130d1</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>PlaybackResult</type>
      <name>_result</name>
      <anchorfile>classpumpkin_1_1PlaybackActionServer.html</anchorfile>
      <anchor>a0f24763b1063faa6ae495a7a1ac60578</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>actionlib::SimpleActionServer&lt; PlaybackAction &gt;</type>
      <name>_server</name>
      <anchorfile>classpumpkin_1_1PlaybackActionServer.html</anchorfile>
      <anchor>ae9bc4eb474410c1a7997ea356688fd3a</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>ros::Publisher</type>
      <name>_ssc</name>
      <anchorfile>classpumpkin_1_1PlaybackActionServer.html</anchorfile>
      <anchor>a7cfe854fa055c9cc0bd852ac88726eca</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>State</type>
      <name>_state</name>
      <anchorfile>classpumpkin_1_1PlaybackActionServer.html</anchorfile>
      <anchor>a4d709e398875d4839094e3826e7b1926</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::vector&lt; YAML::Node &gt;::iterator</type>
      <name>_step_it</name>
      <anchorfile>classpumpkin_1_1PlaybackActionServer.html</anchorfile>
      <anchor>abdcf7a11bb795ef18e0e238ccc9d2439</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="struct">
    <name>pumpkin::PlaybackActionServer::auxiliar_calibration</name>
    <filename>structpumpkin_1_1PlaybackActionServer_1_1auxiliar__calibration.html</filename>
    <member kind="variable">
      <type>int</type>
      <name>arduino_max</name>
      <anchorfile>structpumpkin_1_1PlaybackActionServer_1_1auxiliar__calibration.html</anchorfile>
      <anchor>aab07fe0730c5af8d0216812bae58fcc1</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>int</type>
      <name>arduino_min</name>
      <anchorfile>structpumpkin_1_1PlaybackActionServer_1_1auxiliar__calibration.html</anchorfile>
      <anchor>a560463c0be241f44f2a10e2df40b8f1d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>double</type>
      <name>joint_lower</name>
      <anchorfile>structpumpkin_1_1PlaybackActionServer_1_1auxiliar__calibration.html</anchorfile>
      <anchor>a42e6f2b39301fa15eb1200e99cd007de</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>std::string</type>
      <name>joint_name</name>
      <anchorfile>structpumpkin_1_1PlaybackActionServer_1_1auxiliar__calibration.html</anchorfile>
      <anchor>ad6ad123a7c8f586c1d3b49d49e1c1fd0</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>double</type>
      <name>joint_upper</name>
      <anchorfile>structpumpkin_1_1PlaybackActionServer_1_1auxiliar__calibration.html</anchorfile>
      <anchor>aa21b4d622d6d72f9944b7257dac39d6b</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>int</type>
      <name>ssc_max</name>
      <anchorfile>structpumpkin_1_1PlaybackActionServer_1_1auxiliar__calibration.html</anchorfile>
      <anchor>a18e50c3609c1e41bc15824c2ace96274</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>int</type>
      <name>ssc_min</name>
      <anchorfile>structpumpkin_1_1PlaybackActionServer_1_1auxiliar__calibration.html</anchorfile>
      <anchor>a8a98ebc80f81106d00872173ffd05dcf</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable">
      <type>int</type>
      <name>ssc_pin</name>
      <anchorfile>structpumpkin_1_1PlaybackActionServer_1_1auxiliar__calibration.html</anchorfile>
      <anchor>a0f44fcb96cb5ce587b8b45bbe674df43</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>pumpkin::PumpkinPlanner</name>
    <filename>classpumpkin_1_1PumpkinPlanner.html</filename>
    <member kind="function">
      <type>bool</type>
      <name>plan</name>
      <anchorfile>classpumpkin_1_1PumpkinPlanner.html</anchorfile>
      <anchor>aaf47096f41c3fcd9b4d26ba0fc8401e7</anchor>
      <arglist>(pmsg::PlannerRequest &amp;req, pmsg::PlannerResponse &amp;res)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>PumpkinPlanner</name>
      <anchorfile>classpumpkin_1_1PumpkinPlanner.html</anchorfile>
      <anchor>a613fe79c7c6682b0faf900f37fcd895d</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>rosCall</name>
      <anchorfile>classpumpkin_1_1PumpkinPlanner.html</anchorfile>
      <anchor>ab7a50e679136b6a01e42748478cbb1e8</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>bool</type>
      <name>setJoints</name>
      <anchorfile>classpumpkin_1_1PumpkinPlanner.html</anchorfile>
      <anchor>ae445d7d0009f0fd25be1b525d3a37d60</anchor>
      <arglist>(const std::string &amp;group_name, const std::map&lt; std::string, int &gt; &amp;joints)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~PumpkinPlanner</name>
      <anchorfile>classpumpkin_1_1PumpkinPlanner.html</anchorfile>
      <anchor>ac356c30b97038b1fc26b7b937c89bcac</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::vector&lt; int &gt;</type>
      <name>_indexes</name>
      <anchorfile>classpumpkin_1_1PumpkinPlanner.html</anchorfile>
      <anchor>a83b67562c35dd420444b04b3e1a68c2b</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::vector&lt; robot_state::JointModelGroup * &gt;</type>
      <name>_joint_groups</name>
      <anchorfile>classpumpkin_1_1PumpkinPlanner.html</anchorfile>
      <anchor>a5377a6103d652433c3c525d3bbbedc97</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>robot_model::RobotModelPtr</type>
      <name>_model</name>
      <anchorfile>classpumpkin_1_1PumpkinPlanner.html</anchorfile>
      <anchor>a79bb0d6892b3663780e06178b2ca0b94</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>ros::NodeHandle</type>
      <name>_nh</name>
      <anchorfile>classpumpkin_1_1PumpkinPlanner.html</anchorfile>
      <anchor>affe4cd6eab5938db076fad035b2a2986</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>planning_pipeline::PlanningPipelinePtr</type>
      <name>_plannerPipeline</name>
      <anchorfile>classpumpkin_1_1PumpkinPlanner.html</anchorfile>
      <anchor>a682f460bd9cd48647acd748f569d129d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>planning_scene::PlanningScenePtr</type>
      <name>_planningScene</name>
      <anchorfile>classpumpkin_1_1PumpkinPlanner.html</anchorfile>
      <anchor>a828c3516dc4012523ecd1ce5f59f5d00</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>ros::ServiceServer</type>
      <name>_server</name>
      <anchorfile>classpumpkin_1_1PumpkinPlanner.html</anchorfile>
      <anchor>acca3c3a93fc17454ff2e29fc59570c24</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>ros::CallbackQueue</type>
      <name>_serverQueue</name>
      <anchorfile>classpumpkin_1_1PumpkinPlanner.html</anchorfile>
      <anchor>a9270cecea5728b66f371fbcbc4254bab</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>ros::NodeHandle</type>
      <name>_snh</name>
      <anchorfile>classpumpkin_1_1PumpkinPlanner.html</anchorfile>
      <anchor>a2c51c71b697fde1bacc95ca90b557ca4</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="class">
    <name>pumpkin::RecordActionServer</name>
    <filename>classpumpkin_1_1RecordActionServer.html</filename>
    <member kind="function">
      <type>void</type>
      <name>onGoal</name>
      <anchorfile>classpumpkin_1_1RecordActionServer.html</anchorfile>
      <anchor>a84276d1fc6284333f12e3f0e929c7f03</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>onPreempt</name>
      <anchorfile>classpumpkin_1_1RecordActionServer.html</anchorfile>
      <anchor>a09c043ab4ee4e86d2bab6bbd5f723a83</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="function">
      <type>void</type>
      <name>onRead</name>
      <anchorfile>classpumpkin_1_1RecordActionServer.html</anchorfile>
      <anchor>acfcf2d2b6ffe5bf82ded4c3391ab52a9</anchor>
      <arglist>(const pumpkin_messages::analog_arrayConstPtr &amp;msg)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>RecordActionServer</name>
      <anchorfile>classpumpkin_1_1RecordActionServer.html</anchorfile>
      <anchor>a002a801aa1e2aef8f165023d4957c440</anchor>
      <arglist>(bool setVerbose=false)</arglist>
    </member>
    <member kind="function">
      <type></type>
      <name>~RecordActionServer</name>
      <anchorfile>classpumpkin_1_1RecordActionServer.html</anchorfile>
      <anchor>a660fb36bd31b5c07874a7dc294aa52bc</anchor>
      <arglist>()</arglist>
    </member>
    <member kind="variable" protection="private">
      <type>unsigned int</type>
      <name>_count</name>
      <anchorfile>classpumpkin_1_1RecordActionServer.html</anchorfile>
      <anchor>a22ef2dab88cab295db1cd2b4ab438a2f</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>pumpkin_messages::RecordFeedback</type>
      <name>_feedback</name>
      <anchorfile>classpumpkin_1_1RecordActionServer.html</anchorfile>
      <anchor>a0ce836b1f99dfbdeb7bf2b32f6f9491b</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>std::ofstream</type>
      <name>_file</name>
      <anchorfile>classpumpkin_1_1RecordActionServer.html</anchorfile>
      <anchor>a4418894f5905793c2fb6d1152e6bb931</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>bool</type>
      <name>_first_move</name>
      <anchorfile>classpumpkin_1_1RecordActionServer.html</anchorfile>
      <anchor>a78f2d7374ebe0e2076efb7fdd5453cb5</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>double</type>
      <name>_max_time</name>
      <anchorfile>classpumpkin_1_1RecordActionServer.html</anchorfile>
      <anchor>a0acfbb60b204fbe41dcb01be6ba04473</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>ros::NodeHandle</type>
      <name>_nh</name>
      <anchorfile>classpumpkin_1_1RecordActionServer.html</anchorfile>
      <anchor>a2ad0708351aebdc2ff7629ff4651d7a7</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>YAML::Node</type>
      <name>_node</name>
      <anchorfile>classpumpkin_1_1RecordActionServer.html</anchorfile>
      <anchor>a04f85dab3d380ea7f39de0f12e3e61d7</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>pumpkin_messages::RecordResult</type>
      <name>_result</name>
      <anchorfile>classpumpkin_1_1RecordActionServer.html</anchorfile>
      <anchor>aefd37ee800f8a1aa2c81997d2be7d4e8</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>actionlib::SimpleActionServer&lt; pumpkin_messages::RecordAction &gt;</type>
      <name>_server</name>
      <anchorfile>classpumpkin_1_1RecordActionServer.html</anchorfile>
      <anchor>a1e4bcda965e3d5aa8982b22fe4d3069e</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>ros::Time</type>
      <name>_start</name>
      <anchorfile>classpumpkin_1_1RecordActionServer.html</anchorfile>
      <anchor>ac4194550a2a4abb31316af2f8507681d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>ros::Subscriber</type>
      <name>_sub</name>
      <anchorfile>classpumpkin_1_1RecordActionServer.html</anchorfile>
      <anchor>a3e7ce88a32366befcb77be441dc55a7d</anchor>
      <arglist></arglist>
    </member>
    <member kind="variable" protection="private">
      <type>bool</type>
      <name>_verbose</name>
      <anchorfile>classpumpkin_1_1RecordActionServer.html</anchorfile>
      <anchor>a2f043d452fdea1f3e98ab559745e2b43</anchor>
      <arglist></arglist>
    </member>
  </compound>
  <compound kind="dir">
    <name>old</name>
    <path>/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin/src/old/</path>
    <filename>dir_77960dec01f7fc5224cf962e923b0b85.html</filename>
    <file>mosfet_status_publisher.cpp</file>
    <file>playback.cpp</file>
    <file>pumpkin_trajectory_exec.cpp</file>
  </compound>
  <compound kind="dir">
    <name>pumpkin</name>
    <path>/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin/</path>
    <filename>dir_386ee0f31d88b48c89b6eebccfe75a75.html</filename>
    <dir>src</dir>
  </compound>
  <compound kind="dir">
    <name>src</name>
    <path>/home/rafaelpaiva/workspace/pumpkin_ws/src/pumpkin/src/</path>
    <filename>dir_ec6cac5996f769ddea4c322d6cff7d52.html</filename>
    <dir>old</dir>
    <file>an_read2joint_states.cpp</file>
    <file>arduino_2_rviz.cpp</file>
    <file>calibrate.cpp</file>
    <file>load_config.cpp</file>
    <file>moveit.cpp</file>
    <file>planner.cpp</file>
    <file>playback_action.cpp</file>
    <file>playback_joint_states.cpp</file>
    <file>playback_moveit.cpp</file>
    <file>recorder.cpp</file>
    <file>sensor_reads.cpp</file>
    <file>state_publisher.cpp</file>
  </compound>
</tagfile>
