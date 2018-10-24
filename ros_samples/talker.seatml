<?xml version="1.0" encoding="UTF-8"?>
<seatml>
  <general name="talker" anonymous="1" rate="1">
    <adaptor name="/chatter" type="ros_pub"
              datatype="std_msgs/String" size="1" />

    <onexec>
      <script>
       hello_str = "hello world %s" % rospy.get_time()
       rospy.loginfo(hello_str)
       seat.ros_publish("/chatter", hello_str)
      </script>
    </onexec>

  </general>

  <state name="main_mode">
<!--
    <label text="Talker" colspan="3" bg_color="blue" />
    <brk />
    <button label="Exit">
     <script>
       seat.exit()
     </script>
    </button>
-->
  </state>
</seatml>
