<?xml version="1.0" encoding="UTF-8"?>
<seatml>
  <general name="listener" anonymous="1" rate="10">
    <adaptor name="/chatter" type="ros_sub"
              datatype="std_msgs/String" />

  </general>

  <state name="main_mode">
    <label text="Listener" colspan="3" bg_color="blue" />
    <brk />
    <button label="Exit">
      <script>
        seat.exit()
      </script>
    </button>

    <rule source="/chatter">
      <script>
        print("====",rospy.get_caller_id(), rtc_in_data)
        rospy.loginfo( "I heard %s", rtc_in_data)
      </script>
    </rule>
  </state>

</seatml>
