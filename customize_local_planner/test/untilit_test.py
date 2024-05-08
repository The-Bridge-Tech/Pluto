# the test file for utility in customize_local_planner


from customize_local_planner.untilit import *
def test_roundPwmValue():
    res = roundPwmValue(20,5,10)
    assert res == 10
    
    res = roundPwmValue(20,5,4)
    assert res == 5
    
    res = roundPwmValue(20,5,21)
    assert res == 20
    
    res = roundPwmValue(20,5,20)
    assert res == 20
    
    res = roundPwmValue(20,5,5)
    assert res == 5
    

def test_calculateEulerAngleFromOdometry():
    odomTest = Odometry()


    odomTest.pose.pose.orientation.x = 0.0
    odomTest.pose.pose.orientation.y = 0.0
    odomTest.pose.pose.orientation.z = 0.0
    odomTest.pose.pose.orientation.w = 0.0

    angle = calculateEulerAngleFromOdometry(odomTest)
    assert angle == 0
    # 90 degree
    odomTest.pose.pose.orientation.x = 0.0
    odomTest.pose.pose.orientation.y = 0.0
    odomTest.pose.pose.orientation.z = 0.7071068
    odomTest.pose.pose.orientation.w = 0.7071068
    angle = calculateEulerAngleFromOdometry(odomTest)
    assert round(angle) == 90.00

    odomTest.pose.pose.orientation.x = 0.0
    odomTest.pose.pose.orientation.y = 0.0
    odomTest.pose.pose.orientation.z = 0.9999619
    odomTest.pose.pose.orientation.w =0.0087265
    angle = calculateEulerAngleFromOdometry(odomTest)
    assert round(angle) == 179.0


    odomTest.pose.pose.orientation.x = 0.0
    odomTest.pose.pose.orientation.y = 0.0
    odomTest.pose.pose.orientation.z = 0.9999619
    odomTest.pose.pose.orientation.w =-0.0087265
    angle = calculateEulerAngleFromOdometry(odomTest)
    assert round(angle) == -179.0

def test_calculateEulerAngleFromPoseStamped():
    q = PoseStamped()
    q.pose.orientation.x = 0.0
    q.pose.orientation.y = 0.0
    q.pose.orientation.z = 0.7071068
    q.pose.orientation.w = 0.7071068
    angle = calculateEulerAngleFromPoseStamped(q)
    assert round(angle) == 90.0


def test_relative_angle_between_two_position():
    

    # quadrant 1
    # when goal is ahead of start
    start_position_x = 1.0
    start_position_y = 1.0
    goal_position_x = 2.0
    goal_position_y = 2.0
    relative_angle = relative_angle_between_two_position(start_position_x, start_position_y,math.degrees (atan2(start_position_x, start_position_y)) ,  goal_position_x, goal_position_y)
    assert round(relative_angle) == 0.0

    start_position_x = 2.0
    start_position_y = 2.0
    goal_position_x = 3.0
    goal_position_y = 3.0
    relative_angle = relative_angle_between_two_position(start_position_x, start_position_y,45,  goal_position_x, goal_position_y)
    assert round(relative_angle) == round(0.0)

    start_position_x = 2.0
    start_position_y = 2.0
    goal_position_x = 2.0
    goal_position_y = 3.0
    relative_angle = relative_angle_between_two_position(start_position_x, start_position_y,45,  goal_position_x, goal_position_y)
    assert round(relative_angle) == round(45.0)
