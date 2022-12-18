

def calculate_pwm_from_velocity( goal_velocity:int, know_velocity:int, know_pwm:int)-> int:
    # we assume that the servo is linear
    """
    Calculate the pwm value for a given velocity

        known velocity        goal velocity
        ---------------   ==  ----------------
        know pwm value        goal pwm value

    
    Parameters
    ----------
    goal_velocity : int

    know_velocity : int

    know_pwm : int

    Returns
    -------
        _description_
        
    Note
    ----
    We assume the servo is linear
    """
    
    
    return (goal_velocity * know_pwm) / know_velocity
def calculate_pwm_from_velocity2( goal_velocity:int, full_forward_velocity, full_backward_velocity, max_pwm,min_pwm,neutral_pwm)-> int:
    if(goal_velocity == 0):
        return neutral_pwm
    elif goal_velocity < 0:
        if goal_velocity < full_backward_velocity:
            goal_velocity = full_backward_velocity
        delta = neutral_pwm - min_pwm
        temp =  (goal_velocity * delta)/full_backward_velocity

        return neutral_pwm-temp
    else:
        if goal_velocity  > full_forward_velocity:
            goal_velocity = full_forward_velocity
        delta = max_pwm-neutral_pwm
        temp = (goal_velocity*delta)/full_forward_velocity
        # print(NEUTRAL+temp)
        return neutral_pwm+temp
# def calculate_velocity_from_pwm( goal_pwm:int, know_velocity:int, know_pwm:int, max, neutral,min)-> int:
#     # we assume that the servo is linear
#     """
#     Calculate the pwm value for a given velocity

#         known velocity        goal velocity
#         ---------------   ==  ----------------
#         know pwm value        goal pwm value

    
#     Parameters
#     ----------
#     goal_velocity : int

#     know_velocity : int

#     know_pwm : int

#     Returns
#     -------
#         _description_
        
#     Note
#     ----
#     We assume the servo is linear
#     """
#     temp_velocity = 0
#     if goal_pwm == neutral:
#         return temp_velocity
#     elif goal_pwm > neutral:
#         if goal_pwm > max:
#             goal_pwm = max
#         goal_pwm = goal_pwm - neutral
#         print(goal_pwm)
#     else:
#         if goal_pwm < min:
#             goal_pwm = min
#         goal_pwm =  goal_pwm - neutral  # should be a negative value
#         #print(goal_pwm)
#     know_pwm_remove_neutral = know_pwm-neutral
#     #print(know_pwm_remove_neutral)
#     temp_velocity =  (goal_pwm * know_velocity) / know_pwm_remove_neutral
#     return temp_velocity
    
def calculate_velocity_from_pwm2(goal_pwm:int, full_forward_velocity, full_backward_velocity, max_pwm,min_pwm,neutral_pwm)-> int:
    if goal_pwm == neutral_pwm:
        return 0
    elif goal_pwm > neutral_pwm:
        delta = max_pwm-neutral_pwm
        temp = goal_pwm
        if temp > max_pwm:
            temp = max_pwm
        pwm_delta = temp-neutral_pwm
        velocity = (pwm_delta*full_forward_velocity)/delta
        # print(delta)
        # print(pwm_delta)
        # print(goal_pwm)
        # print(velocity)
        return velocity
    else:
        temp = goal_pwm
        if temp < min_pwm:
            temp = min_pwm
        delta = neutral_pwm-min_pwm
        pwm_delta = neutral_pwm - temp
        
        velocity = (pwm_delta*full_backward_velocity)/delta
        # print(velocity)
        return velocity

# def convert_negative_pwm_to_positive_pwm_value(max:int, neutral:int, min:int, given_pwm:int):
#     if given_pwm >= 0:
#         # is positive value
#         given_pwm += neutral
#         if given_pwm > max:
#             return max
#         else:
#             return given_pwm
#     else:
#         # is negative value
#         new_pwm = neutral + given_pwm
#         if new_pwm < min:
#             return min
#         else:
#             return new_pwm
    
KNOW_LEFT_FULL_BACKWARD_SPEED= -1.31065
KNOW_RIGHT_FULL_BACKWARD_SPEED= -1.0847
KNOW_LEFT_FULL_FORWARD_SPEED= 1.514
KNOW_RIGHT_FULL_FORWARD_SPEED= 1.5366     # (68 *pi*0.4318)/60
LEFT_NEUTRAL= 1376 # Left neutral pwm
RIGHT_NEUTRAL= 1376
RIGHT_MAX= 1765 # RIGHT_NEUTRAL+ Increase in right servo's pwm
LEFT_MAX= 1765 # LEFT_NEUTRAL+ Increase in left servo's pwm
RIGHT_MIN= 992 # LEFT_NEUTRAL+ Decrease in left servo's pwm
LEFT_MIN= 992  # RIGHT_NEUTRAL+ Decrease in right servo's pwm


res1=calculate_pwm_from_velocity2(-1,KNOW_LEFT_FULL_FORWARD_SPEED, KNOW_LEFT_FULL_BACKWARD_SPEED,LEFT_MAX, LEFT_MIN, LEFT_NEUTRAL)

print(res1)
#calculate_pwm_from_velocity2(0.1,KNOW_LEFT_FULL_FORWARD_SPEED, KNOW_LEFT_FULL_BACKWARD_SPEED,LEFT_MAX, LEFT_MIN, LEFT_NEUTRAL)
# # res1 = (calculate_pwm_from_velocity(-0.1,know_velocity= KNOW_LEFT_FULL_BACKWARD_SPEED,know_pwm=LEFT_MIN))
# calculate_velocity_from_pwm2(1300,KNOW_LEFT_FULL_FORWARD_SPEED,KNOW_LEFT_FULL_BACKWARD_SPEED,LEFT_MAX,LEFT_MIN,LEFT_NEUTRAL)


# print(res1)

# res2 = convert_negative_pwm_to_positive_pwm_value(LEFT_MAX,LEFT_NEUTRAL,LEFT_MIN, res1)
# print(res2)