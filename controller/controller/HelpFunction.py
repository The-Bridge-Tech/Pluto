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
    

def calculate_velocity_from_pwm( goal_pwm:int, know_velocity:int, know_pwm:int)-> int:
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
    
    return (goal_pwm * know_velocity) / know_pwm
    
    

def convert_negative_pwm_to_positive_pwm_value(max:int, neutral:int, min:int, given_pwm:int):
    if given_pwm >= 0:
        # is positive value
        given_pwm += neutral
        if given_pwm > max:
            return max
        else:
            return given_pwm
    else:
        # is negative value
        new_pwm = neutral + given_pwm
        if new_pwm < min:
            return min
        else:
            return new_pwm
    
# left= -0.2
# right = 0.2
    
# ans1 = calculate_pwm_from_velocity(left, 1,1500)
# ans2 = calculate_pwm_from_velocity(right, 1,1500)

# ans1 = convert_negative_pwm_to_positive_pwm_value(1785,1376,992, ans1)
# ans2 = convert_negative_pwm_to_positive_pwm_value(1785,1376,992, ans2)
# print(int(ans1))
# print(int(ans2))