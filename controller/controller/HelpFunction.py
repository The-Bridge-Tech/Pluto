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
    
    return (goal_velocity * know_velocity) / know_pwm
    

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
    