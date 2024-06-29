### Legend

| Type	| Description   |
| ----- | ------------- | 
| <|--	| Inheritance   |
| *--	| Composition   |
| o--	| Aggregation   |
| -->	| Association   |
| --	| Link (Solid)  |
| ..>	| Dependency    |
| ..|>	| Realization   |
| ..	| Link (Dashed) |


### Class Diagram

```mermaid
classDiagram
        class Node {
                -create_publisher()
                -create_subscription()
                -create_timer()
        }
        Node <|-- PhaseOneDemo
        Node <|-- LocalPlanner
        Node <|-- PidTuningPublisher

        class PhaseOneDemo {
                -tuning_local_plan_publisher
                -tuning_is_pure_pursuit_controller_mode_publisher
                -tuning_is_autonomous_mode_publisher
                -odom_sub
                -gps_sub
                -publish_tuning_plan()
                -gps_fix_callback()
                -globalOdometryCallback()
        }

        class LocalPlanner {
                local_planner_processor()
                determine_local_controller_strategy()
                strategy_simple_factory()
                publish_left_and_right_pwm()
                globalOdometryCallback()
                is_autonomous_state_callback()
                coverage_area_end_pose_callback()
                is_pure_pursuit_mode_callback()
                pure_pursuit_goal_pose_callback()
                local_plan_callback()
        }
        LocalPlanner <.. Controller
        LocalPlanner <.. MovingStraightPIDController
        LocalPlanner <.. TurningPIDController
        LocalPlanner <.. StopController

        class Controller {
                #angle_error_calculation()
                #pid_movement_algorithm()
        }
        Controller <|-- MovingStraightPIDController
        Controller <|-- StopController
        Controller <|-- TurningPIDController
        
        class MovingStraightPIDController {
                +execute_movement()
        }
        class StopController {
                +execute_movement()
        }
        class TurningPIDController {
                +execute_movement()
        }

        class PidTuningPublisher {

        }

        class untilit {
                +roundPwmValue()
                +calculateEulerAngleFromOdometry()
                +calculateEulerAngleFromPoseStamped()
                +crossProductMag()
                +dotProductMag()
                +angle_difference_in_degree()
                +pidCalculation()
                +calc_goal()
        }
        untilit <.. PhaseOneDemo
        untilit <.. Controller
        untilit <.. PidTuningPublisher
```