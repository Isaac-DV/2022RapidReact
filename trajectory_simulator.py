import math
import numpy as np
import matplotlib.pyplot as plt

# TODO
# 1. Aim for the back of the goal instead of the center
# 2. Look for a trajectory that doesn't collide with the front of the goal
# 3. Iterate over the possible hood angles to find a trajectory with (1) minimum RPM or (2) lowest peak height while still clearing the front of the goal by a good amount?
# 4. Generate values for distance-to-velocity map and velocity-to-rpm map

### Shooter characteristics ###
kBottomWheelRadius = 2.0 # inches
kTopWheelRadius = 1.0 # inches
kBottomToMotorSpeedRatio = 1.0
kTopToBottomSpeedRatio = 24.0 / 18.0
kMotorMaxSpeedRPM = 6380.0
kMinExitAngleDegrees = 90.0 - 42.5
kMaxExitAngleDegrees = 69.5172 # 90.0 - 27.0
kGoalRadius = 26.6875 # inches
kGoalHeight = 78.0 # inches
kBallRadius = 4.75 # inches
kInitialBallHeight = 42.0 # inches
kGravitationalAcceleration = 386.22 # inches/sec^2
kScrubFactor = 310.5153 / 418.879 # time = 1.35, distance from goal center = 146.6875; 307.2217

class Vector2d:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    @staticmethod
    def from_polar(direction_radians, magnitude):
        return Vector2d(math.cos(direction_radians) * magnitude, math.sin(direction_radians) * magnitude)

    def __add__(self, other):
        return Vector2d(self.x + other.x, self.y + other.y)

    def distance(self, other):
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

    def angle_radians(self):
        return math.atan2(self.y, self.x)

    def angle_degrees(self):
        return math.degrees(self.angle_radians())

def getInitialBallVelocity(motor_rpm):
    bottom_wheel_angular_velocity = motor_rpm * kBottomToMotorSpeedRatio / 60.0 * 2.0 * math.pi # radians/sec
    bottom_wheel_edge_velocity = bottom_wheel_angular_velocity * kBottomWheelRadius # inches/sec
    #print("Bottom wheel edge velocity: %f"%(bottom_wheel_edge_velocity))

    top_wheel_angular_velocity = motor_rpm * kBottomToMotorSpeedRatio * kTopToBottomSpeedRatio / 60.0 * 2.0 * math.pi # radians/sec
    top_wheel_edge_velocity = top_wheel_angular_velocity * kTopWheelRadius # inches/sec
    #print("Top wheel edge velocity: %f"%(top_wheel_edge_velocity))

    # Return the average of the two edge velocities
    return kScrubFactor * (bottom_wheel_edge_velocity + top_wheel_edge_velocity) / 2.0

def getInitialVelocityVector(motor_rpm, exit_angle_degrees):
    exit_angle_radians = math.radians(exit_angle_degrees)
    velocity_magnitude = getInitialBallVelocity(motor_rpm)
    #print("Initial ball velocity: %f"%(velocity_magnitude))

    return Vector2d.from_polar(exit_angle_radians, velocity_magnitude)

def findTimeToReachGoal(distance_to_goal, motor_rpm, exit_angle_degrees, tolerance):
    initial_ball_velocity = getInitialVelocityVector(motor_rpm, exit_angle_degrees)
    goal_position = Vector2d(distance_to_goal + 0.25 * kGoalRadius, (kGoalHeight - kInitialBallHeight) + 0.25 * (104 - kGoalHeight))
    t_values = np.arange(start=0, stop=3, step=0.01)
    x_values = []
    y_values = []
    top_x_values = []
    top_y_values = []
    bottom_x_values = []
    bottom_y_values = []
    for t in t_values:
        ball_x = initial_ball_velocity.x * t
        ball_y = (-(kGravitationalAcceleration / 2.0) * (t**2)) + (initial_ball_velocity.y * t)
        x_values.append(ball_x)
        y_values.append(ball_y)
        ball_position = Vector2d(ball_x, ball_y)

        ball_velocity_x = initial_ball_velocity.x
        ball_velocity_y = (-kGravitationalAcceleration * t) + initial_ball_velocity.y
        ball_velocity = Vector2d(ball_velocity_x, ball_velocity_y)
        ball_top_direction = ball_velocity.angle_radians() + (math.pi / 2.0)
        ball_bottom_direction = ball_velocity.angle_radians() - (math.pi / 2.0)
        ball_top_position = ball_position + Vector2d.from_polar(ball_top_direction, kBallRadius)
        top_x_values.append(ball_top_position.x)
        top_y_values.append(ball_top_position.y)
        ball_bottom_position = ball_position + Vector2d.from_polar(ball_bottom_direction, kBallRadius)
        bottom_x_values.append(ball_bottom_position.x)
        bottom_y_values.append(ball_bottom_position.y)
        if ball_velocity_y < 0 and ball_position.distance(goal_position) <= tolerance:
            return t, x_values, y_values, top_x_values, top_y_values, bottom_x_values, bottom_y_values, True
        if ball_velocity_y < 0 and ball_y < goal_position.y:
            break

    return 0, x_values, y_values, top_x_values, top_y_values, bottom_x_values, bottom_y_values, False

def findMotorRPM(distance_to_vision_target, exit_angle_degrees):
    distance_to_goal = distance_to_vision_target + kGoalRadius
    rpm_values = np.arange(start=1000, stop=kMotorMaxSpeedRPM, step=1)
    for rpm in rpm_values:
        t, x_values, y_values, top_x_values, top_y_values, bottom_x_values, bottom_y_values, reached_goal = findTimeToReachGoal(distance_to_goal, rpm, exit_angle_degrees, 1.0)
        if reached_goal:
            return rpm, x_values, y_values, top_x_values, top_y_values, bottom_x_values, bottom_y_values, True
    
    return 0, [], [], [], [], [], [], False

def generateDistanceToHorizontalVelocityMap():
    with open("distanceToHorizontalVelocityMap.txt", "w") as file:
        file.write("    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kDistanceToHorizontalVelocity = new InterpolatingTreeMap<>();\n")
        file.write ("    static {\n")
        file.write("        // Key: Distance (inches), Value: Horizontal Velocity (inches/sec)\n")
        distance_values = np.arange(start=45, stop=231, step=2)
        for distance in distance_values:
            rpm, _, _, _, _, _, _, found_rpm = findMotorRPM(distance, kMaxExitAngleDegrees)
            if found_rpm:
                horizontal_velocity = getInitialVelocityVector(rpm, kMaxExitAngleDegrees).x
                file.write("        kDistanceToHorizontalVelocity.put(new InterpolatingDouble(%.1f), new InterpolatingDouble(%.3f));\n"%(distance, horizontal_velocity))
            else:
                print("Couldn't find an RPM for %d inches!"%(distance))
        file.write("    }")

def generateHorizontalVelocityToRPMMap():
    with open("horizontalVelocityToRPMMap.txt", "w") as file:
        file.write("    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kHorizontalVelocityToRPM = new InterpolatingTreeMap<>();\n")
        file.write ("    static {\n")
        file.write("        // Key: Horizontal Velocity (inches/sec), Value: Shooter RPM\n")
        horizontal_velocity_values = np.arange(start=88, stop=145, step=1)
        for horizontal_velocity in horizontal_velocity_values:
            found = False
            for rpm in np.arange(start=1000, stop=kMotorMaxSpeedRPM, step=1):
                ball_velocity = getInitialVelocityVector(rpm, kMaxExitAngleDegrees)
                if abs(ball_velocity.x - horizontal_velocity) < 0.1:
                    file.write("        kHorizontalVelocityToRPM.put(new InterpolatingDouble(%.1f), new InterpolatingDouble(%.3f));\n"%(horizontal_velocity, rpm))
                    found = True
                    break
            if not found:
                print("RPM not found for a horizontal velocity of %d inches/sec!"%(horizontal_velocity))
        file.write("    }")

if __name__ == "__main__":
    '''_, x_values, y_values, _ = findTimeToReachGoal(146.6875, 2400.0, kMaxExitAngleDegrees, 1.0)
    plt.plot(x_values, y_values)
    plt.show()
    '''
    '''distance_to_vision_target = float(input("Enter a distance from the vision target: "))
    rpm, x_values, y_values, top_x_values, top_y_values, bottom_x_values, bottom_y_values, found_rpm = findMotorRPM(distance_to_vision_target, kMaxExitAngleDegrees)
    if found_rpm:
        print("Shooter RPM should be %f"%(rpm))
        distance_to_goal = distance_to_vision_target + kGoalRadius
        goal_points = [(distance_to_goal - kGoalRadius, 104 - kInitialBallHeight), (distance_to_goal - kBallRadius - 2.6875, kGoalHeight - kInitialBallHeight), \
            (distance_to_goal + kBallRadius + 2.6875, kGoalHeight - kInitialBallHeight), (distance_to_goal + kGoalRadius, 104 - kInitialBallHeight)]
        plt.plot([x for (x, y) in goal_points], [y for (x, y) in goal_points], color="red")
        plt.plot(x_values, y_values, color="blue")
        plt.plot(top_x_values, top_y_values, linestyle="--", color="blue")
        plt.plot(bottom_x_values, bottom_y_values, linestyle="--", color="blue")
        plt.title("Ball Trajectory")
        plt.show()
    else:
        print("RPM not found!")
    '''

    generateDistanceToHorizontalVelocityMap()
    generateHorizontalVelocityToRPMMap()